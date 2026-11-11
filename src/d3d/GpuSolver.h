#pragma once

#include "Buffer.h"
#include "RigidGpuSolver.h"
#include "Utils/LazyShader.h"
#include "smp/GpuRigPacking.h"
#include "smp/Math.h"
#include "smp/SmpRig.h"

#include <BS_thread_pool.hpp>

#include <array>
#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace SmpPhysics
{
	struct GpuSolverConstants;

	/**
	 * @brief Hosts the GPU hair claims: their shared slot pool, collision contact passes and surface prep, with the rigid-body solver stepping the bodies.
	 *
	 * Every claim owns a range of bone slots in buffers shared by all claims, so a single dispatch
	 * advances every claim. Results are read back through a ring of staging buffers and
	 * arrive one to two frames late, which is why they are returned relative to the tree's kinematic
	 * root: the caller applies the current frame's root transform and the attachment never goes stale.
	 */
	class GpuSolver
	{
	public:
		/** @brief Every compute shader the solver host compiles, rigid solver included, for the startup prewarm. */
		[[nodiscard]] static std::vector<Util::ShaderJob> ShaderJobs();

		static constexpr std::uint32_t kBlockSlots = 256;          ///< Slots per pool block; a claim's bones, colliders and drivers live in its own block range.
		static constexpr std::uint32_t kMaximumClaimSlots = 2048;  ///< Widest claim the shared pool serves; a bigger rig is not simulated.
		static constexpr std::size_t kStagingSlots = 3;
		static constexpr std::uint64_t kMaximumResultAge = 4;  ///< Frames after which an unread result is dropped instead of applied.

		/** @brief One claim's inputs for this frame, the compute solver's per-frame input for the claim. */
		struct FrameClaim
		{
			std::uint64_t token{};
			const std::vector<Transform>* kinematicWorld{};                  ///< One per rig bone; dynamic entries are ignored.
			const std::vector<Transform>* driverWorld{};                     ///< One per collision driver, or null for a claim with none.
			const std::vector<GpuRig::ColliderPacked>* externalColliders{};  ///< This frame's kExternalColliderSlots world-space proxies with claim-local driver slots, or null.
			bool hasPreviousInput{};                                         ///< False on the first frame and after the caller reset the solver.
			bool resetRequested{};                                           ///< A teleport, so the tree returns to rest before it is stepped.
			float deltaTime{};
			Vec3 wind{};              ///< The world wind vector, zero for none.
			float windTime{};         ///< The gust clock.
			float windFactor{};       ///< The actor's wind exposure, 0 to 1.
			bool rigidContacts{};     ///< Move the claim's bodies out of the collision proxies every substep.
			std::vector<Grab> grabs;  ///< Points a hand holds, by claim-local rig bone; a tree takes the first kMaxGrabPinsPerTree.
		};

		/** @brief One claim's newest completed result: root-relative, one entry per rig bone. */
		struct ClaimResult
		{
			std::uint64_t token{};
			std::uint64_t frame{};
			bool failed{};  ///< A tree stopped being finite and was reset, so the caller resets its own solver too.
			std::vector<Transform> bones{};
		};

		/** @brief What the GPU collision-surface path holds, for status introspection. */
		struct SurfacePrepInfo
		{
			bool active{};             ///< The compute path produces the frame's surfaces; otherwise the CPU path does.
			std::uint32_t surfaces{};  ///< Metas across every claim.
			std::uint32_t vertices{};
			std::uint32_t triangles{};
			std::uint32_t nodes{};
			std::uint32_t refitLevels{};  ///< Refit dispatches a frame issues.
		};

		GpuSolver() = default;
		~GpuSolver();
		GpuSolver(const GpuSolver&) = delete;
		GpuSolver& operator=(const GpuSolver&) = delete;

		/** @brief False once shader compilation or a buffer creation has failed; the caller then leaves every claim at the engine's pose. */
		[[nodiscard]] bool Available() const noexcept { return available; }
		[[nodiscard]] const std::string& UnavailableReason() const noexcept { return unavailableReason; }
		[[nodiscard]] std::size_t ClaimCount() const noexcept { return claims.size(); }
		/** @brief The collision-surface path's current state and the sizes it was last packed to. */
		[[nodiscard]] SurfacePrepInfo SurfacePrep() const noexcept;

		/** @brief What CheckSurfacePrep found: the last frame's GPU surface pass against the CPU reference. */
		struct SurfacePrepCheckResult
		{
			bool ran{};            ///< False when the check could not run; reason then says why.
			std::string reason{};  ///< Why the check did not run; empty when it ran.
			std::uint32_t triangles{};
			std::uint32_t nodes{};
			float maxTriangleError{};
			float maxNormalError{};
			std::uint32_t triangleMismatches{};
			float maxNodeError{};
			std::uint32_t nodeMismatches{};
			std::uint32_t topologyMismatches{};
		};

		/**
		 * @brief Asks the next SubmitFrame to copy its surface pass back and compare it with GpuRig::ReferenceSurfacePrep.
		 * @details The readback runs on the thread that issues the frame's GPU work, never on the caller's, because the
		 *          immediate context is not thread safe. Safe to call from any thread.
		 */
		void RequestSurfacePrepCheck() const noexcept { surfacePrepCheckRequested.store(true); }

		/**
		 * @return The newest finished check, which compared a frame's GPU output with the driver poses that frame
		 *         wrote, so it is exact up to float rounding; ran is false with a reason until one has run.
		 */
		[[nodiscard]] SurfacePrepCheckResult LastSurfacePrepCheck() const;

		/**
		 * @brief Packs a rig, reserves slots for it and gives its bodies to the rigid-body solver, which starts them
		 *        from that rest pose on the next SubmitFrame.
		 * @param a_collision The claim's packed collision proxies, empty for a rig that has none.
		 * @param a_surfaces The claim's posed collision meshes; boneDriver indexes the driver list every FrameClaim supplies.
		 * @param a_reason Set to why the claim was refused; a rig the pool or the rigid-body solver cannot hold is not
		 *                 simulated without disabling the backend for the claims that do fit.
		 * @return false, registering nothing, when the backend is unavailable or the rig cannot be packed.
		 */
		[[nodiscard]] bool AddClaim(std::uint64_t a_token, const SmpRig& a_rig, const std::vector<Transform>& a_restWorld,
			const SolverParams& a_params, const GpuRig::CollisionData& a_collision, const std::vector<SkinnedSurface>& a_surfaces,
			const std::vector<float>& a_windAreas, std::string& a_reason);
		void RemoveClaim(std::uint64_t a_token);
		/** @brief Uploads every claim's inputs and steps them with the rigid-body solver, then copies the output for readback. */
		[[nodiscard]] bool SubmitFrame(const std::vector<FrameClaim>& a_claims);
		void SetFlushAfterSubmit(bool a_flush) noexcept { flushAfterSubmit = a_flush; }
		/** @return True when a newer completed result was read; a_results then holds one entry per live claim. */
		[[nodiscard]] bool Poll(std::vector<ClaimResult>& a_results);
		/** @brief The counter SubmitFrame stamps into a result's frame; read it right after a successful SubmitFrame. */
		[[nodiscard]] std::uint64_t Frame() const noexcept { return frameCounter; }
		/** @brief Drops the cached compute shaders so the next SubmitFrame recompiles them. */
		void InvalidateShaders();
		/** @brief The rigid-body solver that steps every claim. */
		[[nodiscard]] const RigidGpuSolver& Rigid() const noexcept { return rigidSolver; }
		/** @brief The view over every slot's world pose, which the skinning pass reads; null until the pool exists. The solver's world poses carry no scale. */
		[[nodiscard]] winrt::com_ptr<ID3D11ShaderResourceView> WorldsView() const noexcept { return worldsSrv; }
		/** @return The first slot of a_token's bones, so rig bone i is slot BoneOffset + i; empty for an unknown claim. */
		[[nodiscard]] std::optional<std::uint32_t> BoneOffset(std::uint64_t a_token) const;
		/** @return Whether a_token has been stepped at least once, so its world poses are valid. */
		[[nodiscard]] bool PoseReady(std::uint64_t a_token) const;
		/** @brief The view over every tree's per-frame input; its current translation is the tree's kinematic parent position. */
		[[nodiscard]] winrt::com_ptr<ID3D11ShaderResourceView> InputsView() const noexcept { return inputsSrv; }
		/**
		 * @return For each rig bone of a_token, the slot of the input record of the tree the bone belongs to (the
		 *         same slot as the tree's kinematic parent in InputsView), or kNoTree for a kinematic bone.
		 */
		[[nodiscard]] std::vector<std::uint32_t> BoneTreeSlots(std::uint64_t a_token) const;
		static constexpr std::uint32_t kNoTree = GpuRig::kNoTree;

		/** @brief How many of a claim's bones the last copied contact buffer reported touching a surface, and how deep. */
		struct ContactStat
		{
			std::uint64_t token{};
			std::uint32_t touching{};
			float meanDepth{};
			float maxDepth{};
			std::vector<std::uint32_t> bones;           ///< rig bone indices that were touching
			std::vector<GpuRig::BoneContact> contacts;  ///< the merged contact of each bone in bones, in the same order
		};
		/** @brief Asks for the contact buffer to be copied after the next dispatch; ContactStats reports it a few frames later. */
		void RequestContactStats() const noexcept { contactStatsRequested = true; }
		[[nodiscard]] std::vector<ContactStat> ContactStats() const;

	private:
		struct Claim
		{
			std::uint64_t token{};
			std::uint32_t boneOffset{};
			std::uint32_t boneCount{};
			std::uint32_t slotSpan{};  ///< Slots the claim was allocated: bones, colliders and drivers.
			std::uint32_t colliderOffset{};
			std::uint32_t driverOffset{};
			std::uint32_t driverCount{};
			std::uint32_t sphereBase{};  ///< where this claim's spheres start in the shared sphere list
			GpuRig::RigData rig;         ///< Every stored range is already shifted into the shared slot space.
			std::vector<SkinnedSurface> surfaces;
			std::vector<int> surfaceDrivers;                 ///< the drivers the surfaces are skinned to
			std::vector<GpuRig::SurfaceFrame> surfaceCache;  ///< one per surface, as last built
			std::vector<TriangleBvh> surfaceBvhs;            ///< one per surface, refitted whenever its surface is rebuilt
			std::vector<Transform> cachedDrivers;            ///< the driver list surfaceCache was posed from; empty until built
			bool surfaceTopologyReady{};                     ///< the surfaceBvhs were built from the claim's first supplied drivers
			std::uint32_t surfaceFirst{};                    ///< this claim's first record in the GPU surface metas
			std::uint32_t surfaceCount{};                    ///< how many records the claim added to them
			SolverParams params;
			std::vector<GpuRig::TreeInput> inputs;     ///< One per tree, refreshed every submitted frame.
			std::vector<Transform> previousKinematic;  ///< One per tree: its kinematic parent's transform last frame.
			std::uint64_t firstFrame{};                ///< The first frame this claim's output is in the output buffer; zero until then.
		};

		struct Readback
		{
			winrt::com_ptr<ID3D11Buffer> buffer;
			bool pending{};
			bool hasResult{};
			std::uint64_t serial{};
			std::uint64_t frame{};
			std::vector<ClaimResult> result;
		};

		[[nodiscard]] bool EnsureShaders();
		/** @brief Creates the shared buffers for a_slots slots, carrying the first a_previousSlots slots of state across; false when a buffer could not be allocated. */
		[[nodiscard]] bool CreateBuffers(std::uint32_t a_slots, std::uint32_t a_previousSlots);
		[[nodiscard]] bool CreateBuffersUnchecked(std::uint32_t a_slots, std::uint32_t a_previousSlots);
		[[nodiscard]] std::optional<std::uint32_t> AllocateSlots(std::uint32_t a_needed);
		void PlanCapacity(std::uint32_t a_slots);
		void ReleaseSlots(std::uint32_t a_offset, std::uint32_t a_count);
		void UploadStatics();
		void Fail(const char* a_reason);
		bool EnsureSurfacePrepShaders();
		void RebuildSurfacePrep();
		[[nodiscard]] bool CreateSurfacePrepBuffers();
		void DisableSurfacePrep(const char* a_reason);
		void RunSurfacePrepCheck();
		[[nodiscard]] bool SubmitFrameUnchecked(const std::vector<FrameClaim>& a_claims);

		/** @brief What one frame uploads, gathered from the claims before any buffer is written. */
		struct FramePack
		{
			std::vector<GpuRig::TreeInput> treeInputs;
			std::vector<GpuRig::DriverWorld> driverWorlds;
			std::vector<std::size_t> startedClaims;               ///< Claims whose first-frame number this frame took.
			std::vector<RigidGpuSolver::FrameClaim> rigidClaims;  ///< Claims the rigid-body solver steps this frame.
		};
		void PrepareFrameSurfaces(const std::vector<FrameClaim>& a_claims);
		[[nodiscard]] bool PackFrameClaims(const std::vector<FrameClaim>& a_claims, FramePack& a_pack);
		/** @brief Gives back the first-frame state a pack took, for a frame that failed to upload. */
		void UndoFramePack(const FramePack& a_pack);
		[[nodiscard]] bool UploadFramePack(const FramePack& a_pack);
		void DispatchSurfacePrep();
		/** @brief Records the contact passes, from the bone poses the worlds buffer holds. */
		void DispatchContacts(const GpuSolverConstants& a_values);

		std::vector<Claim> claims;
		std::unordered_map<std::uint64_t, std::size_t> claimIndex;
		RigidGpuSolver rigidSolver;

		std::unique_ptr<Buffer> statics;
		std::unique_ptr<Buffer> trees;
		std::unique_ptr<Buffer> order;
		std::unique_ptr<Buffer> spheres;
		std::unique_ptr<Buffer> colliders;
		std::unique_ptr<Buffer> inputs;
		std::unique_ptr<Buffer> drivers;
		std::unique_ptr<Buffer> boneTrees;   ///< Per bone slot, the tree slot it belongs to, or kNoTree for a kinematic bone.
		std::unique_ptr<Buffer> contacts;    ///< Per bone slot, its merged contact at the start of the substep in flight.
		std::unique_ptr<Buffer> boneCull;    ///< Per bone slot, which surfaces its spheres can reach and whether they are queried this substep.
		std::unique_ptr<Buffer> sphereHits;  ///< Per sphere list entry, its contact at the start of the substep; sized with the list.
		std::unique_ptr<Buffer> sphereList;  ///< Every claim's spheres, rewritten whole whenever the claims change.
		std::uint32_t sphereListCapacity{};
		std::vector<GpuRig::SphereRecord> sphereListShadow;  ///< The sphere list as uploaded, so an added claim appends to it instead of rebuilding it.
		winrt::com_ptr<ID3D11ShaderResourceView> sphereListSrv;
		std::unique_ptr<Buffer> surfaceMetas;  ///< The three surface buffers are rewritten whole every frame and grow on demand.
		std::unique_ptr<Buffer> surfaceTriangles;
		std::unique_ptr<Buffer> surfaceNodes;
		std::uint32_t surfaceMetaCapacity{};
		std::uint32_t surfaceTriangleCapacity{};
		std::uint32_t surfaceNodeCapacity{};
		std::unique_ptr<Buffer> surfacePrepVertices;  ///< The static surface prep records, rebuilt when the claims change.
		std::unique_ptr<Buffer> surfacePrepBones;
		std::unique_ptr<Buffer> surfacePrepTriangles;
		std::unique_ptr<Buffer> surfacePrepNodes;
		std::unique_ptr<Buffer> surfacePrepLeaves;
		std::unique_ptr<Buffer> surfacePrepRefit;
		std::unique_ptr<Buffer> surfacePrepMetas;
		std::unique_ptr<Buffer> posedVertices;  ///< The prep pass's scratch: one posed position per vertex.
		std::unique_ptr<Buffer> skinTranslations;
		std::unique_ptr<Buffer> skinRotations;
		std::unique_ptr<Buffer> gpuSurfaceTriangles;  ///< What the contact pass reads in place of the uploaded arrays.
		std::unique_ptr<Buffer> gpuSurfaceNodes;
		winrt::com_ptr<ID3D11ShaderResourceView> surfacePrepVerticesSrv;
		winrt::com_ptr<ID3D11ShaderResourceView> surfacePrepBonesSrv;
		winrt::com_ptr<ID3D11ShaderResourceView> surfacePrepTrianglesSrv;
		winrt::com_ptr<ID3D11ShaderResourceView> surfacePrepNodesSrv;
		winrt::com_ptr<ID3D11ShaderResourceView> surfacePrepLeavesSrv;
		winrt::com_ptr<ID3D11ShaderResourceView> surfacePrepRefitSrv;
		winrt::com_ptr<ID3D11ShaderResourceView> surfacePrepMetasSrv;
		winrt::com_ptr<ID3D11ShaderResourceView> gpuSurfaceTrianglesSrv;
		winrt::com_ptr<ID3D11ShaderResourceView> gpuSurfaceNodesSrv;
		winrt::com_ptr<ID3D11UnorderedAccessView> posedVerticesUav;
		winrt::com_ptr<ID3D11UnorderedAccessView> skinTranslationsUav;
		winrt::com_ptr<ID3D11UnorderedAccessView> skinRotationsUav;
		winrt::com_ptr<ID3D11UnorderedAccessView> gpuSurfaceTrianglesUav;
		winrt::com_ptr<ID3D11UnorderedAccessView> gpuSurfaceNodesUav;
		GpuRig::SurfacePrepData surfacePrep;
		bool surfacePrepActive{};
		bool surfacePrepDirty{};
		mutable std::atomic<bool> surfacePrepCheckRequested{};
		mutable std::mutex surfacePrepCheckMutex;
		SurfacePrepCheckResult surfacePrepCheckResult{ false, "no surface check has run yet; call surfaceCheck again after the next frame" };
		bool surfacePrepWarned{};  ///< The reason the GPU path was last disabled is already logged.
		std::unique_ptr<Buffer> worlds;
		std::unique_ptr<Buffer> outputs;
		winrt::com_ptr<ID3D11ShaderResourceView> worldsSrv;
		winrt::com_ptr<ID3D11ShaderResourceView> boneTreesSrv;
		winrt::com_ptr<ID3D11ShaderResourceView> contactsSrv;
		winrt::com_ptr<ID3D11ShaderResourceView> staticsSrv;
		winrt::com_ptr<ID3D11ShaderResourceView> treesSrv;
		winrt::com_ptr<ID3D11ShaderResourceView> orderSrv;
		winrt::com_ptr<ID3D11ShaderResourceView> spheresSrv;
		winrt::com_ptr<ID3D11ShaderResourceView> collidersSrv;
		winrt::com_ptr<ID3D11ShaderResourceView> inputsSrv;
		winrt::com_ptr<ID3D11ShaderResourceView> driversSrv;
		winrt::com_ptr<ID3D11ShaderResourceView> surfaceMetasSrv;
		winrt::com_ptr<ID3D11ShaderResourceView> surfaceTrianglesSrv;
		winrt::com_ptr<ID3D11ShaderResourceView> surfaceNodesSrv;
		winrt::com_ptr<ID3D11UnorderedAccessView> worldsUav;
		winrt::com_ptr<ID3D11UnorderedAccessView> outputsUav;
		winrt::com_ptr<ID3D11UnorderedAccessView> contactsUav;
		winrt::com_ptr<ID3D11UnorderedAccessView> boneCullUav;
		winrt::com_ptr<ID3D11UnorderedAccessView> sphereHitsUav;
		std::uint32_t sphereHitCapacity{};
		std::uint32_t sphereCount{};  ///< Entries in the sphere list as last uploaded.
		std::unique_ptr<ConstantBuffer> constants;
		std::array<Readback, kStagingSlots> readbacks;
		mutable std::atomic<bool> contactStatsRequested{};
		bool flushAfterSubmit{ true };
		winrt::com_ptr<ID3D11Buffer> contactStaging;
		std::uint32_t contactStagingSlots{};
		bool contactStagingPending{};
		mutable std::mutex contactStatsMutex;
		std::vector<ContactStat> contactStats;
		std::unique_ptr<BS::thread_pool<>> surfacePool;  ///< refits each claim's surface hierarchies at once
		GpuRig::SurfaceFrame surfaceFrame;               ///< the frame's merged surface arrays, kept so their capacity carries over
		std::unique_ptr<Util::LazyShader<ID3D11ComputeShader>> contactCullShader;
		std::unique_ptr<Util::LazyShader<ID3D11ComputeShader>> contactSphereShader;
		std::unique_ptr<Util::LazyShader<ID3D11ComputeShader>> contactReduceShader;
		std::unique_ptr<ConstantBuffer> surfacePrepConstants;  ///< Mirrored by SurfacePrepCS.hlsl.
		std::unique_ptr<Util::LazyShader<ID3D11ComputeShader>> surfaceBonesShader;
		std::unique_ptr<Util::LazyShader<ID3D11ComputeShader>> surfacePoseShader;
		std::unique_ptr<Util::LazyShader<ID3D11ComputeShader>> surfaceTrianglesShader;
		std::unique_ptr<Util::LazyShader<ID3D11ComputeShader>> surfaceLeavesShader;
		std::unique_ptr<Util::LazyShader<ID3D11ComputeShader>> surfaceRefitShader;

		std::vector<bool> freeBlocks;  ///< Blocks no claim holds, in units of kBlockSlots bone slots.
		std::uint32_t slotCapacity{};
		std::uint32_t treeHighWater{};    ///< One past the highest tree slot any claim uses; the dispatch bound.
		std::uint32_t boneHighWater{};    ///< One past the highest bone slot any claim uses; the contact pass's dispatch bound.
		std::uint32_t driverHighWater{};  ///< One past the highest driver slot any claim uses; how much of the driver buffer a frame writes.
		std::uint64_t frameCounter{};
		std::uint64_t nextSerial{};
		bool staticsDirty{};
		std::vector<std::size_t> pendingStaticClaims;  ///< Claims added since the statics were uploaded; written into their own slots on the next upload unless a rebuild is due.
		bool available{ true };
		std::string unavailableReason;
	};

	/** @brief The contact passes' constant buffer, mirrored by ContactCS.hlsl. */
	struct GpuSolverConstants
	{
		std::uint32_t boneCount{};  ///< One past the highest bone slot any claim uses.
		std::uint32_t sphereCount{};
		std::uint32_t padding[2]{};
	};
	static_assert(sizeof(GpuSolverConstants) == 16);

	/** @brief The surface prep compute shader's constant buffer, mirrored by SurfacePrepCS.hlsl. */
	struct GpuSurfacePrepConstants
	{
		std::uint32_t boneCount{};
		std::uint32_t vertexCount{};
		std::uint32_t triangleCount{};
		std::uint32_t leafCount{};
		std::uint32_t refitFirst{};  ///< The first entry this refit dispatch covers.
		std::uint32_t refitCount{};  ///< How many entries it covers.
		std::uint32_t padding[2]{};
	};
	static_assert(sizeof(GpuSurfacePrepConstants) == 32);
}
