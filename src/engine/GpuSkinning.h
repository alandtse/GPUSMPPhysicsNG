#pragma once

#include "Utils/LazyShader.h"
#include "smp/Math.h"
#include "smp/SkinPalette.h"

#include <array>
#include <cstdint>
#include <d3d11_1.h>
#include <memory>
#include <mutex>
#include <unordered_map>
#include <vector>

class ConstantBuffer;

namespace SmpPhysics
{
	/** @brief The three rows of the 3x4 skinning matrix the engine builds from a bone's world transform and its skin-to-bone transform. */
	[[nodiscard]] SkinRows ExpectedSkinRows(const RE::NiTransform& a_world, const RE::NiTransform& a_skinToBone);

	/**
	 * @brief Feeds the skinning shaders the GPU solver's pose directly, so the hair the compute shader simulates is
	 *        drawn from this frame's result instead of the one the staged readback delivers a few frames later.
	 *
	 * The engine uploads a skin instance's bone matrices to the vertex shader's b10 (current) and b9 (previous) slots
	 * from CPU memory. For a registered skin instance this class starts from those same matrices, overwrites the rows
	 * of the bones the solver simulates with matrices built from the solver's world poses, and rebinds both slots to
	 * its own constant buffers. The matrices of every other bone stay exactly what the engine computed.
	 */
	class GpuSkinning
	{
	public:
		[[nodiscard]] static std::vector<Util::ShaderJob> ShaderJobs();

		static constexpr std::uint32_t kMaximumBones = 80;                     ///< The engine's per draw limit; a skin instance with more bones is left to the engine.
		static constexpr std::uint32_t kStaticBone = SmpPhysics::kStaticBone;  ///< Marks a bone the solver does not simulate.
		static constexpr std::uint32_t kBytesPerBone = 3 * 16;
		static constexpr std::uint32_t kConstantRegisterBytes = 16;     ///< One float4 constant register.
		static constexpr std::uint32_t kFloatsPerWorldRecord = 8;       ///< A solver world transform: rotation quaternion then translation and padding.
		static constexpr std::uint32_t kPaletteBytes = 240 * 16;        ///< What the shaders declare for b10 and b9.
		static constexpr std::uint32_t kBatchPaletteSlots = 768;        ///< Skin instances the batched palette serves; any further one keeps the per-draw path.
		static constexpr std::uint32_t kBatchStaticCapacity = 4096;     ///< Bones outside the solver the batched palette can follow.
		static constexpr std::uint32_t kBatchNodeBone = 0xFFFFFFFEu;    ///< Marks a batched bone whose world pose comes from its node.
		static constexpr std::uint32_t kBatchUnusedBone = 0xFFFFFFFFu;  ///< Marks a batched record past the skin's last bone.

		/** @brief One skin instance's matrices as this class bound them and as the engine computed them, for the devbench check. */
		struct Capture
		{
			bool valid{};
			std::uint32_t engineFrame{};  ///< The engine frame the skin instance was drawn in when this was read.
			std::uint32_t boneCount{};
			std::vector<std::uint32_t> slots;
			std::vector<SkinRows> current;         ///< What this class bound to b10.
			std::vector<SkinRows> previous;        ///< What this class bound to b9.
			std::vector<SkinRows> engineCurrent;   ///< What the engine computed for b10.
			std::vector<SkinRows> enginePrevious;  ///< What the engine computed for b9.
			std::vector<SkinRows> batchCurrent;    ///< What the batched palette holds for the same frame, empty when this skin instance is not batched.
			std::vector<SkinRows> reference;       ///< b10 rebuilt on the CPU from the solver's own world poses; equals the engine's row for a bone the solver does not simulate.
		};

		/** @brief What the devbench kinematic check measured over the Apply calls since it was armed, for the bones the solver does not simulate. */
		struct KinematicCheck
		{
			bool armed{};
			std::uint32_t samples{};
			std::uint32_t staticBones{};
			float engineVsBindPoseMax{};       ///< Largest row difference between the engine's matrix and one rebuilt from the bone's world pose at bind time.
			float engineVsStepPoseMax{};       ///< The same, rebuilt from the world pose recorded when the solver last stepped.
			float stepVsBindTranslationMax{};  ///< Largest distance the bone's world position moved between that step and the bind.
		};

		GpuSkinning();
		~GpuSkinning();
		GpuSkinning(const GpuSkinning&) = delete;
		GpuSkinning& operator=(const GpuSkinning&) = delete;

		/**
		 * @brief Starts drawing a_skin from the solver's poses.
		 * @param a_slots One entry per skin bone: the solver slot of its world pose, or kStaticBone.
		 * @param a_skinToBone One entry per skin bone, from PackSkinToBone.
		 * @return false for a skin instance this class cannot serve, which keeps the engine's matrices.
		 */
		[[nodiscard]] bool Register(RE::NiSkinInstance* a_skin, std::uint64_t a_token, std::vector<std::uint32_t> a_slots, std::vector<SkinRows> a_skinToBone,
			std::vector<std::uint64_t> a_otherTokens = {});
		/** @brief Forgets every skin instance registered for a_token. */
		void RemoveClaim(std::uint64_t a_token);
		/**
		 * @brief Says whether a_token's skin instances are drawn from the solver this frame.
		 * @param a_worlds The solver's world pose view, kept alive here for as long as the claim stays active.
		 * @param a_solverFrame The solver's frame counter, which tells a new pose from the one already applied.
		 */
		void SetClaimState(std::uint64_t a_token, bool a_active, winrt::com_ptr<ID3D11ShaderResourceView> a_worlds, std::uint64_t a_solverFrame);
		/** @brief Turns the whole pass on or off; while off, every skin instance is drawn from the engine's matrices. */
		void SetEnabled(bool a_enabled);
		/** @brief Runs from the engine's bone binding hook on the render thread; a skin instance that is not registered is left alone. */
		void Apply(RE::NiSkinInstance* a_skin);
		/** @brief Drops the compiled shader so the next Apply recompiles it. */
		void InvalidateShader();
		/** @brief Asks the next Apply of a_skin to read its matrices back, and returns what the previous request read. */
		[[nodiscard]] Capture RequestCapture(const RE::NiSkinInstance* a_skin);
		/** @brief Starts recording the engine's own matrices of a skin instance this class does not serve, each time it is drawn, and returns the latest: valid, engineFrame and engineCurrent only. */
		[[nodiscard]] Capture WatchEngine(const RE::NiSkinInstance* a_skin);
		/** @brief Stops every WatchEngine recording. */
		void ClearWatches();
		/** @brief Turns the batched palette on or off; while off, every skin instance builds its own palette at draw time. */
		void SetBatched(bool a_batched);
		/**
		 * @brief Builds the palettes of every active skin instance in one dispatch from this frame's solver pose.
		 *        Call once per frame, after the solver's step is submitted and SetClaimState has run.
		 */
		void BuildBatch();
		/** @return How many skin instances the last BuildBatch served. */
		[[nodiscard]] std::size_t BatchedSkinCount() const;
		/** @brief Arms the kinematic check for a_skin and clears what it measured; false when a_skin is not registered. */
		[[nodiscard]] bool ArmKinematicCheck(const RE::NiSkinInstance* a_skin);
		/** @return What the kinematic check measured for a_skin so far. */
		[[nodiscard]] KinematicCheck GetKinematicCheck(const RE::NiSkinInstance* a_skin) const;
		/** @brief Records the world pose of every bone the solver does not simulate in a_token's armed skin instances; call after the claim's kinematic input is read. */
		void RecordStepPoses(std::uint64_t a_token);
		[[nodiscard]] std::size_t SkinCount() const;
		/** @brief How many times Apply replaced the engine's matrices since this was last called. */
		[[nodiscard]] std::uint64_t TakeAppliedCount();

	private:
		struct Resources;
		struct Entry
		{
			RE::NiPointer<RE::NiSkinInstance> skin;
			std::uint64_t token{};
			std::vector<std::uint64_t> otherTokens;  ///< Other claims whose simulated bones some of the slots name; all must be posed for the mesh to be drawn from the solver.
			std::vector<std::uint32_t> slots;
			std::vector<SkinRows> skinToBone;
			std::shared_ptr<Resources> resources;
			bool resourcesFailed{};
			bool hasLast{};
			std::uint32_t lastEngineFrame{ 0xFFFFFFFFu };
			std::uint64_t lastSolverFrame{ ~0ull };
			bool captureRequested{};
			KinematicCheck kinematicCheck;
			std::vector<RE::NiTransform> stepPoses;
			bool batchable{};                        ///< Every bone has a node, so a palette can be built without the engine's matrices.
			int batchSlot{ -1 };                     ///< Slot in the shared palette buffers, or -1 while none is held.
			std::vector<std::uint32_t> batchStatic;  ///< For each bone, its index in the batch's node-pose array, or kBatchUnusedBone for a simulated bone.
			std::vector<float> batchScales;          ///< The world scale each bone's record was last written with.
			bool batchHasLast{};
			std::uint64_t batchSerial{};  ///< The BuildBatch call that last wrote this entry's palette.
			Capture capture;
		};
		struct ClaimState
		{
			bool active{};
			winrt::com_ptr<ID3D11ShaderResourceView> worlds;
			std::uint64_t solverFrame{};
		};

		struct Batch;
		[[nodiscard]] bool EnsureBatch();
		void ReleaseBatchSlot(Entry& a_entry);
		/** @brief Whether every other claim a_entry draws bones from is posed in the same world buffer. Called with mutex held. */
		[[nodiscard]] bool OthersPosed(const Entry& a_entry, const ID3D11ShaderResourceView* a_worlds) const;
		[[nodiscard]] bool AssignBatchSlot(Entry& a_entry);
		[[nodiscard]] bool EnsureShader();
		[[nodiscard]] bool CreateResources(Entry& a_entry);
		void Refresh(Entry& a_entry, const ClaimState& a_state, RE::NiSkinInstance* a_skin);
		void ReadCapture(Entry& a_entry, const ClaimState& a_state, RE::NiSkinInstance* a_skin);
		void MeasureKinematicBones(Entry& a_entry, const RE::NiSkinInstance* a_skin);

		mutable std::mutex mutex;
		std::unordered_map<const RE::NiSkinInstance*, Entry> entries;
		std::unordered_map<const RE::NiSkinInstance*, Capture> watches;  ///< Guarded by mutex.
		std::unordered_map<std::uint64_t, ClaimState> claimStates;
		std::unique_ptr<Util::LazyShader<ID3D11ComputeShader>> shader;
		std::unique_ptr<ConstantBuffer> constants;
		std::unique_ptr<Batch> batch;
		bool batched{ true };
		bool batchFailed{};
		std::uint64_t batchSerial{};
		std::size_t batchedCount{};
		bool enabled{};
		bool shaderFailed{};
		std::uint64_t appliedCount{};
	};

	/** @brief The skinning compute shader's constant buffer, mirrored by SkinPaletteCS.hlsl. */
	struct SkinPaletteConstants
	{
		std::uint32_t boneCount{};
		std::uint32_t hasLast{};
		std::uint32_t padding0{};
		std::uint32_t padding1{};
	};
	static_assert(sizeof(SkinPaletteConstants) == 16);

	/** @brief One bone's record for the skinning compute shader, mirrored by SkinPaletteCS.hlsl. */
	struct SkinPaletteBone
	{
		std::uint32_t slot{};
		float worldScale{};
		std::uint32_t padding0{};
		std::uint32_t padding1{};
		SkinRows skinToBone{};
	};
	static_assert(sizeof(SkinPaletteBone) == 64);

	/** @brief One bone's record for the batched skinning compute shader, mirrored by SkinPaletteBatchCS.hlsl. */
	struct SkinPaletteBatchBone
	{
		std::uint32_t slot{};
		float worldScale{};
		std::uint32_t staticIndex{};
		std::uint32_t padding{};
		SkinRows skinToBone{};
	};
	static_assert(sizeof(SkinPaletteBatchBone) == 64);

	/** @brief The batched skinning compute shader's constant buffer, mirrored by SkinPaletteBatchCS.hlsl. */
	struct SkinPaletteBatchConstants
	{
		std::uint32_t activeCount{};
		std::uint32_t padding[3]{};
	};
	static_assert(sizeof(SkinPaletteBatchConstants) == 16);
}
