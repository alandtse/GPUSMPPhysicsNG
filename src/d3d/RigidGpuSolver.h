#pragma once

#include "Buffer.h"
#include "Utils/LazyShader.h"
#include "smp/Math.h"
#include "smp/RigidGpuPacking.h"
#include "smp/RigidReference.h"
#include "smp/SolverCommon.h"

#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace SmpPhysics
{
	/**
	 * @brief Steps claims as free rigid bodies joined by 6-DOF spring constraints, the way hdtSMP64's Bullet world does.
	 * @details One dispatch sequence advances every claim: RigidFrameBegin arms the bodies, each substep runs the
	 *          predict, setup, island-solve and integrate kernels, and RigidExport writes the dynamic poses into the
	 *          GPU solver's world and output buffers, so skinning and the CPU readback treat a rigid-driven claim
	 *          like a chain-driven one. Body state lives on the GPU and persists across frames.
	 */
	class RigidGpuSolver
	{
	public:
		/** @brief What a claim needs to be stepped: the world SMP would build and where its bodies sit in the GPU solver's slots. */
		struct ClaimSetup
		{
			std::uint64_t token{};
			std::uint32_t boneOffset{};  ///< GPU solver slot of rig bone 0; rig bone i is slot boneOffset + i.
			RigidReference::World world;
			std::vector<int> rootBody;            ///< Per body, the kinematic body its pose is exported relative to, or -1.
			std::vector<Transform> restRelative;  ///< Per body, its rest pose relative to its root body's rest pose.
			std::vector<int> parentBody;          ///< Per body, the body its joint hangs from, or -1; a pin frees the joints on that path.
			/** @brief What wind drag needs of a body: its strand direction in its own frame, the area it presents and its wind factor. */
			struct WindBody
			{
				Vec3 axis{};
				float area{};
				float factor{};
			};
			std::vector<WindBody> wind;  ///< Per body, or empty for no wind.
		};

		/** @brief One claim's inputs for a frame. */
		struct FrameClaim
		{
			std::uint64_t token{};
			const std::vector<Transform>* kinematicWorld{};  ///< One per rig bone; dynamic entries are ignored.
			bool hasPreviousInput{};
			bool resetRequested{};
			float deltaTime{};
			Vec3 wind{};         ///< The world wind vector, zero for none.
			float windTime{};    ///< The gust clock.
			float windFactor{};  ///< The actor's wind exposure, 0 to 1.
			/** @brief A point of a body a hand holds. */
			struct Pin
			{
				int body{ -1 };
				Vec3 local{};
				Vec3 target{};
				float stiffness{};
			};
			std::vector<Pin> pins;  ///< One per body; a body takes the first pin that names it.
		};

		RigidGpuSolver() = default;
		~RigidGpuSolver();
		RigidGpuSolver(const RigidGpuSolver&) = delete;
		RigidGpuSolver& operator=(const RigidGpuSolver&) = delete;

		[[nodiscard]] bool Available() const noexcept { return available; }
		[[nodiscard]] const std::string& UnavailableReason() const noexcept { return unavailableReason; }
		[[nodiscard]] std::size_t ClaimCount() const noexcept { return claims.size(); }
		[[nodiscard]] bool HasClaim(std::uint64_t a_token) const { return claimIndex.contains(a_token); }

		/** @return false, registering nothing, when the world cannot be packed or the backend is unavailable. */
		[[nodiscard]] bool AddClaim(ClaimSetup&& a_setup, std::string& a_reason);
		void RemoveClaim(std::uint64_t a_token);
		/** @brief Drops the cached compute shaders so the next Dispatch recompiles them. */
		void InvalidateShaders();

		/**
		 * @brief Advances every listed claim one frame and writes the dynamic poses into the GPU solver's buffers.
		 * @param a_worlds The GPU solver's world pose target.
		 * @param a_outputs The GPU solver's root-relative output target.
		 * @param a_contacts The GPU solver's per-bone merged contacts; with a_contactPass, bodies are moved out of the colliders it reports.
		 * @param a_contactPass Records the GPU solver's contact passes for a substep (1-based) once the bodies' poses are exported.
		 */
		[[nodiscard]] bool Dispatch(const std::vector<FrameClaim>& a_frame, ID3D11UnorderedAccessView* a_worlds,
			ID3D11UnorderedAccessView* a_outputs, ID3D11ShaderResourceView* a_contacts = nullptr,
			const std::function<void(std::uint32_t)>& a_contactPass = {});

	private:
		struct Claim
		{
			ClaimSetup setup;
			RigidGpu::StepData data;  ///< Packed once at claim time; indices are claim-local until the shared buffers are built.
			std::uint32_t bodyBase{};
			std::uint32_t constraintBase{};
			std::uint32_t islandBase{};
			bool stepped{};  ///< The claim has been stepped at least once, so its bodies hold live state.
		};

		struct FrameInput
		{
			RigidGpu::Float4 rotation{};
			RigidGpu::Float4 translation{};
			RigidGpu::Uint4 flags{};
			RigidGpu::Float4 wind{};        ///< xyz the world wind vector, w the gust clock
			RigidGpu::Float4 windScale{};   ///< x the actor's wind exposure
			RigidGpu::Float4 grabLocal{};   ///< xyz the held point in the body's frame, w the pin's stiffness
			RigidGpu::Float4 grabTarget{};  ///< xyz where the hand holds it, w 1 for a pin
		};
		static_assert(sizeof(FrameInput) == 112);

		struct BodyMeta
		{
			RigidGpu::Uint4 slots{};
			RigidGpu::Float4 restRotation{};
			RigidGpu::Float4 restTranslation{};
			RigidGpu::Float4 windAxis{};    ///< xyz the strand's direction in the body's frame, w the wind-facing area
			RigidGpu::Float4 windParams{};  ///< x the bone's own wind factor
		};
		static_assert(sizeof(BodyMeta) == 80);

		struct RuntimeConstants
		{
			std::uint32_t bodyCount{};
			float frameTime{};
			std::uint32_t padding[2]{};
		};
		static_assert(sizeof(RuntimeConstants) == 16);

		[[nodiscard]] bool EnsureShaders();
		void Fail(const char* a_reason);
		/** @brief Rebuilds the shared buffers from the claims, carrying the surviving claims' body state across. */
		[[nodiscard]] bool RebuildBuffers();

		std::vector<Claim> claims;
		std::unordered_map<std::uint64_t, std::size_t> claimIndex;
		bool dirty{};
		bool available{ true };
		std::string unavailableReason;

		std::uint32_t bodyCount{};
		std::uint32_t constraintCount{};
		std::uint32_t islandCount{};
		std::unique_ptr<Buffer> bodies;
		std::unique_ptr<Buffer> solverBodies;
		std::unique_ptr<Buffer> rows;
		std::unique_ptr<Buffer> rowMasks;
		std::unique_ptr<Buffer> constraints;
		std::unique_ptr<Buffer> islands;
		std::unique_ptr<Buffer> islandConstraints;
		std::unique_ptr<Buffer> coloredConstraints;
		std::unique_ptr<Buffer> colorOffsets;
		std::unique_ptr<Buffer> frameInputs;
		std::unique_ptr<Buffer> bodyMeta;
		std::unique_ptr<ConstantBuffer> stepConstants;
		std::unique_ptr<ConstantBuffer> runtimeConstants;
		std::vector<FrameInput> frameScratch;
		SmpStepClock stepClock;

		std::unique_ptr<Util::LazyShader<ID3D11ComputeShader>> frameBeginShader;
		std::unique_ptr<Util::LazyShader<ID3D11ComputeShader>> predictShader;
		std::unique_ptr<Util::LazyShader<ID3D11ComputeShader>> setupShader;
		std::unique_ptr<Util::LazyShader<ID3D11ComputeShader>> solveShader;
		std::unique_ptr<Util::LazyShader<ID3D11ComputeShader>> integrateShader;
		std::unique_ptr<Util::LazyShader<ID3D11ComputeShader>> exportShader;
		std::unique_ptr<Util::LazyShader<ID3D11ComputeShader>> worldExportShader;
	};
}
