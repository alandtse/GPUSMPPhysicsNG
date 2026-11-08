#pragma once

#include "Buffer.h"
#include "smp/RigidGpuPacking.h"
#include "smp/RigidReference.h"
#include "smp/SolverCommon.h"
#include "smp/Math.h"
#include "Utils/LazyShader.h"

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
	 *          chain solver's world and output buffers, so skinning and the CPU readback treat a rigid-driven claim
	 *          like a chain-driven one. Body state lives on the GPU and persists across frames.
	 */
	class RigidGpuSolver
	{
	public:
		/** @brief What a claim needs to be stepped: the world SMP would build and where its bodies sit in the chain solver's slots. */
		struct ClaimSetup
		{
			std::uint64_t token{};
			std::uint32_t boneOffset{};  ///< Chain solver slot of rig bone 0; rig bone i is slot boneOffset + i.
			RigidReference::World world;
			std::vector<int> rootBody;            ///< Per body, the kinematic body its pose is exported relative to, or -1.
			std::vector<Transform> restRelative;  ///< Per body, its rest pose relative to its root body's rest pose.
		};

		/** @brief One claim's inputs for a frame. */
		struct FrameClaim
		{
			std::uint64_t token{};
			const std::vector<Transform>* kinematicWorld{};  ///< One per rig bone; dynamic entries are ignored.
			bool hasPreviousInput{};
			bool resetRequested{};
			float deltaTime{};
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
		 * @brief Advances every listed claim one frame and writes the dynamic poses into the chain solver's buffers.
		 * @param a_worlds The chain solver's world pose target.
		 * @param a_outputs The chain solver's root-relative output target.
		 * @param a_contacts The chain solver's per-bone merged contacts; with a_contactPass, bodies are moved out of the colliders it reports.
		 * @param a_contactPass Records the chain solver's contact passes for a substep (1-based) once the bodies' poses are exported.
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
		};
		static_assert(sizeof(FrameInput) == 48);

		struct BodyMeta
		{
			RigidGpu::Uint4 slots{};
			RigidGpu::Float4 restRotation{};
			RigidGpu::Float4 restTranslation{};
		};
		static_assert(sizeof(BodyMeta) == 48);

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
