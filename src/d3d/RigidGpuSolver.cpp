#include "RigidGpuSolver.h"

#include "Globals.h"
#include "GpuPass.h"
#include "Utils/D3D.h"
#include "smp/GpuViews.h"

#include <algorithm>
#include <cstring>
#include <exception>
#include <utility>

namespace SmpPhysics
{
	namespace
	{
		constexpr const wchar_t* kStepShaderPath = L"SmpPhysics\\RigidSolverCS.hlsl";
		constexpr const wchar_t* kRuntimeShaderPath = L"SmpPhysics\\RigidRuntimeCS.hlsl";
		constexpr const char* kShaderTarget = "cs_5_0";
		constexpr const char* kLogPrefix = "[GPU SMP Physics]";
		constexpr std::uint32_t kThreadsPerGroup = 64;
		constexpr std::uint32_t kStepViewCount = 6;
		constexpr std::uint32_t kStepTargetCount = 4;
		constexpr std::uint32_t kRuntimeViewCount = 3;
		constexpr std::uint32_t kRuntimeTargetCount = 3;
		constexpr std::uint32_t kContactTargetCount = 7;  // UAV slots the chain contact passes leave bound; one of them is the contact buffer
		constexpr std::uint32_t kNoBody = 0xFFFFFFFFu;

		std::uint32_t Groups(std::uint32_t a_threads) noexcept
		{
			return std::max(1u, (a_threads + kThreadsPerGroup - 1u) / kThreadsPerGroup);
		}

		template <typename T>
		std::unique_ptr<Buffer> MakeStatic(const std::vector<T>& a_data, const char* a_name)
		{
			const std::uint32_t count = std::max<std::uint32_t>(1u, static_cast<std::uint32_t>(a_data.size()));
			std::vector<T> padded = a_data;
			padded.resize(count);
			D3D11_SUBRESOURCE_DATA init{ padded.data(), 0, 0 };
			auto buffer = std::make_unique<Buffer>(StructuredBufferDesc<T>(count, false, false), &init, a_name);
			buffer->CreateSRV(StructuredSrvDesc(count));
			return buffer;
		}

		template <typename T>
		std::unique_ptr<Buffer> MakeWritable(const std::vector<T>* a_initial, std::uint32_t a_count, const char* a_name, bool a_srv)
		{
			const std::uint32_t count = std::max(1u, a_count);
			std::unique_ptr<Buffer> buffer;
			if (a_initial && !a_initial->empty()) {
				D3D11_SUBRESOURCE_DATA init{ a_initial->data(), 0, 0 };
				buffer = std::make_unique<Buffer>(StructuredBufferDesc<T>(count, true, false), &init, a_name);
			} else {
				buffer = std::make_unique<Buffer>(StructuredBufferDesc<T>(count, true, false), nullptr, a_name);
			}
			buffer->CreateUAV(StructuredUavDesc(count));
			if (a_srv)
				buffer->CreateSRV(StructuredSrvDesc(count));
			return buffer;
		}

		RigidGpu::Float4 Quat4(const Quat& a_value) noexcept
		{
			return { a_value.x, a_value.y, a_value.z, a_value.w };
		}
	}

	RigidGpuSolver::~RigidGpuSolver() = default;

	void RigidGpuSolver::Fail(const char* a_reason)
	{
		available = false;
		unavailableReason = a_reason;
		logger::warn("{} {}", kLogPrefix, a_reason);
	}

	void RigidGpuSolver::InvalidateShaders()
	{
		for (auto* shader : { &frameBeginShader, &predictShader, &setupShader, &solveShader, &integrateShader, &exportShader, &worldExportShader }) {
			if (*shader)
				(*shader)->Reset();
		}
	}

	std::vector<Util::ShaderJob> RigidGpuSolver::ShaderJobs()
	{
		return {
			{ kRuntimeShaderPath, kShaderTarget, "RigidFrameBegin" },
			{ kRuntimeShaderPath, kShaderTarget, "RigidExport" },
			{ kRuntimeShaderPath, kShaderTarget, "RigidWorldExport" },
			{ kStepShaderPath, kShaderTarget, "RigidPredict" },
			{ kStepShaderPath, kShaderTarget, "RigidSetup" },
			{ kStepShaderPath, kShaderTarget, "RigidSolveIslandColored" },
			{ kStepShaderPath, kShaderTarget, "RigidIntegrate" },
		};
	}

	bool RigidGpuSolver::EnsureShaders()
	{
		if (!available)
			return false;
		if (!globals::d3d::device) {
			Fail("the rigid solver needs a Direct3D device; claimed hair stays at the engine's pose");
			return false;
		}
		const auto make = [](std::unique_ptr<Util::LazyShader<ID3D11ComputeShader>>& a_shader) {
			if (!a_shader)
				a_shader = std::make_unique<Util::LazyShader<ID3D11ComputeShader>>();
		};
		make(frameBeginShader);
		make(predictShader);
		make(setupShader);
		make(solveShader);
		make(integrateShader);
		make(exportShader);
		make(worldExportShader);
		const std::vector<std::pair<const char*, const char*>> defines;
		try {
			if (!frameBeginShader->Get(kRuntimeShaderPath, defines, kShaderTarget, "RigidFrameBegin", "SmpPhysics::RigidRuntimeCS RigidFrameBegin") ||
				!exportShader->Get(kRuntimeShaderPath, defines, kShaderTarget, "RigidExport", "SmpPhysics::RigidRuntimeCS RigidExport") ||
				!worldExportShader->Get(kRuntimeShaderPath, defines, kShaderTarget, "RigidWorldExport", "SmpPhysics::RigidRuntimeCS RigidWorldExport") ||
				!predictShader->Get(kStepShaderPath, defines, kShaderTarget, "RigidPredict", "SmpPhysics::RigidSolverCS RigidPredict") ||
				!setupShader->Get(kStepShaderPath, defines, kShaderTarget, "RigidSetup", "SmpPhysics::RigidSolverCS RigidSetup") ||
				!solveShader->Get(kStepShaderPath, defines, kShaderTarget, "RigidSolveIslandColored", "SmpPhysics::RigidSolverCS RigidSolveIslandColored") ||
				!integrateShader->Get(kStepShaderPath, defines, kShaderTarget, "RigidIntegrate", "SmpPhysics::RigidSolverCS RigidIntegrate")) {
				Fail("the rigid solver compute shaders did not compile; claimed hair stays at the engine's pose");
				return false;
			}
		} catch (const std::exception& error) {
			Fail("the rigid solver compute shaders did not compile; claimed hair stays at the engine's pose");
			logger::warn("{} shader error: {}", kLogPrefix, error.what());
			return false;
		}
		return true;
	}

	bool RigidGpuSolver::AddClaim(ClaimSetup&& a_setup, std::string& a_reason)
	{
		ZoneScopedN("SmpPhysics::RigidAddClaim");
		if (!available) {
			a_reason = unavailableReason;
			return false;
		}
		const std::size_t bones = a_setup.world.bodies.size();
		if (bones == 0 || a_setup.rootBody.size() != bones || a_setup.restRelative.size() != bones) {
			a_reason = "the rigid claim's body tables do not match its world";
			return false;
		}
		const auto anchor = std::ranges::find_if(a_setup.world.bodies, [](const RigidReference::Body& a_body) { return a_body.kinematic; });
		if (anchor != a_setup.world.bodies.end()) {
			const int anchorIndex = static_cast<int>(anchor - a_setup.world.bodies.begin());
			for (std::size_t body = 0; body < bones; ++body) {
				if (a_setup.world.bodies[body].kinematic)
					continue;
				RigidReference::Generic6DofSpring2 contact;
				contact.bodyA = static_cast<int>(body);
				contact.bodyB = anchorIndex;
				contact.contactSlot = a_setup.boneOffset + static_cast<std::uint32_t>(body) + 1u;
				contact.contactFriction = a_setup.world.bodies[body].friction * RigidReference::kSurfaceBodyFriction;
				a_setup.world.constraints.push_back(contact);
			}
		}
		Claim claim;
		claim.data = RigidGpu::PackStep(a_setup.world, 1.0f / 60.0f);
		if (!claim.data.rowsFit || !claim.data.colorsFit) {
			a_reason = "the rigid claim needs more constraint rows or colours than the solver holds";
			return false;
		}
		claim.setup = std::move(a_setup);
		if (const auto found = claimIndex.find(claim.setup.token); found != claimIndex.end()) {
			claims[found->second] = std::move(claim);
		} else {
			claimIndex[claim.setup.token] = claims.size();
			claims.push_back(std::move(claim));
		}
		dirty = true;
		return true;
	}

	void RigidGpuSolver::RemoveClaim(std::uint64_t a_token)
	{
		const auto found = claimIndex.find(a_token);
		if (found == claimIndex.end())
			return;
		claims.erase(claims.begin() + static_cast<std::ptrdiff_t>(found->second));
		claimIndex.clear();
		for (std::size_t index = 0; index < claims.size(); ++index)
			claimIndex[claims[index].setup.token] = index;
		dirty = true;
	}

	bool RigidGpuSolver::RebuildBuffers()
	{
		ZoneScopedN("SmpPhysics::RigidRebuildBuffers");
		auto* context = globals::d3d::context;
		struct Carried
		{
			std::uint32_t oldBase{};
			std::uint32_t newBase{};
			std::uint32_t count{};
		};
		std::vector<Carried> carried;
		std::vector<RigidGpu::BodyRecord> allBodies;
		std::vector<RigidGpu::ConstraintRecord> allConstraints;
		std::vector<RigidGpu::IslandRecord> allIslands;
		std::vector<std::uint32_t> allIslandConstraints;
		std::vector<std::uint32_t> allColored;
		std::vector<std::uint32_t> allColorOffsets;
		std::vector<BodyMeta> allMeta;
		std::uint32_t bodyTotal = 0;
		std::uint32_t constraintTotal = 0;
		std::uint32_t islandTotal = 0;
		std::uint32_t listTotal = 0;
		for (Claim& claim : claims) {
			const std::uint32_t oldBase = claim.bodyBase;
			const bool existing = claim.stepped;
			claim.bodyBase = bodyTotal;
			claim.constraintBase = constraintTotal;
			const auto bodiesHere = static_cast<std::uint32_t>(claim.data.bodies.size());
			if (existing)
				carried.push_back({ oldBase, claim.bodyBase, bodiesHere });
			allBodies.insert(allBodies.end(), claim.data.bodies.begin(), claim.data.bodies.end());
			for (RigidGpu::ConstraintRecord record : claim.data.constraints) {
				record.header.x += claim.bodyBase;
				record.header.y += claim.bodyBase;
				allConstraints.push_back(record);
			}
			for (RigidGpu::IslandRecord record : claim.data.islands) {
				record.range.x += listTotal;
				allIslands.push_back(record);
			}
			for (const std::uint32_t index : claim.data.islandConstraints)
				allIslandConstraints.push_back(index + claim.constraintBase);
			for (const std::uint32_t index : claim.data.coloredConstraints)
				allColored.push_back(index + claim.constraintBase);
			allColorOffsets.insert(allColorOffsets.end(), claim.data.colorOffsets.begin(), claim.data.colorOffsets.end());
			for (std::size_t body = 0; body < claim.setup.world.bodies.size(); ++body) {
				BodyMeta meta;
				meta.slots.x = claim.setup.boneOffset + static_cast<std::uint32_t>(body);
				meta.slots.y = claim.setup.rootBody[body] < 0 ? kNoBody : claim.bodyBase + static_cast<std::uint32_t>(claim.setup.rootBody[body]);
				const Transform& rest = claim.setup.restRelative[body];
				meta.restRotation = Quat4(rest.rotation);
				meta.restTranslation = { rest.translation.x, rest.translation.y, rest.translation.z, 0.0f };
				if (body < claim.setup.wind.size()) {
					const ClaimSetup::WindBody& wind = claim.setup.wind[body];
					meta.windAxis = { wind.axis.x, wind.axis.y, wind.axis.z, wind.area };
					meta.windParams = { wind.factor, 0.0f, 0.0f, 0.0f };
				}
				allMeta.push_back(meta);
			}
			bodyTotal += bodiesHere;
			constraintTotal += static_cast<std::uint32_t>(claim.data.constraints.size());
			islandTotal += static_cast<std::uint32_t>(claim.data.islands.size());
			listTotal += static_cast<std::uint32_t>(claim.data.islandConstraints.size());
		}

		try {
			winrt::com_ptr<ID3D11Buffer> previousBodies = bodies ? bodies->resource : nullptr;
			bodies = MakeWritable<RigidGpu::BodyRecord>(&allBodies, bodyTotal, "SmpPhysics::RigidBodies", true);
			if (previousBodies) {
				for (const Carried& range : carried) {
					const D3D11_BOX box{ range.oldBase * static_cast<UINT>(sizeof(RigidGpu::BodyRecord)), 0, 0,
						(range.oldBase + range.count) * static_cast<UINT>(sizeof(RigidGpu::BodyRecord)), 1, 1 };
					context->CopySubresourceRegion(bodies->resource.get(), 0, range.newBase * static_cast<UINT>(sizeof(RigidGpu::BodyRecord)), 0, 0,
						previousBodies.get(), 0, &box);
				}
			}
			solverBodies = MakeWritable<RigidGpu::SolverBodyRecord>(nullptr, bodyTotal, "SmpPhysics::RigidSolverBodies", false);
			rows = MakeWritable<RigidGpu::RowRecord>(nullptr, constraintTotal * RigidGpu::kRowsPerConstraint, "SmpPhysics::RigidRows", false);
			rowMasks = MakeWritable<std::uint32_t>(nullptr, constraintTotal * RigidGpu::kAxisSlots, "SmpPhysics::RigidRowMasks", false);
			constraints = MakeStatic(allConstraints, "SmpPhysics::RigidConstraints");
			islands = MakeStatic(allIslands, "SmpPhysics::RigidIslands");
			islandConstraints = MakeStatic(allIslandConstraints, "SmpPhysics::RigidIslandConstraints");
			coloredConstraints = MakeStatic(allColored, "SmpPhysics::RigidColoredConstraints");
			colorOffsets = MakeStatic(allColorOffsets, "SmpPhysics::RigidColorOffsets");
			bodyMeta = MakeStatic(allMeta, "SmpPhysics::RigidBodyMeta");
			frameInputs = std::make_unique<Buffer>(StructuredBufferDesc<FrameInput>(std::max(1u, bodyTotal), false, true), nullptr, "SmpPhysics::RigidFrameInputs");
			frameInputs->CreateSRV(StructuredSrvDesc(std::max(1u, bodyTotal)));
			if (!stepConstants)
				stepConstants = std::make_unique<ConstantBuffer>(ConstantBufferDesc<RigidGpu::StepConstants>(true), "SmpPhysics::RigidStepConstants");
			if (!runtimeConstants)
				runtimeConstants = std::make_unique<ConstantBuffer>(ConstantBufferDesc<RuntimeConstants>(true), "SmpPhysics::RigidRuntimeConstants");
		} catch (const std::exception& error) {
			Fail("the rigid solver could not allocate its buffers; claimed hair stays at the engine's pose");
			logger::warn("{} buffer error: {}", kLogPrefix, error.what());
			return false;
		}
		bodyCount = bodyTotal;
		constraintCount = constraintTotal;
		islandCount = islandTotal;
		islandSizes = RigidGpu::SummariseIslands(allIslands);
		frameScratch.assign(bodyTotal, FrameInput{});
		dirty = false;
		return true;
	}

	bool RigidGpuSolver::Dispatch(const std::vector<FrameClaim>& a_frame, ID3D11UnorderedAccessView* a_worlds,
		ID3D11UnorderedAccessView* a_outputs, ID3D11ShaderResourceView* a_contacts, const std::function<void(std::uint32_t)>& a_contactPass)
	{
		if (!available || claims.empty() || a_frame.empty() || !a_worlds || !a_outputs)
			return false;
		auto* context = globals::d3d::context;
		if (!context || !EnsureShaders())
			return false;
		if (dirty && !RebuildBuffers())
			return false;

		float frameTime = 0.0f;
		std::vector<std::size_t> active;
		std::ranges::fill(frameScratch, FrameInput{});
		for (const FrameClaim& frame : a_frame) {
			const auto found = claimIndex.find(frame.token);
			if (found == claimIndex.end() || !frame.kinematicWorld)
				continue;
			Claim& claim = claims[found->second];
			const std::vector<Transform>& kinematic = *frame.kinematicWorld;
			if (kinematic.size() != claim.setup.world.bodies.size())
				continue;
			const bool reset = frame.resetRequested || !claim.stepped;
			for (std::size_t body = 0; body < kinematic.size(); ++body) {
				FrameInput& input = frameScratch[claim.bodyBase + body];
				const Transform& pose = kinematic[body];
				input.rotation = Quat4(pose.rotation);
				input.translation = { pose.translation.x, pose.translation.y, pose.translation.z, 0.0f };
				input.flags = { frame.hasPreviousInput && !reset ? 1u : 0u, reset ? 1u : 0u, 0u, 0u };
				input.wind = { frame.wind.x, frame.wind.y, frame.wind.z, frame.windTime };
				input.windScale = { frame.windFactor, 0.0f, 0.0f, 0.0f };
			}
			for (const FrameClaim::Pin& pin : frame.pins) {
				if (pin.body < 0 || static_cast<std::size_t>(pin.body) >= kinematic.size() || !(pin.stiffness > 0.0f))
					continue;
				FrameInput& input = frameScratch[claim.bodyBase + static_cast<std::size_t>(pin.body)];
				if (input.grabTarget.w != 0.0f)
					continue;
				input.grabLocal = { pin.local.x, pin.local.y, pin.local.z, pin.stiffness };
				input.grabTarget = { pin.target.x, pin.target.y, pin.target.z, 1.0f };
				const std::vector<int>& parents = claim.setup.parentBody;
				for (int held = pin.body, hops = 0; held >= 0 && static_cast<std::size_t>(held) < kinematic.size() && hops < static_cast<int>(kinematic.size());
					held = static_cast<std::size_t>(held) < parents.size() ? parents[static_cast<std::size_t>(held)] : -1, ++hops)
					frameScratch[claim.bodyBase + static_cast<std::size_t>(held)].flags.z = 1u;
			}
			claim.stepped = true;
			frameTime = std::max(frameTime, frame.deltaTime);
			active.push_back(found->second);
		}
		if (active.empty())
			return false;

		const SmpStepClock::Frame plan = stepClock.Advance(frameTime);
		if (plan.count <= 0)
			return false;
		frameTime = plan.total;
		D3D11_MAPPED_SUBRESOURCE mapped{};
		if (FAILED(context->Map(frameInputs->resource.get(), 0, D3D11_MAP_WRITE_DISCARD, 0, &mapped)))
			return false;
		std::memcpy(mapped.pData, frameScratch.data(), frameScratch.size() * sizeof(FrameInput));
		context->Unmap(frameInputs->resource.get(), 0);

		RuntimeConstants runtime;
		runtime.bodyCount = bodyCount;
		runtime.frameTime = frameTime;
		runtimeConstants->Update(runtime);
		ID3D11Buffer* runtimeBuffer = runtimeConstants->CB();
		ID3D11ShaderResourceView* runtimeViews[kRuntimeViewCount] = { frameInputs->srv.get(), bodyMeta->srv.get(), a_contacts };
		ID3D11UnorderedAccessView* runtimeTargets[kRuntimeTargetCount] = { bodies->uav.get(), a_worlds, a_outputs };
		{
			CS_GPU_PASS("SmpPhysics::RigidFrameBegin");
			context->CSSetConstantBuffers(0, 1, &runtimeBuffer);
			context->CSSetShaderResources(0, kRuntimeViewCount, runtimeViews);
			context->CSSetUnorderedAccessViews(0, kRuntimeTargetCount, runtimeTargets, nullptr);
			context->CSSetShader(frameBeginShader->get(), nullptr, 0);
			context->Dispatch(Groups(bodyCount), 1, 1);
		}

		const RigidReference::SolverSettings solver;
		RigidGpu::StepConstants step;
		step.bodyCount = bodyCount;
		step.constraintCount = constraintCount;
		step.islandCount = islandCount;
		step.iterations = static_cast<std::uint32_t>(solver.iterations);
		step.erp = solver.erp;
		step.globalCfm = solver.globalCfm;
		step.damping = solver.damping;
		step.residualThreshold = solver.leastSquaresResidualThreshold;
		step.axisSlots = RigidGpu::kAxisSlots;
		ID3D11Buffer* stepBuffer = stepConstants->CB();
		ID3D11ShaderResourceView* stepViews[kStepViewCount] = { constraints->srv.get(), islands->srv.get(), islandConstraints->srv.get(),
			coloredConstraints->srv.get(), colorOffsets->srv.get(), a_contacts };
		ID3D11UnorderedAccessView* stepTargets[kStepTargetCount] = { bodies->uav.get(), solverBodies->uav.get(), rows->uav.get(), rowMasks->uav.get() };
		for (int subStep = 0; subStep < plan.count; ++subStep) {
			if (a_contacts && a_contactPass) {
				CS_GPU_PASS("SmpPhysics::RigidContact");
				context->CSSetConstantBuffers(0, 1, &runtimeBuffer);
				context->CSSetShaderResources(0, kRuntimeViewCount, runtimeViews);
				context->CSSetUnorderedAccessViews(0, kRuntimeTargetCount, runtimeTargets, nullptr);
				context->CSSetShader(worldExportShader->get(), nullptr, 0);
				context->Dispatch(Groups(bodyCount), 1, 1);
				a_contactPass(static_cast<std::uint32_t>(subStep) + 1u);
				ID3D11UnorderedAccessView* releasedTargets[kContactTargetCount] = {};
				context->CSSetUnorderedAccessViews(0, kContactTargetCount, releasedTargets, nullptr);
			}
			step.timeStep = plan.steps[static_cast<std::size_t>(subStep)];
			step.framesPerSecond = 1.0f / step.timeStep;
			stepConstants->Update(step);
			context->CSSetConstantBuffers(0, 1, &stepBuffer);
			context->CSSetShaderResources(0, kStepViewCount, stepViews);
			context->CSSetUnorderedAccessViews(0, kStepTargetCount, stepTargets, nullptr);
			{
				CS_GPU_PASS("SmpPhysics::RigidPredict");
				context->CSSetShader(predictShader->get(), nullptr, 0);
				context->Dispatch(Groups(bodyCount), 1, 1);
			}
			{
				CS_GPU_PASS("SmpPhysics::RigidSetup");
				context->CSSetShader(setupShader->get(), nullptr, 0);
				context->Dispatch(Groups(constraintCount), 1, 1);
			}
			{
				CS_GPU_PASS("SmpPhysics::RigidSolve");
				context->CSSetShader(solveShader->get(), nullptr, 0);
				context->Dispatch(std::max(1u, islandCount), 1, 1);
			}
			{
				CS_GPU_PASS("SmpPhysics::RigidIntegrate");
				context->CSSetShader(integrateShader->get(), nullptr, 0);
				context->Dispatch(Groups(bodyCount), 1, 1);
			}
		}

		context->CSSetConstantBuffers(0, 1, &runtimeBuffer);
		context->CSSetShaderResources(0, kRuntimeViewCount, runtimeViews);
		context->CSSetUnorderedAccessViews(0, kRuntimeTargetCount, runtimeTargets, nullptr);
		{
			CS_GPU_PASS("SmpPhysics::RigidExport");
			context->CSSetShader(exportShader->get(), nullptr, 0);
			context->Dispatch(Groups(bodyCount), 1, 1);
		}

		ID3D11ShaderResourceView* noViews[kStepViewCount] = {};
		ID3D11UnorderedAccessView* noTargets[kStepTargetCount] = {};
		ID3D11Buffer* noConstant = nullptr;
		context->CSSetShaderResources(0, kStepViewCount, noViews);
		context->CSSetUnorderedAccessViews(0, kStepTargetCount, noTargets, nullptr);
		context->CSSetConstantBuffers(0, 1, &noConstant);
		context->CSSetShader(nullptr, nullptr, 0);
		return true;
	}
}
