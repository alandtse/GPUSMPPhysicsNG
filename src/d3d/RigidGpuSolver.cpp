#include "RigidGpuSolver.h"

#include "Globals.h"
#include "GpuPass.h"
#include "Utils/D3D.h"
#include "smp/GpuViews.h"
#include "smp/RigidRigBuilder.h"

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
		constexpr float kDefaultFixedStep = 1.0f / 60.0f;
		constexpr std::uint32_t kStepViewCount = 6;
		constexpr std::uint32_t kStepTargetCount = 4;
		constexpr std::uint32_t kRuntimeViewCount = 3;
		constexpr std::uint32_t kRuntimeTargetCount = 3;
		constexpr std::uint32_t kContactTargetCount = 7;  // UAV slots the chain contact passes leave bound; one of them is the contact buffer
		constexpr std::uint32_t kNoBody = 0xFFFFFFFFu;

		std::uint32_t Groups(std::uint32_t a_threads) noexcept
		{
			return std::max(1u, DispatchGroups(a_threads, kThreadsPerGroup));
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
		claim.data = RigidGpu::PackStep(a_setup.world, kDefaultFixedStep);
		if (!claim.data.rowsFit || !claim.data.colorsFit) {
			a_reason = "the rigid claim needs more constraint rows or colours than the solver holds";
			return false;
		}
		claim.setup = std::move(a_setup);
		if (const auto found = claimIndex.find(claim.setup.token); found != claimIndex.end()) {
			dirty = dirty || found->second < builtClaims;
			claims[found->second] = std::move(claim);
		} else {
			claimIndex[claim.setup.token] = claims.size();
			claims.push_back(std::move(claim));
		}
		dirty = dirty || !bodies;
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

	struct RigidGpuSolver::PackedPools
	{
		std::vector<RigidGpu::BodyRecord> bodies;
		std::vector<RigidGpu::ConstraintRecord> constraints;
		std::vector<RigidGpu::IslandRecord> islands;
		std::vector<std::uint32_t> islandConstraints;
		std::vector<std::uint32_t> colored;
		std::vector<std::uint32_t> colorOffsets;
		std::vector<BodyMeta> meta;
	};

	void RigidGpuSolver::PackClaim(Claim& a_claim, PoolTotals& a_totals, PackedPools& a_pools)
	{
		a_claim.bodyBase = a_totals.bodies;
		a_claim.constraintBase = a_totals.constraints;
		a_pools.bodies.insert(a_pools.bodies.end(), a_claim.data.bodies.begin(), a_claim.data.bodies.end());
		for (RigidGpu::ConstraintRecord record : a_claim.data.constraints) {
			record.header.x += a_claim.bodyBase;
			record.header.y += a_claim.bodyBase;
			a_pools.constraints.push_back(record);
		}
		for (RigidGpu::IslandRecord record : a_claim.data.islands) {
			record.range.x += a_totals.list;
			a_pools.islands.push_back(record);
		}
		for (const std::uint32_t index : a_claim.data.islandConstraints)
			a_pools.islandConstraints.push_back(index + a_claim.constraintBase);
		for (const std::uint32_t index : a_claim.data.coloredConstraints)
			a_pools.colored.push_back(index + a_claim.constraintBase);
		a_pools.colorOffsets.insert(a_pools.colorOffsets.end(), a_claim.data.colorOffsets.begin(), a_claim.data.colorOffsets.end());
		for (std::size_t body = 0; body < a_claim.setup.world.bodies.size(); ++body) {
			BodyMeta meta;
			meta.slots.x = a_claim.setup.boneOffset + static_cast<std::uint32_t>(body);
			meta.slots.y = a_claim.setup.rootBody[body] < 0 ? kNoBody : a_claim.bodyBase + static_cast<std::uint32_t>(a_claim.setup.rootBody[body]);
			const Transform& rest = a_claim.setup.restRelative[body];
			meta.restRotation = Quat4(rest.rotation);
			meta.restTranslation = { rest.translation.x, rest.translation.y, rest.translation.z, 0.0f };
			if (body < a_claim.setup.nodeFromBody.size()) {
				const Transform& node = a_claim.setup.nodeFromBody[body];
				meta.nodeRotation = Quat4(node.rotation);
				meta.nodeTranslation = { node.translation.x, node.translation.y, node.translation.z, 0.0f };
			}
			if (body < a_claim.setup.wind.size()) {
				const ClaimSetup::WindBody& wind = a_claim.setup.wind[body];
				meta.windAxis = { wind.axis.x, wind.axis.y, wind.axis.z, wind.area };
				meta.windParams = { wind.factor, 0.0f, 0.0f, 0.0f };
			}
			a_pools.meta.push_back(meta);
		}
		a_totals.bodies += static_cast<std::uint32_t>(a_claim.data.bodies.size());
		a_totals.constraints += static_cast<std::uint32_t>(a_claim.data.constraints.size());
		a_totals.islands += static_cast<std::uint32_t>(a_claim.data.islands.size());
		a_totals.list += static_cast<std::uint32_t>(a_claim.data.islandConstraints.size());
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
		PackedPools pools;
		PoolTotals totals;
		for (Claim& claim : claims) {
			const std::uint32_t oldBase = claim.bodyBase;
			const bool existing = claim.stepped;
			PackClaim(claim, totals, pools);
			if (existing)
				carried.push_back({ oldBase, claim.bodyBase, static_cast<std::uint32_t>(claim.data.bodies.size()) });
		}
		const PoolTotals room{ RigidGpu::GrownPoolCapacity(capacity.bodies, totals.bodies), RigidGpu::GrownPoolCapacity(capacity.constraints, totals.constraints),
			RigidGpu::GrownPoolCapacity(capacity.islands, totals.islands), RigidGpu::GrownPoolCapacity(capacity.list, totals.list) };
		pools.bodies.resize(room.bodies);
		pools.constraints.resize(room.constraints);
		pools.islands.resize(room.islands);
		pools.islandConstraints.resize(room.list);
		pools.colored.resize(room.list);
		pools.colorOffsets.resize(static_cast<std::size_t>(room.islands) * RigidGpu::kMaxColors);
		pools.meta.resize(room.bodies);

		try {
			winrt::com_ptr<ID3D11Buffer> previousBodies = bodies ? bodies->resource : nullptr;
			bodies = MakeWritable<RigidGpu::BodyRecord>(&pools.bodies, room.bodies, "SmpPhysics::RigidBodies", true);
			if (previousBodies) {
				for (const Carried& range : carried) {
					const D3D11_BOX box{ range.oldBase * static_cast<UINT>(sizeof(RigidGpu::BodyRecord)), 0, 0,
						(range.oldBase + range.count) * static_cast<UINT>(sizeof(RigidGpu::BodyRecord)), 1, 1 };
					context->CopySubresourceRegion(bodies->resource.get(), 0, range.newBase * static_cast<UINT>(sizeof(RigidGpu::BodyRecord)), 0, 0,
						previousBodies.get(), 0, &box);
				}
			}
			solverBodies = MakeWritable<RigidGpu::SolverBodyRecord>(nullptr, room.bodies, "SmpPhysics::RigidSolverBodies", false);
			rows = MakeWritable<RigidGpu::RowRecord>(nullptr, room.constraints * RigidGpu::kRowsPerConstraint, "SmpPhysics::RigidRows", false);
			rowMasks = MakeWritable<std::uint32_t>(nullptr, room.constraints * RigidGpu::kAxisSlots, "SmpPhysics::RigidRowMasks", false);
			constraints = MakeStatic(pools.constraints, "SmpPhysics::RigidConstraints");
			islands = MakeStatic(pools.islands, "SmpPhysics::RigidIslands");
			islandConstraints = MakeStatic(pools.islandConstraints, "SmpPhysics::RigidIslandConstraints");
			coloredConstraints = MakeStatic(pools.colored, "SmpPhysics::RigidColoredConstraints");
			colorOffsets = MakeStatic(pools.colorOffsets, "SmpPhysics::RigidColorOffsets");
			bodyMeta = MakeStatic(pools.meta, "SmpPhysics::RigidBodyMeta");
			frameInputs = std::make_unique<Buffer>(StructuredBufferDesc<FrameInput>(std::max(1u, room.bodies), false, true), nullptr, "SmpPhysics::RigidFrameInputs");
			frameInputs->CreateSRV(StructuredSrvDesc(std::max(1u, room.bodies)));
			if (!stepConstants)
				stepConstants = std::make_unique<ConstantBuffer>(ConstantBufferDesc<RigidGpu::StepConstants>(true), "SmpPhysics::RigidStepConstants");
			if (!runtimeConstants)
				runtimeConstants = std::make_unique<ConstantBuffer>(ConstantBufferDesc<RuntimeConstants>(true), "SmpPhysics::RigidRuntimeConstants");
		} catch (const std::exception& error) {
			Fail("the rigid solver could not allocate its buffers; claimed hair stays at the engine's pose");
			logger::warn("{} buffer error: {}", kLogPrefix, error.what());
			return false;
		}
		capacity = room;
		used = totals;
		bodyCount = totals.bodies;
		constraintCount = totals.constraints;
		islandCount = totals.islands;
		islandSizes = {};
		for (const Claim& claim : claims)
			RigidGpu::MergeIslandSizes(islandSizes, RigidGpu::SummariseIslands(claim.data.islands));
		frameScratch.assign(bodyCount, FrameInput{});
		builtClaims = claims.size();
		dirty = false;
		return true;
	}

	bool RigidGpuSolver::AppendClaims()
	{
		ZoneScopedN("SmpPhysics::RigidAppendClaims");
		PackedPools pools;
		PoolTotals totals = used;
		const PoolTotals first = totals;
		for (std::size_t index = builtClaims; index < claims.size(); ++index)
			PackClaim(claims[index], totals, pools);
		if (totals.bodies > capacity.bodies || totals.constraints > capacity.constraints || totals.islands > capacity.islands || totals.list > capacity.list)
			return RebuildBuffers();

		Util::UpdateBufferSlots(bodies->resource.get(), first.bodies, pools.bodies);
		Util::UpdateBufferSlots(bodyMeta->resource.get(), first.bodies, pools.meta);
		Util::UpdateBufferSlots(constraints->resource.get(), first.constraints, pools.constraints);
		Util::UpdateBufferSlots(islands->resource.get(), first.islands, pools.islands);
		Util::UpdateBufferSlots(islandConstraints->resource.get(), first.list, pools.islandConstraints);
		Util::UpdateBufferSlots(coloredConstraints->resource.get(), first.list, pools.colored);
		Util::UpdateBufferSlots(colorOffsets->resource.get(), first.islands * RigidGpu::kMaxColors, pools.colorOffsets);
		used = totals;
		bodyCount = totals.bodies;
		constraintCount = totals.constraints;
		islandCount = totals.islands;
		for (std::size_t index = builtClaims; index < claims.size(); ++index)
			RigidGpu::MergeIslandSizes(islandSizes, RigidGpu::SummariseIslands(claims[index].data.islands));
		frameScratch.resize(bodyCount, FrameInput{});
		builtClaims = claims.size();
		return true;
	}

	bool RigidGpuSolver::Dispatch(const std::vector<FrameClaim>& a_frame, ID3D11UnorderedAccessView* a_worlds,
		ID3D11UnorderedAccessView* a_outputs, ID3D11ShaderResourceView* a_contacts, const std::function<void(std::uint32_t, float)>& a_contactPass)
	{
		if (!available || claims.empty() || a_frame.empty() || !a_worlds || !a_outputs)
			return false;
		auto* context = globals::d3d::context;
		if (!context || !EnsureShaders())
			return false;
		if (dirty ? !RebuildBuffers() : builtClaims < claims.size() && !AppendClaims())
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
				const Vec3 local = NodePointToBody(claim.setup.nodeFromBody, pin.body, pin.local);
				input.grabLocal = { local.x, local.y, local.z, pin.stiffness };
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
		if (!UploadDiscard(context, frameInputs->resource.get(), frameScratch.data(), frameScratch.size() * sizeof(FrameInput)))
			return false;

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
				a_contactPass(static_cast<std::uint32_t>(subStep) + 1u, plan.StartFraction(subStep));
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
