#include "GpuSolver.h"

#include "DeviceRemoval.h"
#include "Globals.h"
#include "GpuPass.h"
#include "Utils/D3D.h"
#include "smp/GpuViews.h"
#include "smp/RigidRigBuilder.h"

#include <algorithm>
#include <bit>
#include <cstring>
#include <exception>
#include <thread>
#include <utility>

namespace SmpPhysics
{
	namespace
	{
		constexpr const wchar_t* kShaderPath = L"SmpPhysics\\ContactCS.hlsl";
		constexpr const wchar_t* kSurfacePrepPath = L"SmpPhysics\\SurfacePrepCS.hlsl";
		constexpr std::uint32_t kInitialBlocks = 4;
		constexpr int kAllocationPasses = 2;  ///< Search the free blocks, then once more after growing the pool.

		/** @brief A byte range of a buffer, which is how a D3D11 box addresses one. */
		D3D11_BOX ByteRange(std::uint32_t a_offset, std::uint32_t a_bytes) noexcept
		{
			D3D11_BOX box{};
			box.left = a_offset;
			box.right = a_offset + a_bytes;
			box.top = 0;
			box.bottom = 1;
			box.front = 0;
			box.back = 1;
			return box;
		}

		std::uint32_t Groups(std::uint32_t a_threads) noexcept
		{
			return SmpPhysics::DispatchGroups(a_threads, kThreadsPerGroup);
		}

		constexpr std::uint32_t kMinimumSurfaceCapacity = 64;
		constexpr std::uint32_t kStepViewCount = 15;
		constexpr std::uint32_t kStepTargetCount = 8;
		constexpr std::uint32_t kSurfacePrepViewCount = 7;    ///< t0..t6: the prep records and the drivers.
		constexpr std::uint32_t kSurfacePrepTargetCount = 5;  ///< u0..u4: scratch, then the contact pass's inputs.
		constexpr std::size_t kMaxSurfaceThreads = 4;
		/** @brief How far a surface vertex may sit from where its cached pose put it before the surface is rebuilt, in Skyrim units. */
		constexpr float kSurfaceReuseDrift = 0.05f;
		/** @brief The vertex distance from a driver that a rotation of it is charged at, in Skyrim units; the head and torso proxies fit within it. */
		constexpr float kSurfaceReuseRadius = 80.0f;

		/** @brief The most any vertex skinned to a_drivers can have moved between the two lists. */
		float SurfaceDrift(const std::vector<int>& a_drivers, const std::vector<Transform>& a_from, const std::vector<Transform>& a_to) noexcept
		{
			float drift = 0.0f;
			for (const int driver : a_drivers) {
				const Transform& from = a_from[static_cast<std::size_t>(driver)];
				const Transform& to = a_to[static_cast<std::size_t>(driver)];
				const float sign = from.rotation.w * to.rotation.w + from.rotation.x * to.rotation.x +
				                               from.rotation.y * to.rotation.y + from.rotation.z * to.rotation.z <
				                           0.0f ?
				                       -1.0f :
				                       1.0f;
				const float dw = from.rotation.w - sign * to.rotation.w;
				const float dx = from.rotation.x - sign * to.rotation.x;
				const float dy = from.rotation.y - sign * to.rotation.y;
				const float dz = from.rotation.z - sign * to.rotation.z;
				const float angle = 2.0f * std::sqrt(dw * dw + dx * dx + dy * dy + dz * dz);
				drift = std::max(drift, Length(to.translation - from.translation) + kSurfaceReuseRadius * angle);
			}
			return drift;
		}

		/**
		 * @brief Rewrites a_data into a structured buffer, growing it first when it does not fit.
		 * @param a_resident Keeps the buffer in GPU memory rather than CPU-writable. A list every thread of a pass reads each step belongs
		 *        there: read from a CPU-writable buffer, the contact passes took four fifths of the frame.
		 * @return false when the buffer could not be mapped.
		 */
		template <typename T>
		bool UploadDynamic(std::unique_ptr<Buffer>& a_buffer, winrt::com_ptr<ID3D11ShaderResourceView>& a_srv,
			std::uint32_t& a_capacity, const std::vector<T>& a_data, const char* a_name, bool a_resident = false)
		{
			const std::uint32_t needed = std::max<std::uint32_t>(1u, static_cast<std::uint32_t>(a_data.size()));
			if (!a_buffer || needed > a_capacity) {
				try {
					const std::uint32_t capacity = std::max({ needed, a_capacity * 2u, kMinimumSurfaceCapacity });
					auto buffer = std::make_unique<Buffer>(StructuredBufferDesc<T>(capacity, false, !a_resident), nullptr, a_name);
					buffer->CreateSRV(StructuredSrvDesc(capacity));
					a_capacity = capacity;
					a_srv = buffer->srv;
					a_buffer = std::move(buffer);
				} catch (const std::exception& error) {
					logger::warn("{} {} allocation failed: {}", kLogPrefix, a_name, error.what());
					return false;
				}
			}
			if (a_data.empty())
				return true;
			if (a_resident) {
				Util::UpdateBufferSlots(a_buffer->resource.get(), 0u, a_data);
				return true;
			}
			return UploadDiscard(globals::d3d::context, a_buffer->resource.get(), a_data.data(), a_data.size() * sizeof(T));
		}

		/** @brief The rigid-body setup of a rig: SMP's world, the kinematic body each dynamic body hangs from, and its rest pose relative to it. */
		RigidGpuSolver::ClaimSetup BuildRigidSetup(std::uint64_t a_token, std::uint32_t a_boneOffset, const SmpRig& a_rig,
			const std::vector<Transform>& a_restWorld, const SolverParams& a_params)
		{
			ZoneScopedN("SmpPhysics::BuildRigidSetup");
			RigidGpuSolver::ClaimSetup setup;
			setup.token = a_token;
			setup.boneOffset = a_boneOffset;
			const std::vector<Transform> bodyRest = BodyFrames(a_rig, a_restWorld, a_params.bodyScale);
			setup.world = BuildRigidWorld(a_rig, bodyRest, a_params.bodyScale, a_params);
			setup.nodeFromBody = NodeFromBody(a_rig, a_params.bodyScale);
			const std::size_t bones = a_rig.bones.size();
			std::vector<int> parent(bones, -1);
			for (const RigJoint& joint : a_rig.joints)
				parent[static_cast<std::size_t>(joint.child)] = joint.parent;
			setup.parentBody = parent;
			setup.rootBody.assign(bones, -1);
			setup.restRelative.assign(bones, Transform{});
			for (std::size_t bone = 0; bone < bones; ++bone) {
				if (a_rig.bones[bone].kinematic)
					continue;
				int root = static_cast<int>(bone);
				for (std::size_t hops = 0; hops < bones && root >= 0 && !a_rig.bones[static_cast<std::size_t>(root)].kinematic; ++hops)
					root = parent[static_cast<std::size_t>(root)];
				if (root < 0 || !a_rig.bones[static_cast<std::size_t>(root)].kinematic)
					continue;
				setup.rootBody[bone] = root;
				setup.restRelative[bone] = bodyRest[static_cast<std::size_t>(root)].Inverse() * bodyRest[bone];
			}
			return setup;
		}
	}

	GpuSolver::~GpuSolver() = default;

	void GpuSolver::Fail(const char* a_reason)
	{
		if (!available)
			return;
		available = false;
		unavailableReason = a_reason;
		logger::warn("{} {}", kLogPrefix, a_reason);
	}

	bool GpuSolver::CheckDeviceRemoved()
	{
		if (!globals::d3d::device)
			return false;
		const HRESULT reason = globals::d3d::device->GetDeviceRemovedReason();
		if (reason == S_OK)
			return false;
		if (available) {
			logger::error("{} The Direct3D device was removed ({} 0x{:08X}) with {} claims, {} spheres, {} bone slots and frame {}; claims go back to hdtSMP64",
				kLogPrefix, DeviceRemovedName(reason), static_cast<std::uint32_t>(reason), claims.size(), sphereCount, slotCapacity, frameCounter);
			Fail("the Direct3D device was removed");
		}
		return true;
	}

	void GpuSolver::InvalidateShaders()
	{
		if (contactCullShader)
			contactCullShader->Reset();
		if (contactSphereShader)
			contactSphereShader->Reset();
		if (contactReduceShader)
			contactReduceShader->Reset();
		if (clothVertexShader)
			clothVertexShader->Reset();
		if (clothTriangleShader)
			clothTriangleShader->Reset();
		if (clothFoldShader)
			clothFoldShader->Reset();
		if (surfaceBonesShader)
			surfaceBonesShader->Reset();
		if (surfacePoseShader)
			surfacePoseShader->Reset();
		if (surfaceTrianglesShader)
			surfaceTrianglesShader->Reset();
		if (surfaceLeavesShader)
			surfaceLeavesShader->Reset();
		if (surfaceRefitShader)
			surfaceRefitShader->Reset();
		rigidSolver.InvalidateShaders();
		surfacePrepActive = false;
		surfacePrepDirty = true;
	}

	std::vector<Util::ShaderJob> GpuSolver::ShaderJobs()
	{
		auto jobs = RigidGpuSolver::ShaderJobs();
		for (const char* entry : { "ContactCull", "ContactSpheres", "ContactReduce", "ClothVertices", "ClothTriangles" })
			jobs.push_back({ kShaderPath, kShaderTarget, entry });
		for (const char* entry : { "SurfaceBones", "SurfacePose", "SurfaceTriangles", "SurfaceLeaves", "SurfaceRefit" })
			jobs.push_back({ kSurfacePrepPath, kShaderTarget, entry });
		return jobs;
	}

	bool GpuSolver::EnsureShaders()
	{
		ZoneScopedN("SmpPhysics::EnsureShaders");
		if (!available)
			return false;
		if (!globals::d3d::device) {
			Fail("the GPU solver backend needs a Direct3D device; claimed hair stays at the engine's pose");
			return false;
		}
		if (!contactCullShader)
			contactCullShader = std::make_unique<Util::LazyShader<ID3D11ComputeShader>>();
		if (!contactSphereShader)
			contactSphereShader = std::make_unique<Util::LazyShader<ID3D11ComputeShader>>();
		if (!contactReduceShader)
			contactReduceShader = std::make_unique<Util::LazyShader<ID3D11ComputeShader>>();
		if (!clothVertexShader)
			clothVertexShader = std::make_unique<Util::LazyShader<ID3D11ComputeShader>>();
		if (!clothTriangleShader)
			clothTriangleShader = std::make_unique<Util::LazyShader<ID3D11ComputeShader>>();
		if (!clothFoldShader)
			clothFoldShader = std::make_unique<Util::LazyShader<ID3D11ComputeShader>>();
		if (!surfaceBonesShader)
			surfaceBonesShader = std::make_unique<Util::LazyShader<ID3D11ComputeShader>>();
		if (!surfacePoseShader)
			surfacePoseShader = std::make_unique<Util::LazyShader<ID3D11ComputeShader>>();
		if (!surfaceTrianglesShader)
			surfaceTrianglesShader = std::make_unique<Util::LazyShader<ID3D11ComputeShader>>();
		if (!surfaceLeavesShader)
			surfaceLeavesShader = std::make_unique<Util::LazyShader<ID3D11ComputeShader>>();
		if (!surfaceRefitShader)
			surfaceRefitShader = std::make_unique<Util::LazyShader<ID3D11ComputeShader>>();
		const std::vector<std::pair<const char*, const char*>> defines;
		try {
			if (!contactCullShader->Get(kShaderPath, defines, kShaderTarget, "ContactCull", "SmpPhysics::ContactCS ContactCull") ||
				!contactSphereShader->Get(kShaderPath, defines, kShaderTarget, "ContactSpheres", "SmpPhysics::ContactCS ContactSpheres") ||
				!contactReduceShader->Get(kShaderPath, defines, kShaderTarget, "ContactReduce", "SmpPhysics::ContactCS ContactReduce") ||
				!clothVertexShader->Get(kShaderPath, defines, kShaderTarget, "ClothVertices", "SmpPhysics::ContactCS ClothVertices") ||
				!clothTriangleShader->Get(kShaderPath, defines, kShaderTarget, "ClothTriangles", "SmpPhysics::ContactCS ClothTriangles") ||
				!clothFoldShader->Get(kShaderPath, defines, kShaderTarget, "ClothFold", "SmpPhysics::ContactCS ClothFold")) {
				Fail("the GPU solver compute shader did not compile; claimed hair stays at the engine's pose");
				return false;
			}
		} catch (const std::exception& error) {
			Fail("the GPU solver compute shader did not compile; claimed hair stays at the engine's pose");
			logger::warn("{} shader error: {}", kLogPrefix, error.what());
			return false;
		}
		EnsureSurfacePrepShaders();
		return true;
	}

	bool GpuSolver::EnsureSurfacePrepShaders()
	{
		if (!available || !surfaceBonesShader || !surfacePoseShader || !surfaceTrianglesShader || !surfaceLeavesShader || !surfaceRefitShader)
			return false;
		const std::vector<std::pair<const char*, const char*>> defines;
		try {
			return surfaceBonesShader->Get(kSurfacePrepPath, defines, kShaderTarget, "SurfaceBones", "SmpPhysics::SurfacePrepCS SurfaceBones") &&
			       surfacePoseShader->Get(kSurfacePrepPath, defines, kShaderTarget, "SurfacePose", "SmpPhysics::SurfacePrepCS SurfacePose") &&
			       surfaceTrianglesShader->Get(kSurfacePrepPath, defines, kShaderTarget, "SurfaceTriangles", "SmpPhysics::SurfacePrepCS SurfaceTriangles") &&
			       surfaceLeavesShader->Get(kSurfacePrepPath, defines, kShaderTarget, "SurfaceLeaves", "SmpPhysics::SurfacePrepCS SurfaceLeaves") &&
			       surfaceRefitShader->Get(kSurfacePrepPath, defines, kShaderTarget, "SurfaceRefit", "SmpPhysics::SurfacePrepCS SurfaceRefit");
		} catch (const std::exception& error) {
			logger::warn("{} surface prep shader error: {}", kLogPrefix, error.what());
			return false;
		}
	}

	bool GpuSolver::CreateBuffers(std::uint32_t a_slots, std::uint32_t a_previousSlots)
	{
		try {
			return CreateBuffersUnchecked(a_slots, a_previousSlots);
		} catch (const std::exception& error) {
			logger::warn("{} buffer allocation failed: {}", kLogPrefix, error.what());
			return false;
		}
	}

	bool GpuSolver::CreateBuffersUnchecked(std::uint32_t a_slots, std::uint32_t a_previousSlots)
	{
		ZoneScopedN("SmpPhysics::CreateBuffers");
		auto* device = globals::d3d::device;
		auto* context = globals::d3d::context;
		if (!device || !context || a_slots == 0u)
			return false;

		winrt::com_ptr<ID3D11Buffer> previousWorlds;
		winrt::com_ptr<ID3D11Buffer> previousOutputs;
		if (a_previousSlots > 0u && worlds && outputs) {
			previousWorlds = worlds->resource;
			previousOutputs = outputs->resource;
		}

		trees = nullptr;
		spheres = nullptr;
		colliders = nullptr;
		inputs = nullptr;
		drivers = nullptr;
		driverDeltas = nullptr;
		worlds = nullptr;
		outputs = nullptr;
		boneTrees = nullptr;
		boneCloth = nullptr;
		boneClothSrv = nullptr;
		contacts = nullptr;
		boneTreesSrv = nullptr;
		contactsSrv = nullptr;
		contactsUav = nullptr;
		boneCullUav = nullptr;
		sphereHitsUav = nullptr;
		sphereHits = nullptr;
		sphereHitCapacity = 0u;
		treesSrv = nullptr;
		spheresSrv = nullptr;
		collidersSrv = nullptr;
		inputsSrv = nullptr;
		driversSrv = nullptr;
		driverDeltasSrv = nullptr;
		worldsSrv = nullptr;
		worldsUav = nullptr;
		outputsUav = nullptr;

		trees = std::make_unique<Buffer>(StructuredBufferDesc<GpuRig::TreeStatic>(a_slots, false, false), nullptr, "SmpPhysics::PoolTrees");
		trees->CreateSRV(StructuredSrvDesc(a_slots));
		treesSrv = trees->srv;
		spheres = std::make_unique<Buffer>(StructuredBufferDesc<GpuRig::BoneSpheres>(a_slots, false, false), nullptr, "SmpPhysics::PoolSpheres");
		spheres->CreateSRV(StructuredSrvDesc(a_slots));
		spheresSrv = spheres->srv;
		colliders = std::make_unique<Buffer>(StructuredBufferDesc<GpuRig::ColliderPacked>(a_slots, false, false), nullptr, "SmpPhysics::PoolColliders");
		colliders->CreateSRV(StructuredSrvDesc(a_slots));
		collidersSrv = colliders->srv;
		inputs = std::make_unique<Buffer>(StructuredBufferDesc<GpuRig::TreeInput>(a_slots, false, true), nullptr, "SmpPhysics::PoolInputs");
		inputs->CreateSRV(StructuredSrvDesc(a_slots));
		inputsSrv = inputs->srv;
		drivers = std::make_unique<Buffer>(StructuredBufferDesc<GpuRig::DriverWorld>(a_slots, false, true), nullptr, "SmpPhysics::PoolDrivers");
		drivers->CreateSRV(StructuredSrvDesc(a_slots));
		driversSrv = drivers->srv;
		driverDeltas = std::make_unique<Buffer>(StructuredBufferDesc<GpuRig::DriverDelta>(a_slots, false, true), nullptr, "SmpPhysics::PoolDriverDeltas");
		driverDeltas->CreateSRV(StructuredSrvDesc(a_slots));
		driverDeltasSrv = driverDeltas->srv;
		worlds = std::make_unique<Buffer>(StructuredBufferDesc<GpuRig::BoneWorld>(a_slots, true, false), nullptr, "SmpPhysics::PoolWorlds");
		worlds->CreateUAV(StructuredUavDesc(a_slots));
		worlds->CreateSRV(StructuredSrvDesc(a_slots));
		worldsUav = worlds->uav;
		worldsSrv = worlds->srv;
		outputs = std::make_unique<Buffer>(StructuredBufferDesc<GpuRig::BoneOutput>(a_slots, true, false), nullptr, "SmpPhysics::PoolOutputs");
		outputs->CreateUAV(StructuredUavDesc(a_slots));
		outputsUav = outputs->uav;
		boneTrees = std::make_unique<Buffer>(StructuredBufferDesc<std::uint32_t>(a_slots, false, false), nullptr, "SmpPhysics::PoolBoneTrees");
		boneTrees->CreateSRV(StructuredSrvDesc(a_slots));
		boneTreesSrv = boneTrees->srv;
		boneCloth = std::make_unique<Buffer>(StructuredBufferDesc<GpuRig::BoneCloth>(a_slots, false, false), nullptr, "SmpPhysics::PoolBoneCloth");
		boneCloth->CreateSRV(StructuredSrvDesc(a_slots));
		boneClothSrv = boneCloth->srv;
		// The contact pass writes this through the UAV and the rigid solver reads it through the SRV, never both at once.
		const std::vector<GpuRig::BoneContact> noContacts(a_slots * kContactLanes);
		D3D11_SUBRESOURCE_DATA noContactsData{ noContacts.data(), 0, 0 };
		contacts = std::make_unique<Buffer>(StructuredBufferDesc<GpuRig::BoneContact>(a_slots * kContactLanes, true, false), &noContactsData, "SmpPhysics::PoolContacts");
		contacts->CreateUAV(StructuredUavDesc(a_slots * kContactLanes));
		contacts->CreateSRV(StructuredSrvDesc(a_slots * kContactLanes));
		contactsUav = contacts->uav;
		contactsSrv = contacts->srv;
		boneCull = std::make_unique<Buffer>(StructuredBufferDesc<std::array<std::uint32_t, 4>>(a_slots, true, false), nullptr, "SmpPhysics::PoolBoneCull");
		boneCull->CreateUAV(StructuredUavDesc(a_slots));
		boneCullUav = boneCull->uav;

		const std::uint32_t outputBytes = a_slots * static_cast<std::uint32_t>(sizeof(GpuRig::BoneOutput));
		for (std::size_t index = 0; index < readbacks.size(); ++index) {
			readbacks[index] = {};
			const Buffer staging(StagingBufferDesc(outputBytes), nullptr, nullptr);
			readbacks[index].buffer = staging.resource;
			Util::SetResourceName(readbacks[index].buffer.get(), "SmpPhysics::PoolReadback%u", static_cast<unsigned>(index));
		}

		if (!constants)
			constants = std::make_unique<ConstantBuffer>(ConstantBufferDesc<GpuSolverConstants>(true), "SmpPhysics::PoolConstants");

		if (previousWorlds) {
			const D3D11_BOX worldBox = ByteRange(0u, a_previousSlots * static_cast<std::uint32_t>(sizeof(GpuRig::BoneWorld)));
			context->CopySubresourceRegion(worlds->resource.get(), 0, 0, 0, 0, previousWorlds.get(), 0, &worldBox);
			const D3D11_BOX outputBox = ByteRange(0u, a_previousSlots * static_cast<std::uint32_t>(sizeof(GpuRig::BoneOutput)));
			context->CopySubresourceRegion(outputs->resource.get(), 0, 0, 0, 0, previousOutputs.get(), 0, &outputBox);
		}
		return true;
	}

	void GpuSolver::PlanCapacity(std::uint32_t a_slots)
	{
		const std::uint32_t blocks = (a_slots + kBlockSlots - 1u) / kBlockSlots;
		if (blocks > freeBlocks.size())
			freeBlocks.resize(blocks, true);
	}

	std::optional<std::uint32_t> GpuSolver::AllocateSlots(std::uint32_t a_needed)
	{
		const std::uint32_t neededBlocks = (a_needed + kBlockSlots - 1u) / kBlockSlots;
		for (int attempt = 0; attempt < kAllocationPasses; ++attempt) {
			std::uint32_t run = 0;
			for (std::uint32_t block = 0; block < freeBlocks.size(); ++block) {
				run = freeBlocks[block] ? run + 1u : 0u;
				if (run == neededBlocks) {
					const std::uint32_t first = block + 1u - neededBlocks;
					for (std::uint32_t index = first; index <= block; ++index)
						freeBlocks[index] = false;
					return first * kBlockSlots;
				}
			}
			if (attempt + 1 == kAllocationPasses)
				break;
			const std::uint32_t grown = std::max({ static_cast<std::uint32_t>(freeBlocks.size()) * 2u,
				static_cast<std::uint32_t>(freeBlocks.size()) + neededBlocks, kInitialBlocks });
			if (!CreateBuffers(grown * kBlockSlots, slotCapacity)) {
				Fail("the GPU solver buffers could not be created; claimed hair stays at the engine's pose");
				return std::nullopt;
			}
			slotCapacity = grown * kBlockSlots;
			PlanCapacity(slotCapacity);
			staticsDirty = true;
		}
		return std::nullopt;
	}

	void GpuSolver::ClearContacts(std::uint32_t a_offset, std::uint32_t a_count)
	{
		auto* context = globals::d3d::context;
		if (!contacts || !context || a_count == 0)
			return;
		constexpr UINT recordSize = static_cast<UINT>(sizeof(GpuRig::BoneContact));
		const std::vector<GpuRig::BoneContact> noContacts(a_count * kContactLanes);
		const D3D11_BOX range{ a_offset * kContactLanes * recordSize, 0, 0, (a_offset + a_count) * kContactLanes * recordSize, 1, 1 };
		context->UpdateSubresource(contacts->resource.get(), 0, &range, noContacts.data(), 0, 0);
	}

	void GpuSolver::ReleaseSlots(std::uint32_t a_offset, std::uint32_t a_count)
	{
		const std::uint32_t first = a_offset / kBlockSlots;
		const std::uint32_t count = (a_count + kBlockSlots - 1u) / kBlockSlots;
		for (std::uint32_t index = first; index < first + count && index < freeBlocks.size(); ++index)
			freeBlocks[index] = true;
	}

	bool GpuSolver::AddClaim(std::uint64_t a_token, const SmpRig& a_rig, const std::vector<Transform>& a_restWorld,
		const SolverParams& a_params, const GpuRig::CollisionData& a_collision, const std::vector<SkinnedSurface>& a_surfaces,
		const std::vector<float>& a_windAreas, std::string& a_reason)
	{
		ZoneScopedN("SmpPhysics::GpuAddClaim");
		if (!available) {
			a_reason = unavailableReason;
			return false;
		}
		if (claimIndex.contains(a_token))
			return true;
		if (!Util::GetShaderPrewarmStatus().running && !EnsureShaders()) {
			a_reason = unavailableReason;
			return false;
		}

		SmpRig chainRig = a_rig;
		ApplySpringLimits(chainRig, kSmpPhysicsTick, kMinimumSpringDampingRatio);
		GpuRig::RigData rigData;
		{
			ZoneScopedN("SmpPhysics::BuildRigData");
			rigData = GpuRig::BuildRigData(chainRig, a_restWorld, a_windAreas);
		}
		const std::uint32_t boneCount = static_cast<std::uint32_t>(rigData.bones.size());
		if (boneCount == 0u || boneCount != a_rig.bones.size()) {
			a_reason = "the rig could not be packed for the GPU backend";
			return false;
		}
		GpuRig::SetCollision(rigData, a_collision);
		if (rigData.collision.spheres.size() > boneCount) {
			a_reason = "the rig has more collision sphere lists than bones";
			return false;
		}

		const std::uint32_t needed = GpuRig::SlotSpan(rigData);
		for (const GpuRig::TreeStatic& tree : rigData.trees) {
			if (tree.range.y > GpuRig::kMaxTreeBones) {
				a_reason = "a chain has more bones than the compute solver serves";
				return false;
			}
		}
		if (needed > kMaximumClaimSlots) {
			a_reason = "the rig needs " + std::to_string(needed) + " solver slots, past the " +
			           std::to_string(kMaximumClaimSlots) + " this backend serves";
			return false;
		}

		if (freeBlocks.empty()) {
			if (!CreateBuffers(kInitialBlocks * kBlockSlots, 0u)) {
				Fail("the GPU solver buffers could not be created; claimed hair stays at the engine's pose");
				a_reason = unavailableReason;
				return false;
			}
			slotCapacity = kInitialBlocks * kBlockSlots;
			PlanCapacity(slotCapacity);
			staticsDirty = true;
		}

		const std::optional<std::uint32_t> offset = AllocateSlots(needed);
		if (!offset) {
			a_reason = available ? "the GPU solver has no free slot range for this rig" : unavailableReason;
			return false;
		}

		GpuRig::ShiftToSlotBase(rigData, *offset);

		Claim claim;
		claim.token = a_token;
		claim.boneOffset = *offset;
		claim.boneCount = boneCount;
		claim.slotSpan = needed;
		claim.colliderOffset = *offset + boneCount;
		claim.driverOffset = claim.colliderOffset + static_cast<std::uint32_t>(rigData.collision.colliders.size());
		claim.driverCount = rigData.collision.driverCount;
		claim.rig = std::move(rigData);
		claim.surfaces = a_surfaces;
		for (const SkinnedSurface& surface : claim.surfaces) {
			for (const int driver : surface.boneDriver) {
				if (driver >= 0 && static_cast<std::uint32_t>(driver) < claim.driverCount &&
					std::find(claim.surfaceDrivers.begin(), claim.surfaceDrivers.end(), driver) == claim.surfaceDrivers.end())
					claim.surfaceDrivers.push_back(driver);
			}
		}
		claim.surfaceCache.resize(claim.surfaces.size());
		claim.surfaceBvhs.resize(claim.surfaces.size());
		claim.inputs.assign(claim.rig.trees.size(), GpuRig::TreeInput{});

		claimIndex.emplace(a_token, claims.size());
		claims.push_back(std::move(claim));
		treeHighWater = std::max(treeHighWater, *offset + static_cast<std::uint32_t>(claims.back().rig.trees.size()));
		boneHighWater = std::max(boneHighWater, *offset + boneCount);
		driverHighWater = std::max(driverHighWater, claims.back().driverOffset + claims.back().driverCount);
		if (!staticsDirty)
			pendingStaticClaims.push_back(claims.size() - 1u);
		surfacePrepDirty = true;

		ClearContacts(*offset, boneCount);

		RigidGpuSolver::ClaimSetup rigidSetup = BuildRigidSetup(a_token, *offset, a_rig, a_restWorld, a_params);
		rigidSetup.wind.reserve(claims.back().rig.bones.size());
		for (const GpuRig::BoneStatic& bone : claims.back().rig.bones) {
			Vec3 axis{ bone.wind.x, bone.wind.y, bone.wind.z };
			if (rigidSetup.wind.size() < rigidSetup.nodeFromBody.size())
				axis = rigidSetup.nodeFromBody[rigidSetup.wind.size()].rotation.Rotate(axis);
			rigidSetup.wind.push_back({ axis, bone.wind.w, std::bit_cast<float>(bone.topology.z) });
		}
		if (!rigidSolver.AddClaim(std::move(rigidSetup), a_reason)) {
			RemoveClaim(a_token);
			return false;
		}

		UploadStatics();
		return true;
	}

	std::optional<std::uint32_t> GpuSolver::BoneOffset(std::uint64_t a_token) const
	{
		const auto found = claimIndex.find(a_token);
		if (found == claimIndex.end())
			return std::nullopt;
		return claims[found->second].boneOffset;
	}

	bool GpuSolver::PoseReady(std::uint64_t a_token) const
	{
		const auto found = claimIndex.find(a_token);
		return found != claimIndex.end() && claims[found->second].firstFrame != 0u;
	}

	std::vector<std::uint32_t> GpuSolver::BoneTreeSlots(std::uint64_t a_token) const
	{
		const auto found = claimIndex.find(a_token);
		if (found == claimIndex.end())
			return {};
		const Claim& claim = claims[found->second];
		return GpuRig::BoneTreeSlots(claim.rig, claim.boneOffset, claim.boneCount);
	}

	void GpuSolver::RemoveClaim(std::uint64_t a_token)
	{
		const auto found = claimIndex.find(a_token);
		if (found == claimIndex.end())
			return;

		rigidSolver.RemoveClaim(a_token);
		ReleaseSlots(claims[found->second].boneOffset, claims[found->second].slotSpan);
		claims.erase(claims.begin() + static_cast<std::ptrdiff_t>(found->second));

		claimIndex.clear();
		for (std::size_t index = 0; index < claims.size(); ++index)
			claimIndex.emplace(claims[index].token, index);

		treeHighWater = 0u;
		boneHighWater = 0u;
		driverHighWater = 0u;
		for (const Claim& claim : claims) {
			treeHighWater = std::max(treeHighWater, claim.boneOffset + static_cast<std::uint32_t>(claim.rig.trees.size()));
			boneHighWater = std::max(boneHighWater, claim.boneOffset + claim.boneCount);
			driverHighWater = std::max(driverHighWater, claim.driverOffset + claim.driverCount);
		}
		staticsDirty = true;
		pendingStaticClaims.clear();
		surfacePrepDirty = true;
	}

	void GpuSolver::UploadStatics()
	{
		if (!trees)
			return;
		auto* context = globals::d3d::context;
		const auto clothBase = [this] {
			return GpuRig::ClothBase{ static_cast<std::uint32_t>(clothVertexShadow.size()), static_cast<std::uint32_t>(clothTriangleShadow.size()),
				static_cast<std::uint32_t>(clothBoneListShadow.size()) };
		};
		const auto appendCloth = [this](const GpuRig::ClaimStatics& a_packed) {
			clothVertexShadow.insert(clothVertexShadow.end(), a_packed.clothVertices.begin(), a_packed.clothVertices.end());
			clothTriangleShadow.insert(clothTriangleShadow.end(), a_packed.clothTriangles.begin(), a_packed.clothTriangles.end());
			clothBoneListShadow.insert(clothBoneListShadow.end(), a_packed.clothBoneList.begin(), a_packed.clothBoneList.end());
		};
		if (staticsDirty) {
			ZoneScopedN("SmpPhysics::UploadStatics");
			std::vector<GpuRig::TreeStatic> treeData(slotCapacity);
			std::vector<GpuRig::BoneSpheres> sphereData(slotCapacity);
			std::vector<GpuRig::ColliderPacked> colliderData(slotCapacity);
			std::vector<std::uint32_t> boneTreeData(slotCapacity, kNoTree);
			std::vector<GpuRig::BoneCloth> boneClothData(slotCapacity);
			sphereListShadow.clear();
			clothVertexShadow.clear();
			clothTriangleShadow.clear();
			clothBoneListShadow.clear();
			for (Claim& claim : claims) {
				claim.sphereBase = static_cast<std::uint32_t>(sphereListShadow.size());
				const GpuRig::ClaimStatics packed = GpuRig::PackClaimStatics(claim.rig, claim.boneOffset, claim.boneCount, claim.sphereBase, clothBase());
				std::ranges::copy(packed.trees, treeData.begin() + claim.boneOffset);
				std::ranges::copy(packed.spheres, sphereData.begin() + claim.boneOffset);
				std::ranges::copy(packed.boneTrees, boneTreeData.begin() + claim.boneOffset);
				std::ranges::copy(packed.boneCloth, boneClothData.begin() + claim.boneOffset);
				std::ranges::copy(packed.colliders, colliderData.begin() + claim.colliderOffset);
				sphereListShadow.insert(sphereListShadow.end(), packed.sphereList.begin(), packed.sphereList.end());
				appendCloth(packed);
			}
			context->UpdateSubresource(boneCloth->resource.get(), 0, nullptr, boneClothData.data(), 0, 0);
			context->UpdateSubresource(trees->resource.get(), 0, nullptr, treeData.data(), 0, 0);
			context->UpdateSubresource(spheres->resource.get(), 0, nullptr, sphereData.data(), 0, 0);
			context->UpdateSubresource(colliders->resource.get(), 0, nullptr, colliderData.data(), 0, 0);
			context->UpdateSubresource(boneTrees->resource.get(), 0, nullptr, boneTreeData.data(), 0, 0);
			pendingStaticClaims.clear();
			staticsDirty = false;
		} else if (!pendingStaticClaims.empty()) {
			ZoneScopedN("SmpPhysics::AppendStatics");
			for (const std::size_t index : pendingStaticClaims) {
				Claim& claim = claims[index];
				claim.sphereBase = static_cast<std::uint32_t>(sphereListShadow.size());
				const GpuRig::ClaimStatics packed = GpuRig::PackClaimStatics(claim.rig, claim.boneOffset, claim.boneCount, claim.sphereBase, clothBase());
				Util::UpdateBufferSlots(trees->resource.get(), claim.boneOffset, packed.trees);
				Util::UpdateBufferSlots(spheres->resource.get(), claim.boneOffset, packed.spheres);
				Util::UpdateBufferSlots(boneTrees->resource.get(), claim.boneOffset, packed.boneTrees);
				Util::UpdateBufferSlots(boneCloth->resource.get(), claim.boneOffset, packed.boneCloth);
				Util::UpdateBufferSlots(colliders->resource.get(), claim.colliderOffset, packed.colliders);
				sphereListShadow.insert(sphereListShadow.end(), packed.sphereList.begin(), packed.sphereList.end());
				appendCloth(packed);
			}
			pendingStaticClaims.clear();
		} else {
			return;
		}
		if (!UploadDynamic(sphereList, sphereListSrv, sphereListCapacity, sphereListShadow, "SmpPhysics::PoolSphereList", true)) {
			Fail("the GPU solver sphere list could not be uploaded; claimed hair stays at the engine's pose");
			return;
		}
		sphereCount = static_cast<std::uint32_t>(sphereListShadow.size());
		if (!sphereHits || sphereHitCapacity < sphereListCapacity) {
			try {
				auto hits = std::make_unique<Buffer>(StructuredBufferDesc<GpuRig::SphereHit>(sphereListCapacity, true, false), nullptr, "SmpPhysics::PoolSphereHits");
				hits->CreateUAV(StructuredUavDesc(sphereListCapacity));
				sphereHits = std::move(hits);
				sphereHitsUav = sphereHits->uav;
				sphereHitCapacity = sphereListCapacity;
			} catch (const std::exception&) {
				Fail("the GPU solver sphere hit buffer could not be created; claimed hair stays at the engine's pose");
			}
		}
		UploadClothStatics();
	}

	void GpuSolver::UploadClothStatics()
	{
		clothVertexCount = 0u;
		clothTriangleCount = 0u;
		if (clothTriangleShadow.empty())
			return;
		if (!UploadDynamic(clothVertexList, clothVertexListSrv, clothVertexCapacity, clothVertexShadow, "SmpPhysics::PoolClothVertices", true) ||
			!UploadDynamic(clothTriangleList, clothTriangleListSrv, clothTriangleCapacity, clothTriangleShadow, "SmpPhysics::PoolClothTriangles", true) ||
			!UploadDynamic(clothBoneList, clothBoneListSrv, clothBoneListCapacity, clothBoneListShadow, "SmpPhysics::PoolClothBoneList", true)) {
			Fail("the GPU solver cloth triangle lists could not be uploaded; claimed hair stays at the engine's pose");
			return;
		}
		try {
			if (!clothPosed || clothPosedCapacity < clothVertexCapacity) {
				auto posed = std::make_unique<Buffer>(StructuredBufferDesc<GpuRig::Float4>(clothVertexCapacity, true, false), nullptr, "SmpPhysics::PoolClothPosed");
				posed->CreateUAV(StructuredUavDesc(clothVertexCapacity));
				clothPosed = std::move(posed);
				clothPosedUav = clothPosed->uav;
				clothPosedCapacity = clothVertexCapacity;
			}
			if (!clothHits || clothHitCapacity < clothTriangleCapacity) {
				auto hits = std::make_unique<Buffer>(StructuredBufferDesc<GpuRig::SphereHit>(clothTriangleCapacity * kClothTriangleHits, true, false), nullptr, "SmpPhysics::PoolClothHits");
				hits->CreateUAV(StructuredUavDesc(clothTriangleCapacity * kClothTriangleHits));
				auto counts = std::make_unique<Buffer>(StructuredBufferDesc<std::uint32_t>(clothTriangleCapacity, true, false), nullptr, "SmpPhysics::PoolClothHitCounts");
				counts->CreateUAV(StructuredUavDesc(clothTriangleCapacity));
				clothHits = std::move(hits);
				clothHitCounts = std::move(counts);
				clothHitsUav = clothHits->uav;
				clothHitCountsUav = clothHitCounts->uav;
				clothHitCapacity = clothTriangleCapacity;
			}
			const std::uint32_t foldSlots = (clothBoneListCapacity / GpuRig::kClothFoldChunk + 1u) * kContactLanes;
			if (!clothFolds || clothFoldCapacity < foldSlots) {
				auto folds = std::make_unique<Buffer>(StructuredBufferDesc<GpuRig::ClothFold>(foldSlots, true, false), nullptr, "SmpPhysics::PoolClothFolds");
				folds->CreateUAV(StructuredUavDesc(foldSlots));
				clothFolds = std::move(folds);
				clothFoldsUav = clothFolds->uav;
				clothFoldCapacity = foldSlots;
			}
		} catch (const std::exception&) {
			Fail("the GPU solver cloth contact buffers could not be created; claimed hair stays at the engine's pose");
			return;
		}
		clothVertexCount = static_cast<std::uint32_t>(clothVertexShadow.size());
		clothTriangleCount = static_cast<std::uint32_t>(clothTriangleShadow.size());
	}

	GpuSolver::SurfacePrepInfo GpuSolver::SurfacePrep() const noexcept
	{
		return { surfacePrepActive,
			static_cast<std::uint32_t>(surfacePrep.metas.size()),
			static_cast<std::uint32_t>(surfacePrep.vertices.size()),
			static_cast<std::uint32_t>(surfacePrep.triangles.size()),
			static_cast<std::uint32_t>(surfacePrep.nodes.size()),
			surfacePrep.refitLevels };
	}

	GpuSolver::SurfacePrepCheckResult GpuSolver::LastSurfacePrepCheck() const
	{
		const std::lock_guard lock(surfacePrepCheckMutex);
		return surfacePrepCheckResult;
	}

	void GpuSolver::RunSurfacePrepCheck()
	{
		SurfacePrepCheckResult result;
		const auto store = [this](SurfacePrepCheckResult a_result) {
			const std::lock_guard lock(surfacePrepCheckMutex);
			surfacePrepCheckResult = std::move(a_result);
		};
		if (!surfacePrepActive || !drivers || !gpuSurfaceTriangles || !gpuSurfaceNodes) {
			result.reason = "the GPU collision-surface path is not active; collision surfaces are produced on the CPU";
			store(std::move(result));
			return;
		}
		if (frameCounter == 0u) {
			result.reason = "no frame has been submitted yet; submit one with the GPU backend active";
			store(std::move(result));
			return;
		}
		auto* device = globals::d3d::device;
		auto* context = globals::d3d::context;
		if (!device || !context) {
			result.reason = "the D3D11 device or context is unavailable";
			store(std::move(result));
			return;
		}

		const auto read = [device, context](ID3D11Buffer* a_source, std::uint32_t a_bytes, const char* a_name, void* a_destination) {
			if (a_bytes == 0u)
				return true;
			winrt::com_ptr<ID3D11Buffer> staging;
			const D3D11_BUFFER_DESC stagingDesc = StagingBufferDesc(a_bytes);
			if (FAILED(device->CreateBuffer(&stagingDesc, nullptr, staging.put())))
				return false;
			Util::SetResourceName(staging.get(), "%s", a_name);
			const D3D11_BOX box = ByteRange(0u, a_bytes);
			context->CopySubresourceRegion(staging.get(), 0, 0, 0, 0, a_source, 0, &box);
			D3D11_MAPPED_SUBRESOURCE mapped{};
			if (FAILED(context->Map(staging.get(), 0, D3D11_MAP_READ, 0, &mapped)))
				return false;
			std::memcpy(a_destination, mapped.pData, a_bytes);
			context->Unmap(staging.get(), 0);
			return true;
		};

		// The staging copy must see every dispatch of the frame it compares.
		context->Flush();

		const std::uint32_t triangleCount = static_cast<std::uint32_t>(surfacePrep.triangles.size());
		const std::uint32_t nodeCount = static_cast<std::uint32_t>(surfacePrep.nodes.size());
		std::vector<GpuRig::DriverWorld> driverRecords(driverHighWater);
		std::vector<GpuRig::SurfaceTriangle> actualTriangles(triangleCount);
		std::vector<BvhNode> actualNodes(nodeCount);
		if (!read(drivers->resource.get(), driverHighWater * static_cast<std::uint32_t>(sizeof(GpuRig::DriverWorld)),
				"SmpPhysics::SurfacePrepCheckStagingDrivers", driverRecords.data()) ||
			!read(gpuSurfaceTriangles->resource.get(), triangleCount * static_cast<std::uint32_t>(sizeof(GpuRig::SurfaceTriangle)),
				"SmpPhysics::SurfacePrepCheckStagingTriangles", actualTriangles.data()) ||
			!read(gpuSurfaceNodes->resource.get(), nodeCount * static_cast<std::uint32_t>(sizeof(BvhNode)),
				"SmpPhysics::SurfacePrepCheckStagingNodes", actualNodes.data())) {
			result.reason = "a surface prep buffer could not be copied back for reading";
			store(std::move(result));
			return;
		}

		std::vector<Transform> driverWorlds(driverHighWater);
		for (std::uint32_t slot = 0; slot < driverHighWater; ++slot) {
			const GpuRig::DriverWorld& packed = driverRecords[slot];
			driverWorlds[slot] = { Quat{ packed.rotation.x, packed.rotation.y, packed.rotation.z, packed.rotation.w },
				Vec3{ packed.translation.x, packed.translation.y, packed.translation.z } };
		}

		std::vector<GpuRig::SurfaceTriangle> expectedTriangles;
		std::vector<BvhNode> expectedNodes;
		GpuRig::ReferenceSurfacePrep(surfacePrep, driverWorlds, expectedTriangles, expectedNodes);
		const GpuRig::SurfacePrepComparison comparison = GpuRig::CompareSurfacePrep(expectedTriangles, expectedNodes,
			actualTriangles.data(), actualNodes.data(), triangleCount, nodeCount);

		result.ran = true;
		result.triangles = comparison.triangles;
		result.nodes = comparison.nodes;
		result.maxTriangleError = comparison.maxTriangleError;
		result.maxNormalError = comparison.maxNormalError;
		result.triangleMismatches = comparison.triangleMismatches;
		result.maxNodeError = comparison.maxNodeError;
		result.nodeMismatches = comparison.nodeMismatches;
		result.topologyMismatches = comparison.topologyMismatches;
		store(std::move(result));
	}

	void GpuSolver::DisableSurfacePrep(const char* a_reason)
	{
		surfacePrepActive = false;
		if (surfacePrepWarned)
			return;
		surfacePrepWarned = true;
		logger::warn("{} {}", kLogPrefix, a_reason);
	}

	void GpuSolver::RebuildSurfacePrep()
	{
		surfacePrepDirty = false;
		surfacePrep.Clear();
		surfacePrepActive = false;
		if (!EnsureSurfacePrepShaders()) {
			DisableSurfacePrep("the surface prep compute shader did not compile; collision surfaces stay on the CPU path");
			return;
		}
		for (Claim& claim : claims) {
			claim.surfaceFirst = static_cast<std::uint32_t>(surfacePrep.metas.size());
			claim.surfaceCount = 0u;
			if (!claim.surfaceTopologyReady) {
				surfacePrep.Clear();
				DisableSurfacePrep("a claim's collision surface hierarchies are not built yet; collision surfaces stay on the CPU path");
				return;
			}
			for (std::size_t surface = 0; surface < claim.surfaces.size(); ++surface) {
				TriangleBvh& bvh = claim.surfaceBvhs[surface];
				if (!bvh.Ready())
					continue;
				if (!GpuRig::AppendSurfacePrep(surfacePrep, claim.surfaces[surface], bvh, claim.driverOffset, claim.driverCount)) {
					surfacePrep.Clear();
					DisableSurfacePrep("a collision surface hierarchy could not be packed for the GPU; collision surfaces stay on the CPU path");
					return;
				}
			}
			claim.surfaceCount = static_cast<std::uint32_t>(surfacePrep.metas.size()) - claim.surfaceFirst;
		}
		GpuRig::FinalizeSurfacePrep(surfacePrep);
		if (!CreateSurfacePrepBuffers()) {
			surfacePrep.Clear();
			DisableSurfacePrep("the surface prep buffers could not be created; collision surfaces stay on the CPU path");
			return;
		}
		surfacePrepActive = true;
		surfacePrepWarned = false;
	}

	bool GpuSolver::CreateSurfacePrepBuffers()
	{
		auto* context = globals::d3d::context;
		if (!globals::d3d::device || !context)
			return false;

		// A structured buffer needs at least one element; a count of zero is still a valid, never-read resource.
		const auto count = [](const auto& a_data) noexcept {
			return std::max<std::uint32_t>(1u, static_cast<std::uint32_t>(a_data.size()));
		};
		const std::uint32_t vertexCount = count(surfacePrep.vertices);
		const std::uint32_t boneCount = count(surfacePrep.bones);
		const std::uint32_t triangleCount = count(surfacePrep.triangles);
		const std::uint32_t nodeCount = count(surfacePrep.nodes);
		const std::uint32_t leafCount = count(surfacePrep.leaves);
		const std::uint32_t refitCount = count(surfacePrep.refit);
		const std::uint32_t metaCount = count(surfacePrep.metas);
		try {
			surfacePrepVertices = std::make_unique<Buffer>(StructuredBufferDesc<GpuRig::SurfacePrepVertex>(vertexCount, false, false), nullptr, "SmpPhysics::SurfacePrepVertices");
			surfacePrepVertices->CreateSRV(StructuredSrvDesc(vertexCount));
			surfacePrepVerticesSrv = surfacePrepVertices->srv;
			surfacePrepBones = std::make_unique<Buffer>(StructuredBufferDesc<GpuRig::SurfacePrepBone>(boneCount, false, false), nullptr, "SmpPhysics::SurfacePrepBones");
			surfacePrepBones->CreateSRV(StructuredSrvDesc(boneCount));
			surfacePrepBonesSrv = surfacePrepBones->srv;
			surfacePrepTriangles = std::make_unique<Buffer>(StructuredBufferDesc<GpuRig::SurfacePrepTriangle>(triangleCount, false, false), nullptr, "SmpPhysics::SurfacePrepTriangles");
			surfacePrepTriangles->CreateSRV(StructuredSrvDesc(triangleCount));
			surfacePrepTrianglesSrv = surfacePrepTriangles->srv;
			surfacePrepNodes = std::make_unique<Buffer>(StructuredBufferDesc<GpuRig::SurfacePrepNode>(nodeCount, false, false), nullptr, "SmpPhysics::SurfacePrepNodes");
			surfacePrepNodes->CreateSRV(StructuredSrvDesc(nodeCount));
			surfacePrepNodesSrv = surfacePrepNodes->srv;
			surfacePrepLeaves = std::make_unique<Buffer>(StructuredBufferDesc<GpuRig::SurfacePrepLeaf>(leafCount, false, false), nullptr, "SmpPhysics::SurfacePrepLeaves");
			surfacePrepLeaves->CreateSRV(StructuredSrvDesc(leafCount));
			surfacePrepLeavesSrv = surfacePrepLeaves->srv;
			surfacePrepRefit = std::make_unique<Buffer>(StructuredBufferDesc<GpuRig::SurfacePrepRefit>(refitCount, false, false), nullptr, "SmpPhysics::SurfacePrepRefit");
			surfacePrepRefit->CreateSRV(StructuredSrvDesc(refitCount));
			surfacePrepRefitSrv = surfacePrepRefit->srv;
			surfacePrepMetas = std::make_unique<Buffer>(StructuredBufferDesc<GpuRig::SurfaceMeta>(metaCount, false, false), nullptr, "SmpPhysics::SurfacePrepMetas");
			surfacePrepMetas->CreateSRV(StructuredSrvDesc(metaCount));
			surfacePrepMetasSrv = surfacePrepMetas->srv;
			posedVertices = std::make_unique<Buffer>(StructuredBufferDesc<GpuRig::Float4>(vertexCount, true, false), nullptr, "SmpPhysics::SurfacePrepPosedVertices");
			posedVertices->CreateUAV(StructuredUavDesc(vertexCount));
			posedVerticesUav = posedVertices->uav;
			skinTranslations = std::make_unique<Buffer>(StructuredBufferDesc<GpuRig::Float4>(boneCount, true, false), nullptr, "SmpPhysics::SurfacePrepSkinTranslations");
			skinTranslations->CreateUAV(StructuredUavDesc(boneCount));
			skinTranslationsUav = skinTranslations->uav;
			skinRotations = std::make_unique<Buffer>(StructuredBufferDesc<GpuRig::Float4>(boneCount, true, false), nullptr, "SmpPhysics::SurfacePrepSkinRotations");
			skinRotations->CreateUAV(StructuredUavDesc(boneCount));
			skinRotationsUav = skinRotations->uav;
			gpuSurfaceTriangles = std::make_unique<Buffer>(StructuredBufferDesc<GpuRig::SurfaceTriangle>(triangleCount, true, false), nullptr, "SmpPhysics::GpuSurfaceTriangles");
			gpuSurfaceTriangles->CreateUAV(StructuredUavDesc(triangleCount));
			gpuSurfaceTriangles->CreateSRV(StructuredSrvDesc(triangleCount));
			gpuSurfaceTrianglesUav = gpuSurfaceTriangles->uav;
			gpuSurfaceTrianglesSrv = gpuSurfaceTriangles->srv;
			gpuSurfaceNodes = std::make_unique<Buffer>(StructuredBufferDesc<BvhNode>(nodeCount, true, false), nullptr, "SmpPhysics::GpuSurfaceNodes");
			gpuSurfaceNodes->CreateUAV(StructuredUavDesc(nodeCount));
			gpuSurfaceNodes->CreateSRV(StructuredSrvDesc(nodeCount));
			gpuSurfaceNodesUav = gpuSurfaceNodes->uav;
			gpuSurfaceNodesSrv = gpuSurfaceNodes->srv;
			if (!surfacePrepConstants)
				surfacePrepConstants = std::make_unique<ConstantBuffer>(ConstantBufferDesc<GpuSurfacePrepConstants>(true), "SmpPhysics::SurfacePrepConstants");
		} catch (const std::exception&) {
			return false;
		}

		const auto upload = [context](const Buffer& a_buffer, const auto& a_data) {
			if (!a_data.empty())
				context->UpdateSubresource(a_buffer.resource.get(), 0, nullptr, a_data.data(), 0, 0);
		};
		upload(*surfacePrepVertices, surfacePrep.vertices);
		upload(*surfacePrepBones, surfacePrep.bones);
		upload(*surfacePrepTriangles, surfacePrep.triangles);
		upload(*surfacePrepNodes, surfacePrep.nodes);
		upload(*surfacePrepLeaves, surfacePrep.leaves);
		upload(*surfacePrepRefit, surfacePrep.refit);
		upload(*surfacePrepMetas, surfacePrep.metas);
		return true;
	}

	void GpuSolver::PrepareFrameSurfaces(const std::vector<FrameClaim>& a_claims)
	{
		surfaceFrame.Clear();
		{
			ZoneScopedN("SmpPhysics::SurfacePrep");
			for (const FrameClaim& frameClaim : a_claims) {
				const auto found = claimIndex.find(frameClaim.token);
				if (found == claimIndex.end() || !frameClaim.driverWorld)
					continue;
				Claim& claim = claims[found->second];
				if (claim.surfaceTopologyReady || frameClaim.driverWorld->size() != claim.driverCount)
					continue;
				for (std::size_t surface = 0; surface < claim.surfaces.size(); ++surface) {
					const std::vector<Vec3> posed = PoseSurface(claim.surfaces[surface], *frameClaim.driverWorld);
					claim.surfaceBvhs[surface].Build(posed, claim.surfaces[surface].indices);
				}
				claim.surfaceTopologyReady = true;
				surfacePrepDirty = true;
			}
			if (surfacePrepDirty)
				RebuildSurfacePrep();
		}
		if (!surfacePrepActive) {
			ZoneScopedN("SmpPhysics::AppendSurfaces");
			if (!surfacePool) {
				const std::size_t cores = std::max<std::size_t>(std::thread::hardware_concurrency() / 2, 1);
				surfacePool = std::make_unique<BS::thread_pool<>>(std::min(cores, kMaxSurfaceThreads));
			}
			for (std::size_t index = 0; index < a_claims.size(); ++index) {
				const FrameClaim& frameClaim = a_claims[index];
				const auto found = claimIndex.find(frameClaim.token);
				if (found == claimIndex.end() || !frameClaim.kinematicWorld || !frameClaim.driverWorld)
					continue;
				Claim& claim = claims[found->second];
				if (frameClaim.kinematicWorld->size() != claim.boneCount || frameClaim.driverWorld->size() != claim.driverCount)
					continue;
				const bool reusable = claim.cachedDrivers.size() == frameClaim.driverWorld->size() &&
				                      SurfaceDrift(claim.surfaceDrivers, claim.cachedDrivers, *frameClaim.driverWorld) < kSurfaceReuseDrift;
				if (reusable)
					continue;
				claim.cachedDrivers = *frameClaim.driverWorld;
				for (std::size_t surface = 0; surface < claim.surfaces.size(); ++surface) {
					claim.surfaceCache[surface].Clear();
					surfacePool->detach_task([&part = claim.surfaceCache[surface], &claim, &frameClaim, surface] {
						GpuRig::AppendSurface(part, claim.surfaces[surface], *frameClaim.driverWorld, claim.surfaceBvhs[surface]);
					});
				}
			}
			surfacePool->wait();
		}
	}

	bool GpuSolver::PackFrameClaims(const std::vector<FrameClaim>& a_claims, FramePack& a_pack)
	{
		ZoneScopedN("SmpPhysics::PackFrameClaims");
		auto* context = globals::d3d::context;
		a_pack.treeInputs.assign(treeHighWater, GpuRig::TreeInput{});
		a_pack.driverWorlds.assign(driverHighWater, GpuRig::DriverWorld{});
		bool submitted = false;
		for (std::size_t claimSlot = 0; claimSlot < a_claims.size(); ++claimSlot) {
			const FrameClaim& frameClaim = a_claims[claimSlot];
			const auto found = claimIndex.find(frameClaim.token);
			if (found == claimIndex.end() || !frameClaim.kinematicWorld)
				continue;
			Claim& claim = claims[found->second];
			const std::vector<Transform>& kinematic = *frameClaim.kinematicWorld;
			if (kinematic.size() != claim.boneCount)
				continue;

			std::uint32_t firstSurface = claim.surfaceFirst;
			std::uint32_t surfaceCount = claim.surfaceCount;
			if (!surfacePrepActive) {
				firstSurface = static_cast<std::uint32_t>(surfaceFrame.metas.size());
				if (frameClaim.driverWorld && frameClaim.driverWorld->size() == claim.driverCount) {
					for (const GpuRig::SurfaceFrame& part : claim.surfaceCache) {
						std::uint32_t partFirst = 0;
						GpuRig::MergeSurfaceFrame(surfaceFrame, part, partFirst);
					}
				}
				surfaceCount = static_cast<std::uint32_t>(surfaceFrame.metas.size()) - firstSurface;
			}
			for (std::size_t tree = 0; tree < claim.rig.trees.size(); ++tree) {
				claim.inputs[tree] = GpuRig::MakeTreeInput(firstSurface, surfaceCount, frameClaim.rigidContacts, claim.boneCount);
				a_pack.treeInputs[claim.boneOffset + static_cast<std::uint32_t>(tree)] = claim.inputs[tree];
			}
			const bool supplied = frameClaim.driverWorld && frameClaim.driverWorld->size() == claim.driverCount;
			a_pack.packedClaims.push_back(found->second);
			if (!frameClaim.hasPreviousInput || frameClaim.resetRequested || !claim.steppedDriversValid)
				a_pack.freshDrivers.emplace_back(claim.driverOffset, claim.driverCount);
			for (std::uint32_t index = 0; index < claim.driverCount; ++index) {
				const Transform driver = supplied ? (*frameClaim.driverWorld)[index] : Transform{};
				GpuRig::DriverWorld& packed = a_pack.driverWorlds[claim.driverOffset + index];
				packed.rotation = { driver.rotation.w, driver.rotation.x, driver.rotation.y, driver.rotation.z };
				packed.translation = { driver.translation.x, driver.translation.y, driver.translation.z, 0.0f };
			}
			const std::size_t staticColliders = claim.rig.collision.colliders.size();
			if (frameClaim.externalColliders && frameClaim.externalColliders->size() == kExternalColliderSlots &&
				staticColliders >= kExternalColliderSlots) {
				ZoneScopedN("SmpPhysics::ExternalColliders");
				std::vector<GpuRig::ColliderPacked> externals = *frameClaim.externalColliders;
				for (GpuRig::ColliderPacked& external : externals) {
					if (external.kind.y != GpuRig::kInvalidDriverSlot)
						external.kind.y += claim.driverOffset;
				}
				const UINT first = static_cast<UINT>((claim.colliderOffset + staticColliders - kExternalColliderSlots) * sizeof(GpuRig::ColliderPacked));
				const D3D11_BOX region{ first, 0, 0, first + static_cast<UINT>(externals.size() * sizeof(GpuRig::ColliderPacked)), 1, 1 };
				context->UpdateSubresource(colliders->resource.get(), 0, &region, externals.data(), 0, 0);
			}
			RigidGpuSolver::FrameClaim& rigid = a_pack.rigidClaims.emplace_back();
			rigid.token = frameClaim.token;
			rigid.kinematicWorld = frameClaim.kinematicWorld;
			rigid.hasPreviousInput = frameClaim.hasPreviousInput;
			rigid.resetRequested = frameClaim.resetRequested;
			rigid.deltaTime = frameClaim.deltaTime;
			rigid.wind = frameClaim.wind;
			rigid.windTime = frameClaim.windTime;
			rigid.windFactor = frameClaim.windFactor;
			for (const Grab& grab : frameClaim.grabs)
				rigid.pins.push_back({ grab.bone, grab.local, grab.target, grab.stiffness });
			if (claim.firstFrame == 0u) {
				claim.firstFrame = frameCounter + 1u;
				a_pack.startedClaims.push_back(found->second);
			}
			submitted = true;
		}
		return submitted;
	}

	void GpuSolver::UndoFramePack(const FramePack& a_pack)
	{
		for (const std::size_t index : a_pack.startedClaims)
			claims[index].firstFrame = 0u;
	}

	bool GpuSolver::UploadFramePack(const FramePack& a_pack)
	{
		auto* context = globals::d3d::context;
		{
			ZoneScopedN("SmpPhysics::UploadFrame");
			if (!surfacePrepActive) {
				if (!UploadDynamic(surfaceMetas, surfaceMetasSrv, surfaceMetaCapacity, surfaceFrame.metas, "SmpPhysics::PoolSurfaces") ||
					!UploadDynamic(surfaceTriangles, surfaceTrianglesSrv, surfaceTriangleCapacity, surfaceFrame.triangles, "SmpPhysics::PoolSurfaceTriangles") ||
					!UploadDynamic(surfaceNodes, surfaceNodesSrv, surfaceNodeCapacity, surfaceFrame.nodes, "SmpPhysics::PoolSurfaceNodes")) {
					return false;
				}
			}

			if (!UploadDiscard(context, inputs->resource.get(), a_pack.treeInputs.data(), a_pack.treeInputs.size() * sizeof(GpuRig::TreeInput)))
				return false;

			if (!a_pack.driverWorlds.empty() && !UploadDiscard(context, drivers->resource.get(), a_pack.driverWorlds.data(), a_pack.driverWorlds.size() * sizeof(GpuRig::DriverWorld)))
				return false;
		}
		return true;
	}

	void GpuSolver::DispatchSurfacePrep()
	{
		auto* context = globals::d3d::context;
		if (surfacePrepActive) {
			GpuSurfacePrepConstants prep;
			prep.boneCount = static_cast<std::uint32_t>(surfacePrep.bones.size());
			prep.vertexCount = static_cast<std::uint32_t>(surfacePrep.vertices.size());
			prep.triangleCount = static_cast<std::uint32_t>(surfacePrep.triangles.size());
			prep.leafCount = static_cast<std::uint32_t>(surfacePrep.leaves.size());
			surfacePrepConstants->Update(prep);
			ID3D11Buffer* prepConstantView = surfacePrepConstants->CB();
			ID3D11ShaderResourceView* prepViews[kSurfacePrepViewCount] = { surfacePrepVerticesSrv.get(), surfacePrepBonesSrv.get(),
				surfacePrepTrianglesSrv.get(), surfacePrepNodesSrv.get(), surfacePrepLeavesSrv.get(), surfacePrepRefitSrv.get(), driversSrv.get() };
			ID3D11UnorderedAccessView* prepTargets[kSurfacePrepTargetCount] = { posedVerticesUav.get(), skinTranslationsUav.get(),
				skinRotationsUav.get(), gpuSurfaceTrianglesUav.get(), gpuSurfaceNodesUav.get() };
			CS_GPU_PASS("SmpPhysics::SurfacePrep");
			context->CSSetConstantBuffers(0, 1, &prepConstantView);
			context->CSSetShaderResources(0, kSurfacePrepViewCount, prepViews);
			context->CSSetUnorderedAccessViews(0, kSurfacePrepTargetCount, prepTargets, nullptr);
			if (prep.boneCount > 0u) {
				CS_GPU_PASS("SmpPhysics::SP_Bones");
				context->CSSetShader(surfaceBonesShader->get(), nullptr, 0);
				context->Dispatch(Groups(prep.boneCount), 1, 1);
			}
			if (prep.vertexCount > 0u) {
				CS_GPU_PASS("SmpPhysics::SP_Pose");
				context->CSSetShader(surfacePoseShader->get(), nullptr, 0);
				context->Dispatch(Groups(prep.vertexCount), 1, 1);
			}
			if (prep.triangleCount > 0u) {
				CS_GPU_PASS("SmpPhysics::SP_Triangles");
				context->CSSetShader(surfaceTrianglesShader->get(), nullptr, 0);
				context->Dispatch(Groups(prep.triangleCount), 1, 1);
			}
			if (prep.leafCount > 0u) {
				CS_GPU_PASS("SmpPhysics::SP_Leaves");
				context->CSSetShader(surfaceLeavesShader->get(), nullptr, 0);
				context->Dispatch(Groups(prep.leafCount), 1, 1);
			}
			for (std::uint32_t level = 0; level < surfacePrep.refitLevels; ++level) {
				prep.refitFirst = surfacePrep.refitLevelStart[level];
				prep.refitCount = surfacePrep.refitLevelStart[level + 1u] - prep.refitFirst;
				if (prep.refitCount == 0u)
					continue;
				CS_GPU_PASS("SmpPhysics::SP_Refit");
				surfacePrepConstants->Update(prep);
				context->CSSetShader(surfaceRefitShader->get(), nullptr, 0);
				context->Dispatch(Groups(prep.refitCount), 1, 1);
			}
			ID3D11ShaderResourceView* noPrepViews[kSurfacePrepViewCount] = {};
			ID3D11UnorderedAccessView* noPrepTargets[kSurfacePrepTargetCount] = {};
			context->CSSetShaderResources(0, kSurfacePrepViewCount, noPrepViews);
			context->CSSetUnorderedAccessViews(0, kSurfacePrepTargetCount, noPrepTargets, nullptr);
		}
		if (surfacePrepCheckRequested.exchange(false))
			RunSurfacePrepCheck();
	}

	void GpuSolver::DispatchContacts(const GpuSolverConstants& a_values)
	{
		auto* context = globals::d3d::context;
		const bool cloth = a_values.clothTriangleCount > 0u;
		ID3D11ShaderResourceView* views[kStepViewCount] = { cloth ? clothVertexListSrv.get() : nullptr, treesSrv.get(), inputsSrv.get(), cloth ? clothTriangleListSrv.get() : nullptr,
			cloth ? boneClothSrv.get() : nullptr, spheresSrv.get(), collidersSrv.get(), driversSrv.get(),
			surfacePrepActive ? surfacePrepMetasSrv.get() : surfaceMetasSrv.get(),
			surfacePrepActive ? gpuSurfaceTrianglesSrv.get() : surfaceTrianglesSrv.get(),
			surfacePrepActive ? gpuSurfaceNodesSrv.get() : surfaceNodesSrv.get(),
			cloth ? clothBoneListSrv.get() : nullptr, sphereListSrv.get(), boneTreesSrv.get(), driverDeltasSrv.get() };
		ID3D11UnorderedAccessView* contactTargets[kStepTargetCount] = { cloth ? clothPosedUav.get() : nullptr, worldsUav.get(), cloth ? clothHitsUav.get() : nullptr, contactsUav.get(),
			cloth ? clothHitCountsUav.get() : nullptr, sphereHitsUav.get(), boneCullUav.get(), cloth ? clothFoldsUav.get() : nullptr };
		ID3D11Buffer* constantView = constants->CB();
		CS_GPU_PASS("SmpPhysics::ContactPasses");
		context->CSSetConstantBuffers(0, 1, &constantView);
		context->CSSetShaderResources(0, kStepViewCount, views);
		context->CSSetUnorderedAccessViews(0, kStepTargetCount, contactTargets, nullptr);
		{
			CS_GPU_PASS("SmpPhysics::ContactCull");
			context->CSSetShader(contactCullShader->get(), nullptr, 0);
			context->Dispatch(Groups(a_values.boneCount), 1, 1);
		}
		if (cloth) {
			{
				CS_GPU_PASS("SmpPhysics::ClothVertices");
				context->CSSetShader(clothVertexShader->get(), nullptr, 0);
				context->Dispatch(Groups(a_values.clothVertexCount), 1, 1);
			}
			{
				CS_GPU_PASS("SmpPhysics::ClothTriangles");
				context->CSSetShader(clothTriangleShader->get(), nullptr, 0);
				context->Dispatch(Groups(a_values.clothTriangleCount), 1, 1);
			}
			CS_GPU_PASS("SmpPhysics::ClothFold");
			context->CSSetShader(clothFoldShader->get(), nullptr, 0);
			context->Dispatch(Groups(static_cast<std::uint32_t>(clothBoneListShadow.size()) / GpuRig::kClothFoldChunk), 1, 1);
		}
		{
			CS_GPU_PASS("SmpPhysics::ContactSpheres");
			context->CSSetShader(contactSphereShader->get(), nullptr, 0);
			context->Dispatch(Groups(a_values.sphereCount), 1, 1);
		}
		{
			CS_GPU_PASS("SmpPhysics::ContactReduce");
			context->CSSetShader(contactReduceShader->get(), nullptr, 0);
			context->Dispatch(Groups(a_values.boneCount), 1, 1);
		}
	}

	bool GpuSolver::SubmitFrame(const std::vector<FrameClaim>& a_claims)
	{
		ZoneScopedN("GpuSolver::SubmitFrame");
		try {
			return SubmitFrameUnchecked(a_claims);
		} catch (const std::exception& error) {
			const std::string reason = std::format("a GPU frame could not be recorded ({}); claimed hair stays at the engine's pose", error.what());
			Fail(reason.c_str());
			return false;
		}
	}

	bool GpuSolver::SubmitFrameUnchecked(const std::vector<FrameClaim>& a_claims)
	{
		// While the startup compile runs, claims keep the engine pose (PoseReady stays false) instead of the game waiting for it.
		// The shaders are rebuilt here after a shader cache clear, so claims already running do not dispatch nothing.
		if (!available || !trees || claims.empty() || Util::GetShaderPrewarmStatus().running || !EnsureShaders())
			return false;
		auto* context = globals::d3d::context;
		if (!context)
			return false;

		UploadStatics();

		ZoneScopedN("SmpPhysics::SubmitFrame");
		PrepareFrameSurfaces(a_claims);
		FramePack pack;
		if (!PackFrameClaims(a_claims, pack))
			return false;
		if (!UploadFramePack(pack)) {
			UndoFramePack(pack);
			return false;
		}

		ZoneNamedN(recordZone, "SmpPhysics::RecordDispatches", true);
		++frameCounter;
		GpuSolverConstants values;
		values.boneCount = boneHighWater;
		values.sphereCount = sphereCount;
		values.clothVertexCount = clothVertexCount;
		values.clothTriangleCount = clothTriangleCount;
		constants->Update(values);

		Readback* slot = nullptr;
		for (Readback& readback : readbacks) {
			if (!readback.pending && !readback.hasResult) {
				slot = &readback;
				break;
			}
		}

		const bool takeContacts = !pack.rigidClaims.empty() && std::ranges::any_of(a_claims, [](const FrameClaim& a_claim) { return a_claim.rigidContacts; });
		if (!takeContacts)
			DispatchSurfacePrep();
		if (!pack.rigidClaims.empty()) {
			std::vector<GpuRig::DriverWorld> from = steppedDrivers.size() == pack.driverWorlds.size() ? steppedDrivers : pack.driverWorlds;
			for (const auto& [offset, count] : pack.freshDrivers)
				std::copy_n(pack.driverWorlds.begin() + offset, count, from.begin() + offset);
			std::vector<GpuRig::DriverRange> liveDrivers;
			std::vector<GpuRig::DriverRange> clothDrivers;
			std::vector<GpuRig::DriverRange> otherDrivers;
			liveDrivers.reserve(pack.packedClaims.size());
			for (const std::size_t index : pack.packedClaims) {
				liveDrivers.emplace_back(claims[index].driverOffset, claims[index].driverCount);
				(claims[index].rig.collision.clothTriangles.empty() ? otherDrivers : clothDrivers).emplace_back(claims[index].driverOffset, claims[index].driverCount);
			}
			std::vector<GpuRig::DriverWorld> blended = pack.driverWorlds;
			std::vector<GpuRig::DriverDelta> deltas(pack.driverWorlds.size());
			bool deltasUploaded = false;
			const auto contactPass = [this, &values, &pack, &from, &blended, &deltas, &liveDrivers, &clothDrivers, &otherDrivers, &deltasUploaded](std::uint32_t, float a_start, float a_fraction, float a_total) {
				if (!pack.driverWorlds.empty()) {
					GpuRig::BlendDriverRanges(from, pack.driverWorlds, otherDrivers, a_fraction, blended);
					// Cloth triangles are posed where the step begins; a proxy posed where it ends is a step ahead of them and passes through.
					GpuRig::BlendDriverRanges(from, pack.driverWorlds, clothDrivers, a_start, blended);
					(void)UploadDiscard(globals::d3d::context, drivers->resource.get(), blended.data(), blended.size() * sizeof(GpuRig::DriverWorld));
				}
				if (!deltas.empty() && !deltasUploaded) {
					GpuRig::FillDriverDeltas(from, pack.driverWorlds, liveDrivers, a_total, deltas);
					(void)UploadDiscard(globals::d3d::context, driverDeltas->resource.get(), deltas.data(), deltas.size() * sizeof(GpuRig::DriverDelta));
					deltasUploaded = true;
				}
				DispatchSurfacePrep();
				DispatchContacts(values);
			};
			const bool stepped = rigidSolver.Dispatch(pack.rigidClaims, worldsUav.get(), outputsUav.get(), takeContacts ? contactsSrv.get() : nullptr, contactPass);
			if (!rigidSolver.Available()) {
				UndoFramePack(pack);
				Fail("the rigid solver became unavailable; claimed hair stays at the engine's pose");
				return false;
			}
			if (stepped) {
				if (takeContacts && !pack.driverWorlds.empty())
					(void)UploadDiscard(globals::d3d::context, drivers->resource.get(), pack.driverWorlds.data(), pack.driverWorlds.size() * sizeof(GpuRig::DriverWorld));
				steppedDrivers = pack.driverWorlds;
				for (Claim& claim : claims)
					claim.steppedDriversValid = false;
				for (const std::size_t index : pack.packedClaims)
					claims[index].steppedDriversValid = true;
			} else if (takeContacts) {
				DispatchSurfacePrep();
			}
		}

		if (!contactStagingPending && contactStatsRequested.exchange(false)) {
			if (!contactStaging || contactStagingSlots != slotCapacity) {
				const Buffer staging(StagingBufferDesc(slotCapacity * kContactLanes * static_cast<std::uint32_t>(sizeof(GpuRig::BoneContact))), nullptr, "SmpPhysics::PoolContactReadback");
				contactStaging = staging.resource;
				contactStagingSlots = slotCapacity;
			}
			context->CopyResource(contactStaging.get(), contacts->resource.get());
			contactStagingPending = true;
		}

		if (slot) {
			context->CopyResource(slot->buffer.get(), outputs->resource.get());
			slot->pending = true;
			slot->hasResult = false;
			slot->result.clear();
			slot->serial = ++nextSerial;
			slot->frame = frameCounter;
		}

		ID3D11ShaderResourceView* noViews[kStepViewCount] = {};
		ID3D11UnorderedAccessView* noTargets[kStepTargetCount] = {};
		ID3D11Buffer* noConstant = nullptr;
		context->CSSetShaderResources(0, kStepViewCount, noViews);
		context->CSSetUnorderedAccessViews(0, kStepTargetCount, noTargets, nullptr);
		context->CSSetConstantBuffers(0, 1, &noConstant);
		context->CSSetShader(nullptr, nullptr, 0);
		if (flushAfterSubmit)
			context->Flush();
		return true;
	}

	std::vector<GpuSolver::ContactStat> GpuSolver::ContactStats() const
	{
		const std::scoped_lock lock(contactStatsMutex);
		return contactStats;
	}

	bool GpuSolver::Poll(std::vector<ClaimResult>& a_results)
	{
		ZoneScopedN("GpuSolver::Poll");
		a_results.clear();
		if (!available || !outputs || claims.empty())
			return false;
		auto* context = globals::d3d::context;
		if (!context)
			return false;

		if (contactStagingPending) {
			D3D11_MAPPED_SUBRESOURCE mapped{};
			if (context->Map(contactStaging.get(), 0, D3D11_MAP_READ, D3D11_MAP_FLAG_DO_NOT_WAIT, &mapped) == S_OK) {
				const auto* data = static_cast<const GpuRig::BoneContact*>(mapped.pData);
				std::vector<ContactStat> stats;
				for (const Claim& claim : claims) {
					ContactStat stat;
					stat.token = claim.token;
					for (std::uint32_t bone = 0; bone < claim.boneCount && claim.boneOffset + bone < contactStagingSlots; ++bone) {
						bool touching = false;
						for (std::uint32_t lane = 0; lane < kContactLanes; ++lane) {
							const GpuRig::BoneContact& contact = data[(claim.boneOffset + bone) * kContactLanes + lane];
							const float depth = contact.depth;
							if (!(depth > 0.0f))
								continue;
							touching = true;
							stat.bones.push_back(bone);
							stat.contacts.push_back(contact);
							stat.meanDepth += depth;
							stat.maxDepth = std::max(stat.maxDepth, depth);
						}
						if (touching)
							++stat.touching;
					}
					if (!stat.contacts.empty())
						stat.meanDepth /= static_cast<float>(stat.contacts.size());
					stats.push_back(stat);
				}
				context->Unmap(contactStaging.get(), 0);
				contactStagingPending = false;
				const std::scoped_lock lock(contactStatsMutex);
				contactStats = std::move(stats);
			}
		}

		for (Readback& readback : readbacks) {
			if (!readback.pending)
				continue;
			D3D11_MAPPED_SUBRESOURCE mapped{};
			const HRESULT result = context->Map(readback.buffer.get(), 0, D3D11_MAP_READ, waitForReadback ? 0 : D3D11_MAP_FLAG_DO_NOT_WAIT, &mapped);
			if (result == DXGI_ERROR_WAS_STILL_DRAWING)
				continue;
			if (FAILED(result)) {
				readback.pending = false;
				Fail("the GPU solver readback failed; claimed hair stays at the engine's pose");
				return false;
			}
			const auto* data = static_cast<const GpuRig::BoneOutput*>(mapped.pData);
			readback.result.clear();
			readback.result.reserve(claims.size());
			for (const Claim& claim : claims) {
				// A snapshot taken before the claim's first dispatch holds whatever its slots held then.
				if (claim.firstFrame == 0u || claim.firstFrame > readback.frame)
					continue;
				ClaimResult claimResult;
				claimResult.token = claim.token;
				claimResult.frame = readback.frame;
				claimResult.bones.assign(claim.boneCount, Transform{});
				for (const std::uint32_t bone : claim.rig.order) {
					const GpuRig::BoneOutput& output = data[bone];
					Transform& transform = claimResult.bones[bone - claim.boneOffset];
					transform.rotation = { output.rotation.x, output.rotation.y, output.rotation.z, output.rotation.w };
					transform.translation = { output.translation.x, output.translation.y, output.translation.z };
					claimResult.failed = claimResult.failed || output.status.x != 0u;
				}
				readback.result.push_back(std::move(claimResult));
			}
			context->Unmap(readback.buffer.get(), 0);
			readback.pending = false;
			readback.hasResult = true;
		}

		Readback* newest = nullptr;
		for (Readback& readback : readbacks) {
			if (readback.hasResult && (!newest || readback.serial > newest->serial))
				newest = &readback;
		}
		if (!newest)
			return false;

		for (Readback& readback : readbacks) {
			if (&readback != newest) {
				readback.hasResult = false;
				readback.result.clear();
			}
		}

		if (frameCounter > newest->frame && frameCounter - newest->frame > kMaximumResultAge) {
			newest->hasResult = false;
			newest->result.clear();
			return false;
		}

		a_results = std::move(newest->result);
		newest->hasResult = false;
		newest->result.clear();
		return true;
	}
}
