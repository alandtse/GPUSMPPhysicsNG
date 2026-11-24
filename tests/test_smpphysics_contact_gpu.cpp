// Runs the contact passes (ContactCS.hlsl: ContactCull, ContactSpheres, ContactReduce) on a WARP
// device and compares every bone's merged contact with the CPU's QueryColliders and ContactMerge, then checks which
// trees the passes serve. A scene with cloth triangles also runs ClothVertices and ClothTriangles, against
// TestVertexAgainstTriangle.

#include "rigid_gpu_harness.h"
#include "smp/ClothTriangles.h"
#include "smp/Collision.h"
#include "smp/GpuRigPacking.h"
#include "smp/SolverCommon.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <array>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <initializer_list>
#include <utility>
#include <vector>

using namespace SmpPhysics;
using namespace SmpPhysics::RigidGpuTest;
using Catch::Approx;

namespace
{
	constexpr std::size_t kBoneCount{ 4 };
	constexpr std::size_t kTreeCount{ 2 };
	constexpr float kContactSkin{ 0.02f };
	constexpr float kSentinelDepth{ 123.0f };
	constexpr float kTolerance{ 1.0e-4f };

	/** @brief Bones 0 and 1 form tree 0 and bones 2 and 3 tree 1. Bone 3 is out of every collider's reach. */
	constexpr std::array<std::uint32_t, kBoneCount> kBoneTrees{ 0u, 0u, 1u, 1u };

	struct ContactKernels
	{
		GpuDevice gpu;
		ComPtr<ID3D11ComputeShader> cull;
		ComPtr<ID3D11ComputeShader> spheres;
		ComPtr<ID3D11ComputeShader> reduce;
		ComPtr<ID3D11ComputeShader> clothVertices;
		ComPtr<ID3D11ComputeShader> clothTriangles;
	};

	const ContactKernels& LoadKernels()
	{
		static const ContactKernels built = [] {
			ContactKernels value;
			value.gpu = CreateDevice(D3D_DRIVER_TYPE_WARP);
			const std::filesystem::path path = std::filesystem::path(SMPPHYSICS_FEATURE_SHADER_DIR) / "SmpPhysics" / "ContactCS.hlsl";
			ID3D11Device* device = value.gpu.device.Get();
			value.cull = CompileKernel(device, "ContactCull", path);
			value.spheres = CompileKernel(device, "ContactSpheres", path);
			value.reduce = CompileKernel(device, "ContactReduce", path);
			value.clothVertices = CompileKernel(device, "ClothVertices", path);
			value.clothTriangles = CompileKernel(device, "ClothTriangles", path);
			return value;
		}();
		return built;
	}

	/** @brief The claim the passes run over: two trees of two bones each, sharing one collider set. */
	struct Scene
	{
		std::vector<Collider> colliders;
		std::vector<std::vector<BoneSphere>> spheres;
		std::vector<Transform> boneWorld;
		Transform driver;
		std::vector<SkinnedSurface> surfaces;
		float contactSkin{ kContactSkin };
		std::uint32_t driverCount{ 1 };                    ///< every driver sits at driver
		ClothTriangleSet cloth;                            ///< empty runs the passes a claim without cloth triangles gets
		std::vector<std::vector<std::uint32_t>> faceKeys;  ///< per surface and source face, the driver slot plus one the GPU surface pass writes into the posed face; a missing entry is 0
	};

	Scene MakeScene()
	{
		Scene scene;
		scene.driver = Transform{};

		Collider ellipsoid;
		ellipsoid.type = Collider::Type::Ellipsoid;
		ellipsoid.driverIndex = 0;
		ellipsoid.halfExtents = { 1.0f, 1.0f, 1.0f };
		Collider capsule;
		capsule.type = Collider::Type::Capsule;
		capsule.driverIndex = 0;
		capsule.segmentA = { -1.0f, 0.0f, -1.5f };
		capsule.segmentB = { 1.0f, 0.0f, -1.5f };
		capsule.radius = 0.3f;
		scene.colliders = { ellipsoid, capsule };

		// Bones 0 and 2 sink into the ellipsoid, bone 1 into the capsule, and bone 3 is far from both.
		scene.boneWorld.resize(kBoneCount);
		scene.boneWorld[0].translation = { 0.1f, 0.0f, 0.97f };
		scene.boneWorld[1].translation = { 0.2f, 0.0f, -1.12f };
		scene.boneWorld[2].translation = { -0.1f, 0.2f, 0.95f };
		scene.boneWorld[3].translation = { 0.0f, 0.0f, 40.0f };

		scene.spheres.resize(kBoneCount);
		for (std::size_t bone = 0; bone < kBoneCount; ++bone) {
			scene.spheres[bone].push_back({ { 0.0f, 0.0f, 0.0f }, 0.1f, 0u });
			scene.spheres[bone].push_back({ { 0.05f, 0.03f, -0.02f }, 0.08f, 0u });
		}
		// Bone 1's first sphere was authored inside the ellipsoid, so it must not be pushed by it.
		scene.spheres[1][0].exemptMask = 1u;
		return scene;
	}

	GpuRig::BoneContact ToBoneContact(const ContactMerge& a_merge)
	{
		GpuRig::BoneContact expected;
		Vec3 normal{};
		Vec3 position{};
		expected.depth = a_merge.Result(normal, position);
		expected.deepest = a_merge.deepest;
		expected.collider = a_merge.collider;
		expected.normal[0] = normal.x;
		expected.normal[1] = normal.y;
		expected.normal[2] = normal.z;
		expected.position[0] = position.x;
		expected.position[1] = position.y;
		expected.position[2] = position.z;
		return expected;
	}

	/** @brief The single contact the CPU merges for one bone, the oracle for a bone that touches one driver. */
	GpuRig::BoneContact ExpectedContact(const Scene& a_scene, std::size_t a_bone)
	{
		ContactMerge merge;
		for (const BoneSphere& sphere : a_scene.spheres[a_bone]) {
			const Vec3 centre = a_scene.boneWorld[a_bone].TransformPoint(sphere.centerLocal);
			const Contact contact = QueryColliders(a_scene.colliders, std::vector<Transform>(a_scene.driverCount, a_scene.driver), centre, sphere.radius + a_scene.contactSkin, sphere.exemptMask);
			merge.Add(contact, centre);
		}
		return ToBoneContact(merge);
	}

	/** @brief The contact lanes the CPU makes for one bone of a scene without surfaces, each collider keyed by its driver. */
	std::array<GpuRig::BoneContact, kContactLanes> ExpectedLanes(const Scene& a_scene, std::size_t a_bone)
	{
		ContactLanes lanes;
		for (const BoneSphere& sphere : a_scene.spheres[a_bone]) {
			const Vec3 centre = a_scene.boneWorld[a_bone].TransformPoint(sphere.centerLocal);
			const Contact contact = QueryColliders(a_scene.colliders, std::vector<Transform>(a_scene.driverCount, a_scene.driver), centre, sphere.radius + a_scene.contactSkin, sphere.exemptMask);
			if (contact.collider >= 0)
				lanes.Add(contact, centre, static_cast<std::uint32_t>(a_scene.colliders[static_cast<std::size_t>(contact.collider)].driverIndex + 1));
		}
		std::array<GpuRig::BoneContact, kContactLanes> expected;
		for (std::size_t lane = 0; lane < kContactLanes; ++lane)
			expected[lane] = ToBoneContact(lanes.lanes[lane]);
		return expected;
	}

	/** @brief A flat quad at a_height facing +Z, skinned to driver 0 so it stays put under an identity driver. */
	SkinnedSurface MakeFloorSurface(float a_height)
	{
		SkinnedSurface surface;
		surface.name = "floor";
		for (const Vec3& corner : { Vec3{ -10.0f, -10.0f, a_height }, Vec3{ 10.0f, -10.0f, a_height }, Vec3{ 10.0f, 10.0f, a_height }, Vec3{ -10.0f, 10.0f, a_height } }) {
			SkinnedVertex vertex;
			vertex.position = corner;
			vertex.bone[0] = 0;
			vertex.weight[0] = 1.0f;
			surface.vertices.push_back(vertex);
		}
		surface.indices = { 0, 1, 2, 0, 2, 3 };
		surface.skinToBone = { Transform{} };
		surface.boneDriver = { 0 };
		return surface;
	}

	/** @brief One sphere per bone over a floor at z = 0: two bones sink into it, one rests above, one is far above. */
	Scene MakeSurfaceScene()
	{
		Scene scene;
		scene.surfaces = { MakeFloorSurface(0.0f) };
		scene.contactSkin = 0.0f;
		scene.boneWorld.resize(kBoneCount);
		const std::array<float, kBoneCount> heights{ 0.4f, 1.6f, 0.7f, 30.0f };
		scene.spheres.resize(kBoneCount);
		for (std::size_t bone = 0; bone < kBoneCount; ++bone) {
			scene.boneWorld[bone].translation = { static_cast<float>(bone), 0.0f, heights[bone] };
			scene.spheres[bone].push_back({ { 0.0f, 0.0f, 0.0f }, 1.0f, 0u });
		}
		return scene;
	}

	/** @brief A floor that keeps pushing a sphere a little below it, with spheres from just above to far under it. */
	Scene MakeDeepReachScene()
	{
		Scene scene = MakeSurfaceScene();
		scene.surfaces[0].margin = 0.2f;
		scene.surfaces[0].contactPenetration = 0.4f;
		const std::array<float, kBoneCount> heights{ 0.5f, -0.2f, -0.5f, -2.5f };
		for (std::size_t bone = 0; bone < kBoneCount; ++bone)
			scene.boneWorld[bone].translation.z = heights[bone];
		return scene;
	}

	/** @brief Twelve triangles in two clusters at z = 0, far enough apart that the BVH splits and the gap between them lies inside its root box. */
	SkinnedSurface MakeClusteredSurface()
	{
		SkinnedSurface surface;
		surface.name = "clusters";
		for (const float x : { -9.0f, 9.0f }) {
			for (const float y : { 0.0f, 3.0f, 6.0f }) {
				const std::uint32_t first = static_cast<std::uint32_t>(surface.vertices.size());
				for (const Vec3& corner : { Vec3{ x - 1.0f, y, 0.0f }, Vec3{ x + 1.0f, y, 0.0f }, Vec3{ x + 1.0f, y + 2.0f, 0.0f }, Vec3{ x - 1.0f, y + 2.0f, 0.0f } }) {
					SkinnedVertex vertex;
					vertex.position = corner;
					vertex.bone[0] = 0;
					vertex.weight[0] = 1.0f;
					surface.vertices.push_back(vertex);
				}
				for (const std::uint32_t offset : { 0u, 1u, 2u, 0u, 2u, 3u })
					surface.indices.push_back(first + offset);
			}
		}
		surface.skinToBone = { Transform{} };
		surface.boneDriver = { 0 };
		return surface;
	}

	/** @brief One sphere per bone over two surface clusters: one bone sinks into each cluster, one hangs in the gap inside the root box, one is far above. */
	Scene MakeClusteredSurfaceScene()
	{
		Scene scene;
		scene.surfaces = { MakeClusteredSurface() };
		scene.contactSkin = 0.0f;
		const std::array<Vec3, kBoneCount> positions{ Vec3{ -9.0f, 1.0f, 0.4f }, Vec3{ 0.0f, 3.0f, 0.3f }, Vec3{ 9.0f, 4.0f, 0.7f }, Vec3{ 0.0f, 3.0f, 30.0f } };
		scene.boneWorld.resize(kBoneCount);
		scene.spheres.resize(kBoneCount);
		for (std::size_t bone = 0; bone < kBoneCount; ++bone) {
			scene.boneWorld[bone].translation = positions[bone];
			scene.spheres[bone].push_back({ { 0.0f, 0.0f, 0.0f }, 1.0f, 0u });
		}
		return scene;
	}

	GpuRig::BoneContact Sentinel()
	{
		GpuRig::BoneContact value;
		value.depth = kSentinelDepth;
		return value;
	}

	struct PassOptions
	{
		std::array<bool, kTreeCount> takesContacts{ true, true };
		std::vector<GpuRig::DriverDelta> driverDeltas;
	};

	/** @brief Dispatches the contact passes over a scene, the two cloth passes among them when it has cloth triangles, and returns every bone's contact lanes, kContactLanes per bone; a skipped bone keeps the sentinel in each. */
	std::vector<GpuRig::BoneContact> RunLanes(const Scene& a_scene, const PassOptions& a_options)
	{
		const ContactKernels& kernels = LoadKernels();
		ID3D11Device* device = kernels.gpu.device.Get();
		ID3D11DeviceContext* context = kernels.gpu.context.Get();

		GpuRig::CollisionData collision = GpuRig::PackCollision(a_scene.spheres, a_scene.colliders, a_scene.driverCount);
		GpuRig::PackClothTriangles(collision, a_scene.cloth);
		const bool cloth = !collision.clothTriangles.empty();
		std::vector<GpuRig::SphereRecord> sphereList = collision.sphereList;
		std::size_t cursor = 0;
		for (std::size_t bone = 0; bone < kBoneCount; ++bone)
			for (std::size_t index = 0; index < a_scene.spheres[bone].size(); ++index)
				sphereList[cursor++].exempt.y = static_cast<std::uint32_t>(bone);

		std::vector<GpuRig::TreeStatic> trees(kTreeCount);
		for (GpuRig::TreeStatic& tree : trees)
			tree.contact = { 0u, static_cast<std::uint32_t>(collision.colliders.size()), 0u, a_scene.driverCount };

		std::vector<GpuRig::TreeInput> inputs(kTreeCount);
		for (GpuRig::TreeInput& input : inputs)
			input.contact.z = a_scene.contactSkin;
		for (std::size_t tree = 0; tree < kTreeCount; ++tree)
			inputs[tree].surface.z = a_options.takesContacts[tree] ? 1u : 0u;

		std::vector<GpuRig::BoneWorld> worlds(kBoneCount);
		for (std::size_t bone = 0; bone < kBoneCount; ++bone) {
			const Vec3& translation = a_scene.boneWorld[bone].translation;
			worlds[bone].rotation = { 1.0f, 0.0f, 0.0f, 0.0f };
			worlds[bone].translation = { translation.x, translation.y, translation.z, 0.0f };
		}
		const std::vector<GpuRig::DriverWorld> drivers(a_scene.driverCount, { { 1.0f, 0.0f, 0.0f, 0.0f }, { a_scene.driver.translation.x, a_scene.driver.translation.y, a_scene.driver.translation.z, 0.0f } });
		GpuRig::SurfaceFrame surfaceFrame;
		std::uint32_t firstMeta = 0;
		std::uint32_t metaCount = 0;
		if (!a_scene.surfaces.empty())
			GpuRig::AppendSurfaces(surfaceFrame, a_scene.surfaces, { a_scene.driver }, firstMeta, metaCount);
		for (GpuRig::TreeInput& input : inputs) {
			input.surface.x = firstMeta;
			input.surface.y = metaCount;
		}
		for (std::size_t surface = 0; surface < a_scene.faceKeys.size() && surface < surfaceFrame.metas.size(); ++surface) {
			const GpuRig::SurfaceMeta& meta = surfaceFrame.metas[surface];
			for (std::uint32_t slot = 0; slot < meta.dimensions.w; ++slot) {
				GpuRig::SurfaceTriangle& face = surfaceFrame.triangles[meta.base.x + slot];
				const std::uint32_t source = std::bit_cast<std::uint32_t>(face.a.w);
				if (source < a_scene.faceKeys[surface].size())
					face.b.w = std::bit_cast<float>(a_scene.faceKeys[surface][source]);
			}
		}
		const std::vector<GpuRig::BoneContact> contacts(kBoneCount * kContactLanes, Sentinel());
		const std::vector<std::uint32_t> boneTrees(kBoneTrees.begin(), kBoneTrees.end());
		const std::vector<float> hits(sphereList.size() * 12);
		const std::vector<std::uint32_t> cull(kBoneCount * 4);
		const std::vector<std::uint32_t> unused(4);

		std::array<ComPtr<ID3D11Buffer>, 7> targetBuffers;
		targetBuffers[1] = CreateTyped(device, worlds, D3D11_BIND_UNORDERED_ACCESS);
		targetBuffers[3] = CreateTyped(device, contacts, D3D11_BIND_UNORDERED_ACCESS);
		targetBuffers[5] = CreateStructured(device, sizeof(GpuRig::SphereHit), sphereList.size(), D3D11_BIND_UNORDERED_ACCESS);
		targetBuffers[6] = CreateStructured(device, 16, kBoneCount, D3D11_BIND_UNORDERED_ACCESS);
		const std::size_t clothHitCount = collision.clothTriangles.size() * kClothTriangleHits;
		if (cloth) {
			targetBuffers[0] = CreateStructured(device, sizeof(GpuRig::Float4), collision.clothVertices.size(), D3D11_BIND_UNORDERED_ACCESS);
			targetBuffers[2] = CreateStructured(device, sizeof(GpuRig::SphereHit), clothHitCount, D3D11_BIND_UNORDERED_ACCESS);
		}
		const std::array<std::size_t, 7> targetCounts{ collision.clothVertices.size(), kBoneCount, clothHitCount, kBoneCount * kContactLanes, 0, sphereList.size(), kBoneCount };
		std::array<ID3D11UnorderedAccessView*, 7> targets{};
		std::array<ComPtr<ID3D11UnorderedAccessView>, 7> targetViews;
		for (std::size_t slot = 0; slot < targets.size(); ++slot) {
			if (!targetBuffers[slot])
				continue;
			targetViews[slot] = CreateUav(device, targetBuffers[slot].Get(), targetCounts[slot]);
			targets[slot] = targetViews[slot].Get();
		}

		std::array<ComPtr<ID3D11Buffer>, 15> sourceBuffers;
		std::array<std::size_t, 15> sourceCounts{};
		const auto bind = [&](std::size_t slot, auto& values) {
			sourceBuffers[slot] = CreateTyped(device, values, D3D11_BIND_SHADER_RESOURCE);
			sourceCounts[slot] = values.size();
		};
		if (cloth) {
			bind(0, collision.clothVertices);
			bind(3, collision.clothTriangles);
			bind(4, collision.boneCloth);
			bind(11, collision.clothBoneList);
		}
		bind(1, trees);
		bind(2, inputs);
		bind(5, collision.spheres);
		bind(6, collision.colliders);
		bind(7, drivers);
		bind(8, surfaceFrame.metas);
		bind(9, surfaceFrame.triangles);
		bind(10, surfaceFrame.nodes);
		bind(12, sphereList);
		bind(13, boneTrees);
		if (!a_options.driverDeltas.empty())
			bind(14, a_options.driverDeltas);
		std::array<ComPtr<ID3D11ShaderResourceView>, 15> sourceViews;
		std::array<ID3D11ShaderResourceView*, 15> sources{};
		for (std::size_t slot = 0; slot < sources.size(); ++slot) {
			if (!sourceBuffers[slot])
				continue;
			sourceViews[slot] = CreateSrv(device, sourceBuffers[slot].Get(), sourceCounts[slot]);
			sources[slot] = sourceViews[slot].Get();
		}

		struct Constants
		{
			std::uint32_t boneCount, sphereCount, clothVertexCount, clothTriangleCount;
		};
		const Constants constants{ static_cast<std::uint32_t>(kBoneCount), static_cast<std::uint32_t>(sphereList.size()),
			static_cast<std::uint32_t>(collision.clothVertices.size()), static_cast<std::uint32_t>(collision.clothTriangles.size()) };
		D3D11_BUFFER_DESC constantsDesc{};
		constantsDesc.ByteWidth = sizeof(Constants);
		constantsDesc.Usage = D3D11_USAGE_IMMUTABLE;
		constantsDesc.BindFlags = D3D11_BIND_CONSTANT_BUFFER;
		D3D11_SUBRESOURCE_DATA constantsData{};
		constantsData.pSysMem = &constants;
		ComPtr<ID3D11Buffer> constantBuffer;
		REQUIRE(device->CreateBuffer(&constantsDesc, &constantsData, constantBuffer.GetAddressOf()) == S_OK);

		context->CSSetConstantBuffers(0, 1, constantBuffer.GetAddressOf());
		context->CSSetShaderResources(0, static_cast<UINT>(sources.size()), sources.data());
		context->CSSetUnorderedAccessViews(0, static_cast<UINT>(targets.size()), targets.data(), nullptr);
		context->CSSetShader(kernels.cull.Get(), nullptr, 0);
		context->Dispatch(GroupCount(kBoneCount), 1, 1);
		if (cloth) {
			context->CSSetShader(kernels.clothVertices.Get(), nullptr, 0);
			context->Dispatch(GroupCount(collision.clothVertices.size()), 1, 1);
			context->CSSetShader(kernels.clothTriangles.Get(), nullptr, 0);
			context->Dispatch(GroupCount(collision.clothTriangles.size()), 1, 1);
		}
		context->CSSetShader(kernels.spheres.Get(), nullptr, 0);
		context->Dispatch(GroupCount(sphereList.size()), 1, 1);
		context->CSSetShader(kernels.reduce.Get(), nullptr, 0);
		context->Dispatch(GroupCount(kBoneCount), 1, 1);

		std::array<ID3D11UnorderedAccessView*, 7> noTargets{};
		std::array<ID3D11ShaderResourceView*, 15> noSources{};
		context->CSSetUnorderedAccessViews(0, static_cast<UINT>(noTargets.size()), noTargets.data(), nullptr);
		context->CSSetShaderResources(0, static_cast<UINT>(noSources.size()), noSources.data());
		context->CSSetShader(nullptr, nullptr, 0);
		return ReadBack<GpuRig::BoneContact>(device, context, targetBuffers[3].Get(), kBoneCount * kContactLanes);
	}

	bool IsSkipped(const GpuRig::BoneContact& a_contact)
	{
		return a_contact.depth == kSentinelDepth;
	}

	bool IsEmptyLane(const GpuRig::BoneContact& a_contact)
	{
		const GpuRig::BoneContact empty;
		return std::memcmp(&a_contact, &empty, sizeof(empty)) == 0;
	}

	/** @brief Runs a scene whose bones each touch one driver at most and returns lane 0 of every bone; the other lanes must be empty, or skipped with it. */
	std::vector<GpuRig::BoneContact> RunPasses(const Scene& a_scene, const PassOptions& a_options)
	{
		const std::vector<GpuRig::BoneContact> lanes = RunLanes(a_scene, a_options);
		std::vector<GpuRig::BoneContact> first(kBoneCount);
		for (std::size_t bone = 0; bone < kBoneCount; ++bone) {
			first[bone] = lanes[bone * kContactLanes];
			for (std::size_t lane = 1; lane < kContactLanes; ++lane) {
				INFO("bone " << bone << " lane " << lane);
				const GpuRig::BoneContact& other = lanes[bone * kContactLanes + lane];
				REQUIRE((IsSkipped(first[bone]) ? IsSkipped(other) : IsEmptyLane(other)));
			}
		}
		return first;
	}

	void RequireMatches(const GpuRig::BoneContact& a_actual, const GpuRig::BoneContact& a_expected, std::size_t a_bone)
	{
		INFO("bone " << a_bone << " depth " << a_actual.depth << " expected " << a_expected.depth);
		REQUIRE(a_actual.depth == Approx(a_expected.depth).margin(kTolerance));
		REQUIRE(a_actual.deepest == Approx(a_expected.deepest).margin(kTolerance));
		REQUIRE(a_actual.collider == a_expected.collider);
		for (int axis = 0; axis < 3; ++axis) {
			REQUIRE(a_actual.normal[axis] == Approx(a_expected.normal[axis]).margin(kTolerance));
			REQUIRE(a_actual.position[axis] == Approx(a_expected.position[axis]).margin(kTolerance));
		}
	}

}

TEST_CASE("The scene exercises a contact, an exemption and a bone out of reach", "[smpphysics][contact_gpu]")
{
	const Scene scene = MakeScene();
	CHECK(ExpectedContact(scene, 0).depth > 0.0f);
	CHECK(ExpectedContact(scene, 0).collider == 0);
	CHECK(ExpectedContact(scene, 1).depth > 0.0f);
	CHECK(ExpectedContact(scene, 1).collider == 1);
	CHECK(ExpectedContact(scene, 2).depth > 0.0f);
	CHECK(ExpectedContact(scene, 3).depth == 0.0f);
}

TEST_CASE("The pass merges the contacts of a tree that takes them and leaves the other tree's", "[smpphysics][contact_gpu]")
{
	const Scene scene = MakeScene();
	PassOptions options;
	options.takesContacts = { false, true };
	const std::vector<GpuRig::BoneContact> contacts = RunPasses(scene, options);
	CHECK(IsSkipped(contacts[0]));
	CHECK(IsSkipped(contacts[1]));
	RequireMatches(contacts[2], ExpectedContact(scene, 2), 2);
	RequireMatches(contacts[3], ExpectedContact(scene, 3), 3);
}

TEST_CASE("The pass merges every bone's contact as the CPU does", "[smpphysics][contact_gpu]")
{
	const Scene scene = MakeScene();
	const std::vector<GpuRig::BoneContact> contacts = RunPasses(scene, {});
	for (std::size_t bone = 0; bone < kBoneCount; ++bone)
		RequireMatches(contacts[bone], ExpectedContact(scene, bone), bone);
}

TEST_CASE("A bone out of reach merges to no contact rather than keeping the last one", "[smpphysics][contact_gpu]")
{
	const std::vector<GpuRig::BoneContact> contacts = RunPasses(MakeScene(), {});
	CHECK_FALSE(IsSkipped(contacts[3]));
	CHECK(contacts[3].depth == 0.0f);
	CHECK(contacts[3].collider == -1);
}

/** @brief Runs the passes over a surface scene and checks every bone's depth against the CPU probe. */
void CheckSurfaceDepthsMatchProbe(const Scene& a_scene, std::vector<SphereProbe>& a_probes)
{
	ProbeSpheres(a_scene.spheres, a_scene.colliders, a_scene.surfaces, a_scene.boneWorld, { a_scene.driver }, a_probes);
	REQUIRE(a_probes.size() == kBoneCount);

	const std::vector<GpuRig::BoneContact> contacts = RunPasses(a_scene, {});
	for (std::size_t bone = 0; bone < kBoneCount; ++bone) {
		INFO("bone " << bone << " depth " << contacts[bone].depth << " probe " << a_probes[bone].depth);
		CHECK(contacts[bone].depth == Approx(a_probes[bone].depth).margin(1.0e-3f));
	}
}

TEST_CASE("The passes find a bone's penetration of a skinned surface as the CPU probe does", "[smpphysics][contact_gpu]")
{
	std::vector<SphereProbe> probes;
	CheckSurfaceDepthsMatchProbe(MakeSurfaceScene(), probes);
	CHECK(probes[0].depth > 0.0f);
	CHECK(probes[3].depth == 0.0f);
}

TEST_CASE("A sphere sits at the blend of its vertex's bones and its depth is scaled by flexible", "[smpphysics][contact_gpu]")
{
	const Scene rigid = MakeSurfaceScene();
	const std::vector<GpuRig::BoneContact> before = RunPasses(rigid, {});
	REQUIRE(before[0].depth > 0.0f);
	REQUIRE(before[2].depth > 0.0f);

	Scene blended = rigid;
	blended.spheres[0][0].weight = 0.75f;
	blended.spheres[0][0].others[0] = { 1, { 0.0f, 0.0f, 0.0f }, 0.25f };
	blended.spheres[2][0].flexible = 0.5f;
	const std::vector<GpuRig::BoneContact> after = RunPasses(blended, {});

	const Vec3 expected = rigid.boneWorld[0].translation * 0.75f + rigid.boneWorld[1].translation * 0.25f;
	CHECK(after[0].position[0] == Approx(expected.x).margin(1.0e-4f));
	CHECK(after[0].position[2] == Approx(expected.z).margin(1.0e-4f));
	CHECK(after[0].depth == Approx(before[0].depth - (expected.z - rigid.boneWorld[0].translation.z)).margin(1.0e-3f));
	CHECK(after[2].depth == Approx(before[2].depth * 0.5f).margin(1.0e-4f));
}

TEST_CASE("The passes find the deep push reach of a surface as the CPU probe does", "[smpphysics][contact_gpu]")
{
	std::vector<SphereProbe> probes;
	CheckSurfaceDepthsMatchProbe(MakeDeepReachScene(), probes);
	CHECK(probes[0].depth > 0.0f);
	CHECK(probes[1].depth > 0.0f);
}

TEST_CASE("A bone between two surface clusters, inside the BVH root box, finds no contact", "[smpphysics][contact_gpu]")
{
	std::vector<SphereProbe> probes;
	CheckSurfaceDepthsMatchProbe(MakeClusteredSurfaceScene(), probes);
	CHECK(probes[0].depth > 0.0f);
	CHECK(probes[1].depth == 0.0f);
	CHECK(probes[2].depth > 0.0f);
	CHECK(probes[3].depth == 0.0f);
}

TEST_CASE("A contact carries the velocity its collider's driver gives the contact point", "[smpphysics][contact_gpu]")
{
	const Scene scene = MakeScene();
	const float half = std::sqrt(0.5f);
	PassOptions options;
	GpuRig::DriverDelta delta;
	delta.rotation = { half, 0.0f, 0.0f, half };
	delta.translation = { 2.0f, -1.0f, 0.5f, 60.0f };
	options.driverDeltas = { delta };
	const std::vector<GpuRig::BoneContact> contacts = RunPasses(scene, options);
	for (std::size_t bone = 0; bone < kBoneCount; ++bone) {
		INFO("bone " << bone);
		if (!(contacts[bone].depth > 0.0f)) {
			CHECK(contacts[bone].velocityValid == 0u);
			continue;
		}
		REQUIRE(contacts[bone].velocityValid == 1u);
		const Vec3 expected = GpuRig::DriverPointVelocity(delta, { contacts[bone].position[0], contacts[bone].position[1], contacts[bone].position[2] });
		CHECK(contacts[bone].velocity[0] == Approx(expected.x).margin(5.0e-3f));
		CHECK(contacts[bone].velocity[1] == Approx(expected.y).margin(5.0e-3f));
		CHECK(contacts[bone].velocity[2] == Approx(expected.z).margin(5.0e-3f));
	}
}

TEST_CASE("A contact with no driver delta bound reports no velocity", "[smpphysics][contact_gpu]")
{
	const std::vector<GpuRig::BoneContact> contacts = RunPasses(MakeScene(), {});
	for (const GpuRig::BoneContact& contact : contacts)
		CHECK(contact.velocityValid == 0u);
}

/** @brief Two unit balls on two drivers, with bone 0 resting one sphere on top of the first and one against the side of the second. */
Scene MakeTwoDriverScene()
{
	Scene scene;
	scene.driver = Transform{};
	scene.driverCount = 2;
	Collider under;
	under.type = Collider::Type::Ellipsoid;
	under.driverIndex = 0;
	under.halfExtents = { 1.0f, 1.0f, 1.0f };
	Collider beside = under;
	beside.driverIndex = 1;
	beside.center = { 1.5f, 0.0f, 1.5f };
	scene.colliders = { under, beside };

	scene.boneWorld.resize(kBoneCount);
	scene.spheres.resize(kBoneCount);
	for (std::size_t bone = 0; bone < kBoneCount; ++bone) {
		scene.boneWorld[bone].translation = { 0.0f, 0.0f, 1.0f + 40.0f * static_cast<float>(bone) };
		scene.spheres[bone].push_back({ { 0.0f, 0.0f, 0.0f }, 0.1f, 0u });
	}
	scene.spheres[0].push_back({ { 0.55f, 0.0f, 0.5f }, 0.1f, 0u });
	return scene;
}

TEST_CASE("A bone touching two drivers keeps one contact for each", "[smpphysics][contact_gpu]")
{
	const Scene scene = MakeTwoDriverScene();
	const GpuRig::BoneContact merged = ExpectedContact(scene, 0);
	// One merged contact would point between the two surfaces and hold the bone off neither.
	REQUIRE(merged.normal[0] < -0.5f);
	REQUIRE(merged.normal[2] > 0.2f);

	const std::vector<GpuRig::BoneContact> lanes = RunLanes(scene, {});
	const std::array<GpuRig::BoneContact, kContactLanes> expected = ExpectedLanes(scene, 0);
	for (std::size_t lane = 0; lane < kContactLanes; ++lane)
		RequireMatches(lanes[lane], expected[lane], lane);

	CHECK(lanes[0].collider == 0);
	CHECK(lanes[0].depth == Approx(0.12f).margin(1.0e-3f));
	CHECK(lanes[0].normal[2] == Approx(1.0f).margin(1.0e-3f));
	CHECK(lanes[0].position[2] == Approx(1.0f).margin(1.0e-4f));
	CHECK(lanes[1].collider == 1);
	CHECK(lanes[1].depth == Approx(0.17f).margin(1.0e-3f));
	CHECK(lanes[1].normal[0] == Approx(-1.0f).margin(1.0e-3f));
	CHECK(lanes[1].position[0] == Approx(0.55f).margin(1.0e-4f));
	CHECK(lanes[1].position[2] == Approx(1.5f).margin(1.0e-4f));
	for (std::size_t lane = 2; lane < kContactLanes; ++lane)
		CHECK(IsEmptyLane(lanes[lane]));
	for (std::size_t lane = 0; lane < kContactLanes; ++lane)
		CHECK(IsEmptyLane(lanes[kContactLanes + lane]));
}

TEST_CASE("A bone touching one driver keeps the single merged contact in lane 0", "[smpphysics][contact_gpu]")
{
	const Scene scene = MakeScene();
	const std::vector<GpuRig::BoneContact> lanes = RunLanes(scene, {});
	for (std::size_t bone = 0; bone < kBoneCount; ++bone) {
		RequireMatches(lanes[bone * kContactLanes], ExpectedContact(scene, bone), bone);
		for (std::size_t lane = 1; lane < kContactLanes; ++lane) {
			INFO("bone " << bone << " lane " << lane);
			CHECK(IsEmptyLane(lanes[bone * kContactLanes + lane]));
		}
	}
}

namespace
{
	constexpr std::size_t kCrowdedDrivers{ kContactLanes + 1 };
	constexpr float kCrowdedSpacing{ 3.0f };
	/** @brief The tilt of each sphere's normal about Y; the last is nearest the third, whose lane it must join. */
	constexpr std::array<float, kCrowdedDrivers> kCrowdedTilt{ -0.6f, -0.2f, 0.2f, 0.6f, 0.25f };
	constexpr std::size_t kCrowdedSharedLane{ 2 };

	/** @brief A row of unit balls, one per driver and one more than there are lanes, with bone 0 resting one sphere on each. */
	Scene MakeCrowdedScene()
	{
		static_assert(kContactLanes == 4, "kCrowdedTilt lists one sphere per lane and one more");
		Scene scene;
		scene.driver = Transform{};
		scene.driverCount = static_cast<std::uint32_t>(kCrowdedDrivers);
		scene.boneWorld.resize(kBoneCount);
		scene.spheres.resize(kBoneCount);
		for (std::size_t bone = 1; bone < kBoneCount; ++bone) {
			scene.boneWorld[bone].translation = { 0.0f, 0.0f, 40.0f * static_cast<float>(bone) };
			scene.spheres[bone].push_back({ { 0.0f, 0.0f, 0.0f }, 0.1f, 0u });
		}
		for (std::size_t index = 0; index < kCrowdedDrivers; ++index) {
			Collider ball;
			ball.type = Collider::Type::Ellipsoid;
			ball.driverIndex = static_cast<int>(index);
			ball.halfExtents = { 1.0f, 1.0f, 1.0f };
			ball.center = { kCrowdedSpacing * static_cast<float>(index), 0.0f, 0.0f };
			scene.colliders.push_back(ball);
			const float distance = index + 1 == kCrowdedDrivers ? 0.97f : 0.95f;
			scene.spheres[0].push_back({ { ball.center.x + distance * std::sin(kCrowdedTilt[index]), 0.0f, distance * std::cos(kCrowdedTilt[index]) }, 0.1f, 0u });
		}
		return scene;
	}
}

TEST_CASE("More contacted drivers than lanes fold into the best aligned lane", "[smpphysics][contact_gpu]")
{
	const Scene scene = MakeCrowdedScene();
	const std::vector<GpuRig::BoneContact> lanes = RunLanes(scene, {});
	const std::array<GpuRig::BoneContact, kContactLanes> expected = ExpectedLanes(scene, 0);
	for (std::size_t lane = 0; lane < kContactLanes; ++lane) {
		RequireMatches(lanes[lane], expected[lane], lane);
		CHECK(lanes[lane].depth > 0.0f);
		CHECK(lanes[lane].collider == static_cast<int>(lane));
	}

	// The fifth sphere is not dropped: it pulls the shared lane's position off the third sphere, toward its own.
	const Vec3& own = scene.spheres[0][kCrowdedSharedLane].centerLocal;
	const Vec3& extra = scene.spheres[0][kContactLanes].centerLocal;
	const float ownWeight = 0.17f * 0.17f;
	const float extraWeight = 0.15f * 0.15f;
	CHECK(lanes[kCrowdedSharedLane].position[0] == Approx((own.x * ownWeight + extra.x * extraWeight) / (ownWeight + extraWeight)).margin(0.1f));
	CHECK(lanes[kCrowdedSharedLane].position[0] > own.x + 1.0f);
	CHECK(lanes[kCrowdedSharedLane].deepest == Approx(0.17f).margin(1.0e-3f));
	for (std::size_t lane = 0; lane < kContactLanes; ++lane) {
		if (lane != kCrowdedSharedLane)
			CHECK(lanes[lane].position[0] == Approx(scene.spheres[0][lane].centerLocal.x).margin(1.0e-4f));
	}
}

namespace
{
	// tara2.xml's dress and body shapes: a body vertex is held from 0.32 behind the cloth's plane to 0.87 in front.
	constexpr float kClothMargin{ 0.1f };
	constexpr float kClothPenetration{ -0.1f };
	constexpr float kClothMultiplier{ 3.2f };
	constexpr float kBodyRadius{ 0.55f };
	constexpr float kClothReach{ kBodyRadius + kClothMargin * kClothMultiplier };
	constexpr std::uint32_t kLeftKey{ 1 };
	constexpr std::uint32_t kRightKey{ 2 };

	SkinnedVertex RigidVertex(const Vec3& a_position)
	{
		SkinnedVertex vertex;
		vertex.position = a_position;
		vertex.bone[0] = 0;
		vertex.weight[0] = 1.0f;
		return vertex;
	}

	/**
	 * @brief A 4 by 2 sheet in the z = 0 plane, its x = 0 edge skinned to bone 0, its x = 4 edge to bone 1 and the middle to both.
	 * @details Triangles 0 and 1 span x 0 to 2 below and above the diagonal, triangles 2 and 3 span x 2 to 4. Their normals
	 *          are +Z and the negative penetration turns them, so the body belongs below the sheet.
	 */
	ClothTriangleSet MakeSheet(const std::vector<bool>& a_dynamicBone)
	{
		std::vector<SkinnedVertex> vertices;
		for (const float y : { 0.0f, 2.0f }) {
			for (const float x : { 0.0f, 2.0f, 4.0f }) {
				SkinnedVertex vertex = RigidVertex({ x, y, 0.0f });
				vertex.weight[0] = 1.0f - x / 4.0f;
				vertex.bone[1] = 1;
				vertex.weight[1] = x / 4.0f;
				vertices.push_back(vertex);
			}
		}
		const std::vector<std::uint16_t> triangles{ 0, 1, 4, 0, 4, 3, 1, 2, 5, 1, 5, 4 };
		const std::vector<int> rigBoneOfSkinBone{ 0, 1 };
		const std::vector<Transform> skinToBone(2);
		const std::vector<float> marginMultiplier(kBoneCount, kClothMultiplier);
		const std::vector<float> weightThreshold(kBoneCount, 0.0f);
		ClothTriangleSet cloth;
		AppendClothTriangles(cloth, { "sheet", &vertices, &triangles, &rigBoneOfSkinBone, &skinToBone, &a_dynamicBone, &marginMultiplier, &weightThreshold, kClothMargin, kClothPenetration });
		return cloth;
	}

	/**
	 * @brief A body proxy built from a per-vertex shape: three slivers whose vertices sit under, above and far from the sheet.
	 * @details Face 0, on the left driver, holds a vertex 0.5 under triangle 1 and one 0.3 under triangle 0. Face 1, on the
	 *          right driver, holds one 0.2 under triangle 1 and one 0.4 under triangle 2. Face 2, on the left driver, holds
	 *          one 0.2 above triangle 3, inside its window, and one 0.5 above it, past the window.
	 */
	SkinnedSurface MakeBodyProxy()
	{
		SkinnedSurface body;
		body.name = "body";
		body.margin = kBodyRadius;
		body.perVertex = true;
		for (const Vec3& position : { Vec3{ 0.5f, 1.2f, -0.5f }, Vec3{ 1.2f, 0.4f, -0.3f }, Vec3{ 0.8f, 0.9f, -3.0f },
				 Vec3{ 0.6f, 1.5f, -0.2f }, Vec3{ 3.0f, 0.5f, -0.4f }, Vec3{ 3.0f, 1.0f, -6.0f },
				 Vec3{ 2.5f, 1.5f, 0.2f }, Vec3{ 2.6f, 1.6f, 0.5f }, Vec3{ 2.4f, 1.8f, 3.0f } })
			body.vertices.push_back(RigidVertex(position));
		body.indices = { 0, 1, 2, 3, 4, 5, 6, 7, 8 };
		body.skinToBone = { Transform{} };
		body.boneDriver = { 0 };
		return body;
	}

	/** @brief The sheet on bones 0 and 1 over the body proxy; bones 2 and 3 are far away and own no triangle. */
	Scene MakeClothScene()
	{
		Scene scene;
		scene.contactSkin = 0.0f;
		scene.driverCount = 2;
		scene.boneWorld.resize(kBoneCount);
		scene.boneWorld[2].translation = { 0.0f, 0.0f, 40.0f };
		scene.boneWorld[3].translation = { 0.0f, 0.0f, 80.0f };
		scene.spheres.resize(kBoneCount);
		scene.spheres[3].push_back({ { 0.0f, 0.0f, 0.0f }, 0.1f, 0u });
		scene.cloth = MakeSheet(std::vector<bool>(kBoneCount, true));
		scene.surfaces = { MakeBodyProxy() };
		scene.faceKeys = { { kLeftKey, kRightKey, kLeftKey } };
		return scene;
	}

	struct ClothHit
	{
		TriangleContact contact;
		std::uint32_t key{};
		int collider{ -1 };
	};

	/** @brief Per cloth triangle, its deepest hit and its deepest on another driver, found by testing every vertex of every per-vertex surface. */
	std::vector<std::array<ClothHit, kClothTriangleHits>> ExpectedClothHits(const Scene& a_scene)
	{
		std::vector<Vec3> posed;
		for (const ClothVertex& vertex : a_scene.cloth.vertices)
			posed.push_back(PoseClothVertex(vertex, a_scene.boneWorld));
		std::vector<std::array<ClothHit, kClothTriangleHits>> expected(a_scene.cloth.triangles.size());
		for (std::size_t index = 0; index < expected.size(); ++index) {
			const ClothTriangle& triangle = a_scene.cloth.triangles[index];
			std::vector<ClothHit> hits;
			for (std::size_t surface = 0; surface < a_scene.surfaces.size(); ++surface) {
				const SkinnedSurface& body = a_scene.surfaces[surface];
				if (!body.perVertex || (surface < kMaskedSurfaceLimit && (triangle.surfaceMask & (1u << surface)) == 0u))
					continue;
				const std::vector<Vec3> bodyPosed = PoseSurface(body, std::vector<Transform>(a_scene.driverCount, a_scene.driver));
				for (std::size_t corner = 0; corner < body.indices.size(); ++corner) {
					const TriangleContact contact = TestVertexAgainstTriangle(bodyPosed[body.indices[corner]], body.margin, posed[triangle.corners[0]],
						posed[triangle.corners[1]], posed[triangle.corners[2]], triangle.margin, triangle.penetration);
					const std::size_t face = corner / 3;
					if (contact.depth > 0.0f)
						hits.push_back({ contact, surface < a_scene.faceKeys.size() && face < a_scene.faceKeys[surface].size() ? a_scene.faceKeys[surface][face] : 0u,
							static_cast<int>(a_scene.colliders.size() + surface) });
				}
			}
			ClothHit& best = expected[index][0];
			for (const ClothHit& hit : hits) {
				if (hit.contact.depth > best.contact.depth)
					best = hit;
			}
			for (const ClothHit& hit : hits) {
				if (hit.key != best.key && hit.contact.depth > expected[index][1].contact.depth)
					expected[index][1] = hit;
			}
		}
		return expected;
	}

	/** @brief The lanes the CPU makes for one bone from its cloth triangles' hits, added to a_lanes, which holds what its spheres gave. */
	std::array<GpuRig::BoneContact, kContactLanes> ExpectedClothLanes(const Scene& a_scene, std::size_t a_bone, ContactLanes a_lanes = {})
	{
		const std::vector<std::array<ClothHit, kClothTriangleHits>> hits = ExpectedClothHits(a_scene);
		for (const ClothBoneTriangle& owned : a_scene.cloth.boneTriangles[a_bone]) {
			for (const ClothHit& hit : hits[owned.triangle]) {
				if (hit.contact.depth > 0.0f)
					a_lanes.Add({ hit.contact.depth * owned.flexible, hit.contact.normal, hit.collider }, hit.contact.point, hit.key, static_cast<float>(owned.corners));
			}
		}
		std::array<GpuRig::BoneContact, kContactLanes> expected;
		for (std::size_t lane = 0; lane < kContactLanes; ++lane)
			expected[lane] = ToBoneContact(a_lanes.lanes[lane]);
		return expected;
	}

	void RequireClothLanes(const Scene& a_scene, const std::vector<GpuRig::BoneContact>& a_lanes, std::size_t a_bone, const ContactLanes& a_sphereLanes = {})
	{
		const std::array<GpuRig::BoneContact, kContactLanes> expected = ExpectedClothLanes(a_scene, a_bone, a_sphereLanes);
		for (std::size_t lane = 0; lane < kContactLanes; ++lane) {
			INFO("lane " << lane);
			RequireMatches(a_lanes[a_bone * kContactLanes + lane], expected[lane], a_bone);
		}
	}

	/** @brief hdtSMP64's merged depth of contacts on one lane: each a depth and how many times it is added. */
	float MergedDepth(std::initializer_list<std::pair<float, float>> a_contacts)
	{
		float cubed = 0.0f;
		float squared = 0.0f;
		for (const auto& [depth, count] : a_contacts) {
			cubed += count * depth * depth * depth;
			squared += count * depth * depth;
		}
		return cubed / squared;
	}
}

TEST_CASE("A cloth's triangles take contacts from a body proxy's vertices inside the triangle's window", "[smpphysics][contact_gpu][clothtriangles]")
{
	const Scene scene = MakeClothScene();
	const std::vector<std::array<ClothHit, kClothTriangleHits>> hits = ExpectedClothHits(scene);
	REQUIRE(hits.size() == 4);
	CHECK(hits[0][0].contact.depth == Approx(kClothReach - 0.3f));
	CHECK(hits[0][1].contact.depth == 0.0f);
	CHECK(hits[1][0].contact.depth == Approx(kClothReach - 0.2f));
	CHECK(hits[1][0].key == kRightKey);
	CHECK(hits[1][1].contact.depth == Approx(kClothReach - 0.5f));
	CHECK(hits[1][1].key == kLeftKey);
	CHECK(hits[2][0].contact.depth == Approx(kClothReach - 0.4f));
	// The vertex above the sheet is inside the window and is the deepest; the one past the window is let go.
	CHECK(hits[3][0].contact.depth == Approx(kClothReach + 0.2f));
	CHECK(hits[3][1].contact.depth == 0.0f);

	const std::vector<GpuRig::BoneContact> lanes = RunLanes(scene, {});
	RequireClothLanes(scene, lanes, 0);
	RequireClothLanes(scene, lanes, 1);

	// Bone 0 holds three corner weights of triangles 0 and 1, one of triangle 2 and two of triangle 3; each driver has its lane.
	const GpuRig::BoneContact& left = lanes[0];
	CHECK(left.depth == Approx(MergedDepth({ { 0.57f, 3.0f }, { 0.37f, 3.0f }, { 1.07f, 2.0f } })).margin(kTolerance));
	CHECK(left.deepest == Approx(1.07f).margin(kTolerance));
	CHECK(left.collider == 0);
	CHECK(left.normal[2] == Approx(1.0f).margin(kTolerance));
	const GpuRig::BoneContact& right = lanes[1];
	CHECK(right.depth == Approx(MergedDepth({ { 0.67f, 3.0f }, { 0.47f, 1.0f } })).margin(kTolerance));
	CHECK(right.normal[2] == Approx(1.0f).margin(kTolerance));
	// A lane's position is on the cloth, where the vertices project.
	CHECK(right.position[2] == Approx(0.0f).margin(kTolerance));
	CHECK(right.position[0] == Approx((3.0f * 0.67f * 0.67f * 0.6f + 0.47f * 0.47f * 3.0f) / (3.0f * 0.67f * 0.67f + 0.47f * 0.47f)).margin(kTolerance));
	for (std::size_t lane = 2; lane < kContactLanes; ++lane)
		CHECK(IsEmptyLane(lanes[lane]));
	for (std::size_t bone = 2; bone < kBoneCount; ++bone) {
		for (std::size_t lane = 0; lane < kContactLanes; ++lane)
			CHECK(IsEmptyLane(lanes[bone * kContactLanes + lane]));
	}
}

TEST_CASE("Cloth triangles are tested where their bones pose them, and their depth is scaled by flexible", "[smpphysics][contact_gpu][clothtriangles]")
{
	Scene tilted = MakeClothScene();
	tilted.boneWorld[1].translation = { 0.0f, 0.0f, 0.3f };
	const std::vector<GpuRig::BoneContact> moved = RunLanes(tilted, {});
	RequireClothLanes(tilted, moved, 0);
	RequireClothLanes(tilted, moved, 1);
	const std::vector<GpuRig::BoneContact> flat = RunLanes(MakeClothScene(), {});
	CHECK(moved[kContactLanes].depth > 0.0f);
	CHECK(std::fabs(moved[kContactLanes].depth - flat[kContactLanes].depth) > 0.01f);

	// With bone 1 not simulated, bone 0 owns every triangle alone and each depth is scaled by the triangle's simulated weight.
	Scene half = MakeClothScene();
	half.cloth = MakeSheet({ true, false, true, true });
	REQUIRE(half.cloth.boneTriangles[1].empty());
	const std::vector<GpuRig::BoneContact> scaled = RunLanes(half, {});
	RequireClothLanes(half, scaled, 0);
	CHECK(scaled[0].deepest == Approx(0.57f * 2.0f / 3.0f).margin(kTolerance));
	for (std::size_t lane = 0; lane < kContactLanes; ++lane)
		CHECK(IsEmptyLane(scaled[kContactLanes + lane]));
}

TEST_CASE("A cloth triangle leaves out a masked surface and a tree that takes no contacts, and is two-sided without penetration", "[smpphysics][contact_gpu][clothtriangles]")
{
	Scene masked = MakeClothScene();
	for (ClothTriangle& triangle : masked.cloth.triangles)
		triangle.surfaceMask = ~1u;
	const std::vector<GpuRig::BoneContact> none = RunLanes(masked, {});
	for (std::size_t lane = 0; lane < 2 * kContactLanes; ++lane)
		CHECK(IsEmptyLane(none[lane]));

	// A surface that was not built from a per-vertex shape is not tested against the triangles at all.
	Scene faces = MakeClothScene();
	faces.surfaces[0].perVertex = false;
	const std::vector<GpuRig::BoneContact> untested = RunLanes(faces, {});
	for (std::size_t lane = 0; lane < 2 * kContactLanes; ++lane)
		CHECK(IsEmptyLane(untested[lane]));

	// With no penetration a triangle is two-sided, so the vertex past the one-sided window above triangle 3 is now held from above.
	Scene twoSided = MakeClothScene();
	for (ClothTriangle& triangle : twoSided.cloth.triangles)
		triangle.penetration = 0.0f;
	const std::vector<std::array<ClothHit, kClothTriangleHits>> hits = ExpectedClothHits(twoSided);
	CHECK(hits[3][0].contact.depth == Approx(kClothReach - 0.2f));
	CHECK(hits[3][0].contact.normal.z == Approx(-1.0f));
	const std::vector<GpuRig::BoneContact> both = RunLanes(twoSided, {});
	RequireClothLanes(twoSided, both, 0);
	RequireClothLanes(twoSided, both, 1);

	PassOptions options;
	options.takesContacts = { false, true };
	const std::vector<GpuRig::BoneContact> skipped = RunLanes(MakeClothScene(), options);
	for (std::size_t lane = 0; lane < 2 * kContactLanes; ++lane)
		CHECK(IsSkipped(skipped[lane]));
}

TEST_CASE("A cloth triangle's contact carries the velocity of the body vertex's driver", "[smpphysics][contact_gpu][clothtriangles]")
{
	Scene scene = MakeClothScene();
	// Only the vertex under triangle 2 is left, so bone 1's lane is that one hit.
	scene.surfaces[0].indices = { 3, 4, 5 };
	scene.faceKeys = { { kRightKey } };
	scene.surfaces[0].vertices[3].position = { 0.6f, 1.5f, -9.0f };
	PassOptions options;
	GpuRig::DriverDelta still;
	still.translation.w = 60.0f;
	GpuRig::DriverDelta moving;
	moving.translation = { 0.5f, -1.0f, 2.0f, 60.0f };
	options.driverDeltas = { still, moving };
	const std::vector<GpuRig::BoneContact> lanes = RunLanes(scene, options);
	const GpuRig::BoneContact& lane = lanes[kContactLanes];
	REQUIRE(lane.depth == Approx(kClothReach - 0.4f).margin(kTolerance));
	REQUIRE(lane.velocityValid == 1u);
	CHECK(lane.velocity[0] == Approx(30.0f).margin(5.0e-3f));
	CHECK(lane.velocity[1] == Approx(-60.0f).margin(5.0e-3f));
	CHECK(lane.velocity[2] == Approx(120.0f).margin(5.0e-3f));
}

namespace
{
	/**
	 * @brief The cloth scene with a sphere on bones 0 and 1 over two floors beside the sheet: the body proxy's own, at z = -1,
	 *        and a second surface built from triangles, at z = -1.3. Bone 0's sphere reaches only the first, bone 1's both.
	 */
	Scene MakeClothAndSphereScene()
	{
		Scene scene = MakeClothScene();
		SkinnedSurface& body = scene.surfaces[0];
		const auto first = static_cast<std::uint32_t>(body.vertices.size());
		for (const Vec3& corner : { Vec3{ 5.0f, -3.0f, -1.0f }, Vec3{ 12.0f, -3.0f, -1.0f }, Vec3{ 12.0f, 5.0f, -1.0f }, Vec3{ 5.0f, 5.0f, -1.0f } })
			body.vertices.push_back(RigidVertex(corner));
		for (const std::uint32_t offset : { 0u, 1u, 2u, 0u, 2u, 3u })
			body.indices.push_back(first + offset);
		SkinnedSurface lower = MakeFloorSurface(-1.3f);
		for (SkinnedVertex& vertex : lower.vertices)
			vertex.position.x += 8.0f;
		scene.surfaces.push_back(lower);
		scene.spheres[0].push_back({ { 7.0f, 1.0f, -0.8f }, 0.4f, 0u });
		scene.spheres[1].push_back({ { 9.0f, 1.0f, -1.1f }, 0.4f, 0u });
		return scene;
	}
}

TEST_CASE("Without cloth triangles a per-vertex surface is collided as its faces, as before", "[smpphysics][contact_gpu][clothtriangles]")
{
	Scene scene = MakeClothAndSphereScene();
	scene.cloth = {};
	std::vector<SphereProbe> probes;
	ProbeSpheres(scene.spheres, scene.colliders, scene.surfaces, scene.boneWorld, { scene.driver, scene.driver }, probes);
	REQUIRE(probes.size() == 3);
	CHECK(probes[0].depth == Approx(0.4f + kBodyRadius - 0.2f));
	CHECK(probes[1].depth == Approx(0.4f + kBodyRadius - 0.1f));

	const std::vector<GpuRig::BoneContact> lanes = RunLanes(scene, {});
	CHECK(lanes[0].depth == Approx(probes[0].depth).margin(1.0e-3f));
	CHECK(lanes[0].collider == 0);
	CHECK(lanes[kContactLanes].depth == Approx(probes[1].depth).margin(1.0e-3f));
	CHECK(lanes[kContactLanes].collider == 0);

	// The mark alone changes nothing: the same surface built from triangles gives the same contacts.
	Scene unmarked = scene;
	unmarked.surfaces[0].perVertex = false;
	const std::vector<GpuRig::BoneContact> same = RunLanes(unmarked, {});
	REQUIRE(same.size() == lanes.size());
	CHECK(std::memcmp(same.data(), lanes.data(), lanes.size() * sizeof(GpuRig::BoneContact)) == 0);
}

TEST_CASE("A sphere of a shape with cloth triangles leaves per-vertex surfaces to them and keeps the others", "[smpphysics][contact_gpu][clothtriangles]")
{
	Scene scene = MakeClothAndSphereScene();
	for (std::vector<BoneSphere>& spheres : scene.spheres) {
		for (BoneSphere& sphere : spheres)
			sphere.triangleContacts = true;
	}
	const std::vector<GpuRig::BoneContact> lanes = RunLanes(scene, {});

	// Bone 0's sphere touched only the body proxy, so its lanes are its triangles' alone.
	RequireClothLanes(scene, lanes, 0);
	// Bone 1's sphere still rests on the surface built from triangles, whose faces carry no driver here.
	ContactLanes sphereLanes;
	sphereLanes.Add({ 0.2f, { 0.0f, 0.0f, 1.0f }, 1 }, { 9.0f, 1.0f, -1.1f }, 0u);
	RequireClothLanes(scene, lanes, 1, sphereLanes);
	CHECK(lanes[kContactLanes].collider == 1);
	CHECK(lanes[kContactLanes].depth == Approx(0.2f).margin(kTolerance));
	CHECK(lanes[kContactLanes + 1].depth > 0.0f);
}
