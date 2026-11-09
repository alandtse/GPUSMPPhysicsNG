// Runs the contact passes (ContactCS.hlsl: ContactCull, ContactSpheres, ContactReduce) on a WARP
// device and compares every bone's merged contact with the CPU's QueryColliders and ContactMerge, then checks which
// trees the passes serve.

#include "rigid_gpu_harness.h"
#include "smp/Collision.h"
#include "smp/GpuRigPacking.h"
#include "smp/SolverCommon.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
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
	};

	const ContactKernels& LoadKernels()
	{
		static const ContactKernels built = [] {
			ContactKernels value;
			value.gpu = CreateDevice(D3D_DRIVER_TYPE_WARP);
			ShaderInclude include({ std::filesystem::path(SMPPHYSICS_SHADER_DIR), std::filesystem::path(SMPPHYSICS_FEATURE_SHADER_DIR) });
			const std::filesystem::path path = std::filesystem::path(SMPPHYSICS_FEATURE_SHADER_DIR) / "SmpPhysics" / "ContactCS.hlsl";
			ID3D11Device* device = value.gpu.device.Get();
			value.cull = CompileKernel(device, &include, "ContactCull", path);
			value.spheres = CompileKernel(device, &include, "ContactSpheres", path);
			value.reduce = CompileKernel(device, &include, "ContactReduce", path);
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

	/** @brief The merged contact the CPU makes for one bone, the oracle for ContactReduce. */
	GpuRig::BoneContact ExpectedContact(const Scene& a_scene, std::size_t a_bone)
	{
		ContactMerge merge;
		for (const BoneSphere& sphere : a_scene.spheres[a_bone]) {
			const Vec3 centre = a_scene.boneWorld[a_bone].TransformPoint(sphere.centerLocal);
			const Contact contact = QueryColliders(a_scene.colliders, { a_scene.driver }, centre, sphere.radius + a_scene.contactSkin, sphere.exemptMask);
			merge.Add(contact, centre);
		}
		GpuRig::BoneContact expected;
		Vec3 normal{};
		Vec3 position{};
		expected.depth = merge.Result(normal, position);
		expected.deepest = merge.deepest;
		expected.collider = merge.collider;
		expected.normal[0] = normal.x;
		expected.normal[1] = normal.y;
		expected.normal[2] = normal.z;
		expected.position[0] = position.x;
		expected.position[1] = position.y;
		expected.position[2] = position.z;
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
	};

	/** @brief Dispatches the three contact passes over a scene and returns each bone's merged contact; a skipped bone keeps the sentinel. */
	std::vector<GpuRig::BoneContact> RunPasses(const Scene& a_scene, const PassOptions& a_options)
	{
		const ContactKernels& kernels = LoadKernels();
		ID3D11Device* device = kernels.gpu.device.Get();
		ID3D11DeviceContext* context = kernels.gpu.context.Get();

		const GpuRig::CollisionData collision = GpuRig::PackCollision(a_scene.spheres, a_scene.colliders, 1);
		std::vector<GpuRig::SphereRecord> sphereList = collision.sphereList;
		std::size_t cursor = 0;
		for (std::size_t bone = 0; bone < kBoneCount; ++bone)
			for (std::size_t index = 0; index < a_scene.spheres[bone].size(); ++index)
				sphereList[cursor++].exempt.y = static_cast<std::uint32_t>(bone);

		std::vector<GpuRig::TreeStatic> trees(kTreeCount);
		for (GpuRig::TreeStatic& tree : trees)
			tree.contact = { 0u, static_cast<std::uint32_t>(collision.colliders.size()), 0u, 1u };

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
		const std::vector<GpuRig::DriverWorld> drivers{ { { 1.0f, 0.0f, 0.0f, 0.0f }, { a_scene.driver.translation.x, a_scene.driver.translation.y, a_scene.driver.translation.z, 0.0f } } };
		GpuRig::SurfaceFrame surfaceFrame;
		std::uint32_t firstMeta = 0;
		std::uint32_t metaCount = 0;
		if (!a_scene.surfaces.empty())
			GpuRig::AppendSurfaces(surfaceFrame, a_scene.surfaces, { a_scene.driver }, firstMeta, metaCount);
		for (GpuRig::TreeInput& input : inputs) {
			input.surface.x = firstMeta;
			input.surface.y = metaCount;
		}
		const std::vector<GpuRig::BoneContact> contacts(kBoneCount, Sentinel());
		const std::vector<std::uint32_t> boneTrees(kBoneTrees.begin(), kBoneTrees.end());
		const std::vector<float> hits(sphereList.size() * 12);
		const std::vector<std::uint32_t> cull(kBoneCount * 4);
		const std::vector<std::uint32_t> unused(4);

		std::array<ComPtr<ID3D11Buffer>, 7> targetBuffers;
		targetBuffers[1] = CreateTyped(device, worlds, D3D11_BIND_UNORDERED_ACCESS);
		targetBuffers[3] = CreateTyped(device, contacts, D3D11_BIND_UNORDERED_ACCESS);
		targetBuffers[5] = CreateStructured(device, 48, sphereList.size(), D3D11_BIND_UNORDERED_ACCESS);
		targetBuffers[6] = CreateStructured(device, 16, kBoneCount, D3D11_BIND_UNORDERED_ACCESS);
		const std::array<std::size_t, 7> targetCounts{ 0, kBoneCount, 0, kBoneCount, 0, sphereList.size(), kBoneCount };
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
			std::uint32_t boneCount, sphereCount, padding[2];
		};
		const Constants constants{ static_cast<std::uint32_t>(kBoneCount), static_cast<std::uint32_t>(sphereList.size()), {} };
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
		context->CSSetShader(kernels.spheres.Get(), nullptr, 0);
		context->Dispatch(GroupCount(sphereList.size()), 1, 1);
		context->CSSetShader(kernels.reduce.Get(), nullptr, 0);
		context->Dispatch(GroupCount(kBoneCount), 1, 1);

		std::array<ID3D11UnorderedAccessView*, 7> noTargets{};
		std::array<ID3D11ShaderResourceView*, 15> noSources{};
		context->CSSetUnorderedAccessViews(0, static_cast<UINT>(noTargets.size()), noTargets.data(), nullptr);
		context->CSSetShaderResources(0, static_cast<UINT>(noSources.size()), noSources.data());
		context->CSSetShader(nullptr, nullptr, 0);
		return ReadBack<GpuRig::BoneContact>(device, context, targetBuffers[3].Get(), kBoneCount);
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

	bool IsSkipped(const GpuRig::BoneContact& a_contact)
	{
		return a_contact.depth == kSentinelDepth;
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

TEST_CASE("A bone between two surface clusters, inside the BVH root box, finds no contact", "[smpphysics][contact_gpu]")
{
	std::vector<SphereProbe> probes;
	CheckSurfaceDepthsMatchProbe(MakeClusteredSurfaceScene(), probes);
	CHECK(probes[0].depth > 0.0f);
	CHECK(probes[1].depth == 0.0f);
	CHECK(probes[2].depth > 0.0f);
	CHECK(probes[3].depth == 0.0f);
}
