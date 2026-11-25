// Runs SurfaceBones, SurfacePose and SurfaceTriangles (SurfacePrepCS.hlsl) on a WARP device and checks the driver
// each posed triangle records for the contact pass.

#include "rigid_gpu_harness.h"
#include "smp/GpuRigPacking.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <array>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <vector>

using namespace SmpPhysics;
using namespace SmpPhysics::RigidGpuTest;
using Catch::Approx;

namespace
{
	constexpr std::uint32_t kFirstDriverSlot{ 5 };
	constexpr std::uint32_t kSecondDriverSlot{ 7 };
	constexpr std::size_t kDriverSlots{ 8 };

	/** @brief Bone 0 follows driver 5, bone 1 driver 7 and bone 2 no driver. */
	std::vector<GpuRig::SurfacePrepBone> MakeBones()
	{
		std::vector<GpuRig::SurfacePrepBone> bones(3);
		for (GpuRig::SurfacePrepBone& bone : bones)
			bone.rotation = { 1.0f, 0.0f, 0.0f, 0.0f };
		bones[0].driver.x = kFirstDriverSlot;
		bones[1].driver.x = kSecondDriverSlot;
		bones[2].driver.x = GpuRig::kInvalidDriverSlot;
		return bones;
	}

	GpuRig::SurfacePrepVertex MakeVertex(float a_x, float a_y, std::uint32_t a_bone, float a_weight, std::uint32_t a_other = GpuRig::kInvalidDriverSlot, float a_otherWeight = 0.0f)
	{
		GpuRig::SurfacePrepVertex vertex;
		vertex.position = { a_x, a_y, 0.0f, 1.0f };
		vertex.bones = { a_bone, a_other, GpuRig::kInvalidDriverSlot, GpuRig::kInvalidDriverSlot };
		vertex.weights = { a_weight, a_otherWeight, 0.0f, 0.0f };
		return vertex;
	}

	std::uint32_t DriverOf(const GpuRig::SurfaceTriangle& a_triangle)
	{
		return std::bit_cast<std::uint32_t>(a_triangle.b.w);
	}

	/** @brief Poses the vertices of three triangles, each over its own three vertices, and returns their records. */
	std::vector<GpuRig::SurfaceTriangle> Run(const std::vector<GpuRig::SurfacePrepVertex>& a_vertices, const std::vector<GpuRig::SurfacePrepTriangle>& a_triangles,
		const std::vector<GpuRig::SurfacePrepBone>& a_bones = MakeBones())
	{
		static const GpuDevice gpu = CreateDevice(D3D_DRIVER_TYPE_WARP);
		static const std::filesystem::path path = std::filesystem::path(SMPPHYSICS_FEATURE_SHADER_DIR) / "SmpPhysics" / "SurfacePrepCS.hlsl";
		ID3D11Device* device = gpu.device.Get();
		ID3D11DeviceContext* context = gpu.context.Get();
		static const ComPtr<ID3D11ComputeShader> boneKernel = CompileKernel(device, "SurfaceBones", path);
		static const ComPtr<ID3D11ComputeShader> poseKernel = CompileKernel(device, "SurfacePose", path);
		static const ComPtr<ID3D11ComputeShader> triangleKernel = CompileKernel(device, "SurfaceTriangles", path);

		const std::vector<GpuRig::SurfacePrepBone>& bones = a_bones;
		std::vector<GpuRig::DriverWorld> drivers(kDriverSlots);
		for (GpuRig::DriverWorld& driver : drivers)
			driver.rotation = { 1.0f, 0.0f, 0.0f, 0.0f };
		drivers[kFirstDriverSlot].translation = { 10.0f, 0.0f, 0.0f, 0.0f };
		drivers[kSecondDriverSlot].translation = { 0.0f, 20.0f, 0.0f, 0.0f };

		std::array<ComPtr<ID3D11Buffer>, 7> sourceBuffers;
		std::array<std::size_t, 7> sourceCounts{};
		const auto bind = [&](std::size_t slot, const auto& values) {
			sourceBuffers[slot] = CreateTyped(device, values, D3D11_BIND_SHADER_RESOURCE);
			sourceCounts[slot] = values.size();
		};
		bind(0, a_vertices);
		bind(1, bones);
		bind(2, a_triangles);
		bind(6, drivers);
		const std::vector<GpuRig::Float4> posed(a_vertices.size());
		const std::vector<GpuRig::Float4> skinTranslations(bones.size());
		const std::vector<GpuRig::Float4> skinRotations(bones.size());
		const std::vector<GpuRig::SurfaceTriangle> records(a_triangles.size());
		const std::vector<std::uint32_t> unusedNodes(8);

		std::array<ComPtr<ID3D11Buffer>, 5> targetBuffers;
		std::array<std::size_t, 5> targetCounts{ posed.size(), bones.size(), bones.size(), records.size(), 2 };
		targetBuffers[0] = CreateTyped(device, posed, D3D11_BIND_UNORDERED_ACCESS);
		targetBuffers[1] = CreateTyped(device, skinTranslations, D3D11_BIND_UNORDERED_ACCESS);
		targetBuffers[2] = CreateTyped(device, skinRotations, D3D11_BIND_UNORDERED_ACCESS);
		targetBuffers[3] = CreateTyped(device, records, D3D11_BIND_UNORDERED_ACCESS);
		targetBuffers[4] = CreateStructured(device, 32, 2, D3D11_BIND_UNORDERED_ACCESS);

		std::array<ComPtr<ID3D11ShaderResourceView>, 7> sourceViews;
		std::array<ID3D11ShaderResourceView*, 7> sources{};
		for (std::size_t slot = 0; slot < sources.size(); ++slot) {
			if (!sourceBuffers[slot])
				continue;
			sourceViews[slot] = CreateSrv(device, sourceBuffers[slot].Get(), sourceCounts[slot]);
			sources[slot] = sourceViews[slot].Get();
		}
		std::array<ComPtr<ID3D11UnorderedAccessView>, 5> targetViews;
		std::array<ID3D11UnorderedAccessView*, 5> targets{};
		for (std::size_t slot = 0; slot < targets.size(); ++slot) {
			targetViews[slot] = CreateUav(device, targetBuffers[slot].Get(), targetCounts[slot]);
			targets[slot] = targetViews[slot].Get();
		}

		const std::array<std::uint32_t, 8> constants{ static_cast<std::uint32_t>(bones.size()), static_cast<std::uint32_t>(a_vertices.size()),
			static_cast<std::uint32_t>(a_triangles.size()), 0u, 0u, 0u, 0u, 0u };
		D3D11_BUFFER_DESC constantsDesc{};
		constantsDesc.ByteWidth = sizeof(constants);
		constantsDesc.Usage = D3D11_USAGE_IMMUTABLE;
		constantsDesc.BindFlags = D3D11_BIND_CONSTANT_BUFFER;
		D3D11_SUBRESOURCE_DATA constantsData{};
		constantsData.pSysMem = constants.data();
		ComPtr<ID3D11Buffer> constantBuffer;
		REQUIRE(device->CreateBuffer(&constantsDesc, &constantsData, constantBuffer.GetAddressOf()) == S_OK);

		context->CSSetConstantBuffers(0, 1, constantBuffer.GetAddressOf());
		context->CSSetShaderResources(0, static_cast<UINT>(sources.size()), sources.data());
		context->CSSetUnorderedAccessViews(0, static_cast<UINT>(targets.size()), targets.data(), nullptr);
		context->CSSetShader(boneKernel.Get(), nullptr, 0);
		context->Dispatch(GroupCount(bones.size()), 1, 1);
		context->CSSetShader(poseKernel.Get(), nullptr, 0);
		context->Dispatch(GroupCount(a_vertices.size()), 1, 1);
		context->CSSetShader(triangleKernel.Get(), nullptr, 0);
		context->Dispatch(GroupCount(a_triangles.size()), 1, 1);

		std::array<ID3D11UnorderedAccessView*, 5> noTargets{};
		std::array<ID3D11ShaderResourceView*, 7> noSources{};
		context->CSSetUnorderedAccessViews(0, static_cast<UINT>(noTargets.size()), noTargets.data(), nullptr);
		context->CSSetShaderResources(0, static_cast<UINT>(noSources.size()), noSources.data());
		context->CSSetShader(nullptr, nullptr, 0);
		return ReadBack<GpuRig::SurfaceTriangle>(device, context, targetBuffers[3].Get(), a_triangles.size());
	}
}

TEST_CASE("A posed triangle records the driver that carries most of its weight, plus one", "[smpphysics][surface_prep_gpu]")
{
	const std::vector<GpuRig::SurfacePrepVertex> vertices{
		MakeVertex(0.0f, 0.0f, 0, 1.0f), MakeVertex(1.0f, 0.0f, 0, 1.0f), MakeVertex(0.0f, 1.0f, 0, 0.6f, 1, 0.4f),
		MakeVertex(0.0f, 0.0f, 1, 1.0f), MakeVertex(1.0f, 0.0f, 1, 1.0f), MakeVertex(0.0f, 1.0f, 1, 1.0f),
		MakeVertex(0.0f, 0.0f, 0, 1.0f), MakeVertex(1.0f, 0.0f, 1, 1.0f), MakeVertex(0.0f, 1.0f, 2, 1.0f),
		MakeVertex(0.0f, 0.0f, 2, 1.0f), MakeVertex(1.0f, 0.0f, 2, 1.0f), MakeVertex(0.0f, 1.0f, 2, 1.0f)
	};
	std::vector<GpuRig::SurfacePrepTriangle> triangles(4);
	for (std::uint32_t index = 0; index < triangles.size(); ++index)
		triangles[index].corners = { index * 3u, index * 3u + 1u, index * 3u + 2u, index };

	const std::vector<GpuRig::SurfaceTriangle> records = Run(vertices, triangles);
	REQUIRE(records.size() == 4);
	CHECK(DriverOf(records[0]) == kFirstDriverSlot + 1u);
	CHECK(DriverOf(records[1]) == kSecondDriverSlot + 1u);
	CHECK(DriverOf(records[2]) == kFirstDriverSlot + 1u);
	CHECK(DriverOf(records[3]) == 0u);

	CHECK(records[0].a.x == Approx(10.0f).margin(1e-5f));
	CHECK(records[1].a.y == Approx(20.0f).margin(1e-5f));
	CHECK(records[3].a.x == Approx(0.0f).margin(1e-5f));
	CHECK(std::bit_cast<std::uint32_t>(records[2].a.w) == 2u);
}

TEST_CASE("A posed vertex is scaled about its bone by the bone's skin scale", "[smpphysics][surface_prep_gpu]")
{
	std::vector<GpuRig::SurfacePrepBone> bones = MakeBones();
	bones[0].translation = { -4.0f, 0.0f, 0.0f, 1.5f };
	const std::vector<GpuRig::SurfacePrepVertex> vertices{ MakeVertex(4.0f, 0.0f, 0, 1.0f), MakeVertex(6.0f, 0.0f, 0, 1.0f), MakeVertex(4.0f, 2.0f, 0, 1.0f) };
	std::vector<GpuRig::SurfacePrepTriangle> triangles(1);
	triangles[0].corners = { 0u, 1u, 2u, 0u };

	const std::vector<GpuRig::SurfaceTriangle> records = Run(vertices, triangles, bones);
	REQUIRE(records.size() == 1);
	// Driver 5 sits at x = 10; the bind translation is already in world units, so only the vertex is scaled.
	CHECK(records[0].a.x == Approx(10.0f - 4.0f + 1.5f * 4.0f).margin(1e-4f));
	CHECK(records[0].b.x == Approx(10.0f - 4.0f + 1.5f * 6.0f).margin(1e-4f));
	CHECK(records[0].c.y == Approx(1.5f * 2.0f).margin(1e-4f));
}
