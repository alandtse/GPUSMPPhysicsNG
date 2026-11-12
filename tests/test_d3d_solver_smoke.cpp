// Smoke test for the D3D11 layer (src/d3d): a hair rig is added to the GPU solver on a WARP device and stepped
// through the rigid path, which proves the shader root, runtime compile, buffer helpers and dispatch wiring.

#include "Utils/D3D.h"
#include "d3d/GpuSolver.h"
#include "shader_cache_dir.h"
#include "smp/SmpRig.h"

#include <catch2/catch_test_macros.hpp>

#include <d3d11.h>
#include <wrl/client.h>

#include <chrono>
#include <cmath>
#include <filesystem>
#include <functional>
#include <iostream>
#include <string_view>
#include <thread>

using namespace SmpPhysics;
using Microsoft::WRL::ComPtr;

namespace
{
	constexpr std::string_view kRigXml = R"XML(<?xml version="1.0" encoding="utf-8"?>
<system xmlns="http://www.example.com/hdt">
	<bone name="NPC Head [Head]">
		<mass>0</mass>
	</bone>
	<bone-default name="hair-default">
		<mass>0.1</mass>
		<inertia x="16" y="8" z="4"/>
		<linearDamping>0.75</linearDamping>
		<angularDamping>0.5</angularDamping>
		<gravity-factor>0.5</gravity-factor>
	</bone-default>
	<bone name="Hair01" template="hair-default"/>
	<bone name="Hair02" template="hair-default"/>
	<generic-constraint-default name="hair-joint">
		<frameInB>
			<basis x="0" y="0" z="0" w="1"/>
			<origin x="0" y="0" z="4"/>
		</frameInB>
		<linearLowerLimit x="0" y="0" z="0"/>
		<linearUpperLimit x="0" y="0" z="0"/>
		<angularLowerLimit x="-0.6" y="-0.5" z="-0.4"/>
		<angularUpperLimit x="0.6" y="0.5" z="0.4"/>
		<angularStiffness x="3" y="4" z="5"/>
		<angularDamping x="0.1" y="0.2" z="0.3"/>
		<angularEquilibrium x="0.05" y="0" z="-0.05"/>
	</generic-constraint-default>
	<constraint-group>
		<generic-constraint bodyA="Hair01" bodyB="NPC Head [Head]" template="hair-joint"/>
		<generic-constraint bodyA="Hair02" bodyB="Hair01" template="hair-joint"/>
	</constraint-group>
</system>
)XML";
}

TEST_CASE("The GPU solver steps a rigid hair claim on a WARP device", "[smpphysics][d3d]")
{
	ComPtr<ID3D11Device> device;
	ComPtr<ID3D11DeviceContext> context;
	const D3D_FEATURE_LEVEL levels[] = { D3D_FEATURE_LEVEL_11_0 };
	REQUIRE(D3D11CreateDevice(nullptr, D3D_DRIVER_TYPE_WARP, nullptr, 0, levels, 1, D3D11_SDK_VERSION,
				device.GetAddressOf(), nullptr, context.GetAddressOf()) == S_OK);
	globals::d3d::device = device.Get();
	globals::d3d::context = context.Get();
	logger::GetSink() = [](logger::Level, const std::string& a_message) { std::cerr << a_message << '\n'; };
	Util::SetShaderRoot(std::filesystem::path(SMPPHYSICS_SHADER_DIR).wstring());
	Util::SetShaderCacheDir(TestShaderCacheDir());

	const ParseResult parsed = ParseSmpXml(kRigXml);
	REQUIRE(parsed.status == ParseStatus::Ok);
	const std::vector<Transform> rest(parsed.rig.bones.size());

	GpuSolver solver;
	std::string reason;
	const bool added = solver.AddClaim(1, parsed.rig, rest, SolverParams{}, {}, {}, {}, reason);
	INFO("reason: " << reason << " unavailable: " << solver.UnavailableReason());
	REQUIRE(added);

	std::vector<GpuSolver::FrameClaim> frame(1);
	frame[0].token = 1;
	frame[0].kinematicWorld = &rest;
	frame[0].deltaTime = 1.0f / 60.0f;

	std::vector<GpuSolver::ClaimResult> results;
	bool polled = false;
	// WARP finishes a frame some time after it is submitted, so the readback is awaited rather than counted in frames.
	for (int step = 0; step < 400 && !polled; ++step) {
		frame[0].hasPreviousInput = step > 0;
		REQUIRE(solver.SubmitFrame(frame));
		context->Flush();
		polled = solver.Poll(results);
		if (!polled)
			std::this_thread::sleep_for(std::chrono::milliseconds(5));
	}
	INFO(solver.UnavailableReason());
	REQUIRE(solver.Available());
	REQUIRE(polled);
	REQUIRE(results.size() == 1);
	CHECK_FALSE(results[0].failed);
	REQUIRE(results[0].bones.size() == rest.size());
	for (const Transform& bone : results[0].bones)
		CHECK(std::isfinite(bone.translation.x));

	globals::d3d::device = nullptr;
	globals::d3d::context = nullptr;
}

namespace
{
	struct WarpDevice
	{
		ComPtr<ID3D11Device> device;
		ComPtr<ID3D11DeviceContext> context;

		WarpDevice()
		{
			const D3D_FEATURE_LEVEL levels[] = { D3D_FEATURE_LEVEL_11_0 };
			REQUIRE(D3D11CreateDevice(nullptr, D3D_DRIVER_TYPE_WARP, nullptr, 0, levels, 1, D3D11_SDK_VERSION,
						device.GetAddressOf(), nullptr, context.GetAddressOf()) == S_OK);
			globals::d3d::device = device.Get();
			globals::d3d::context = context.Get();
			Util::SetShaderRoot(std::filesystem::path(SMPPHYSICS_SHADER_DIR).wstring());
			Util::SetShaderCacheDir(TestShaderCacheDir());
		}

		~WarpDevice()
		{
			globals::d3d::device = nullptr;
			globals::d3d::context = nullptr;
		}
	};

	/** @brief Steps the two-bone rig for a_frames frames with a_claim's wind and pins and returns the newest root-relative bones. */
	std::vector<Transform> StepRig(WarpDevice& a_gpu, const std::function<void(GpuSolver::FrameClaim&)>& a_configure, int a_frames)
	{
		const ParseResult parsed = ParseSmpXml(kRigXml);
		REQUIRE(parsed.status == ParseStatus::Ok);
		const std::vector<Transform> rest(parsed.rig.bones.size());
		GpuSolver solver;
		std::string reason;
		REQUIRE(solver.AddClaim(1, parsed.rig, rest, SolverParams{}, {}, {}, {}, reason));

		std::vector<GpuSolver::FrameClaim> frame(1);
		frame[0].token = 1;
		frame[0].kinematicWorld = &rest;
		frame[0].deltaTime = 1.0f / 60.0f;
		a_configure(frame[0]);
		std::vector<GpuSolver::ClaimResult> results;
		std::vector<Transform> newest;
		for (int step = 0; step < a_frames; ++step) {
			frame[0].hasPreviousInput = step > 0;
			REQUIRE(solver.SubmitFrame(frame));
			a_gpu.context->Flush();
			if (solver.Poll(results) && !results.empty())
				newest = results[0].bones;
			std::this_thread::sleep_for(std::chrono::milliseconds(2));
		}
		REQUIRE(solver.Available());
		REQUIRE_FALSE(newest.empty());
		return newest;
	}
}

TEST_CASE("Wind pushes a rigid hair claim downwind and a sheltered claim stays put", "[smpphysics][d3d]")
{
	WarpDevice gpu;
	const std::vector<Transform> calm = StepRig(gpu, [](GpuSolver::FrameClaim&) {}, 120);
	const std::vector<Transform> windy = StepRig(gpu, [](GpuSolver::FrameClaim& a_claim) {
		a_claim.wind = { 600.0f, 0.0f, 0.0f };
		a_claim.windFactor = 1.0f; }, 120);
	const std::vector<Transform> sheltered = StepRig(gpu, [](GpuSolver::FrameClaim& a_claim) {
		a_claim.wind = { 600.0f, 0.0f, 0.0f };
		a_claim.windFactor = 0.0f; }, 120);
	REQUIRE(calm.size() == windy.size());
	const std::size_t tip = windy.size() - 1;
	CHECK(windy[tip].translation.x - calm[tip].translation.x > 0.05f);
	CHECK(std::fabs(sheltered[tip].translation.x - calm[tip].translation.x) < 1.0e-3f);
}

TEST_CASE("A grab pin pulls a rigid hair claim's bone toward the hand", "[smpphysics][d3d]")
{
	WarpDevice gpu;
	const std::vector<Transform> free = StepRig(gpu, [](GpuSolver::FrameClaim&) {}, 120);
	const std::size_t tip = free.size() - 1;
	const Vec3 target = free[tip].translation + Vec3{ 0.0f, 8.0f, 0.0f };
	const std::vector<Transform> held = StepRig(gpu, [&](GpuSolver::FrameClaim& a_claim) {
		Grab grab;
		grab.bone = static_cast<int>(tip);
		grab.target = target;
		a_claim.grabs.push_back(grab); }, 120);
	for (const Transform& bone : held)
		REQUIRE(std::isfinite(bone.translation.y));
	CHECK(held[tip].translation.y - free[tip].translation.y > 1.0f);
}

TEST_CASE("Resource names are stored under the narrow debug-name GUID so RenderDoc reads them as text", "[smpphysics][d3d]")
{
	ComPtr<ID3D11Device> device;
	const D3D_FEATURE_LEVEL levels[] = { D3D_FEATURE_LEVEL_11_0 };
	REQUIRE(D3D11CreateDevice(nullptr, D3D_DRIVER_TYPE_WARP, nullptr, 0, levels, 1, D3D11_SDK_VERSION,
				device.GetAddressOf(), nullptr, nullptr) == S_OK);
	D3D11_BUFFER_DESC desc{};
	desc.ByteWidth = 16;
	desc.BindFlags = D3D11_BIND_SHADER_RESOURCE;
	ComPtr<ID3D11Buffer> buffer;
	REQUIRE(device->CreateBuffer(&desc, nullptr, buffer.GetAddressOf()) == S_OK);

	Util::SetResourceName(buffer.Get(), "SmpPhysics::%s", "Probe");

	constexpr GUID kNarrowNameGuid = { 0x429b8c22, 0x9188, 0x4b0c, { 0x87, 0x42, 0xac, 0xb0, 0xbf, 0x85, 0xc2, 0x00 } };
	char name[64]{};
	UINT size = sizeof(name);
	REQUIRE(buffer->GetPrivateData(kNarrowNameGuid, &size, name) == S_OK);
	CHECK(std::string_view(name, size) == "SmpPhysics::Probe");
}
