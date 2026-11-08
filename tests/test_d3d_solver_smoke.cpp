// Smoke test for the D3D11 layer (src/d3d): a hair rig is added to the GPU solver on a WARP device and stepped
// through the rigid path, which proves the shader root, runtime compile, buffer helpers and dispatch wiring.

#include "d3d/GpuSolver.h"
#include "smp/SmpRig.h"

#include <catch2/catch_test_macros.hpp>

#include <d3d11.h>
#include <wrl/client.h>

#include <chrono>
#include <cmath>
#include <filesystem>
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
	frame[0].rigid = true;
	frame[0].deltaTime = 1.0f / 60.0f;

	std::vector<GpuSolver::ClaimResult> results;
	bool polled = false;
	for (int step = 0; step < 12 && !polled; ++step) {
		frame[0].hasPreviousInput = step > 0;
		REQUIRE(solver.SubmitFrame(frame));
		polled = solver.Poll(results);
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
