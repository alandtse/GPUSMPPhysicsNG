// Unit tests for the SMP rig to rigid world conversion (src/smp/RigidRigBuilder.cpp).
// Pure data; no game/D3D dependency.

#include "smp/RigidRigBuilder.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <string_view>

using namespace SmpPhysics;
using Catch::Approx;

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

	RigidReference::World Build(float a_scale)
	{
		const ParseResult parsed = ParseSmpXml(kRigXml);
		REQUIRE(parsed.status == ParseStatus::Ok);
		const std::vector<Transform> rest(parsed.rig.bones.size());
		return BuildRigidWorld(parsed.rig, rest, a_scale, SolverParams{});
	}
}

TEST_CASE("The rigid world has one body per bone and one constraint per joint", "[smpphysics][rigid_rig]")
{
	const RigidReference::World world = Build(1.0f);
	REQUIRE(world.bodies.size() == 3);
	REQUIRE(world.constraints.size() == 2);
	CHECK(world.bodies[0].kinematic);
	CHECK(world.bodies[0].inverseMass == 0.0f);
	CHECK(world.constraints[0].bodyA == 1);
	CHECK(world.constraints[0].bodyB == 0);
	CHECK(world.constraints[1].bodyA == 2);
	CHECK(world.constraints[1].bodyB == 1);
}

TEST_CASE("A dynamic body carries the file's mass, inertia, damping and weighted gravity", "[smpphysics][rigid_rig]")
{
	const RigidReference::World world = Build(1.0f);
	const RigidReference::Body& body = world.bodies[1];
	CHECK_FALSE(body.kinematic);
	CHECK(body.inverseMass == Approx(10.0f));
	CHECK(body.inverseInertiaLocal.x == Approx(1.0f / 16.0f));
	CHECK(body.inverseInertiaLocal.y == Approx(1.0f / 8.0f));
	CHECK(body.inverseInertiaLocal.z == Approx(1.0f / 4.0f));
	CHECK(body.linearDamping == Approx(0.75f));
	CHECK(body.angularDamping == Approx(0.5f));
	CHECK(body.gravity.z == Approx(-0.5f * 0.1f * SolverParams{}.gravity));
}

TEST_CASE("A joint carries its angular limits, springs and SMP's stop defaults", "[smpphysics][rigid_rig]")
{
	const RigidReference::World world = Build(1.0f);
	const RigidReference::Generic6DofSpring2& joint = world.constraints[0];
	CHECK(joint.angular[0].lower == Approx(-0.6f));
	CHECK(joint.angular[1].upper == Approx(0.5f));
	CHECK(joint.angular[2].springStiffness == Approx(5.0f));
	CHECK(joint.angular[1].springDamping == Approx(0.2f));
	CHECK(joint.angular[0].equilibrium == Approx(0.05f));
	CHECK(joint.angular[0].stopCfm == Approx(kRigidAngularStopCfm));
	CHECK(joint.linear.stopCfm.x == 0.0f);
	CHECK(joint.linear.stopErp.x == Approx(0.2f));
}

TEST_CASE("A joint carries its linear window, spring and damping, scaled as SMP scales them", "[smpphysics][rigid_rig]")
{
	constexpr std::string_view kSlidingXml = R"XML(<?xml version="1.0" encoding="utf-8"?>
<system xmlns="http://www.example.com/hdt">
	<bone name="Head"><mass>0</mass></bone>
	<bone name="Hair01"><mass>0.1</mass></bone>
	<generic-constraint bodyA="Hair01" bodyB="Head">
		<linearLowerLimit x="-1" y="-2" z="-3"/>
		<linearUpperLimit x="1" y="2" z="3"/>
		<linearStiffness x="4" y="5" z="6"/>
		<linearDamping x="0.1" y="0.2" z="0.3"/>
		<linearEquilibrium x="0.5" y="0" z="-0.5"/>
		<linearBounce x="0.25" y="0" z="0"/>
		<linearStiffnessLimited>false</linearStiffnessLimited>
		<enableLinearSprings>false</enableLinearSprings>
	</generic-constraint>
</system>
)XML";
	const ParseResult parsed = ParseSmpXml(kSlidingXml);
	REQUIRE(parsed.status == ParseStatus::Ok);
	const std::vector<Transform> rest(parsed.rig.bones.size());

	constexpr float kScale = 2.0f;
	const RigidReference::World unit = BuildRigidWorld(parsed.rig, rest, 1.0f, SolverParams{});
	const RigidReference::World scaled = BuildRigidWorld(parsed.rig, rest, kScale, SolverParams{});
	const RigidReference::LinearAxisLimits& linear = unit.constraints[0].linear;
	CHECK(linear.lower.y == Approx(-2.0f));
	CHECK(linear.upper.z == Approx(3.0f));
	CHECK(linear.springStiffness.z == Approx(6.0f));
	CHECK(linear.springDamping.y == Approx(0.2f));
	CHECK(linear.equilibrium.z == Approx(-0.5f));
	CHECK(linear.bounce.x == Approx(0.25f));
	CHECK_FALSE(linear.springEnabled[0]);
	CHECK_FALSE(linear.springStiffnessLimited[1]);
	CHECK(linear.springDampingLimited[2]);

	const RigidReference::LinearAxisLimits& big = scaled.constraints[0].linear;
	CHECK(big.upper.z == Approx(linear.upper.z * kScale));
	CHECK(big.equilibrium.z == Approx(linear.equilibrium.z * kScale));
	CHECK(big.springStiffness.z == Approx(linear.springStiffness.z * kScale * kScale * kScale));
	CHECK(big.springDamping.y == Approx(linear.springDamping.y));
}

TEST_CASE("The actor's scale follows SMP: mass by s^3, inertia and angular stiffness by s^5", "[smpphysics][rigid_rig]")
{
	constexpr float kScale = 1.5f;
	const RigidReference::World unit = Build(1.0f);
	const RigidReference::World scaled = Build(kScale);
	const float scale3 = kScale * kScale * kScale;
	const float scale5 = scale3 * kScale * kScale;
	CHECK(scaled.bodies[1].inverseMass == Approx(unit.bodies[1].inverseMass / scale3));
	CHECK(scaled.bodies[1].inverseInertiaLocal.x == Approx(unit.bodies[1].inverseInertiaLocal.x / scale5));
	CHECK(scaled.constraints[0].angular[2].springStiffness == Approx(unit.constraints[0].angular[2].springStiffness * scale5));
	CHECK(scaled.constraints[0].frameB.origin.z == Approx(unit.constraints[0].frameB.origin.z * kScale));
}

TEST_CASE("A joint carries its motors, servos, ERP and CFM, bounce and non-Hookean factors", "[smpphysics][rigid_rig]")
{
	constexpr std::string_view kMotorXml = R"XML(<?xml version="1.0" encoding="utf-8"?>
<system xmlns="http://www.example.com/hdt">
	<bone name="Head"><mass>0</mass></bone>
	<bone name="Hair01"><mass>0.1</mass></bone>
	<generic-constraint bodyA="Hair01" bodyB="Head">
		<linearMotors>true</linearMotors>
		<angularServoMotors>true</angularServoMotors>
		<linearTargetVelocity x="1" y="2" z="3"/>
		<angularTargetVelocity x="4" y="5" z="6"/>
		<linearMaxMotorForce x="7" y="8" z="9"/>
		<angularMaxMotorForce x="10" y="11" z="12"/>
		<linearEquilibrium x="0.5" y="0" z="0"/>
		<angularEquilibrium x="0" y="0.25" z="0"/>
		<angularBounce x="0.1" y="0.2" z="0.3"/>
		<linearNonHookeanStiffness x="0.5" y="0" z="0"/>
		<angularNonHookeanDamping x="0" y="0.75" z="0"/>
		<stopERP>0.3</stopERP>
		<stopCFM>0.002</stopCFM>
		<motorERP>0.6</motorERP>
		<motorCFM>0.004</motorCFM>
	</generic-constraint>
</system>
)XML";
	const ParseResult parsed = ParseSmpXml(kMotorXml);
	REQUIRE(parsed.status == ParseStatus::Ok);
	const std::vector<Transform> rest(parsed.rig.bones.size());
	const RigidReference::World world = BuildRigidWorld(parsed.rig, rest, 2.0f, SolverParams{});
	const RigidReference::Generic6DofSpring2& joint = world.constraints[0];

	CHECK(joint.linear.motorEnabled[1]);
	CHECK_FALSE(joint.linear.servoEnabled[1]);
	CHECK(joint.linear.targetVelocity.z == Approx(3.0f));
	CHECK(joint.linear.maxMotorForce.x == Approx(7.0f));
	CHECK(joint.linear.servoTarget.x == Approx(1.0f));
	CHECK(joint.linear.nonHookeanStiffness.x == Approx(0.5f));
	CHECK(joint.linear.stopErp.y == Approx(0.3f));
	CHECK(joint.linear.stopCfm.z == Approx(0.002f));
	CHECK(joint.linear.motorErp.x == Approx(0.6f));
	CHECK(joint.linear.motorCfm.x == Approx(0.004f));

	CHECK(joint.angular[1].servoEnabled);
	CHECK_FALSE(joint.angular[1].motorEnabled);
	CHECK(joint.angular[1].servoTarget == Approx(0.25f));
	CHECK(joint.angular[0].targetVelocity == Approx(4.0f));
	CHECK(joint.angular[2].maxMotorForce == Approx(12.0f));
	CHECK(joint.angular[2].bounce == Approx(0.3f));
	CHECK(joint.angular[1].nonHookeanDamping == Approx(0.75f));
	CHECK(joint.angular[0].stopCfm == Approx(0.002f));
	CHECK(joint.angular[0].motorErp == Approx(0.6f));
}

TEST_CASE("useLinearReferenceFrameA makes the parent body A, as SMP swaps the constraint's bodies", "[smpphysics][rigid_rig]")
{
	constexpr std::string_view kSwapXml = R"XML(<?xml version="1.0" encoding="utf-8"?>
<system xmlns="http://www.example.com/hdt">
	<bone name="Head"><mass>0</mass></bone>
	<bone name="Hair01"><mass>0.1</mass></bone>
	<generic-constraint bodyA="Hair01" bodyB="Head">
		<useLinearReferenceFrameA>true</useLinearReferenceFrameA>
	</generic-constraint>
</system>
)XML";
	const ParseResult parsed = ParseSmpXml(kSwapXml);
	REQUIRE(parsed.status == ParseStatus::Ok);
	const std::vector<Transform> rest(parsed.rig.bones.size());
	const RigidReference::World world = BuildRigidWorld(parsed.rig, rest, 1.0f, SolverParams{});
	CHECK(world.constraints[0].bodyA == 0);
	CHECK(world.constraints[0].bodyB == 1);
}

TEST_CASE("A joint the topology pass turned around still steps with its authored bodies", "[smpphysics][rigid_rig]")
{
	constexpr std::string_view kHeldRootXml = R"XML(<?xml version="1.0" encoding="utf-8"?>
<system xmlns="http://www.example.com/hdt">
	<bone name="Head"><mass>0</mass></bone>
	<bone name="Hang"><mass>0.1</mass></bone>
	<bone name="Root"><mass>0.1</mass></bone>
	<bone name="Leaf"><mass>0.1</mass></bone>
	<generic-constraint bodyA="Hang" bodyB="Head"/>
	<generic-constraint bodyA="Leaf" bodyB="Root"/>
	<generic-constraint bodyA="Leaf" bodyB="Hang"/>
</system>
)XML";
	ParseResult parsed = ParseSmpXml(kHeldRootXml);
	REQUIRE(parsed.status == ParseStatus::Ok);
	std::string reason;
	REQUIRE(NormalizeRigTopology(parsed.rig, reason));
	const std::vector<Transform> rest(parsed.rig.bones.size());
	const RigidReference::World world = BuildRigidWorld(parsed.rig, rest, 1.0f, SolverParams{});

	int leaf = -1;
	int root = -1;
	for (std::size_t index = 0; index < parsed.rig.bones.size(); ++index) {
		if (parsed.rig.bones[index].name == "Leaf")
			leaf = static_cast<int>(index);
		if (parsed.rig.bones[index].name == "Root")
			root = static_cast<int>(index);
	}
	bool found = false;
	for (const RigidReference::Generic6DofSpring2& constraint : world.constraints) {
		if (constraint.bodyA == leaf && constraint.bodyB == root)
			found = true;
		CHECK_FALSE(constraint.bodyA == root);
	}
	CHECK(found);
}

namespace
{
	constexpr std::string_view kComRigXml = R"XML(<?xml version="1.0" encoding="utf-8"?>
<system xmlns="http://www.example.com/hdt">
	<bone name="NPC Pelvis [Pelv]">
		<mass>0</mass>
	</bone>
	<bone name="NPC Belly">
		<mass>5</mass>
		<inertia x="2" y="2" z="2"/>
		<centerOfMassTransform>
			<basis x="0" y="0" z="0" w="1"/>
			<origin x="0" y="0" z="-1"/>
		</centerOfMassTransform>
	</bone>
	<bone name="NPC L FrontThigh">
		<mass>2</mass>
		<inertia x="2" y="2" z="2"/>
		<centerOfMassTransform>
			<basis x="2" y="0" z="0" w="1"/>
			<origin x="0" y="0" z="0"/>
		</centerOfMassTransform>
	</bone>
	<bone name="NPC Plain">
		<mass>1</mass>
		<inertia x="1" y="1" z="1"/>
	</bone>
	<generic-constraint bodyA="NPC Belly" bodyB="NPC Pelvis [Pelv]"/>
</system>
)XML";
}

TEST_CASE("A dynamic bone keeps its centerOfMassTransform", "[smpphysics][rigid_rig]")
{
	const ParseResult parsed = ParseSmpXml(kComRigXml);
	REQUIRE(parsed.status == ParseStatus::Ok);
	REQUIRE(parsed.rig.bones.size() == 4);
	CHECK(parsed.rig.bones[1].comTransform.translation.z == Approx(-1.0f));
	CHECK(parsed.rig.bones[2].comTransform.rotation.x == Approx(2.0f / std::sqrt(5.0f)));
	CHECK(parsed.rig.bones[2].comTransform.rotation.w == Approx(1.0f / std::sqrt(5.0f)));
	CHECK(parsed.rig.bones[3].comTransform.translation.z == 0.0f);
}

TEST_CASE("A kinematic bone with a centerOfMassTransform is declined", "[smpphysics][rigid_rig]")
{
	const ParseResult parsed = ParseSmpXml(R"XML(<?xml version="1.0" encoding="utf-8"?>
<system xmlns="http://www.example.com/hdt">
	<bone name="Root">
		<mass>0</mass>
		<centerOfMassTransform>
			<basis x="0" y="0" z="0" w="1"/>
			<origin x="0" y="0" z="2"/>
		</centerOfMassTransform>
	</bone>
</system>
)XML");
	CHECK(parsed.status == ParseStatus::Unsupported);
	CHECK(parsed.reason.find("Root") != std::string::npos);
}

TEST_CASE("A dynamic body sits on its node composed with the centerOfMassTransform, a kinematic one on its node", "[smpphysics][rigid_rig]")
{
	const ParseResult parsed = ParseSmpXml(kComRigXml);
	REQUIRE(parsed.status == ParseStatus::Ok);
	std::vector<Transform> nodes(parsed.rig.bones.size());
	for (Transform& node : nodes)
		node.translation = { 10.0f, 20.0f, 30.0f };
	const std::vector<Transform> frames = BodyFrames(parsed.rig, nodes, 1.0f);
	CHECK(frames[0].translation.z == Approx(30.0f));
	CHECK(frames[1].translation.z == Approx(29.0f));
	CHECK(frames[3].translation.z == Approx(30.0f));
	const Transform back = frames[2] * NodeFromBody(parsed.rig, 1.0f)[2];
	CHECK(back.translation.x == Approx(10.0f));
	CHECK(back.rotation.w == Approx(1.0f));
}

TEST_CASE("NodeFromBody inverts each centerOfMassTransform and is empty when none is set", "[smpphysics][rigid_rig]")
{
	const ParseResult parsed = ParseSmpXml(kComRigXml);
	REQUIRE(parsed.status == ParseStatus::Ok);
	const std::vector<Transform> inverses = NodeFromBody(parsed.rig, 1.0f);
	REQUIRE(inverses.size() == 4);
	const Transform identity = parsed.rig.bones[1].comTransform * inverses[1];
	CHECK(identity.translation.z == Approx(0.0f).margin(1e-5f));
	CHECK(NodeFromBody(ParseSmpXml(kRigXml).rig, 1.0f).empty());
}

TEST_CASE("The actor's scale scales the centerOfMassTransform origin as SMP scales it", "[smpphysics][rigid_rig]")
{
	const ParseResult parsed = ParseSmpXml(kComRigXml);
	REQUIRE(parsed.status == ParseStatus::Ok);
	const std::vector<Transform> nodes(parsed.rig.bones.size());
	const std::vector<Transform> frames = BodyFrames(parsed.rig, nodes, 2.0f);
	CHECK(frames[1].translation.z == Approx(-2.0f));
	const Transform round = frames[1] * NodeFromBody(parsed.rig, 2.0f)[1];
	CHECK(round.translation.z == Approx(0.0f).margin(1e-5f));
	CHECK(frames[0].translation.z == 0.0f);
}

TEST_CASE("A grab point in the node frame lands on the same world point through the body frame", "[smpphysics][rigid_rig]")
{
	const ParseResult parsed = ParseSmpXml(kComRigXml);
	REQUIRE(parsed.status == ParseStatus::Ok);
	const std::vector<Transform> nodes(parsed.rig.bones.size());
	const std::vector<Transform> frames = BodyFrames(parsed.rig, nodes, 1.0f);
	const std::vector<Transform> inverses = NodeFromBody(parsed.rig, 1.0f);
	const Vec3 point{ 3.0f, -2.0f, 5.0f };
	const Vec3 viaBody = frames[1].TransformPoint(NodePointToBody(inverses, 1, point));
	CHECK(viaBody.x == Approx(point.x).margin(1e-4f));
	CHECK(viaBody.y == Approx(point.y).margin(1e-4f));
	CHECK(viaBody.z == Approx(point.z).margin(1e-4f));
	CHECK(NodePointToBody({}, 1, point).z == point.z);
}

TEST_CASE("A centerOfMassTransform rotation too small for the old check still keeps its inverse", "[smpphysics][rigid_rig]")
{
	ParseResult parsed = ParseSmpXml(kRigXml);
	REQUIRE(parsed.status == ParseStatus::Ok);
	REQUIRE(NodeFromBody(parsed.rig, 1.0f).empty());
	parsed.rig.bones.back().comTransform.rotation = Quat{ 0.9999995f, 0.001f, 0.0f, 0.0f };
	CHECK_FALSE(NodeFromBody(parsed.rig, 1.0f).empty());
}
