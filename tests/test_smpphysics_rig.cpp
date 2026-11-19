// Unit tests for the hdtSMP64 XML subset reader and rig builder
// (src/smp/SmpRig.cpp). Pure string parsing; no game/D3D dependency.

#include "smp/SmpRig.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstddef>
#include <string>
#include <string_view>

using namespace SmpPhysics;
using Catch::Approx;

namespace
{
	constexpr std::string_view kHairRigXml = R"XML(<?xml version="1.0" encoding="utf-8"?>
<!-- trimmed hdtSMP64 hair rig: one kinematic head and a two-bone pendulum chain -->
<system xmlns="http://www.example.com/hdt" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:schemaLocation="hdt.xsd">
	<bone name="NPC Head [Head]">
		<mass>0</mass>
	</bone>
	<bone-default name="hair-default">
		<mass>0.01</mass>
		<inertia x="1" y="1" z="1"/>
		<angularDamping>0.85</angularDamping>
		<gravity-factor>0.75</gravity-factor>
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

	std::string WrapSystem(std::string_view a_body)
	{
		std::string xml = "<?xml version=\"1.0\"?>\n<system xmlns=\"http://www.example.com/hdt\">\n";
		xml.append(a_body);
		xml.append("\n</system>\n");
		return xml;
	}

	int FindBone(const SmpRig& a_rig, std::string_view a_name)
	{
		for (std::size_t index = 0; index < a_rig.bones.size(); ++index) {
			if (a_rig.bones[index].name == a_name)
				return static_cast<int>(index);
		}
		return -1;
	}

	bool NamesElement(const ParseResult& a_result, std::string_view a_name)
	{
		return a_result.status == ParseStatus::Unsupported && a_result.reason.find(a_name) != std::string::npos;
	}
}

TEST_CASE("A full SMP hair rig parses into bones, joints and their parameters", "[SmpPhysics]")
{
	const ParseResult result = ParseSmpXml(kHairRigXml);

	REQUIRE(result.status == ParseStatus::Ok);
	REQUIRE(result.reason.empty());
	REQUIRE(result.rig.bones.size() == 3);
	REQUIRE(result.rig.joints.size() == 2);

	REQUIRE(result.rig.bones[0].name == "NPC Head [Head]");
	REQUIRE(result.rig.bones[0].kinematic);
	REQUIRE(result.rig.bones[0].mass == Approx(0.0f));
	REQUIRE(result.rig.bones[0].gravityFactor == Approx(1.0f));

	REQUIRE(result.rig.bones[1].name == "Hair01");
	REQUIRE_FALSE(result.rig.bones[1].kinematic);
	REQUIRE(result.rig.bones[1].mass == Approx(0.01f));
	REQUIRE(result.rig.bones[1].inertia.x == Approx(1.0f));
	REQUIRE(result.rig.bones[1].inertia.y == Approx(1.0f));
	REQUIRE(result.rig.bones[1].inertia.z == Approx(1.0f));
	REQUIRE(result.rig.bones[1].angularDamping == Approx(0.85f));
	REQUIRE(result.rig.bones[1].gravityFactor == Approx(0.75f));

	REQUIRE(result.rig.joints[0].child == 1);
	REQUIRE(result.rig.joints[0].parent == 0);
	REQUIRE(result.rig.joints[1].child == 2);
	REQUIRE(result.rig.joints[1].parent == 1);

	REQUIRE(result.rig.joints[0].frameKind == FrameKind::FrameInB);
	REQUIRE(result.rig.joints[0].frame.rotation.w == Approx(1.0f));
	REQUIRE(result.rig.joints[0].frame.translation.z == Approx(4.0f));
	REQUIRE(result.rig.joints[0].linearLower.x == Approx(0.0f));
	REQUIRE(result.rig.joints[0].linearUpper.z == Approx(0.0f));
	REQUIRE(result.rig.joints[0].angularLower.x == Approx(-0.6f));
	REQUIRE(result.rig.joints[0].angularLower.y == Approx(-0.5f));
	REQUIRE(result.rig.joints[0].angularUpper.z == Approx(0.4f));
	REQUIRE(result.rig.joints[0].angularStiffness.x == Approx(3.0f));
	REQUIRE(result.rig.joints[0].angularStiffness.z == Approx(5.0f));
	REQUIRE(result.rig.joints[0].angularDamping.y == Approx(0.2f));
	REQUIRE(result.rig.joints[0].angularEquilibrium.x == Approx(0.05f));
	REQUIRE(result.rig.joints[0].angularEquilibrium.z == Approx(-0.05f));
	REQUIRE(result.rig.joints[0].angularSpringsEnabled);
}

TEST_CASE("Successive unnamed bone defaults accumulate", "[SmpPhysics]")
{
	const ParseResult result = ParseSmpXml(WrapSystem(R"XML(
		<bone-default><angularDamping>0.25</angularDamping></bone-default>
		<bone-default><mass>0.02</mass></bone-default>
		<bone name="Head"><mass>0</mass></bone>
		<bone name="Hair01"/>
		<generic-constraint bodyA="Hair01" bodyB="Head">
			<linearLowerLimit x="0" y="0" z="0"/>
			<linearUpperLimit x="0" y="0" z="0"/>
		</generic-constraint>
	)XML"));

	REQUIRE(result.status == ParseStatus::Ok);
	const int hair = FindBone(result.rig, "Hair01");
	REQUIRE(hair >= 0);
	REQUIRE(result.rig.bones[static_cast<std::size_t>(hair)].mass == Approx(0.02f));
	REQUIRE(result.rig.bones[static_cast<std::size_t>(hair)].angularDamping == Approx(0.25f));
}

TEST_CASE("Empty numbers are reported; an element keeps its default and a vector axis reads as 0", "[SmpPhysics]")
{
	const ParseResult result = ParseSmpXml(WrapSystem(R"XML(
		<bone name="Head"><mass>0</mass></bone>
		<bone name="Hair01"><mass>0.01</mass><linearDamping></linearDamping></bone>
		<generic-constraint bodyA="Hair01" bodyB="Head">
			<linearLowerLimit x="" y="0" z="0"/>
			<linearUpperLimit x="0" y="0" z="0"/>
		</generic-constraint>
	)XML"));

	REQUIRE(result.status == ParseStatus::Ok);
	REQUIRE(result.warnings.size() == 2);
	REQUIRE(result.rig.joints.size() == 1);
	REQUIRE(result.rig.joints[0].linearLower.x == Approx(0.0f));
}

TEST_CASE("A constraint joining a bone to itself is dropped and reported", "[SmpPhysics]")
{
	const ParseResult result = ParseSmpXml(WrapSystem(R"XML(
		<bone name="Head"><mass>0</mass></bone>
		<bone name="Hair01"><mass>0.01</mass></bone>
		<generic-constraint bodyA="Hair01" bodyB="Hair01"/>
		<generic-constraint bodyA="Hair01" bodyB="Head"/>
	)XML"));

	REQUIRE(result.status == ParseStatus::Ok);
	REQUIRE(result.warnings.size() == 1);
	REQUIRE(result.rig.joints.size() == 1);
}

TEST_CASE("A non-numeric value still fails the file", "[SmpPhysics]")
{
	const ParseResult result = ParseSmpXml(WrapSystem(R"XML(
		<bone name="Head"><mass>heavy</mass></bone>
	)XML"));

	REQUIRE(result.status != ParseStatus::Ok);
}

TEST_CASE("Joint defaults extend by name and inherit the rest", "[SmpPhysics]")
{
	const ParseResult result = ParseSmpXml(WrapSystem(R"XML(
		<bone name="Head"><mass>0</mass></bone>
		<bone name="Hair01"><mass>0.01</mass></bone>
		<generic-constraint-default name="base">
			<linearLowerLimit x="0" y="0" z="0"/>
			<linearUpperLimit x="0" y="0" z="0"/>
			<angularStiffness x="1" y="1" z="1"/>
		</generic-constraint-default>
		<generic-constraint-default name="stiff" extends="base">
			<angularStiffness x="9" y="9" z="9"/>
			<angularDamping x="0.2" y="0.2" z="0.2"/>
		</generic-constraint-default>
		<generic-constraint bodyA="Hair01" bodyB="Head" template="stiff"/>
	)XML"));

	REQUIRE(result.status == ParseStatus::Ok);
	REQUIRE(result.rig.joints.size() == 1);
	REQUIRE(result.rig.joints[0].linearLower.z == Approx(0.0f));
	REQUIRE(result.rig.joints[0].linearUpper.x == Approx(0.0f));
	REQUIRE(result.rig.joints[0].angularStiffness.x == Approx(9.0f));
	REQUIRE(result.rig.joints[0].angularDamping.x == Approx(0.2f));
	REQUIRE(result.rig.joints[0].angularDamping.z == Approx(0.2f));
}

TEST_CASE("A joint template naming an unknown default falls back to the current default", "[SmpPhysics]")
{
	const ParseResult result = ParseSmpXml(WrapSystem(R"XML(
		<bone name="Head"><mass>0</mass></bone>
		<bone name="Hair01"><mass>0.01</mass></bone>
		<generic-constraint-default>
			<linearLowerLimit x="0" y="0" z="0"/>
			<linearUpperLimit x="0" y="0" z="0"/>
			<angularStiffness x="7" y="7" z="7"/>
		</generic-constraint-default>
		<generic-constraint bodyA="Hair01" bodyB="Head" template="missing-template"/>
	)XML"));

	REQUIRE(result.status == ParseStatus::Ok);
	REQUIRE(result.rig.joints.size() == 1);
	REQUIRE(result.rig.joints[0].linearLower.x == Approx(0.0f));
	REQUIRE(result.rig.joints[0].angularStiffness.x == Approx(7.0f));
}

TEST_CASE("A constraint group is flattened into the top level", "[SmpPhysics]")
{
	const ParseResult result = ParseSmpXml(kHairRigXml);

	REQUIRE(result.status == ParseStatus::Ok);
	REQUIRE(result.rig.joints.size() == 2);
	REQUIRE(result.rig.joints[1].child == 2);
}

TEST_CASE("A duplicate bone name is skipped", "[SmpPhysics]")
{
	const ParseResult result = ParseSmpXml(WrapSystem(R"XML(
		<bone name="Head"><mass>0</mass></bone>
		<bone name="Hair01"><mass>0.01</mass></bone>
		<bone name="Hair01"><mass>0</mass></bone>
		<generic-constraint bodyA="Hair01" bodyB="Head">
			<linearLowerLimit x="0" y="0" z="0"/>
			<linearUpperLimit x="0" y="0" z="0"/>
		</generic-constraint>
	)XML"));

	REQUIRE(result.status == ParseStatus::Ok);
	REQUIRE(result.rig.bones.size() == 2);
	REQUIRE_FALSE(result.rig.bones[1].kinematic);
}

TEST_CASE("A constraint naming an unknown bone is skipped", "[SmpPhysics]")
{
	const ParseResult result = ParseSmpXml(WrapSystem(R"XML(
		<bone name="Head"><mass>0</mass></bone>
		<bone name="Hair01"><mass>0.01</mass></bone>
		<generic-constraint bodyA="Hair01" bodyB="Head">
			<linearLowerLimit x="0" y="0" z="0"/>
			<linearUpperLimit x="0" y="0" z="0"/>
		</generic-constraint>
		<generic-constraint bodyA="Ghost" bodyB="Head">
			<linearLowerLimit x="0" y="0" z="0"/>
			<linearUpperLimit x="0" y="0" z="0"/>
		</generic-constraint>
	)XML"));

	REQUIRE(result.status == ParseStatus::Ok);
	REQUIRE(result.rig.joints.size() == 1);
	REQUIRE(result.rig.joints[0].child == FindBone(result.rig, "Hair01"));
}

TEST_CASE("Elements outside the simulated subset are unsupported", "[SmpPhysics]")
{
	SECTION("a collision shape inside a bone")
	{
		REQUIRE(NamesElement(ParseSmpXml(WrapSystem(R"XML(
			<bone name="Head"><mass>0</mass></bone>
			<bone name="Hair01"><mass>0.01</mass><shape name="Capsule" /></bone>
		)XML")),
			"shape"));
	}

	SECTION("a stiff spring constraint")
	{
		REQUIRE(NamesElement(ParseSmpXml(WrapSystem("<stiffspring-constraint bodyA=\"A\" bodyB=\"B\" />")),
			"stiffspring-constraint"));
	}

	SECTION("a pattern, which SMP does not know either, is skipped as SMP skips it")
	{
		REQUIRE(ParseSmpXml(WrapSystem("<pattern>hair</pattern>")).status == ParseStatus::Ok);
	}

	SECTION("an element nothing here knows is skipped as SMP skips it")
	{
		REQUIRE(ParseSmpXml(WrapSystem("<frobnicate />")).status == ParseStatus::Ok);
	}
}

TEST_CASE("A shape reads a decimal comma and the correctly spelled penetration like hdtSMP64", "[SmpPhysics]")
{
	const ParseResult result = ParseSmpXml(WrapSystem(R"XML(
		<per-triangle-shape name="VirtualButt">
			<margin>0,1</margin>
			<penetration>0,5</penetration>
		</per-triangle-shape>
		<bone name="Head"><mass>0</mass></bone>
		<bone name="Hair01"><mass>0.1</mass></bone>
		<generic-constraint bodyA="Hair01" bodyB="Head">
			<linearLowerLimit x="0" y="0" z="0"/>
			<linearUpperLimit x="0" y="0" z="0"/>
		</generic-constraint>
	)XML"));

	REQUIRE(result.status == ParseStatus::Ok);
	REQUIRE(result.rig.collision.triangleShapes.size() == 1);
	REQUIRE(result.rig.collision.triangleShapes[0].margin == Approx(0.1f));
	REQUIRE(result.rig.collision.triangleShapes[0].penetration == Approx(0.5f));
}

// Shaped from the heavy hair XMLs under Xing Hairs: a per-vertex hair shape, a per-triangle head and
// ground target, a frameInLerp joint template, and a ladder of cross-strand constraints.
TEST_CASE("A heavy hair rig's collision shapes parse into the rig", "[SmpPhysics]")
{
	const ParseResult result = ParseSmpXml(WrapSystem(R"XML(
		<per-triangle-shape name="VirtualGround">
			<margin>1</margin>
			<prenetration>4</prenetration>
			<shared>private</shared>
			<no-collide-with-tag>Xingcb</no-collide-with-tag>
			<can-collide-with-tag>XingHair</can-collide-with-tag>
		</per-triangle-shape>
		<per-triangle-shape name="collision head">
			<margin>0.2</margin>
			<priority>0</priority>
			<prenetration>0.1</prenetration>
			<shared>private</shared>
			<tag>Xingch</tag>
			<can-collide-with-tag>XingHair</can-collide-with-tag>
			<no-collide-with-tag>ground</no-collide-with-tag>
			<weight-threshold bone="NPC Spine2 [Spn2]">0.3</weight-threshold>
			<weight-threshold bone="NPC Head [Head]">0.5</weight-threshold>
		</per-triangle-shape>
		<per-vertex-shape name="XingHair">
			<margin>0.01</margin>
			<priority>1</priority>
			<shared>Internal</shared>
			<tag>XingHair</tag>
			<no-collide-with-tag>XingHair</no-collide-with-tag>
			<can-collide-with-tag>Xingch</can-collide-with-tag>
			<can-collide-with-tag>ground</can-collide-with-tag>
			<no-collide-with-bone>NPC Head [Head]</no-collide-with-bone>
			<weight-threshold bone="Hair01">0.5</weight-threshold>
			<disable-tag>XingHair</disable-tag>
		</per-vertex-shape>
		<bone name="Head"><mass>0</mass></bone>
		<bone name="Hair01"><mass>0.1</mass></bone>
		<generic-constraint bodyA="Hair01" bodyB="Head">
			<linearLowerLimit x="0" y="0" z="0"/>
			<linearUpperLimit x="0" y="0" z="0"/>
		</generic-constraint>
	)XML"));

	REQUIRE(result.status == ParseStatus::Ok);
	REQUIRE(result.rig.collision.triangleShapes.size() == 2);
	REQUIRE(result.rig.collision.vertexShapes.size() == 1);

	const RigShape& ground = result.rig.collision.triangleShapes[0];
	REQUIRE(ground.name == "VirtualGround");
	REQUIRE(ground.margin == Approx(1.0f));
	REQUIRE(ground.penetration == Approx(4.0f));
	REQUIRE(ground.shared == "private");
	REQUIRE(ground.tag.empty());
	REQUIRE(ground.collidesWith == std::vector<std::string>{ "XingHair" });
	REQUIRE(ground.noCollidesWith == std::vector<std::string>{ "Xingcb" });

	const RigShape& head = result.rig.collision.triangleShapes[1];
	REQUIRE(head.name == "collision head");
	REQUIRE(head.margin == Approx(0.2f));
	REQUIRE(head.penetration == Approx(0.1f));
	REQUIRE(head.priority == 0);
	REQUIRE(head.tag == "Xingch");
	REQUIRE(head.weights.size() == 2);
	REQUIRE(head.weights[0].bone == "NPC Spine2 [Spn2]");
	REQUIRE(head.weights[0].threshold == Approx(0.3f));
	REQUIRE(head.weights[1].bone == "NPC Head [Head]");
	REQUIRE(head.weights[1].threshold == Approx(0.5f));

	const RigShape& hair = result.rig.collision.vertexShapes[0];
	REQUIRE(hair.name == "XingHair");
	REQUIRE(hair.margin == Approx(0.01f));
	REQUIRE(hair.penetration == Approx(0.0f));
	REQUIRE(hair.priority == 1);
	REQUIRE(hair.shared == "Internal");
	REQUIRE(hair.collidesWith == (std::vector<std::string>{ "Xingch", "ground" }));
	REQUIRE(hair.noCollidesWithBones == std::vector<std::string>{ "NPC Head [Head]" });
	REQUIRE(hair.weights.size() == 1);
}

TEST_CASE("A collision shape's name loses its surrounding whitespace", "[SmpPhysics]")
{
	const ParseResult result = ParseSmpXml(WrapSystem(R"XML(
		<per-vertex-shape name=" FlyingMinaj145 ">
			<margin>0.01</margin>
		</per-vertex-shape>
		<per-triangle-shape name="	Collision Head	">
			<margin>0.2</margin>
		</per-triangle-shape>
		<bone name="Head"><mass>0</mass></bone>
		<bone name="Hair01"><mass>0.1</mass></bone>
		<generic-constraint bodyA="Hair01" bodyB="Head">
			<linearLowerLimit x="0" y="0" z="0"/>
			<linearUpperLimit x="0" y="0" z="0"/>
		</generic-constraint>
	)XML"));

	REQUIRE(result.status == ParseStatus::Ok);
	REQUIRE(result.rig.collision.vertexShapes.size() == 1);
	REQUIRE(result.rig.collision.triangleShapes.size() == 1);
	REQUIRE(result.rig.collision.vertexShapes[0].name == "FlyingMinaj145");
	REQUIRE(result.rig.collision.triangleShapes[0].name == "Collision Head");
}

TEST_CASE("A frameInLerp joint blends both bodies' rest frames", "[SmpPhysics]")
{
	const ParseResult result = ParseSmpXml(WrapSystem(R"XML(
		<bone name="Head"><mass>0</mass></bone>
		<bone name="Hair01"><mass>0.1</mass></bone>
		<bone name="Hair02"><mass>0.1</mass></bone>
		<generic-constraint-default>
			<frameInLerp>
				<translationLerp>0</translationLerp>
				<rotationLerp>0</rotationLerp>
			</frameInLerp>
			<linearLowerLimit x="0" y="0" z="0"/>
			<linearUpperLimit x="0" y="0" z="0"/>
			<angularLowerLimit x="-9" y="-9" z="-9"/>
			<angularUpperLimit x="9" y="9" z="9"/>
		</generic-constraint-default>
		<generic-constraint bodyA="Hair01" bodyB="Head"/>
		<generic-constraint bodyA="Hair02" bodyB="Hair01"/>
	)XML"));

	REQUIRE(result.status == ParseStatus::Ok);
	REQUIRE(result.rig.joints.size() == 2);
	REQUIRE(result.rig.joints[0].frameKind == FrameKind::FrameInLerp);
	REQUIRE(result.rig.joints[0].translationLerp == Approx(0.0f));
	REQUIRE(result.rig.joints[0].rotationLerp == Approx(0.0f));
}

TEST_CASE("A heavy template's bounded linear box is kept", "[SmpPhysics]")
{
	// The heavy hair XMLs give their cross-strand joints a (-2,-2,-4) to (5,5,4) box that SMP stretches within.
	const ParseResult result = ParseSmpXml(WrapSystem(R"XML(
		<bone name="Head"><mass>0</mass></bone>
		<bone name="Hair01"><mass>0.1</mass></bone>
		<generic-constraint-default>
			<linearLowerLimit x="-2" y="-2" z="-4"/>
			<linearUpperLimit x="5" y="5" z="4"/>
		</generic-constraint-default>
		<generic-constraint bodyA="Hair01" bodyB="Head"/>
	)XML"));

	REQUIRE(result.status == ParseStatus::Ok);
	REQUIRE(result.rig.joints.size() == 1);
	CHECK(result.rig.joints[0].linearLower.z == Approx(-4.0f));
	CHECK(result.rig.joints[0].linearUpper.x == Approx(5.0f));
}

TEST_CASE("An unknown element inside frameInLerp is caught rather than ignored", "[SmpPhysics]")
{
	const ParseResult result = ParseSmpXml(WrapSystem(R"XML(
		<bone name="Head"><mass>0</mass></bone>
		<bone name="Hair01"><mass>0.1</mass></bone>
		<generic-constraint bodyA="Hair01" bodyB="Head">
			<frameInLerp>
				<translationLerp>0</translationLerp>
				<twistLerp>1</twistLerp>
			</frameInLerp>
		</generic-constraint>
	)XML"));

	REQUIRE(result.status == ParseStatus::Malformed);
	REQUIRE(result.reason.find("twistLerp") != std::string::npos);
}

TEST_CASE("A bone pinned to two parents keeps the first joint and records the rest", "[SmpPhysics]")
{
	const ParseResult result = ParseSmpXml(WrapSystem(R"XML(
		<bone name="Head"><mass>0</mass></bone>
		<bone name="Hair01"><mass>0.1</mass></bone>
		<bone name="Hair02"><mass>0.1</mass></bone>
		<constraint-group>
			<generic-constraint bodyA="Hair01" bodyB="Head">
				<linearLowerLimit x="0" y="0" z="0"/>
				<linearUpperLimit x="0" y="0" z="0"/>
			</generic-constraint>
		</constraint-group>
		<constraint-group>
			<generic-constraint bodyA="Hair01" bodyB="Head">
				<linearLowerLimit x="0" y="0" z="0"/>
				<linearUpperLimit x="0" y="0" z="0"/>
			</generic-constraint>
			<generic-constraint bodyA="Hair02" bodyB="Hair01">
				<linearLowerLimit x="0" y="0" z="0"/>
				<linearUpperLimit x="0" y="0" z="0"/>
			</generic-constraint>
		</constraint-group>
	)XML"));

	// SMP pins one strand's root to both the head and its neighbour; a single pivot per bone cannot express
	// that, so the rig keeps the first joint and the extra one is recorded rather than dropped.
	REQUIRE(result.status == ParseStatus::Ok);
	REQUIRE(result.rig.joints.size() == 2);
	REQUIRE(result.rig.joints[0].child == FindBone(result.rig, "Hair01"));
	REQUIRE(result.rig.additionalJoints.size() == 1);
	REQUIRE(result.rig.additionalJoints[0].child == FindBone(result.rig, "Hair01"));
	REQUIRE(result.rig.additionalJoints[0].parent == FindBone(result.rig, "Head"));
}

TEST_CASE("An unknown element of a collision shape is caught rather than ignored", "[SmpPhysics]")
{
	const ParseResult result = ParseSmpXml(WrapSystem(R"XML(
		<per-vertex-shape name="Hair">
			<margin>0.01</margin>
			<wobble>3</wobble>
		</per-vertex-shape>
	)XML"));

	REQUIRE(NamesElement(result, "wobble"));
}

TEST_CASE("A joint that lets its bone slide is accepted", "[SmpPhysics]")
{
	const ParseResult result = ParseSmpXml(WrapSystem(R"XML(
		<bone name="Head"><mass>0</mass></bone>
		<bone name="Hair01"><mass>0.01</mass></bone>
		<bone name="Hair02"><mass>0.01</mass></bone>
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
		<generic-constraint bodyA="Hair02" bodyB="Hair01">
			<linearLowerLimit x="0" y="0" z="0"/>
			<linearUpperLimit x="0" y="0" z="0"/>
		</generic-constraint>
	)XML"));

	REQUIRE(result.status == ParseStatus::Ok);
	REQUIRE(result.rig.joints.size() == 2);
	const RigJoint& sliding = result.rig.joints[0];
	CHECK(sliding.linearLower.y == Approx(-2.0f));
	CHECK(sliding.linearUpper.z == Approx(3.0f));
	CHECK(sliding.linearStiffness.z == Approx(6.0f));
	CHECK(sliding.linearDamping.y == Approx(0.2f));
	CHECK(sliding.linearEquilibrium.z == Approx(-0.5f));
	CHECK(sliding.linearBounce.x == Approx(0.25f));
	CHECK_FALSE(sliding.linearStiffnessLimited);
	CHECK_FALSE(sliding.linearSpringsEnabled);
}

TEST_CASE("A joint that leaves a linear axis unset is free", "[SmpPhysics]")
{
	const ParseResult result = ParseSmpXml(WrapSystem(R"XML(
		<bone name="Head"><mass>0</mass></bone>
		<bone name="Hair01"><mass>0.01</mass></bone>
		<generic-constraint bodyA="Hair01" bodyB="Head"/>
	)XML"));

	REQUIRE(result.status == ParseStatus::Ok);
	CHECK(result.rig.joints[0].linearLower.x > result.rig.joints[0].linearUpper.x);
}

TEST_CASE("Rigs the solver cannot drive fail validation or lose the bones it cannot hold", "[SmpPhysics]")
{
	SECTION("a dynamic bone with no joint is dropped as free-falling")
	{
		const ParseResult result = ParseSmpXml(WrapSystem(R"XML(
			<bone name="Head"><mass>0</mass></bone>
			<bone name="Hair01"><mass>0.01</mass></bone>
		)XML"));
		REQUIRE(result.status == ParseStatus::Ok);
		std::string reason;
		CHECK_FALSE(ValidateRig(result.rig, reason));
		REQUIRE(reason.find("Hair01") != std::string::npos);

		SmpRig rig = result.rig;
		REQUIRE(NormalizeRigTopology(rig, reason));
		CHECK(rig.bones.size() == 1);
	}

	SECTION("a cyclic joint chain")
	{
		const ParseResult result = ParseSmpXml(WrapSystem(R"XML(
			<bone name="Head"><mass>0</mass></bone>
			<bone name="Hair01"><mass>0.01</mass></bone>
			<bone name="Hair02"><mass>0.01</mass></bone>
			<generic-constraint bodyA="Hair01" bodyB="Hair02">
				<linearLowerLimit x="0" y="0" z="0"/>
				<linearUpperLimit x="0" y="0" z="0"/>
			</generic-constraint>
			<generic-constraint bodyA="Hair02" bodyB="Hair01">
				<linearLowerLimit x="0" y="0" z="0"/>
				<linearUpperLimit x="0" y="0" z="0"/>
			</generic-constraint>
		)XML"));
		REQUIRE(result.status == ParseStatus::Ok);
		std::string reason;
		REQUIRE_FALSE(ValidateRig(result.rig, reason));
		REQUIRE(reason.find("cyclic") != std::string::npos);
	}
}

TEST_CASE("XML the reader cannot accept is malformed", "[SmpPhysics]")
{
	SECTION("a truncated document")
	{
		REQUIRE(ParseSmpXml("<?xml version=\"1.0\"?><system><bone name=\"Head\">").status == ParseStatus::Malformed);
	}

	SECTION("a mismatched closing tag")
	{
		REQUIRE(ParseSmpXml(WrapSystem("<bone></joint>")).status == ParseStatus::Malformed);
	}

	SECTION("an unterminated attribute")
	{
		REQUIRE(ParseSmpXml("<?xml version=\"1.0\"?><system><bone name=\"Head></bone></system>").status ==
				ParseStatus::Malformed);
	}

	SECTION("an unterminated comment")
	{
		REQUIRE(ParseSmpXml("<?xml version=\"1.0\"?><!-- no end <system/>").status == ParseStatus::Malformed);
	}

	SECTION("a root element that is not system")
	{
		const ParseResult result = ParseSmpXml("<?xml version=\"1.0\"?><bones/>");
		REQUIRE(result.status == ParseStatus::Malformed);
		REQUIRE(result.reason.find("system") != std::string::npos);
	}

	SECTION("a DOCTYPE declaration")
	{
		REQUIRE(ParseSmpXml("<?xml version=\"1.0\"?><!DOCTYPE system><system/>").status == ParseStatus::Malformed);
	}

	SECTION("a CDATA section")
	{
		REQUIRE(ParseSmpXml(WrapSystem("<bone><![CDATA[x]]></bone>")).status == ParseStatus::Malformed);
	}

	SECTION("text where a number belongs")
	{
		REQUIRE(ParseSmpXml(WrapSystem("<bone name=\"Head\"><mass>heavy</mass></bone>")).status ==
				ParseStatus::Malformed);
	}

	SECTION("a NaN number")
	{
		REQUIRE(ParseSmpXml(WrapSystem("<bone name=\"Head\"><mass>nan</mass></bone>")).status ==
				ParseStatus::Malformed);
	}

	SECTION("an infinity")
	{
		REQUIRE(ParseSmpXml(WrapSystem("<bone name=\"Head\"><mass>inf</mass></bone>")).status ==
				ParseStatus::Malformed);
	}

	SECTION("a vector missing an axis attribute")
	{
		REQUIRE(ParseSmpXml(WrapSystem(R"XML(
			<bone name="Head"><mass>0</mass></bone>
			<bone name="Hair01"><mass>0.01</mass></bone>
			<generic-constraint bodyA="Hair01" bodyB="Head">
				<linearLowerLimit x="0" y="0"/>
				<linearUpperLimit x="0" y="0" z="0"/>
			</generic-constraint>
		)XML"))
					.status == ParseStatus::Malformed);
	}

	SECTION("a zero quaternion basis")
	{
		REQUIRE(ParseSmpXml(WrapSystem(R"XML(
			<bone name="Head"><mass>0</mass></bone>
			<bone name="Hair01"><mass>0.01</mass></bone>
			<generic-constraint bodyA="Hair01" bodyB="Head">
				<frameInB><basis x="0" y="0" z="0" w="0"/></frameInB>
				<linearLowerLimit x="0" y="0" z="0"/>
				<linearUpperLimit x="0" y="0" z="0"/>
			</generic-constraint>
		)XML"))
					.status == ParseStatus::Malformed);
	}

	SECTION("input above the byte limit")
	{
		const std::string oversized(4u * 1024u * 1024u + 1u, ' ');
		REQUIRE(ParseSmpXml(oversized).status == ParseStatus::Malformed);
	}

	SECTION("nesting above the depth limit")
	{
		std::string xml = "<?xml version=\"1.0\"?><system>";
		for (int level = 0; level < 33; ++level)
			xml += "<a>";
		for (int level = 0; level < 33; ++level)
			xml += "</a>";
		xml += "</system>";
		REQUIRE(ParseSmpXml(xml).status == ParseStatus::Malformed);
	}

	SECTION("an empty document")
	{
		REQUIRE(ParseSmpXml("").status == ParseStatus::Malformed);
	}

	SECTION("arbitrary bytes")
	{
		const std::string bytes = std::string("\x01\x02\x03") + std::string("\xff\xfe<<<&&&=\"'");
		const ParseResult result = ParseSmpXml(bytes);
		REQUIRE(result.status == ParseStatus::Malformed);
		REQUIRE_FALSE(result.reason.empty());
	}
}

TEST_CASE("Entity references decode in attribute values", "[SmpPhysics]")
{
	const ParseResult result = ParseSmpXml(WrapSystem("<bone name=\"Head &amp; Neck\"><mass>0</mass></bone>"));

	REQUIRE(result.status == ParseStatus::Ok);
	REQUIRE(result.rig.bones.size() == 1);
	REQUIRE(result.rig.bones[0].name == "Head & Neck");
}

TEST_CASE("Limited joint springs are capped the way Bullet's spring constraint caps them", "[SmpPhysics]")
{
	SmpRig rig;
	rig.bones.resize(3);
	rig.bones[0].kinematic = true;
	for (std::size_t index = 1; index < 3; ++index) {
		rig.bones[index].mass = 0.1f;
		rig.bones[index].inertia = { 16.0f, 16.0f, 16.0f };
	}
	rig.joints.resize(2);
	rig.joints[0].child = 1;
	rig.joints[0].parent = 0;
	rig.joints[1].child = 2;
	rig.joints[1].parent = 1;
	for (RigJoint& joint : rig.joints) {
		joint.angularStiffness = { 44440.0f, 44440.0f, 44440.0f };
		joint.angularDamping = { 1.0e6f, 5.0f, 5.0f };
	}

	constexpr float kStep = 1.0f / 60.0f;
	ApplySpringLimits(rig, kStep);

	// A kinematic parent leaves the child's inertia alone; a dynamic parent halves it.
	REQUIRE(rig.joints[0].angularStiffness.x == Approx(16.0f / (16.0f * kStep * kStep)));
	REQUIRE(rig.joints[1].angularStiffness.y == Approx(8.0f / (16.0f * kStep * kStep)));
	REQUIRE(rig.joints[0].angularDamping.x == Approx(16.0f / kStep));
	REQUIRE(rig.joints[0].angularDamping.y == Approx(5.0f));

	// A stiffness already under the cap, and a joint that clears the flags, are left as they were.
	rig.joints[0].angularStiffness = { 100.0f, 100.0f, 100.0f };
	rig.joints[1].angularStiffness = { 44440.0f, 44440.0f, 44440.0f };
	rig.joints[1].angularStiffnessLimited = false;
	ApplySpringLimits(rig, kStep);
	REQUIRE(rig.joints[0].angularStiffness.x == Approx(100.0f));
	REQUIRE(rig.joints[1].angularStiffness.x == Approx(44440.0f));

	// A minimum damping ratio lifts a joint's damping to 2 * ratio * sqrt(stiffness * reduced inertia).
	rig.joints[0].angularStiffness = { 400.0f, 400.0f, 400.0f };
	rig.joints[0].angularDamping = { 0.0f, 1.0e5f, 0.0f };
	rig.joints[0].springDampingLimited = false;
	ApplySpringLimits(rig, kStep, 0.5f);
	REQUIRE(rig.joints[0].angularDamping.x == Approx(std::sqrt(400.0f * 16.0f)));
	REQUIRE(rig.joints[0].angularDamping.y == Approx(1.0e5f));
}

TEST_CASE("Shapes collide by the can-collide and no-collide tag lists of both sides", "[SmpPhysics]")
{
	RigShape cloth;
	cloth.tag = "cloth";
	RigShape body;
	body.tag = "body";

	SECTION("two shapes with empty lists collide")
	{
		REQUIRE(ShapesCanCollide(cloth, body));
	}
	SECTION("an empty can-collide list still honours the no-collide list")
	{
		cloth.noCollidesWith = { "body" };
		REQUIRE_FALSE(ShapesCanCollide(cloth, body));
		REQUIRE_FALSE(ShapesCanCollide(body, cloth));
	}
	SECTION("a non-empty can-collide list collides only with the tags it names")
	{
		cloth.collidesWith = { "ground" };
		REQUIRE_FALSE(ShapesCanCollide(cloth, body));
		cloth.collidesWith.push_back("body");
		REQUIRE(ShapesCanCollide(cloth, body));
	}
	SECTION("the other side can veto the pair")
	{
		body.collidesWith = { "ground" };
		REQUIRE_FALSE(ShapesCanCollide(cloth, body));
	}
}

namespace
{
	constexpr std::string_view kRingTipsXml = R"XML(<?xml version="1.0" encoding="utf-8"?>
<system xmlns="http://www.example.com/hdt">
	<bone name="Head"><mass>0</mass></bone>
	<bone name="Hang"><mass>0.1</mass></bone>
	<bone name="TipA"><mass>0.1</mass></bone>
	<bone name="TipB"><mass>0.1</mass></bone>
	<generic-constraint bodyA="Hang" bodyB="Head"/>
	<generic-constraint bodyA="TipA" bodyB="TipB"/>
</system>
)XML";

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
}

TEST_CASE("A group of dynamic bones no joint ties to a kinematic bone is dropped", "[smpphysics][rig]")
{
	const ParseResult parsed = ParseSmpXml(kRingTipsXml);
	REQUIRE(parsed.status == ParseStatus::Ok);
	std::string reason;
	CHECK_FALSE(ValidateRig(parsed.rig, reason));

	SmpRig rig = parsed.rig;
	REQUIRE(NormalizeRigTopology(rig, reason));
	REQUIRE(rig.bones.size() == 2);
	CHECK(rig.bones[0].name == "Head");
	CHECK(rig.bones[1].name == "Hang");
	CHECK(rig.joints.size() == 1);
	CHECK(ValidateRig(rig, reason));
}

TEST_CASE("A dynamic bone that only hangs other bones is held through them and re-rooted", "[smpphysics][rig]")
{
	const ParseResult parsed = ParseSmpXml(kHeldRootXml);
	REQUIRE(parsed.status == ParseStatus::Ok);
	std::string reason;
	CHECK_FALSE(ValidateRig(parsed.rig, reason));

	SmpRig rig = parsed.rig;
	REQUIRE(NormalizeRigTopology(rig, reason));
	CHECK(ValidateRig(rig, reason));
	CHECK(rig.bones.size() == 4);
	REQUIRE(rig.joints.size() == 3);

	const auto indexOf = [&rig](std::string_view a_name) {
		for (std::size_t index = 0; index < rig.bones.size(); ++index) {
			if (rig.bones[index].name == a_name)
				return static_cast<int>(index);
		}
		return -1;
	};
	const auto jointOf = [&rig](int a_child) -> const RigJoint& {
		for (const RigJoint& joint : rig.joints) {
			if (joint.child == a_child)
				return joint;
		}
		FAIL("no joint has the bone as its child");
		return rig.joints.front();
	};
	CHECK(jointOf(indexOf("Leaf")).parent == indexOf("Hang"));
	CHECK_FALSE(jointOf(indexOf("Leaf")).reversed);
	CHECK(jointOf(indexOf("Root")).parent == indexOf("Leaf"));
	CHECK(jointOf(indexOf("Root")).reversed);
}

TEST_CASE("Dropping a bone drops the joints that name it", "[smpphysics][rig]")
{
	const ParseResult parsed = ParseSmpXml(kHeldRootXml);
	REQUIRE(parsed.status == ParseStatus::Ok);
	std::vector<bool> drop(parsed.rig.bones.size(), false);
	drop[2] = true;
	const SmpRig kept = DropBones(parsed.rig, drop);
	CHECK(kept.bones.size() == 3);
	CHECK(kept.joints.size() == 1);
	CHECK(kept.additionalJoints.size() == 1);
}

TEST_CASE("A top-level element SMP does not know is skipped, one it knows but we do not simulate is declined", "[smpphysics][rig]")
{
	const ParseResult skipped = ParseSmpXml(WrapSystem(R"XML(
		<bone name="Head"><mass>0</mass></bone>
		<bone name="Hair01"><mass>0.01</mass></bone>
		<generic-constraint bodyA="Hair01" bodyB="Head"/>
		<weight-threshold bone="Head">0.3</weight-threshold>
	)XML"));
	CHECK(skipped.status == ParseStatus::Ok);

	const ParseResult declined = ParseSmpXml(WrapSystem(R"XML(
		<bone name="Head"><mass>0</mass></bone>
		<bone name="Hair01"><mass>0.01</mass></bone>
		<conetwist-constraint bodyA="Hair01" bodyB="Head"/>
	)XML"));
	CHECK(declined.status == ParseStatus::Unsupported);
}

TEST_CASE("A bone's surface mask follows the collide tags of its shape and spares the surface it was built from", "[SmpPhysics]")
{
	RigShape tassel;
	tassel.name = "Tassel";
	tassel.tag = "tassel";
	tassel.collidesWith = { "skirt1", "skirt2" };
	RigShape skirt1;
	skirt1.name = "Skirt1";
	skirt1.tag = "skirt1";
	skirt1.noCollidesWith = { "skirt1" };
	skirt1.collidesWith = { "skirt2", "tassel" };
	RigShape skirt2;
	skirt2.name = "Skirt2";
	skirt2.tag = "skirt2";
	skirt2.noCollidesWith = { "skirt2" };

	const std::vector<const RigShape*> surfaces = { &skirt1, &skirt2, nullptr, &tassel };
	CHECK(AllowedSurfaceMask(&tassel, surfaces) == 0b0111u);
	CHECK(AllowedSurfaceMask(&skirt1, surfaces) == 0b1110u);
	CHECK(AllowedSurfaceMask(&skirt2, surfaces) == 0b1101u);
	CHECK(AllowedSurfaceMask(nullptr, surfaces) == 0xFFFFFFFFu);
}

namespace
{
	std::string SpringRigXml(std::string_view a_flag)
	{
		std::string body = R"XML(
		<bone name="Head"><mass>0</mass></bone>
		<bone name="Hair01"><mass>0.01</mass><inertia x="1" y="1" z="1"/></bone>
		<generic-constraint bodyA="Hair01" bodyB="Head">
			<enableAngularSprings>)XML";
		body.append(a_flag);
		body.append("</enableAngularSprings></generic-constraint>");
		return WrapSystem(body);
	}
}

TEST_CASE("A boolean element reads 1 and 0 as hdtSMP64 does", "[SmpPhysics]")
{
	const ParseResult enabled = ParseSmpXml(SpringRigXml("1"));
	REQUIRE(enabled.status == ParseStatus::Ok);
	REQUIRE(enabled.rig.joints.size() == 1);
	CHECK(enabled.rig.joints[0].angularSpringsEnabled);

	const ParseResult disabled = ParseSmpXml(SpringRigXml("0"));
	REQUIRE(disabled.status == ParseStatus::Ok);
	CHECK_FALSE(disabled.rig.joints[0].angularSpringsEnabled);

	CHECK(ParseSmpXml(SpringRigXml("yes")).status != ParseStatus::Ok);
}

TEST_CASE("A UTF-8 byte order mark before the XML prolog is skipped", "[SmpPhysics]")
{
	const std::string bare = WrapSystem(R"XML(<bone name="Head"><mass>0</mass></bone>)XML");
	const ParseResult withMark = ParseSmpXml("\xEF\xBB\xBF" + bare);
	CHECK(withMark.status == ParseStatus::Ok);
	CHECK(withMark.rig.bones.size() == ParseSmpXml(bare).rig.bones.size());
}

TEST_CASE("wind-effect in a collision shape is accepted without a warning", "[SmpPhysics]")
{
	const ParseResult result = ParseSmpXml(WrapSystem(R"XML(
		<bone name="Head"><mass>0</mass></bone>
		<per-triangle-shape name="Body">
			<wind-effect>0</wind-effect>
			<margin>0.5</margin>
		</per-triangle-shape>
		<per-triangle-shape name="Head">
			<wind-effect>0</wind-effect>
		</per-triangle-shape>
	)XML"));

	REQUIRE(result.status == ParseStatus::Ok);
	REQUIRE(result.rig.collision.triangleShapes.size() == 2);
	CHECK(result.rig.collision.triangleShapes[0].margin == Approx(0.5f));
	CHECK(result.warnings.empty());
}

TEST_CASE("A shape element hdtSMP64 reads but this plugin does not model still declines the file", "[SmpPhysics]")
{
	const ParseResult result = ParseSmpXml(WrapSystem(R"XML(
		<bone name="Head"><mass>0</mass></bone>
		<per-triangle-shape name="Body">
			<can-collide-with-bone>Head</can-collide-with-bone>
		</per-triangle-shape>
	)XML"));

	CHECK(result.status == ParseStatus::Unsupported);
}
