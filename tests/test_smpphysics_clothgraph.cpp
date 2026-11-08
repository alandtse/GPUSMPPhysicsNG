// Unit tests for the rig-to-cloth-graph builder
// (src/smp/ClothGraph.cpp). Pure maths; no game/D3D dependency.

#include "smp/ClothGraph.h"
#include "smp/SmpRig.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

using namespace SmpPhysics;
using Catch::Approx;

namespace
{
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

	/** @brief One rest transform per bone: the given translation, with the rotation turned a little about Z. */
	std::vector<Transform> MakeRest(const SmpRig& a_rig, const std::vector<Vec3>& a_positions)
	{
		std::vector<Transform> rest(a_rig.bones.size());
		for (std::size_t index = 0; index < rest.size(); ++index) {
			rest[index].rotation = FromAxisAngle(Vec3{ 0.0f, 0.0f, 1.0f }, 0.2f * static_cast<float>(index));
			rest[index].translation = a_positions[index];
		}
		return rest;
	}

	std::size_t CountKind(const ClothGraph& a_graph, ClothConstraintKind a_kind)
	{
		std::size_t count = 0;
		for (const ClothConstraint& constraint : a_graph.constraints) {
			if (constraint.kind == a_kind)
				++count;
		}
		return count;
	}

	void RequireNoBatchSharesParticle(const ClothGraph& a_graph)
	{
		for (const ClothConstraint& constraint : a_graph.constraints)
			REQUIRE(constraint.batch < a_graph.batchCount);
		for (std::size_t first = 0; first < a_graph.constraints.size(); ++first) {
			for (std::size_t second = first + 1; second < a_graph.constraints.size(); ++second) {
				const ClothConstraint& left = a_graph.constraints[first];
				const ClothConstraint& right = a_graph.constraints[second];
				if (left.batch != right.batch)
					continue;
				REQUIRE(left.first != right.first);
				REQUIRE(left.first != right.second);
				REQUIRE(left.second != right.first);
				REQUIRE(left.second != right.second);
			}
		}
	}

	constexpr std::string_view kLockedLimits = R"XML(
		<generic-constraint-default name="locked">
			<linearLowerLimit x="0" y="0" z="0"/>
			<linearUpperLimit x="0" y="0" z="0"/>
		</generic-constraint-default>)XML";

	constexpr std::string_view kLateralLimits = R"XML(
		<generic-constraint-default name="lateral">
			<linearLowerLimit x="-0.1" y="-0.15" z="-0.2"/>
			<linearUpperLimit x="0.1" y="0.15" z="0.2"/>
		</generic-constraint-default>)XML";

	std::string ChainXml()
	{
		std::string body = R"XML(
	<bone name="Head"><mass>0</mass></bone>
	<bone name="Hair01"><mass>0.01</mass></bone>
	<bone name="Hair02"><mass>0.02</mass></bone>
	<bone name="Hair03"><mass>0.04</mass></bone>)XML";
		body.append(kLockedLimits);
		body.append(R"XML(
	<generic-constraint bodyA="Hair01" bodyB="Head" template="locked"/>
	<generic-constraint bodyA="Hair02" bodyB="Hair01" template="locked"/>
	<generic-constraint bodyA="Hair03" bodyB="Hair02" template="locked"/>)XML");
		return WrapSystem(body);
	}

	std::string LatticeXml(std::string_view a_extraConstraint = {})
	{
		std::string body = R"XML(
	<bone name="Head"><mass>0</mass></bone>
	<bone name="A1"><mass>0.01</mass></bone>
	<bone name="A2"><mass>0.02</mass></bone>
	<bone name="A3"><mass>0.03</mass></bone>
	<bone name="B1"><mass>0.04</mass></bone>
	<bone name="B2"><mass>0.05</mass></bone>
	<bone name="B3"><mass>0.06</mass></bone>)XML";
		body.append(kLockedLimits);
		body.append(kLateralLimits);
		body.append(R"XML(
	<generic-constraint bodyA="A1" bodyB="Head" template="locked"/>
	<generic-constraint bodyA="A2" bodyB="A1" template="locked"/>
	<generic-constraint bodyA="A3" bodyB="A2" template="locked"/>
	<generic-constraint bodyA="B1" bodyB="Head" template="locked"/>
	<generic-constraint bodyA="B2" bodyB="B1" template="locked"/>
	<generic-constraint bodyA="B3" bodyB="B2" template="locked"/>
	<generic-constraint bodyA="A1" bodyB="B1" template="lateral"/>
	<generic-constraint bodyA="A2" bodyB="B2" template="lateral"/>
	<generic-constraint bodyA="A3" bodyB="B3" template="lateral"/>)XML");
		body.append(a_extraConstraint);
		return WrapSystem(body);
	}

	std::vector<Transform> LatticeRest(const SmpRig& a_rig)
	{
		return MakeRest(a_rig,
			{ Vec3{ 0.0f, 0.0f, 0.0f }, Vec3{ -1.0f, 3.0f, 0.0f }, Vec3{ -1.0f, 6.0f, 0.0f },
				Vec3{ -1.0f, 9.0f, 0.0f }, Vec3{ 1.0f, 3.0f, 0.0f }, Vec3{ 1.0f, 6.0f, 0.0f },
				Vec3{ 1.0f, 9.0f, 0.0f } });
	}
}

TEST_CASE("A plain chain builds only chain and bend constraints", "[SmpPhysics]")
{
	const ParseResult result = ParseSmpXml(ChainXml());
	REQUIRE(result.status == ParseStatus::Ok);

	const std::vector<Transform> rest = MakeRest(result.rig,
		{ Vec3{ 0.0f, 0.0f, 0.0f }, Vec3{ 0.0f, 0.0f, -4.0f }, Vec3{ 0.0f, 0.0f, -8.0f }, Vec3{ 0.0f, 0.0f, -12.0f } });
	const ClothGraph graph = BuildClothGraph(result.rig, rest);

	REQUIRE_FALSE(IsLatticeRig(result.rig));
	REQUIRE(graph.particles.size() == 4);
	REQUIRE(graph.particles[0].bone == 0u);
	REQUIRE(graph.particles[0].inverseMass == 0.0f);
	REQUIRE(graph.particles[1].inverseMass == Approx(100.0f));
	REQUIRE(graph.particles[2].inverseMass == Approx(50.0f));
	REQUIRE(graph.particles[3].inverseMass == Approx(25.0f));

	REQUIRE(graph.constraints.size() == 5);
	REQUIRE(CountKind(graph, ClothConstraintKind::ChainLength) == 3);
	REQUIRE(CountKind(graph, ClothConstraintKind::Bend) == 2);
	REQUIRE(CountKind(graph, ClothConstraintKind::Lateral) == 0);
	RequireNoBatchSharesParticle(graph);

	const int hair01 = FindBone(result.rig, "Hair01");
	const int hair03 = FindBone(result.rig, "Hair03");
	std::size_t hair03Bends = 0;
	for (const ClothConstraint& constraint : graph.constraints) {
		if (constraint.kind != ClothConstraintKind::Bend)
			continue;
		REQUIRE(constraint.restLength == Approx(8.0f).margin(1e-5f));
		REQUIRE(constraint.minLength == Approx(kBendMinimumFraction * 8.0f).margin(1e-5f));
		REQUIRE(constraint.maxLength == Approx(8.0f).margin(1e-5f));
		if (constraint.first != static_cast<std::uint32_t>(hair03) && constraint.second != static_cast<std::uint32_t>(hair03))
			continue;
		++hair03Bends;
		REQUIRE((constraint.first == static_cast<std::uint32_t>(hair01) || constraint.second == static_cast<std::uint32_t>(hair01)));
	}
	REQUIRE(hair03Bends == 1);
}

TEST_CASE("A 2x3 lattice builds lateral constraints with a window around the rest length", "[SmpPhysics]")
{
	const ParseResult result = ParseSmpXml(LatticeXml());
	REQUIRE(result.status == ParseStatus::Ok);
	REQUIRE(result.rig.joints.size() == 6);
	REQUIRE(result.rig.additionalJoints.size() == 3);

	const ClothGraph graph = BuildClothGraph(result.rig, LatticeRest(result.rig));

	REQUIRE(IsLatticeRig(result.rig));
	REQUIRE(graph.particles.size() == 7);
	REQUIRE(graph.particles[static_cast<std::size_t>(FindBone(result.rig, "Head"))].inverseMass == 0.0f);
	REQUIRE(graph.particles[static_cast<std::size_t>(FindBone(result.rig, "A2"))].inverseMass == Approx(50.0f));

	REQUIRE(graph.constraints.size() == 13);
	REQUIRE(CountKind(graph, ClothConstraintKind::ChainLength) == 6);
	REQUIRE(CountKind(graph, ClothConstraintKind::Bend) == 4);
	REQUIRE(CountKind(graph, ClothConstraintKind::Lateral) == 3);
	RequireNoBatchSharesParticle(graph);

	const float slack = std::sqrt(0.1f * 0.1f + 0.15f * 0.15f + 0.2f * 0.2f);
	for (const ClothConstraint& constraint : graph.constraints) {
		if (constraint.kind != ClothConstraintKind::Lateral)
			continue;
		REQUIRE(constraint.restLength == Approx(2.0f).margin(1e-5f));
		REQUIRE(constraint.minLength == Approx(std::max(0.0f, 2.0f - slack)).margin(1e-5f));
		REQUIRE(constraint.maxLength == Approx(2.0f + slack).margin(1e-5f));
		REQUIRE(constraint.minLength <= constraint.restLength);
		REQUIRE(constraint.restLength <= constraint.maxLength);
	}
}

TEST_CASE("A repeated lateral joint is added once", "[SmpPhysics]")
{
	const ParseResult result = ParseSmpXml(LatticeXml(R"XML(
		<generic-constraint bodyA="B2" bodyB="A2" template="lateral"/>)XML"));
	REQUIRE(result.status == ParseStatus::Ok);
	REQUIRE(result.rig.additionalJoints.size() == 4);

	const ClothGraph graph = BuildClothGraph(result.rig, LatticeRest(result.rig));

	REQUIRE(graph.constraints.size() == 13);
	REQUIRE(CountKind(graph, ClothConstraintKind::Lateral) == 3);
	RequireNoBatchSharesParticle(graph);
}

TEST_CASE("A lateral axis with no limits leaves the window unbounded", "[SmpPhysics]")
{
	const ParseResult result = ParseSmpXml(LatticeXml());
	REQUIRE(result.status == ParseStatus::Ok);
	SmpRig rig = result.rig;
	for (RigJoint& joint : rig.additionalJoints) {
		joint.linearLower = { 1.0f, 1.0f, 1.0f };
		joint.linearUpper = { -1.0f, -1.0f, -1.0f };
	}

	const ClothGraph graph = BuildClothGraph(rig, LatticeRest(rig));

	const float slack = std::sqrt(3.0f) * kUnlimitedLateralLength;
	for (const ClothConstraint& constraint : graph.constraints) {
		if (constraint.kind != ClothConstraintKind::Lateral)
			continue;
		REQUIRE(constraint.minLength == 0.0f);
		REQUIRE(constraint.maxLength == Approx(2.0f + slack).margin(1.0f));
	}
}

TEST_CASE("Two builds of the same rig produce the same graph", "[SmpPhysics]")
{
	const ParseResult result = ParseSmpXml(LatticeXml());
	REQUIRE(result.status == ParseStatus::Ok);
	const std::vector<Transform> rest = LatticeRest(result.rig);

	const ClothGraph first = BuildClothGraph(result.rig, rest);
	const ClothGraph second = BuildClothGraph(result.rig, rest);

	REQUIRE(first.particles.size() == second.particles.size());
	for (std::size_t index = 0; index < first.particles.size(); ++index) {
		REQUIRE(first.particles[index].bone == second.particles[index].bone);
		REQUIRE(first.particles[index].inverseMass == second.particles[index].inverseMass);
	}
	REQUIRE(first.batchCount == second.batchCount);
	REQUIRE(first.constraints.size() == second.constraints.size());
	for (std::size_t index = 0; index < first.constraints.size(); ++index) {
		const ClothConstraint& left = first.constraints[index];
		const ClothConstraint& right = second.constraints[index];
		REQUIRE(left.first == right.first);
		REQUIRE(left.second == right.second);
		REQUIRE(left.restLength == right.restLength);
		REQUIRE(left.minLength == right.minLength);
		REQUIRE(left.maxLength == right.maxLength);
		REQUIRE(left.kind == right.kind);
		REQUIRE(left.batch == right.batch);
	}
}
