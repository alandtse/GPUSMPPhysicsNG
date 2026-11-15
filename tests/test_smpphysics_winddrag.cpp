// Unit tests for the wind-facing geometry of a rig's bones (src/smp/WindDrag.cpp) and the world's shelter estimate.

#include "Features/Wind/WindShelter.h"
#include "smp/WindDrag.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <limits>

using namespace SmpPhysics;
using Catch::Approx;

namespace
{
	/** @brief A kinematic root with a two-bone chain hanging 10 units apart along -Z: root, bone 1, tip bone 2. */
	SmpRig MakeChain()
	{
		SmpRig rig;
		rig.bones.resize(3);
		rig.bones[0].kinematic = true;
		rig.bones[1].mass = 0.1f;
		rig.bones[2].mass = 0.1f;
		rig.joints.resize(2);
		rig.joints[0].parent = 0;
		rig.joints[0].child = 1;
		rig.joints[1].parent = 1;
		rig.joints[1].child = 2;
		return rig;
	}

	std::vector<Transform> MakeChainRest()
	{
		std::vector<Transform> rest(3);
		rest[1].translation = { 0.0f, 0.0f, -10.0f };
		rest[2].translation = { 0.0f, 0.0f, -20.0f };
		return rest;
	}
}

TEST_CASE("A bone's wind segment runs to its first child and a tip takes its parent's", "[SmpPhysics][wind]")
{
	const SmpRig rig = MakeChain();
	const auto geometry = ComputeBoneWindGeometry(rig, MakeChainRest(), {});
	REQUIRE(geometry.size() == 3);

	CHECK(geometry[0].area == 0.0f);
	CHECK(geometry[1].axis.z == Approx(-1.0f));
	CHECK(geometry[1].area == Approx(10.0f * kDefaultWindWidth));
	CHECK(geometry[2].axis.z == Approx(-1.0f));
	CHECK(geometry[2].area == Approx(10.0f * kDefaultWindWidth));
}

TEST_CASE("A measured wind area is kept within the width limits", "[SmpPhysics][wind]")
{
	const SmpRig rig = MakeChain();
	const auto rest = MakeChainRest();

	CHECK(ComputeBoneWindGeometry(rig, rest, { 0.0f, 55.0f, 0.0f })[1].area == Approx(55.0f));
	CHECK(ComputeBoneWindGeometry(rig, rest, { 0.0f, 0.5f, 0.0f })[1].area == Approx(10.0f * kMinWindWidth));
	CHECK(ComputeBoneWindGeometry(rig, rest, { 0.0f, 9000.0f, 0.0f })[1].area == Approx(10.0f * kMaxWindWidth));
}

TEST_CASE("Coincident bones still present the minimum length to the wind", "[SmpPhysics][wind]")
{
	const SmpRig rig = MakeChain();
	std::vector<Transform> rest(3);
	const auto geometry = ComputeBoneWindGeometry(rig, rest, {});
	CHECK(geometry[1].area == Approx(kMinWindLength * kDefaultWindWidth));
	CHECK(geometry[1].axis.z == Approx(-1.0f));
}

TEST_CASE("A rest pose that does not match the rig yields no area", "[SmpPhysics][wind]")
{
	const SmpRig rig = MakeChain();
	const auto geometry = ComputeBoneWindGeometry(rig, std::vector<Transform>(2), {});
	REQUIRE(geometry.size() == 3);
	CHECK(geometry[1].area == 0.0f);
}

TEST_CASE("A triangle's area is shared between its vertices' bones by weight", "[SmpPhysics][wind]")
{
	const SmpRig rig = MakeChain();
	CardSkin skin;
	skin.rigBone = { 1, 2 };
	skin.vertices.resize(3);
	skin.vertices[0].position = { 0.0f, 0.0f, 0.0f };
	skin.vertices[1].position = { 4.0f, 0.0f, 0.0f };
	skin.vertices[2].position = { 0.0f, 3.0f, 0.0f };
	for (SkinnedVertex& vertex : skin.vertices) {
		vertex.bone[0] = 0;
		vertex.weight[0] = 1.0f;
	}
	skin.vertices[2].bone[0] = 1;
	skin.triangles = { 0, 1, 2 };

	const auto areas = EstimateBoneWindAreas(rig, { skin });
	REQUIRE(areas.size() == 3);
	CHECK(areas[0] == 0.0f);
	CHECK(areas[1] == Approx(4.0f));
	CHECK(areas[2] == Approx(2.0f));
}

TEST_CASE("Degenerate and out-of-range triangles add no wind area", "[SmpPhysics][wind]")
{
	const SmpRig rig = MakeChain();
	CardSkin skin;
	skin.rigBone = { 1 };
	skin.vertices.resize(3);
	for (SkinnedVertex& vertex : skin.vertices) {
		vertex.bone[0] = 0;
		vertex.weight[0] = 1.0f;
	}
	skin.triangles = { 0, 1, 2, 0, 1, 7 };

	const auto areas = EstimateBoneWindAreas(rig, { skin });
	CHECK(areas[1] == 0.0f);
}

TEST_CASE("A wall against the point shelters it fully and each blocked ray takes its weight", "[SmpPhysics][wind]")
{
	using WindShelter::Exposure;
	const float clear = -1.0f;
	CHECK(Exposure({ clear, clear, clear }) == Approx(1.0f));
	CHECK(Exposure({ WindShelter::kNearWall - 1.0f, clear, clear }) == 0.0f);
	CHECK(Exposure({ WindShelter::kNearWall + 1.0f, clear, clear }) == Approx(1.0f - WindShelter::kRays[0].weight));
	CHECK(Exposure({ clear, 100.0f, clear }) == Approx(1.0f - WindShelter::kRays[1].weight));
	CHECK(Exposure({ 500.0f, 100.0f, 100.0f }) == Approx(0.0f).margin(1e-6f));
}
