// Unit tests for telling a skirt-style cloth rig from a hair rig (src/smp/ClothRig.h).

#include "smp/ClothRig.h"

#include <catch2/catch_test_macros.hpp>

using namespace SmpPhysics;

TEST_CASE("A rig whose per-vertex shapes skin to simulated bones is a hair rig", "[smpphysics][clothrig]")
{
	CHECK_FALSE(IsClothRig(1, 1, 0));
	CHECK_FALSE(IsClothRig(4, 2, 1));
}

TEST_CASE("A rig with only body collision per-vertex shapes and a simulated triangle shape is a cloth rig", "[smpphysics][clothrig]")
{
	CHECK(IsClothRig(4, 0, 1));
}

TEST_CASE("A rig with no per-vertex shape, or no simulated triangle shape, is not decided here", "[smpphysics][clothrig]")
{
	CHECK_FALSE(IsClothRig(0, 0, 1));
	CHECK_FALSE(IsClothRig(4, 0, 0));
}

TEST_CASE("Spheres are built per shape for a rig without per-vertex shapes, a cloth rig, or a simulated triangle shape", "[smpphysics][clothrig]")
{
	CHECK(BuildsSpheresPerShape(0, false, 0));
	CHECK(BuildsSpheresPerShape(4, true, 1));
	CHECK(BuildsSpheresPerShape(6, false, 1));
	CHECK_FALSE(BuildsSpheresPerShape(1, false, 0));
}

TEST_CASE("A rig's own triangle shape holds a sphere resting deep behind it only when its file makes it deep", "[smpphysics][clothrig]")
{
	STATIC_REQUIRE(HoldsDeepSpheres(2, false, 0.0f));
	STATIC_REQUIRE_FALSE(HoldsDeepSpheres(2, true, 0.01f));
	STATIC_REQUIRE(HoldsDeepSpheres(2, true, 1.0f));
	STATIC_REQUIRE_FALSE(HoldsDeepSpheres(1, false, 1.0f));
	STATIC_REQUIRE_FALSE(HoldsDeepSpheres(1, true, 1.0f));
}
