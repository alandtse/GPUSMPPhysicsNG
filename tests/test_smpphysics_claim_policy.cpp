// Unit tests for the claim policy (src/smp/ClaimPolicy.h). Pure; no game dependency.

#include "smp/ClaimPolicy.h"

#include <catch2/catch_test_macros.hpp>

using SmpPhysics::HasNoCollisionGeometry;
using SmpPhysics::MayClaimWithoutCollision;

TEST_CASE("A collision add-on with dynamic bones runs without its absent meshes", "[smpphysics][claimpolicy]")
{
	CHECK(MayClaimWithoutCollision(true, 7));
}

TEST_CASE("A rig with nothing to simulate is still declined", "[smpphysics][claimpolicy]")
{
	CHECK_FALSE(MayClaimWithoutCollision(true, 0));
}

TEST_CASE("A rig whose meshes were found is not a fallback case", "[smpphysics][claimpolicy]")
{
	CHECK_FALSE(MayClaimWithoutCollision(false, 7));
}

TEST_CASE("A hair that shares skin bones with its rig but weights none to a dynamic bone has no collision geometry", "[smpphysics][claimpolicy]")
{
	CHECK(HasNoCollisionGeometry(24, 0));
}

TEST_CASE("A mesh sharing no skin bone with the rig, or one that built spheres, is not a no-geometry case", "[smpphysics][claimpolicy]")
{
	CHECK_FALSE(HasNoCollisionGeometry(0, 0));
	CHECK_FALSE(HasNoCollisionGeometry(24, 90));
}
