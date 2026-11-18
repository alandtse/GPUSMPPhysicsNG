// Unit tests for the claim policy (src/smp/ClaimPolicy.h). Pure; no game dependency.

#include "smp/ClaimPolicy.h"

#include <catch2/catch_test_macros.hpp>

using SmpPhysics::MayClaimWithoutCollision;

TEST_CASE("A collision add-on with dynamic bones runs without its absent meshes", "[smpphysics][claimpolicy]")
{
	CHECK(MayClaimWithoutCollision(true, false, 7));
}

TEST_CASE("A rig with nothing to simulate or a cloth claim is still declined", "[smpphysics][claimpolicy]")
{
	CHECK_FALSE(MayClaimWithoutCollision(true, false, 0));
	CHECK_FALSE(MayClaimWithoutCollision(true, true, 7));
}

TEST_CASE("A rig whose meshes were found is not a fallback case", "[smpphysics][claimpolicy]")
{
	CHECK_FALSE(MayClaimWithoutCollision(false, false, 7));
}
