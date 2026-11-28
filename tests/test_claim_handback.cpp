// Unit tests for the rule that returns claims the GPU cannot simulate to hdtSMP64.

#include "Features/SmpPhysics/ClaimHandBack.h"

#include <catch2/catch_test_macros.hpp>

using SmpPhysics::RegistrationFailed;
using SmpPhysics::ShouldHandBack;

TEST_CASE("Only claims the GPU cannot simulate go back to hdtSMP64", "[smpphysics][handback]")
{
	REQUIRE_FALSE(ShouldHandBack(true, false));
	REQUIRE(ShouldHandBack(true, true));
	REQUIRE(ShouldHandBack(false, false));
	REQUIRE(ShouldHandBack(false, true));
}

TEST_CASE("A registration refused only because the renderer lock was busy is not a failure", "[smpphysics][handback]")
{
	// Another mod holding the renderer lock for a frame must not cost the claim its GPU solver; it is tried again next frame.
	REQUIRE(RegistrationFailed(false, false));
	REQUIRE_FALSE(RegistrationFailed(false, true));
	REQUIRE_FALSE(RegistrationFailed(true, false));
	REQUIRE_FALSE(RegistrationFailed(true, true));
}
