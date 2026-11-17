// Unit tests for the rule that returns claims the GPU cannot simulate to hdtSMP64.

#include "Features/SmpPhysics/ClaimHandBack.h"

#include <catch2/catch_test_macros.hpp>

using SmpPhysics::ShouldHandBack;

TEST_CASE("Only claims the GPU cannot simulate go back to hdtSMP64", "[smpphysics][handback]")
{
	REQUIRE_FALSE(ShouldHandBack(true, false));
	REQUIRE(ShouldHandBack(true, true));
	REQUIRE(ShouldHandBack(false, false));
	REQUIRE(ShouldHandBack(false, true));
}
