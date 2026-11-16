// Unit tests for telling Community Shaders Expanded from upstream Community Shaders by its DLL FileVersion.

#include "Features/Wind/CsBuild.h"

#include <catch2/catch_test_macros.hpp>

TEST_CASE("A FileVersion with a CSX provenance tag is the expanded build", "[wind]")
{
	REQUIRE(CsBuild::IsExpanded("3.21.0.0 [CSX commit=0d0c607278c5134d998cd0c2842193863b8675cb; source=dirty]"));
}

TEST_CASE("A plain FileVersion is upstream Community Shaders", "[wind]")
{
	REQUIRE_FALSE(CsBuild::IsExpanded("1.4.3.0"));
	REQUIRE_FALSE(CsBuild::IsExpanded(""));
}
