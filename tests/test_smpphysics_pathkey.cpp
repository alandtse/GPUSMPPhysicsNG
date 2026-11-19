// Unit tests for comparing paths regardless of case and slash style (src/smp/PathKey.h).

#include <catch2/catch_test_macros.hpp>

#include "smp/PathKey.h"

using SmpPhysics::PathKey;

TEST_CASE("The same file written with another case or slash has the same key", "[smpphysics][pathkey]")
{
	CHECK(PathKey("SKSE/Plugins/Cloak.XML") == PathKey("skse\\plugins\\cloak.xml"));
	CHECK(PathKey("A\\b.xml") == "a\\b.xml");
}

TEST_CASE("Different files, even a typo apart, have different keys", "[smpphysics][pathkey]")
{
	CHECK(PathKey("HTD-SMP\\cloak.xml") != PathKey("HDT-SMP\\cloak.xml"));
}
