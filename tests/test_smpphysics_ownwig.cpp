// Unit tests for which wig counts as a character's own look (src/smp/OwnWig.h).

#include <catch2/catch_test_macros.hpp>

#include "smp/OwnWig.h"

using namespace SmpPhysics;

TEST_CASE("With no record the worn wig is the character's own", "[smpphysics][ownwig]")
{
	CHECK(NaturalWigId(std::nullopt, 0) == 0);
	CHECK(NaturalWigId(std::nullopt, 7) == 7);
}

TEST_CASE("While the tool's wig is on, the wig worn before it is the character's own", "[smpphysics][ownwig]")
{
	CHECK(NaturalWigId(OwnWig{ 5, 9 }, 9) == 5);
	CHECK(NaturalWigId(OwnWig{ 0, 9 }, 9) == 0);
}

TEST_CASE("A record is stale once another wig or none is worn", "[smpphysics][ownwig]")
{
	CHECK(NaturalWigId(OwnWig{ 0, 9 }, 4) == 4);
	CHECK(NaturalWigId(OwnWig{ 5, 9 }, 0) == 0);
}
