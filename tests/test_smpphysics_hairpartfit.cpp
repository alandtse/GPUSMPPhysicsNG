// Unit tests for which hair head parts suit a character (src/smp/HairPartFit.h).

#include "smp/HairPartFit.h"

#include <catch2/catch_test_macros.hpp>

using namespace SmpPhysics;

TEST_CASE("A hair part fits the sex its flags name", "[smpphysics][hairpartfit]")
{
	CHECK(HairPartFits(kHairPartFemaleFlag, true));
	CHECK_FALSE(HairPartFits(kHairPartFemaleFlag, false));
	CHECK(HairPartFits(kHairPartMaleFlag, false));
	CHECK_FALSE(HairPartFits(kHairPartMaleFlag, true));
	CHECK(HairPartFits(kHairPartMaleFlag | kHairPartFemaleFlag, true));
	CHECK(HairPartFits(kHairPartMaleFlag | kHairPartFemaleFlag, false));
}

TEST_CASE("A part with no sex flag, or an extra part, fits nobody", "[smpphysics][hairpartfit]")
{
	CHECK_FALSE(HairPartFits(0, true));
	CHECK_FALSE(HairPartFits(0, false));
	CHECK_FALSE(HairPartFits(kHairPartMaleFlag | kHairPartFemaleFlag | kHairPartExtraFlag, true));
}
