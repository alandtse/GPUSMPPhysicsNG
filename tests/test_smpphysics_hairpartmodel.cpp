// Unit tests for which mesh stands for a hair head part (src/smp/HairPartModel.h).

#include <catch2/catch_test_macros.hpp>

#include "smp/HairPartModel.h"

#include <string>
#include <vector>

using SmpPhysics::HairPartModel;

TEST_CASE("A hair head part with a model of its own uses it", "[smpphysics][hairpartmodel]")
{
	const std::vector<std::string> extras{ "wig\Whole.nif" };
	CHECK(HairPartModel("hair\Own.nif", extras) == "hair\Own.nif");
}

TEST_CASE("Without one the whole-hair extra part stands for it, not a split piece or a collision mesh", "[smpphysics][hairpartmodel]")
{
	const std::vector<std::string> extras{
		"KS\wig\_CollisionBodyX_1.nif",
		"KS\wig\Hourglass.nif__HDPT_extraParts\003_1Hourglass.nif",
		"",
		"KS\wig\Hourglass.nif",
		"KS\wig\Other.nif",
	};
	CHECK(HairPartModel("", extras) == "KS\wig\Hourglass.nif");
}

TEST_CASE("A hair head part with no usable mesh gives none", "[smpphysics][hairpartmodel]")
{
	const std::vector<std::string> extras{ "a\collision.nif", "" };
	CHECK(HairPartModel("", extras).empty());
	CHECK(HairPartModel("", {}).empty());
}
