// Unit tests for the hover text of the hair target (src/smp/HairTargetDetail.h).

#include <catch2/catch_test_macros.hpp>

#include "smp/HairTargetDetail.h"

using namespace SmpPhysics;

TEST_CASE("A target with nothing on its head and no ranking yet says so", "[smpphysics][hairtargetdetail]")
{
	CHECK(FormatHairTargetDetail({}) == "Hair head part: none\nWorn on the head: nothing");
}

TEST_CASE("A ranked target reports the hair matched, its mesh, its head part and what it wears", "[smpphysics][hairtargetdetail]")
{
	HairTargetDetail detail;
	detail.matchedName = "HDPT_Hourglass";
	detail.matchedModel = "wig\\Hourglass.nif";
	detail.triangles = 17483;
	detail.hairPart = "HDPT_Hourglass";
	detail.extraParts = 12;
	CHECK(FormatHairTargetDetail(detail) ==
		  "Matched by hair head part: HDPT_Hourglass\nMesh: wig\\Hourglass.nif (17483 triangles)\nHair head part: HDPT_Hourglass (12 extra parts)\nWorn on the head: nothing");

	detail.matchedByWig = true;
	detail.worn = { { "Alany", true, false }, { "Hood", false, true }, { "Both", true, true } };
	CHECK(FormatHairTargetDetail(detail).ends_with("Worn on the head: Alany (hair slot), Hood (head slot), Both (hair and head slots)"));
	CHECK(FormatHairTargetDetail(detail).starts_with("Matched by worn wig: HDPT_Hourglass"));
}
