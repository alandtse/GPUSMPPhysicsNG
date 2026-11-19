// Unit tests for choosing the character the hair tools act on (src/smp/HairTargetPick.cpp).

#include "smp/HairTargetPick.h"

#include <catch2/catch_test_macros.hpp>

using namespace SmpPhysics;

TEST_CASE("The player is the target until something is selected", "[smpphysics][hairtargetpick]")
{
	HairTargetPick pick;
	UpdatePick(pick, 0);

	CHECK(pick.source == HairTargetSource::Player);
	CHECK(pick.formID == 0);
}

TEST_CASE("A new console selection becomes the target and a selection that has not changed does not", "[smpphysics][hairtargetpick]")
{
	HairTargetPick pick;
	UpdatePick(pick, 0x1A67F);
	CHECK(pick.source == HairTargetSource::Console);
	CHECK(pick.formID == 0x1A67F);

	CHECK(PinPick(pick, HairTargetSource::Player));
	UnpinPick(pick);
	CHECK(pick.formID == 0x1A67F);

	CHECK(PinPick(pick, HairTargetSource::Player));
	UpdatePick(pick, 0x1A67F);
	CHECK(pick.formID == 0);
}

TEST_CASE("Clearing the console selection returns an unpinned target to the player", "[smpphysics][hairtargetpick]")
{
	HairTargetPick pick;
	UpdatePick(pick, 0x1A67F);
	UpdatePick(pick, 0);

	CHECK(pick.source == HairTargetSource::Player);
	CHECK(pick.formID == 0);
}

TEST_CASE("A pinned source is not taken over by a new console selection", "[smpphysics][hairtargetpick]")
{
	HairTargetPick pick;
	UpdatePick(pick, 0x1A67F);
	NoteCrosshair(pick, 0x13BAB);
	REQUIRE(PinPick(pick, HairTargetSource::Crosshair));
	UpdatePick(pick, 0x1C0DA);
	NoteCrosshair(pick, 0x13BAB);

	CHECK(pick.source == HairTargetSource::Crosshair);
	CHECK(pick.formID == 0x13BAB);
	CHECK(pick.console == 0x1C0DA);
}

TEST_CASE("The crosshair's character is kept when it leaves the crosshair", "[smpphysics][hairtargetpick]")
{
	HairTargetPick pick;
	NoteCrosshair(pick, 0x13BAB);
	NoteCrosshair(pick, 0);

	CHECK(pick.crosshair == 0x13BAB);
	CHECK(pick.formID == 0);
	NoteCrosshair(pick, 0x1C0DA);
	CHECK(pick.crosshair == 0x1C0DA);
}

TEST_CASE("Pinning a source with no character changes nothing", "[smpphysics][hairtargetpick]")
{
	HairTargetPick pick;

	CHECK_FALSE(PinPick(pick, HairTargetSource::Console));
	CHECK_FALSE(PinPick(pick, HairTargetSource::Crosshair));
	CHECK_FALSE(pick.pinned);
	CHECK(PinPick(pick, HairTargetSource::Player));
	CHECK(pick.pinned);
}

TEST_CASE("A character that is gone is forgotten and a target on it falls back to the player", "[smpphysics][hairtargetpick]")
{
	HairTargetPick pick;
	UpdatePick(pick, 0x1A67F);
	NoteCrosshair(pick, 0x1A67F);
	REQUIRE(PinPick(pick, HairTargetSource::Console));
	ForgetPick(pick, 0x1A67F);

	CHECK(pick.console == 0);
	CHECK(pick.crosshair == 0);
	CHECK(pick.source == HairTargetSource::Player);
	CHECK_FALSE(pick.pinned);
	ForgetPick(pick, 0);
	CHECK(pick.formID == 0);
}
