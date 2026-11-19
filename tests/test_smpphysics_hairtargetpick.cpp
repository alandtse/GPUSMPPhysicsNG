// Unit tests for choosing the character the hair tools act on (src/smp/HairTargetPick.cpp).

#include "smp/HairTargetPick.h"

#include <catch2/catch_test_macros.hpp>

using namespace SmpPhysics;

TEST_CASE("The player is the target until something is selected", "[smpphysics][hairtargetpick]")
{
	HairTargetPick pick;
	UpdatePick(pick, {});

	CHECK(pick.source == HairTargetSource::Player);
	CHECK(pick.formID == 0);
}

TEST_CASE("A new console selection becomes the target and a selection that has not changed does not", "[smpphysics][hairtargetpick]")
{
	HairTargetPick pick;
	UpdatePick(pick, { 0x1A67F, 0, false });
	CHECK(pick.source == HairTargetSource::Console);
	CHECK(pick.formID == 0x1A67F);

	CHECK(PinPick(pick, HairTargetSource::Player));
	UnpinPick(pick);
	CHECK(pick.formID == 0x1A67F);

	CHECK(PinPick(pick, HairTargetSource::Player));
	UpdatePick(pick, { 0x1A67F, 0, false });
	CHECK(pick.formID == 0);
}

TEST_CASE("Clearing the console selection returns an unpinned target to the player", "[smpphysics][hairtargetpick]")
{
	HairTargetPick pick;
	UpdatePick(pick, { 0x1A67F, 0, false });
	UpdatePick(pick, { 0, 0, false });

	CHECK(pick.source == HairTargetSource::Player);
	CHECK(pick.formID == 0);
}

TEST_CASE("A pinned source is not taken over by a new console selection", "[smpphysics][hairtargetpick]")
{
	HairTargetPick pick;
	UpdatePick(pick, { 0x1A67F, 0x13BAB, false });
	REQUIRE(PinPick(pick, HairTargetSource::Crosshair));
	UpdatePick(pick, { 0x1C0DA, 0x13BAB, false });

	CHECK(pick.source == HairTargetSource::Crosshair);
	CHECK(pick.formID == 0x13BAB);
	CHECK(pick.console == 0x1C0DA);
}

TEST_CASE("The crosshair's character is remembered only while no menu is open", "[smpphysics][hairtargetpick]")
{
	HairTargetPick pick;
	UpdatePick(pick, { 0, 0x13BAB, false });
	UpdatePick(pick, { 0, 0x1C0DA, true });
	UpdatePick(pick, { 0, 0, false });

	CHECK(pick.crosshair == 0x13BAB);
	CHECK(pick.formID == 0);
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
	UpdatePick(pick, { 0x1A67F, 0x1A67F, false });
	REQUIRE(PinPick(pick, HairTargetSource::Console));
	ForgetPick(pick, 0x1A67F);

	CHECK(pick.console == 0);
	CHECK(pick.crosshair == 0);
	CHECK(pick.source == HairTargetSource::Player);
	CHECK_FALSE(pick.pinned);
	ForgetPick(pick, 0);
	CHECK(pick.formID == 0);
}
