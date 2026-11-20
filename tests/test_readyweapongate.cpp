// Unit tests for the press and grip decisions of the Ready Weapon gate (src/engine/ReadyWeaponGateLogic.h).

#include "engine/ReadyWeaponGateLogic.h"

#include <catch2/catch_test_macros.hpp>

using namespace SmpPhysics::ReadyWeaponGate;

namespace
{
	constexpr std::size_t kLeft = 0;
	constexpr std::size_t kRight = 1;
}

TEST_CASE("A press of a held hand is swallowed along with its release", "[readyweapongate]")
{
	PressTracker tracker;
	CHECK(tracker.Press(kRight, kRightHand));
	CHECK(tracker.Follow(kRight, false));
	CHECK(tracker.Follow(kRight, true));
	CHECK_FALSE(tracker.Follow(kRight, true));
}

TEST_CASE("A press of a hand that is not held passes through, and so does its release", "[readyweapongate]")
{
	PressTracker tracker;
	CHECK_FALSE(tracker.Press(kLeft, kRightHand));
	CHECK_FALSE(tracker.Follow(kLeft, false));
	CHECK_FALSE(tracker.Follow(kLeft, true));
}

TEST_CASE("Letting go of the hair mid press does not release the swallowed press early", "[readyweapongate]")
{
	PressTracker tracker;
	REQUIRE(tracker.Press(kLeft, kLeftHand));
	CHECK(tracker.Follow(kLeft, false));
	CHECK(tracker.Follow(kLeft, true));
}

TEST_CASE("Each hand keeps its own press state", "[readyweapongate]")
{
	PressTracker tracker;
	REQUIRE(tracker.Press(kLeft, kLeftHand));
	REQUIRE_FALSE(tracker.Press(kRight, kLeftHand));
	CHECK(tracker.Follow(kLeft, false));
	CHECK_FALSE(tracker.Follow(kRight, false));
}

TEST_CASE("A new press decides again after a swallowed one", "[readyweapongate]")
{
	PressTracker tracker;
	REQUIRE(tracker.Press(kRight, kRightHand));
	REQUIRE(tracker.Follow(kRight, true));
	CHECK_FALSE(tracker.Press(kRight, 0u));
	CHECK_FALSE(tracker.Follow(kRight, true));
}

TEST_CASE("A hand index outside the two hands is never swallowed", "[readyweapongate]")
{
	PressTracker tracker;
	CHECK_FALSE(tracker.Press(2, kLeftHand | kRightHand));
	CHECK_FALSE(tracker.Follow(2, true));
}

TEST_CASE("A grip is closed by its button bit or by squeezing past the threshold", "[readyweapongate]")
{
	CHECK(IsGripClosed(true, 0.0f));
	CHECK(IsGripClosed(false, kGripAxisClosedAbove + 0.01f));
	CHECK_FALSE(IsGripClosed(false, kGripAxisClosedAbove));
	CHECK_FALSE(IsGripClosed(false, 0.0f));
}

TEST_CASE("A grip is read as open without polling while a menu is up", "[readyweapongate]")
{
	int polls = 0;
	const auto squeezed = [&] {
		++polls;
		return true;
	};

	CHECK_FALSE(ReadGrip(true, squeezed));
	CHECK(polls == 0);
	CHECK(ReadGrip(false, squeezed));
	CHECK(polls == 1);
}
