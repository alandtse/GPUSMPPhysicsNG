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

TEST_CASE("A grip event is closed past the squeeze threshold", "[readyweapongate]")
{
	CHECK(IsGripValueClosed(1.0f));
	CHECK(IsGripValueClosed(kGripClosedAbove + 0.01f));
	CHECK_FALSE(IsGripValueClosed(kGripClosedAbove));
	CHECK_FALSE(IsGripValueClosed(0.0f));
}

TEST_CASE("A grip is closed from its press until its release", "[readyweapongate]")
{
	GripLatch latch;
	CHECK_FALSE(latch.IsClosed(kRight, false));
	latch.Set(kRight, true);
	CHECK(latch.IsClosed(kRight, false));
	latch.Set(kRight, false);
	CHECK_FALSE(latch.IsClosed(kRight, false));
}

TEST_CASE("Each hand keeps its own grip", "[readyweapongate]")
{
	GripLatch latch;
	latch.Set(kLeft, true);
	CHECK(latch.IsClosed(kLeft, false));
	CHECK_FALSE(latch.IsClosed(kRight, false));
}

TEST_CASE("A menu reads a grip as open and forgets the press", "[readyweapongate]")
{
	GripLatch latch;
	latch.Set(kLeft, true);
	CHECK_FALSE(latch.IsClosed(kLeft, true));
	CHECK_FALSE(latch.IsClosed(kLeft, false));
}

TEST_CASE("A hand index outside the two hands has no grip", "[readyweapongate]")
{
	GripLatch latch;
	latch.Set(2, true);
	CHECK_FALSE(latch.IsClosed(2, false));
}
