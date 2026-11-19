// Unit tests for the refit policy (src/smp/RefitGuard.h).
// Pure logic; no game/D3D dependency.

#include "smp/RefitGuard.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

using namespace SmpPhysics;
using Catch::Approx;

TEST_CASE("The first refit of an actor is always admitted", "[smpphysics][refit]")
{
	RefitGuard guard;
	const RefitGuard::Decision first = guard.Admit(0x14, 100.0);
	CHECK(first.allowed);
	CHECK(first.streak == 0);
}

TEST_CASE("A refit asked for inside the cooldown is held back and says when it may run", "[smpphysics][refit]")
{
	RefitGuard guard;
	REQUIRE(guard.Admit(0x14, 100.0).allowed);
	const RefitGuard::Decision held = guard.Admit(0x14, 100.5);
	CHECK_FALSE(held.allowed);
	CHECK(held.retryIn == Approx(kRefitBackoffBaseSeconds - 0.5));
}

TEST_CASE("Each repeated refit waits twice as long, up to the cap", "[smpphysics][refit]")
{
	RefitGuard guard;
	double now = 0.0;
	double expected = kRefitBackoffBaseSeconds;
	for (int refit = 0; refit < 8; ++refit) {
		const RefitGuard::Decision admitted = guard.Admit(0x14, now);
		REQUIRE(admitted.allowed);
		CHECK(admitted.streak == refit);
		const RefitGuard::Decision held = guard.Admit(0x14, now);
		REQUIRE_FALSE(held.allowed);
		CHECK(held.retryIn == Approx(std::min(expected, kRefitBackoffMaxSeconds)));
		now += held.retryIn;
		expected *= 2.0;
	}
}

TEST_CASE("The streak ends once the actor has been quiet for the window", "[smpphysics][refit]")
{
	RefitGuard guard;
	REQUIRE(guard.Admit(0x14, 0.0).allowed);
	REQUIRE(guard.Admit(0x14, 2.0).allowed);
	const RefitGuard::Decision later = guard.Admit(0x14, 2.0 + kRefitStreakWindowSeconds + 1.0);
	CHECK(later.allowed);
	CHECK(later.streak == 0);
}

TEST_CASE("Each actor has its own cooldown", "[smpphysics][refit]")
{
	RefitGuard guard;
	REQUIRE(guard.Admit(0x14, 10.0).allowed);
	CHECK(guard.Admit(0x1000, 10.0).allowed);
	CHECK_FALSE(guard.Admit(0x14, 10.1).allowed);
}

TEST_CASE("A claim's baseline waits for its model, then is adopted once, then compares", "[smpphysics][refit]")
{
	CHECK(NextBaselineStep(false, false, 0) == BaselineStep::Wait);
	CHECK(NextBaselineStep(false, true, 0) == BaselineStep::Adopt);
	CHECK(NextBaselineStep(true, true, 0) == BaselineStep::Compare);
	CHECK(NextBaselineStep(true, false, 0) == BaselineStep::Compare);
}

TEST_CASE("A model that never attaches stops holding the baseline back", "[smpphysics][refit]")
{
	CHECK(NextBaselineStep(false, false, kMaxSettleChecks - 1) == BaselineStep::Wait);
	CHECK(NextBaselineStep(false, false, kMaxSettleChecks) == BaselineStep::Adopt);
}
