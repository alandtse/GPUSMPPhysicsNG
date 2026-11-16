// Unit tests for the guard that keeps a call through a missing interface function from taking the game down.

#include "Features/Wind/GuardedCall.h"

#include <catch2/catch_test_macros.hpp>

TEST_CASE("A guarded call that succeeds returns its result", "[wind]")
{
	bool faulted = false;
	REQUIRE(GuardedCall::Invoke([](void*) { return true; }, nullptr, faulted));
	REQUIRE_FALSE(faulted);
}

TEST_CASE("A guarded call that returns false is not a fault", "[wind]")
{
	bool faulted = false;
	REQUIRE_FALSE(GuardedCall::Invoke([](void*) { return false; }, nullptr, faulted));
	REQUIRE_FALSE(faulted);
}

TEST_CASE("A guarded call through a null function is caught", "[wind]")
{
	bool faulted = false;
	REQUIRE_FALSE(GuardedCall::Invoke(nullptr, nullptr, faulted));
	REQUIRE(faulted);
}
