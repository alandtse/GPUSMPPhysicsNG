// Unit tests for the engine-compatible bone matrix composition
// (src/smp/SkinPalette.h). Pure maths; no game/D3D dependency.

#include "smp/SkinPalette.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>

using namespace SmpPhysics;
using Catch::Approx;

namespace
{
	constexpr float kQuarterTurn = 1.5707963f;

	Quat AboutZ(float a_angle)
	{
		return { std::cos(a_angle * 0.5f), 0.0f, 0.0f, std::sin(a_angle * 0.5f) };
	}

	const float kIdentity[3][3] = { { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 } };
}

TEST_CASE("Composing an identity skin transform returns the world pose", "[smpphysics][skinpalette]")
{
	const Transform world{ AboutZ(kQuarterTurn), { 10.0f, 20.0f, 30.0f } };
	const SkinRows rows = ComposeSkinRows(world, 1.0f, PackSkinToBone(kIdentity, 1.0f, 0.0f, 0.0f, 0.0f));

	// A quarter turn about Z sends x to y: column zero is (0, 1, 0).
	CHECK(rows[0] == Approx(0.0f).margin(1e-5f));
	CHECK(rows[4] == Approx(1.0f));
	CHECK(rows[1] == Approx(-1.0f));
	CHECK(rows[10] == Approx(1.0f));
	CHECK(rows[3] == Approx(10.0f));
	CHECK(rows[7] == Approx(20.0f));
	CHECK(rows[11] == Approx(30.0f));
}

TEST_CASE("The skin translation is rotated and scaled by the node before the world translation is added", "[smpphysics][skinpalette]")
{
	const Transform world{ AboutZ(kQuarterTurn), { 1.0f, 2.0f, 3.0f } };
	const SkinRows rows = ComposeSkinRows(world, 2.0f, PackSkinToBone(kIdentity, 1.0f, 5.0f, 0.0f, 0.0f));

	// (5, 0, 0) turned a quarter about Z is (0, 5, 0), doubled by the node scale to (0, 10, 0).
	CHECK(rows[3] == Approx(1.0f).margin(1e-4f));
	CHECK(rows[7] == Approx(12.0f));
	CHECK(rows[11] == Approx(3.0f));
	CHECK(rows[4] == Approx(2.0f));
}

TEST_CASE("Both scales multiply into the rotation", "[smpphysics][skinpalette]")
{
	const SkinRows rows = ComposeSkinRows({ Quat{ 1, 0, 0, 0 }, {} }, 1.5f, PackSkinToBone(kIdentity, 2.0f, 0.0f, 0.0f, 0.0f));
	CHECK(rows[0] == Approx(3.0f));
	CHECK(rows[5] == Approx(3.0f));
	CHECK(rows[10] == Approx(3.0f));
	CHECK(rows[1] == Approx(0.0f).margin(1e-6f));
}
