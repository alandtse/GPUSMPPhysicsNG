// Unit tests for the frustum and screen-size maths the culling uses (src/smp/ViewMath.h).

#include "smp/ViewMath.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

using namespace SmpPhysics;
using Catch::Approx;

namespace
{
	/** @brief Looks down +Y with a 90 degree field of view both ways: clip x,y = x,z and w = y, as a row-vector matrix lays it out. */
	constexpr float kLookAlongY[4][4] = {
		{ 1.0f, 0.0f, 0.0f, 0.0f },
		{ 0.0f, 0.0f, 0.0f, 1.0f },
		{ 0.0f, 1.0f, 0.0f, 0.0f },
		{ 0.0f, 0.0f, 0.0f, 0.0f },
	};
	constexpr float kZero[4][4] = {};
}

TEST_CASE("A sphere in front of the camera is in the frustum", "[smpphysics][view]")
{
	CHECK(SphereInFrustum(kLookAlongY, 0.0f, 10.0f, 0.0f, 1.0f));
	CHECK(SphereInFrustum(kLookAlongY, 9.0f, 10.0f, -9.0f, 0.0f));
}

TEST_CASE("A sphere outside a side plane is culled unless its radius reaches in", "[smpphysics][view]")
{
	CHECK_FALSE(SphereInFrustum(kLookAlongY, 30.0f, 10.0f, 0.0f, 1.0f));
	CHECK_FALSE(SphereInFrustum(kLookAlongY, -30.0f, 10.0f, 0.0f, 1.0f));
	CHECK_FALSE(SphereInFrustum(kLookAlongY, 0.0f, 10.0f, 30.0f, 1.0f));
	CHECK(SphereInFrustum(kLookAlongY, 12.0f, 10.0f, 0.0f, 5.0f));
}

TEST_CASE("A degenerate matrix culls nothing", "[smpphysics][view]")
{
	CHECK(SphereInFrustum(kZero, 100.0f, 100.0f, 100.0f, 1.0f));
}

TEST_CASE("Screen height falls with depth and is zero behind the eye", "[smpphysics][view]")
{
	CHECK(SphereScreenHeightPercent(kLookAlongY, 0.0f, 10.0f, 0.0f, 1.0f) == Approx(10.0f));
	CHECK(SphereScreenHeightPercent(kLookAlongY, 0.0f, 20.0f, 0.0f, 1.0f) == Approx(5.0f));
	CHECK(SphereScreenHeightPercent(kLookAlongY, 0.0f, -10.0f, 0.0f, 1.0f) == 0.0f);
	CHECK(SphereScreenHeightPercent(kLookAlongY, 0.0f, 0.0f, 0.0f, 1.0f) == 0.0f);
}
