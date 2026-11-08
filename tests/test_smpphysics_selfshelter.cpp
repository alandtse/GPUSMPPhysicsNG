// Unit tests for the wearer's own wind shelter (src/smp/SelfWindShelter.cpp).

#include "smp/SelfWindShelter.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

using namespace SmpPhysics;
using Catch::Approx;

namespace
{
	const Vec3 kTowardWind{ 1.0f, 0.0f, 0.0f };
	const std::vector<Transform> kIdentityDriver{ Transform{} };

	Collider MakeCapsule(const Vec3& a_a, const Vec3& a_b, float a_radius)
	{
		Collider collider;
		collider.type = Collider::Type::Capsule;
		collider.driverIndex = 0;
		collider.segmentA = a_a;
		collider.segmentB = a_b;
		collider.radius = a_radius;
		return collider;
	}

	Collider MakeEllipsoid(const Vec3& a_centre, const Vec3& a_halfExtents)
	{
		Collider collider;
		collider.type = Collider::Type::Ellipsoid;
		collider.driverIndex = 0;
		collider.center = a_centre;
		collider.halfExtents = a_halfExtents;
		return collider;
	}
}

TEST_CASE("Hair with nothing between it and the wind is fully exposed", "[SmpPhysics]")
{
	REQUIRE(SelfWindExposure({}, {}, kIdentityDriver, { Vec3{} }, kTowardWind, 100.0f) == Approx(1.0f));
	REQUIRE(SelfWindExposure({ MakeEllipsoid({ 50.0f, 0.0f, 0.0f }, { 5.0f, 5.0f, 5.0f }) }, {}, kIdentityDriver, {}, kTowardWind, 100.0f) == Approx(1.0f));
}

TEST_CASE("A head ellipsoid on the windward side shelters the hair, and one behind it does not", "[SmpPhysics]")
{
	const std::vector<Vec3> hair{ Vec3{} };
	REQUIRE(SelfWindExposure({ MakeEllipsoid({ 20.0f, 0.0f, 0.0f }, { 5.0f, 5.0f, 5.0f }) }, {}, kIdentityDriver, hair, kTowardWind, 100.0f) == Approx(0.0f));
	REQUIRE(SelfWindExposure({ MakeEllipsoid({ -20.0f, 0.0f, 0.0f }, { 5.0f, 5.0f, 5.0f }) }, {}, kIdentityDriver, hair, kTowardWind, 100.0f) == Approx(1.0f));
	REQUIRE(SelfWindExposure({ MakeEllipsoid({ 200.0f, 0.0f, 0.0f }, { 5.0f, 5.0f, 5.0f }) }, {}, kIdentityDriver, hair, kTowardWind, 100.0f) == Approx(1.0f));
}

TEST_CASE("A capsule shelters only the samples whose ray crosses it", "[SmpPhysics]")
{
	const std::vector<Collider> body{ MakeCapsule({ 30.0f, 0.0f, -10.0f }, { 30.0f, 0.0f, 10.0f }, 4.0f) };
	const std::vector<Vec3> hair{ Vec3{ 0.0f, 0.0f, 0.0f }, Vec3{ 0.0f, 0.0f, 50.0f } };
	REQUIRE(SelfWindExposure(body, {}, kIdentityDriver, hair, kTowardWind, 100.0f) == Approx(0.5f));
}

TEST_CASE("A hair point already inside a collider is not sheltered by it, and ground planes never shelter", "[SmpPhysics]")
{
	const std::vector<Vec3> hair{ Vec3{} };
	REQUIRE(SelfWindExposure({ MakeEllipsoid({}, { 5.0f, 5.0f, 5.0f }) }, {}, kIdentityDriver, hair, kTowardWind, 100.0f) == Approx(1.0f));
	Collider plane;
	plane.type = Collider::Type::Plane;
	plane.driverIndex = 0;
	plane.center = { 10.0f, 0.0f, 0.0f };
	plane.normal = { -1.0f, 0.0f, 0.0f };
	REQUIRE(SelfWindExposure({ plane }, {}, kIdentityDriver, hair, kTowardWind, 100.0f) == Approx(1.0f));
}

TEST_CASE("A posed armour triangle shelters the hair behind it", "[SmpPhysics]")
{
	SkinnedSurface surface;
	surface.vertices.resize(3);
	surface.vertices[0].position = { 20.0f, -10.0f, -10.0f };
	surface.vertices[1].position = { 20.0f, 10.0f, -10.0f };
	surface.vertices[2].position = { 20.0f, 0.0f, 10.0f };
	surface.indices = { 0, 1, 2 };
	const std::vector<Vec3> hair{ Vec3{} };
	REQUIRE(SelfWindExposure({}, { surface }, kIdentityDriver, hair, kTowardWind, 100.0f) == Approx(0.0f));
	REQUIRE(SelfWindExposure({}, { surface }, kIdentityDriver, hair, kTowardWind, 10.0f) == Approx(1.0f));
	REQUIRE(SelfWindExposure({}, { surface }, kIdentityDriver, hair, Vec3{ -1.0f, 0.0f, 0.0f }, 100.0f) == Approx(1.0f));
}
