// Unit tests for the outlines the collider debug overlay draws (src/smp/DebugShapes.cpp).

#include "smp/DebugShapes.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <limits>
#include <vector>

using namespace SmpPhysics;
using Catch::Approx;

TEST_CASE("A capsule outline stays one radius from its axis", "[SmpPhysics]")
{
	std::vector<DebugSegment> lines;
	AppendCapsuleOutline(lines, { 0.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, 10.0f }, 2.0f, DebugColor::Collider);
	REQUIRE(lines.size() == 40);
	for (const DebugSegment& line : lines) {
		REQUIRE(line.color == DebugColor::Collider);
		for (const Vec3& point : { line.start, line.end })
			REQUIRE(std::sqrt(point.x * point.x + point.y * point.y) == Approx(2.0f).margin(1e-4f));
	}
}

TEST_CASE("A zero-length capsule outlines a sphere", "[SmpPhysics]")
{
	std::vector<DebugSegment> lines;
	AppendCapsuleOutline(lines, { 1.0f, 2.0f, 3.0f }, { 1.0f, 2.0f, 3.0f }, 4.0f, DebugColor::Hand);
	REQUIRE(lines.size() == 36);
	for (const DebugSegment& line : lines)
		REQUIRE(Length(line.start - Vec3{ 1.0f, 2.0f, 3.0f }) == Approx(4.0f).margin(1e-4f));
}

TEST_CASE("A capsule with no usable radius draws nothing", "[SmpPhysics]")
{
	std::vector<DebugSegment> lines;
	AppendCapsuleOutline(lines, { 0.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, 10.0f }, 0.0f, DebugColor::Collider);
	AppendCapsuleOutline(lines, { 0.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, 10.0f }, std::numeric_limits<float>::quiet_NaN(), DebugColor::Collider);
	REQUIRE(lines.empty());
}

TEST_CASE("An ellipsoid outline lies on the ellipsoid", "[SmpPhysics]")
{
	std::vector<DebugSegment> lines;
	AppendEllipsoidOutline(lines, { 5.0f, 0.0f, 0.0f }, Quat{}, { 3.0f, 2.0f, 1.0f }, DebugColor::Collider);
	REQUIRE(lines.size() == 36);
	for (const DebugSegment& line : lines) {
		const Vec3 offset = line.start - Vec3{ 5.0f, 0.0f, 0.0f };
		const float scaled = (offset.x / 3.0f) * (offset.x / 3.0f) + (offset.y / 2.0f) * (offset.y / 2.0f) + offset.z * offset.z;
		REQUIRE(scaled == Approx(1.0f).margin(1e-4f));
	}
}

TEST_CASE("A cross has one line along each axis through its centre", "[SmpPhysics]")
{
	std::vector<DebugSegment> lines;
	AppendCross(lines, { 1.0f, 2.0f, 3.0f }, 0.5f, DebugColor::Joint);
	REQUIRE(lines.size() == 3);
	for (const DebugSegment& line : lines) {
		REQUIRE(Length(line.end - line.start) == Approx(1.0f));
		REQUIRE(Length((line.start + line.end) * 0.5f - Vec3{ 1.0f, 2.0f, 3.0f }) == Approx(0.0f).margin(1e-5f));
	}
}

TEST_CASE("A ring lies in the plane perpendicular to its axis", "[SmpPhysics]")
{
	std::vector<DebugSegment> lines;
	AppendRing(lines, { 1.0f, 2.0f, 3.0f }, { 0.0f, 0.0f, 5.0f }, 2.0f, DebugColor::SphereTouch);
	REQUIRE(lines.size() == 12);
	for (const DebugSegment& line : lines) {
		REQUIRE(line.start.z == Approx(3.0f).margin(1e-4f));
		REQUIRE(Length(line.start - Vec3{ 1.0f, 2.0f, 3.0f }) == Approx(2.0f).margin(1e-4f));
	}
	std::vector<DebugSegment> none;
	AppendRing(none, { 0.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, 0.0f }, 2.0f, DebugColor::SphereTouch);
	REQUIRE(none.empty());
}

TEST_CASE("The distance to a triangle is measured to its face, edge or corner", "[SmpPhysics]")
{
	const Vec3 a{ 0.0f, 0.0f, 0.0f };
	const Vec3 b{ 4.0f, 0.0f, 0.0f };
	const Vec3 c{ 0.0f, 4.0f, 0.0f };
	Vec3 closest;
	REQUIRE(DistanceToTriangle({ 1.0f, 1.0f, 3.0f }, a, b, c, closest) == Approx(3.0f));
	REQUIRE(closest.x == Approx(1.0f));
	REQUIRE(closest.y == Approx(1.0f));
	REQUIRE(DistanceToTriangle({ 2.0f, -2.0f, 0.0f }, a, b, c, closest) == Approx(2.0f));
	REQUIRE(closest.x == Approx(2.0f));
	REQUIRE(closest.y == Approx(0.0f));
	REQUIRE(DistanceToTriangle({ -3.0f, -4.0f, 0.0f }, a, b, c, closest) == Approx(5.0f));
	REQUIRE(Length(closest) == Approx(0.0f));
}

TEST_CASE("The nearest actors are chosen nearest first, with each actor counted once", "[SmpPhysics]")
{
	const std::vector<ActorDistance> candidates{ { 7, 30.0f }, { 3, 10.0f }, { 7, 5.0f }, { 9, 20.0f } };
	REQUIRE(NearestActors(candidates, 2) == std::vector<std::uint32_t>{ 7, 3 });
	REQUIRE(NearestActors(candidates, 10) == std::vector<std::uint32_t>{ 7, 3, 9 });
	REQUIRE(NearestActors(candidates, 0).empty());
	REQUIRE(NearestActors({}, 3).empty());
}
