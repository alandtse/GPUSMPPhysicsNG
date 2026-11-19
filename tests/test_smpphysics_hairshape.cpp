// Unit tests for comparing hair meshes by shape (src/smp/HairShape.cpp).

#include "smp/HairShape.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <array>
#include <cmath>
#include <limits>

using namespace SmpPhysics;
using Catch::Approx;

namespace
{
	/** @brief A flat square in the XZ plane with its corner at a_origin, a_size on a side, cut into a_cuts by a_cuts cells of two triangles. */
	std::vector<HairTriangle> Square(Float3 a_origin, float a_size, int a_cuts)
	{
		std::vector<HairTriangle> triangles;
		const float step = a_size / static_cast<float>(a_cuts);
		for (int row = 0; row < a_cuts; ++row) {
			for (int column = 0; column < a_cuts; ++column) {
				const float x = a_origin[0] + static_cast<float>(column) * step;
				const float z = a_origin[2] + static_cast<float>(row) * step;
				const Float3 p00{ x, a_origin[1], z };
				const Float3 p10{ x + step, a_origin[1], z };
				const Float3 p01{ x, a_origin[1], z + step };
				const Float3 p11{ x + step, a_origin[1], z + step };
				triangles.push_back({ p00, p10, p11 });
				triangles.push_back({ p00, p11, p01 });
			}
		}
		return triangles;
	}
}

TEST_CASE("A mesh is described by its area, box and an even sample of its surface", "[smpphysics][hairshape]")
{
	const auto square = Square({ 0, 0, 0 }, 100.0f, 1);
	const HairDescriptor described = DescribeHair(square);

	CHECK(described.area == Approx(10000.0f));
	CHECK(described.samples.size() == kHairSamples);
	CHECK(described.boundsMin[0] == Approx(0.0f));
	CHECK(described.boundsMax[2] == Approx(100.0f));
	for (const Float3& point : described.samples) {
		CHECK(point[0] >= -0.001f);
		CHECK(point[0] <= 100.001f);
		CHECK(point[1] == Approx(0.0f));
	}
	CHECK(DescribeHair(square).samples == described.samples);
}

TEST_CASE("Triangles with no area or a bad coordinate add nothing", "[smpphysics][hairshape]")
{
	std::vector<HairTriangle> triangles = Square({ 0, 0, 0 }, 10.0f, 1);
	triangles.push_back({ { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 } });
	triangles.push_back({ { 0, 0, 0 }, { std::numeric_limits<float>::quiet_NaN(), 0, 0 }, { 0, 5, 5 } });

	CHECK(DescribeHair(triangles).area == Approx(100.0f));
	CHECK(DescribeHair({}).samples.empty());
	CHECK(CompareHair(DescribeHair({}), DescribeHair(triangles)).chamfer == std::numeric_limits<float>::max());
}

TEST_CASE("The same shape matches itself exactly", "[smpphysics][hairshape]")
{
	const HairDescriptor shape = DescribeHair(Square({ 0, 0, 0 }, 100.0f, 2));
	const HairMatch match = CompareHair(shape, shape);

	CHECK(match.chamfer == Approx(0.0f));
	CHECK(match.reach == Approx(0.0f));
	CHECK(match.coverage == Approx(0.0f));
	CHECK(match.overlap == Approx(1.0f));
	CHECK(match.boxOverlap == Approx(1.0f));
	CHECK(match.areaRatio == Approx(1.0f));
}

TEST_CASE("The same surface cut into more triangles still matches closely", "[smpphysics][hairshape]")
{
	const HairMatch match = CompareHair(DescribeHair(Square({ 0, 0, 0 }, 100.0f, 1)), DescribeHair(Square({ 0, 0, 0 }, 100.0f, 8)));

	CHECK(match.chamfer < 3.0f);
	CHECK(match.coverage == Approx(0.0f));
	CHECK(match.areaRatio == Approx(1.0f));
	CHECK(match.boxOverlap > 0.99f);
}

TEST_CASE("A shape moved sideways is measured by how far", "[smpphysics][hairshape]")
{
	const HairDescriptor base = DescribeHair(Square({ 0, 0, 0 }, 100.0f, 4));
	const HairMatch near = CompareHair(base, DescribeHair(Square({ 5, 0, 0 }, 100.0f, 4)));
	const HairMatch far = CompareHair(base, DescribeHair(Square({ 400, 0, 0 }, 100.0f, 4)));

	CHECK(near.reach >= near.chamfer);
	CHECK(near.chamfer > 0.5f);
	CHECK(near.chamfer < 8.0f);
	CHECK(near.overlap > far.overlap);
	CHECK(far.chamfer > 300.0f);
	CHECK(far.coverage == Approx(1.0f));
	CHECK(far.boxOverlap == Approx(0.0f));
	CHECK(far.overlap == Approx(0.0f));
}

TEST_CASE("A smaller version of a shape has a smaller area ratio", "[smpphysics][hairshape]")
{
	const HairMatch match = CompareHair(DescribeHair(Square({ 0, 0, 0 }, 100.0f, 2)), DescribeHair(Square({ 0, 0, 0 }, 50.0f, 2)));

	CHECK(match.areaRatio == Approx(0.25f));
	CHECK(match.boxOverlap < 0.5f);
}

TEST_CASE("The strongest eligible slot places a vertex, skipping bones that are not eligible", "[smpphysics][hairshape]")
{
	const std::array<float, 4> weights{ 0.1f, 0.6f, 0.3f, 0.0f };

	CHECK(StrongestEligibleSlot(weights, std::array<bool, 4>{ true, true, true, true }) == 1);
	CHECK(StrongestEligibleSlot(weights, std::array<bool, 4>{ true, false, true, true }) == 2);
	CHECK(StrongestEligibleSlot(weights, std::array<bool, 4>{ false, false, false, true }) == std::nullopt);
	CHECK(StrongestEligibleSlot(weights, std::array<bool, 4>{ false, false, false, false }) == std::nullopt);
}
