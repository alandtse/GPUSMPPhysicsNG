// Unit tests for the skinned-surface contact queries (src/smp/TriangleGrid.cpp).
// Pure maths; no game/D3D dependency.

#include "smp/TriangleGrid.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstdint>
#include <vector>

using namespace SmpPhysics;
using Catch::Approx;

namespace
{
	struct Mesh
	{
		std::vector<Vec3> vertices;
		std::vector<std::uint32_t> indices;
	};

	/** @brief A square in the z = 0 plane, a_half to each side, facing +Z. */
	Mesh MakeFloor(float a_half)
	{
		Mesh mesh;
		mesh.vertices = { { -a_half, -a_half, 0.0f }, { a_half, -a_half, 0.0f }, { a_half, a_half, 0.0f }, { -a_half, a_half, 0.0f } };
		mesh.indices = { 0, 1, 2, 0, 2, 3 };
		return mesh;
	}

	TriangleGrid BuildGrid(const Mesh& a_mesh)
	{
		TriangleGrid grid;
		grid.Build(a_mesh.vertices, a_mesh.indices, 2.0f);
		return grid;
	}
}

TEST_CASE("A sphere in front of a face touches it by its reach minus the gap", "[smpphysics][trianglegrid]")
{
	const TriangleGrid grid = BuildGrid(MakeFloor(5.0f));
	REQUIRE(grid.TriangleCount() == 2);

	const Contact contact = grid.Query({ 1.0f, 1.0f, 0.5f }, 1.0f, 0.4f);
	CHECK(contact.depth == Approx(0.5f));
	CHECK(contact.normal.z == Approx(1.0f));
}

TEST_CASE("A one-sided face holds a sphere up to its allowance behind it and ignores one past the deep push reach", "[smpphysics][trianglegrid]")
{
	const TriangleGrid grid = BuildGrid(MakeFloor(5.0f));

	CHECK(grid.Query({ 0.0f, 0.0f, -0.3f }, 1.0f, 0.4f).depth == Approx(1.3f));
	CHECK(grid.Query({ 0.0f, 0.0f, -(0.4f + kDeepPushReach + 0.1f) }, 1.0f, 0.4f).depth == 0.0f);
}

TEST_CASE("A one-sided face reaches kDeepPushReach behind itself at its full allowance", "[smpphysics][trianglegrid]")
{
	const TriangleGrid grid = BuildGrid(MakeFloor(5.0f));
	constexpr float kPenetration = 0.4f;

	// A sphere a unit behind the face is past the allowance alone, so only the deep push reach finds it.
	const Contact deep = grid.Query({ 0.0f, 0.0f, -1.0f }, 1.0f, kPenetration);
	CHECK(deep.depth == Approx(2.0f));
	CHECK(deep.normal.z == Approx(1.0f));

	// The reach ends at the allowance plus kDeepPushReach, and the depth itself is not capped.
	CHECK(grid.Query({ 0.0f, 0.0f, -(kPenetration + kDeepPushReach) }, 1.0f, kPenetration).depth > 0.0f);
	CHECK(grid.Query({ 0.0f, 0.0f, -(kPenetration + kDeepPushReach + 0.1f) }, 1.0f, kPenetration).depth == 0.0f);

	// A two-sided and a flipped face keep today's rule at the same offsets.
	CHECK(grid.Query({ 0.0f, 0.0f, -1.0f }, 1.0f, 0.0f).depth == 0.0f);
	CHECK(grid.Query({ 0.0f, 0.0f, -2.5f }, 1.0f, 0.0f).depth == 0.0f);
	CHECK(grid.Query({ 0.0f, 0.0f, -1.0f }, 1.0f, -kPenetration).depth == 0.0f);
	CHECK(grid.Query({ 0.0f, 0.0f, -2.5f }, 1.0f, -kPenetration).depth == 0.0f);
}

TEST_CASE("A zero deep reach leaves a one-sided face holding only its allowance", "[smpphysics][trianglegrid]")
{
	const TriangleGrid grid = BuildGrid(MakeFloor(5.0f));

	CHECK(grid.Query({ 0.0f, 0.0f, -1.0f }, 1.0f, 0.4f).depth > 0.0f);
	CHECK(grid.Query({ 0.0f, 0.0f, -1.0f }, 1.0f, 0.4f, 0.0f).depth == 0.0f);
	CHECK(grid.Query({ 0.0f, 0.0f, -0.3f }, 1.0f, 0.4f, 0.0f).depth == Approx(1.3f));
}

TEST_CASE("A zero allowance makes a face two-sided", "[smpphysics][trianglegrid]")
{
	const TriangleGrid grid = BuildGrid(MakeFloor(5.0f));

	const Contact above = grid.Query({ 0.0f, 0.0f, 0.25f }, 1.0f, 0.0f);
	CHECK(above.depth == Approx(0.75f));
	CHECK(above.normal.z == Approx(1.0f));

	const Contact below = grid.Query({ 0.0f, 0.0f, -0.25f }, 1.0f, 0.0f);
	CHECK(below.depth == Approx(0.75f));
	CHECK(below.normal.z == Approx(-1.0f));
}

TEST_CASE("A negative allowance flips the faces", "[smpphysics][trianglegrid]")
{
	const TriangleGrid grid = BuildGrid(MakeFloor(5.0f));

	const Contact contact = grid.Query({ 0.0f, 0.0f, -0.5f }, 1.0f, -0.4f);
	CHECK(contact.depth == Approx(0.5f));
	CHECK(contact.normal.z == Approx(-1.0f));
	CHECK(grid.Query({ 0.0f, 0.0f, 0.5f }, 1.0f, -0.4f).depth == 0.0f);
}

TEST_CASE("A sphere whose centre projects off the triangle touches nothing", "[smpphysics][trianglegrid]")
{
	const TriangleGrid grid = BuildGrid(MakeFloor(5.0f));

	CHECK(grid.Query({ 5.5f, 0.0f, 0.2f }, 1.0f, 0.4f).depth == 0.0f);
	CHECK(grid.Query({ 0.0f, 0.0f, 9.0f }, 1.0f, 0.4f).depth == 0.0f);
	CHECK(grid.Query({ 100.0f, -50.0f, 0.3f }, 1.0f, 0.4f).depth == 0.0f);
}

TEST_CASE("The deepest of several faces wins", "[smpphysics][trianglegrid]")
{
	Mesh stack = MakeFloor(5.0f);
	for (const Vec3& corner : { Vec3{ -5.0f, -5.0f, 0.4f }, Vec3{ 5.0f, -5.0f, 0.4f }, Vec3{ 5.0f, 5.0f, 0.4f } }) {
		stack.indices.push_back(static_cast<std::uint32_t>(stack.vertices.size()));
		stack.vertices.push_back(corner);
	}
	const TriangleGrid grid = BuildGrid(stack);

	CHECK(grid.Query({ 2.0f, -2.0f, 0.5f }, 1.0f, 0.4f).depth == Approx(0.9f));
}

TEST_CASE("Degenerate and out-of-range triangles are skipped and a huge mesh gets bigger cells", "[smpphysics][trianglegrid]")
{
	Mesh mesh;
	mesh.vertices = { { 0, 0, 0 }, { 1, 0, 0 }, { 2, 0, 0 }, { 0, 1, 0 }, { 5000, 5000, 5000 } };
	mesh.indices = { 0, 1, 2, 0, 1, 99, 0, 1, 3, 0, 3, 4 };
	TriangleGrid grid;
	grid.Build(mesh.vertices, mesh.indices, 0.01f);
	CHECK(grid.TriangleCount() == 2);
	CHECK(grid.Query({ 0.3f, 0.3f, 0.2f }, 0.5f, 0.4f).depth > 0.0f);
}

TEST_CASE("An empty grid answers every query with no contact", "[smpphysics][trianglegrid]")
{
	TriangleGrid grid;
	grid.Build({}, {}, 2.0f);
	CHECK(grid.TriangleCount() == 0);
	CHECK(grid.Query({ 0, 0, 0 }, 1.0f, 0.4f).depth == 0.0f);
}

TEST_CASE("Faces at exactly equal depth resolve to the lower triangle index", "[smpphysics][trianglegrid]")
{
	// Both faces are 0.5 from the point, so their depths tie exactly and only the triangle index can decide.
	const Vec3 facingX0{ 0.0f, -2.0f, -2.0f };
	const Vec3 facingX1{ 0.0f, 2.0f, 2.0f };
	const Vec3 facingX2{ 0.0f, -2.0f, 2.0f };
	const Vec3 facingZ0{ -2.0f, -2.0f, 0.0f };
	const Vec3 facingZ1{ 2.0f, -2.0f, 0.0f };
	const Vec3 facingZ2{ 2.0f, 2.0f, 0.0f };
	const Vec3 point{ 0.5f, 0.0f, 0.5f };

	Mesh xFirst;
	xFirst.vertices = { facingX0, facingX1, facingX2, facingZ0, facingZ1, facingZ2 };
	xFirst.indices = { 0, 1, 2, 3, 4, 5 };
	const Contact xWins = BuildGrid(xFirst).Query(point, 1.0f, 0.4f);
	CHECK(xWins.depth == Approx(0.5f));
	CHECK(xWins.normal.x == Approx(1.0f));
	CHECK(xWins.normal.z == Approx(0.0f));

	Mesh zFirst;
	zFirst.vertices = { facingZ0, facingZ1, facingZ2, facingX0, facingX1, facingX2 };
	zFirst.indices = { 0, 1, 2, 3, 4, 5 };
	const Contact zWins = BuildGrid(zFirst).Query(point, 1.0f, 0.4f);
	CHECK(zWins.depth == Approx(0.5f));
	CHECK(zWins.normal.z == Approx(1.0f));
	CHECK(zWins.normal.x == Approx(0.0f));
}
