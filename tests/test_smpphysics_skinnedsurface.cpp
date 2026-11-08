// Unit tests for skinned collision surfaces (src/smp/SkinnedSurface.cpp).
// Pure maths; no game/D3D dependency.

#include "smp/SkinnedSurface.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cstdint>
#include <vector>

using namespace SmpPhysics;
using Catch::Approx;

namespace
{
	SkinnedVertex Vertex(Vec3 a_position, int a_boneA, float a_weightA, int a_boneB = -1, float a_weightB = 0.0f)
	{
		SkinnedVertex vertex;
		vertex.position = a_position;
		vertex.bone[0] = a_boneA;
		vertex.weight[0] = a_weightA;
		vertex.bone[1] = a_boneB;
		vertex.weight[1] = a_weightB;
		return vertex;
	}

	Transform Translated(Vec3 a_offset)
	{
		Transform transform;
		transform.translation = a_offset;
		return transform;
	}

	void CheckSamePosition(Vec3 a_left, Vec3 a_right)
	{
		CHECK(a_left.x == Approx(a_right.x));
		CHECK(a_left.y == Approx(a_right.y));
		CHECK(a_left.z == Approx(a_right.z));
	}
}

TEST_CASE("A triangle is kept when a bone's weight summed over its vertices is above the bone's threshold", "[smpphysics][skinnedsurface]")
{
	const std::vector<SkinnedVertex> vertices = {
		Vertex({ 0, 0, 0 }, 0, 0.3f),
		Vertex({ 1, 0, 0 }, 0, 0.3f),
		Vertex({ 0, 1, 0 }, 0, 0.3f),
		Vertex({ 5, 5, 5 }, 1, 0.9f),
		Vertex({ 6, 5, 5 }, 1, 0.9f),
		Vertex({ 5, 6, 5 }, 1, 0.9f)
	};
	const std::vector<std::uint32_t> indices = { 0, 1, 2, 3, 4, 5 };

	SECTION("bones with no listed threshold keep anything weighted")
	{
		CHECK(KeepWeightedTriangles(vertices, indices, { 0.0f, 0.0f }).size() == 6);
	}
	SECTION("a bone's threshold removes triangles whose summed weight to it is below it")
	{
		const auto kept = KeepWeightedTriangles(vertices, indices, { 1.0f, 0.0f });
		REQUIRE(kept.size() == 3);
		CHECK(kept[0] == 3);
	}
	SECTION("weights split across vertices add up to pass a threshold no single vertex passes")
	{
		CHECK(KeepWeightedTriangles(vertices, indices, { 0.5f, 0.0f }).size() == 6);
	}
	SECTION("a threshold at or above the summed weight removes it too")
	{
		CHECK(KeepWeightedTriangles(vertices, indices, { 1.0f, 3.0f }).empty());
	}
}

TEST_CASE("Triangles naming a missing vertex are dropped", "[smpphysics][skinnedsurface]")
{
	const std::vector<SkinnedVertex> vertices = { Vertex({ 0, 0, 0 }, 0, 1.0f), Vertex({ 1, 0, 0 }, 0, 1.0f), Vertex({ 0, 1, 0 }, 0, 1.0f) };
	CHECK(KeepWeightedTriangles(vertices, { 0, 1, 2, 0, 1, 7 }, { 0.0f }).size() == 3);
}

TEST_CASE("A posed vertex follows the drivers its bones are weighted to", "[smpphysics][skinnedsurface]")
{
	SkinnedSurface surface;
	surface.vertices = { Vertex({ 1, 0, 0 }, 0, 1.0f), Vertex({ 1, 0, 0 }, 0, 0.5f, 1, 0.5f) };
	surface.skinToBone = { Transform{}, Transform{} };
	surface.boneDriver = { 0, 1 };

	const std::vector<Transform> drivers = { Translated({ 10, 0, 0 }), Translated({ 0, 20, 0 }) };
	const std::vector<Vec3> positions = PoseSurface(surface, drivers);
	REQUIRE(positions.size() == 2);
	CHECK(positions[0].x == Approx(11.0f));
	CHECK(positions[0].y == Approx(0.0f));
	CHECK(positions[1].x == Approx(6.0f));
	CHECK(positions[1].y == Approx(10.0f));
}

TEST_CASE("A bone nothing drives is skipped and a vertex with no driven bone keeps its bind position", "[smpphysics][skinnedsurface]")
{
	SkinnedSurface surface;
	surface.vertices = { Vertex({ 1, 2, 3 }, 0, 1.0f), Vertex({ 1, 0, 0 }, 0, 0.5f, 1, 0.5f) };
	surface.skinToBone = { Transform{}, Transform{} };
	surface.boneDriver = { -1, 0 };

	const std::vector<Vec3> positions = PoseSurface(surface, { Translated({ 10, 0, 0 }) });
	REQUIRE(positions.size() == 2);
	CHECK(positions[0].x == Approx(1.0f));
	CHECK(positions[0].y == Approx(2.0f));
	CHECK(positions[0].z == Approx(3.0f));
	CHECK(positions[1].x == Approx(11.0f));
}

TEST_CASE("Only triangles whose three vertices all lie within the radius are kept", "[smpphysics][skinnedsurface]")
{
	const std::vector<Vec3> positions = {
		{ 0, 0, 0 }, { 10, 0, 0 }, { 0, 10, 0 }, { 20, 0, 0 }, { 100, 0, 0 }
	};

	SECTION("a triangle whose vertices are all inside the radius is kept whole")
	{
		const auto kept = KeepTrianglesNear(positions, { 0, 1, 2 }, { 0, 0, 0 }, 20.0f);
		CHECK(kept == std::vector<std::uint32_t>{ 0, 1, 2 });
	}
	SECTION("a single vertex outside the radius drops the whole triangle")
	{
		const auto kept = KeepTrianglesNear(positions, { 0, 1, 3, 0, 1, 4 }, { 0, 0, 0 }, 20.0f);
		CHECK(kept == std::vector<std::uint32_t>{ 0, 1, 3 });
	}
	SECTION("a vertex exactly on the radius is inside it")
	{
		CHECK(KeepTrianglesNear(positions, { 0, 1, 3 }, { 0, 0, 0 }, 20.0f).size() == 3);
	}
	SECTION("empty input, and a triangle naming a missing vertex, keep nothing")
	{
		CHECK(KeepTrianglesNear(positions, {}, { 0, 0, 0 }, 20.0f).empty());
		CHECK(KeepTrianglesNear(positions, { 0, 1, 9 }, { 0, 0, 0 }, 20.0f).empty());
		CHECK(KeepTrianglesNear({}, { 0, 1, 2 }, { 0, 0, 0 }, 20.0f).empty());
	}
}

TEST_CASE("A compacted surface drops the vertices its kept triangles do not use", "[smpphysics][skinnedsurface]")
{
	SkinnedSurface surface;
	surface.vertices = {
		Vertex({ 0, 0, 0 }, 0, 1.0f), Vertex({ 1, 0, 0 }, 0, 1.0f), Vertex({ 2, 0, 0 }, 0, 1.0f),
		Vertex({ 3, 0, 0 }, 0, 1.0f), Vertex({ 4, 0, 0 }, 0, 1.0f), Vertex({ 5, 0, 0 }, 0, 1.0f)
	};
	// Six vertices, of which these triangles reach only 1, 2, 4 and 5.
	surface.indices = { 5, 1, 4, 4, 2, 5 };
	surface.skinToBone = { Transform{} };
	surface.boneDriver = { 0 };

	std::vector<Vec3> before;
	for (const std::uint32_t index : surface.indices)
		before.push_back(surface.vertices[index].position);

	SECTION("the surface keeps four vertices and the triangles keep their corners")
	{
		CompactSurface(surface);
		CHECK(surface.vertices.size() == 4);
		REQUIRE(surface.indices.size() == 6);
		for (std::size_t corner = 0; corner < surface.indices.size(); ++corner) {
			REQUIRE(surface.indices[corner] < surface.vertices.size());
			CheckSamePosition(surface.vertices[surface.indices[corner]].position, before[corner]);
		}
	}
	SECTION("a surface whose triangles use every vertex comes out unchanged")
	{
		surface.indices = { 0, 1, 2, 3, 4, 5 };
		CompactSurface(surface);
		CHECK(surface.vertices.size() == 6);
		CHECK(surface.indices == std::vector<std::uint32_t>{ 0, 1, 2, 3, 4, 5 });
		for (std::size_t vertex = 0; vertex < surface.vertices.size(); ++vertex)
			CheckSamePosition(surface.vertices[vertex].position, { static_cast<float>(vertex), 0.0f, 0.0f });
	}
}
