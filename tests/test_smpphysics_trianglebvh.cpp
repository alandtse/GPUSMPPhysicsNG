// Unit tests for the depth-first triangle BVH over a skinned surface (src/smp/TriangleBvh.cpp).
// Pure maths; no game/D3D dependency.

#include "smp/TriangleBvh.h"
#include "smp/TriangleGrid.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <random>
#include <vector>

using namespace SmpPhysics;
using Catch::Approx;

namespace
{
	constexpr float kTriangleAreaEpsilon = 1e-8f;

	struct Mesh
	{
		std::vector<Vec3> vertices;
		std::vector<std::uint32_t> indices;
	};

	float SurfaceHeight(float a_x, float a_y)
	{
		return 0.5f * std::sin(0.7f * a_x) * std::cos(0.5f * a_y);
	}

	/** @brief A bumpy quad grid in the z = 0 plane, two counter-clockwise triangles per quad, facing +Z. */
	Mesh MakeBumpySurface(int a_columns, int a_rows)
	{
		Mesh mesh;
		for (int row = 0; row <= a_rows; ++row) {
			for (int column = 0; column <= a_columns; ++column) {
				const float x = static_cast<float>(column);
				const float y = static_cast<float>(row);
				mesh.vertices.push_back({ x, y, SurfaceHeight(x, y) });
			}
		}
		const auto vertex = [a_columns](int a_column, int a_row) {
			return static_cast<std::uint32_t>(a_row * (a_columns + 1) + a_column);
		};
		for (int row = 0; row < a_rows; ++row) {
			for (int column = 0; column < a_columns; ++column) {
				const std::uint32_t v00 = vertex(column, row);
				const std::uint32_t v10 = vertex(column + 1, row);
				const std::uint32_t v01 = vertex(column, row + 1);
				const std::uint32_t v11 = vertex(column + 1, row + 1);
				mesh.indices.insert(mesh.indices.end(), { v00, v10, v11, v00, v11, v01 });
			}
		}
		return mesh;
	}

	void TranslateMesh(Mesh& a_mesh, const Vec3& a_offset)
	{
		for (Vec3& vertex : a_mesh.vertices)
			vertex = vertex + a_offset;
	}

	void RotateRightHalf(Mesh& a_mesh, float a_angle)
	{
		Vec3 low = a_mesh.vertices.front();
		Vec3 high = low;
		for (const Vec3& vertex : a_mesh.vertices) {
			low = { std::min(low.x, vertex.x), std::min(low.y, vertex.y), std::min(low.z, vertex.z) };
			high = { std::max(high.x, vertex.x), std::max(high.y, vertex.y), std::max(high.z, vertex.z) };
		}
		const float centreX = (low.x + high.x) * 0.5f;
		const float centreY = (low.y + high.y) * 0.5f;
		const float cosine = std::cos(a_angle);
		const float sine = std::sin(a_angle);
		for (Vec3& vertex : a_mesh.vertices) {
			if (vertex.x <= centreX)
				continue;
			const float x = vertex.x - centreX;
			const float y = vertex.y - centreY;
			vertex.x = centreX + x * cosine - y * sine;
			vertex.y = centreY + x * sine + y * cosine;
		}
	}

	std::vector<Vec3> NearSurfacePoints(const Mesh& a_mesh, int a_count, std::uint32_t a_seed)
	{
		std::mt19937 generator(a_seed);
		std::uniform_int_distribution<std::size_t> pickTriangle(0, a_mesh.indices.size() / 3 - 1);
		std::uniform_real_distribution<float> offset(-2.0f, 2.0f);
		std::vector<Vec3> points;
		points.reserve(a_count);
		for (int index = 0; index < a_count; ++index) {
			const std::size_t triangle = pickTriangle(generator);
			const Vec3 centre = (a_mesh.vertices[a_mesh.indices[triangle * 3]] + a_mesh.vertices[a_mesh.indices[triangle * 3 + 1]] +
									a_mesh.vertices[a_mesh.indices[triangle * 3 + 2]]) /
			                    3.0f;
			points.push_back({ centre.x + offset(generator), centre.y + offset(generator), centre.z + offset(generator) });
		}
		return points;
	}

	/** @brief TriangleGrid::Query over a stackless walk of the BVH, with the grid's per-triangle predicate. */
	Contact QueryBvh(const TriangleBvh& a_bvh, const std::vector<Vec3>& a_vertices, const std::vector<std::uint32_t>& a_indices,
		const Vec3& a_point, float a_reach, float a_penetration, float a_deepReach)
	{
		Contact best;
		float bestRank = -std::numeric_limits<float>::infinity();
		std::uint32_t bestIndex = std::numeric_limits<std::uint32_t>::max();
		const std::vector<BvhNode>& nodes = a_bvh.Nodes();
		if (nodes.empty() || !IsFiniteVector(a_point) || !std::isfinite(a_reach) || !(a_reach > 0.0f) || !std::isfinite(a_penetration))
			return best;

		const bool flipped = a_penetration < 0.0f;
		const float allowance = std::fabs(a_penetration);
		const float behind = a_penetration > 0.0f ? allowance + a_deepReach : allowance;
		const float extent = std::max(a_reach, behind);
		const Vec3 span{ extent, extent, extent };
		const Vec3 queryLow = a_point - span;
		const Vec3 queryHigh = a_point + span;

		std::size_t node = 0;
		while (node < nodes.size()) {
			const BvhNode& entry = nodes[node];
			const bool missed = entry.low[0] > queryHigh.x || entry.high[0] < queryLow.x || entry.low[1] > queryHigh.y ||
			                    entry.high[1] < queryLow.y || entry.low[2] > queryHigh.z || entry.high[2] < queryLow.z;
			if (missed) {
				node = entry.skip;
				continue;
			}
			if (entry.leaf == 0) {
				node = node + 1;
				continue;
			}
			const std::uint32_t firstSlot = entry.leaf >> 3;
			const std::uint32_t count = entry.leaf & 7u;
			for (std::uint32_t slot = firstSlot; slot < firstSlot + count; ++slot) {
				const std::uint32_t triangleIndex = a_bvh.LeafOrder()[slot];
				const std::uint32_t i0 = a_indices[triangleIndex * 3];
				const std::uint32_t i1 = a_indices[triangleIndex * 3 + 1];
				const std::uint32_t i2 = a_indices[triangleIndex * 3 + 2];
				if (i0 >= a_vertices.size() || i1 >= a_vertices.size() || i2 >= a_vertices.size())
					continue;
				const Vec3& a = a_vertices[i0];
				const Vec3& b = a_vertices[i1];
				const Vec3& c = a_vertices[i2];
				if (!IsFiniteVector(a) || !IsFiniteVector(b) || !IsFiniteVector(c))
					continue;
				const Vec3 cross = Cross(b - a, c - a);
				const float twiceArea = Length(cross);
				if (!(twiceArea > kTriangleAreaEpsilon))
					continue;
				Vec3 normal = cross / twiceArea;
				if (flipped)
					normal = normal * -1.0f;
				float distance = Dot(a_point - a, normal);
				if (allowance > 0.0f) {
					if (distance >= a_reach || distance < -behind)
						continue;
				} else {
					if (distance < 0.0f) {
						distance = -distance;
						normal = normal * -1.0f;
					}
					if (distance >= a_reach)
						continue;
				}
				const float depth = a_reach - distance;
				const float rank = -std::fabs(distance);
				if (rank < bestRank || (rank == bestRank && triangleIndex > bestIndex))
					continue;
				const Vec3 projected = a_point - normal * distance;
				const Vec3 toA = a - projected;
				const Vec3 toB = b - projected;
				const Vec3 toC = c - projected;
				const float areaA = Length(Cross(toB, toC));
				const float areaB = Length(Cross(toC, toA));
				const float areaC = Length(Cross(toA, toB));
				if (areaA + areaB > twiceArea || areaB + areaC > twiceArea || areaC + areaA > twiceArea)
					continue;
				best.depth = depth;
				best.normal = normal;
				bestRank = rank;
				bestIndex = triangleIndex;
			}
			node = entry.skip;
		}
		return best;
	}

	void CheckQueriesMatch(const TriangleBvh& a_bvh, const TriangleGrid& a_grid, const Mesh& a_mesh, const std::vector<Vec3>& a_points)
	{
		for (const Vec3& point : a_points) {
			for (const float penetration : { 0.5f, 0.0f, -0.5f }) {
				const float deepReach = penetration > 0.0f ? 2.0f : 0.0f;
				const Contact expected = a_grid.Query(point, 1.5f, penetration, deepReach);
				const Contact actual = QueryBvh(a_bvh, a_mesh.vertices, a_mesh.indices, point, 1.5f, penetration, deepReach);
				CHECK(actual.depth == Approx(expected.depth).margin(1e-5f));
				CHECK(actual.normal.x == Approx(expected.normal.x).margin(1e-5f));
				CHECK(actual.normal.y == Approx(expected.normal.y).margin(1e-5f));
				CHECK(actual.normal.z == Approx(expected.normal.z).margin(1e-5f));
			}
		}
	}

	TriangleBvh BuildBvh(const Mesh& a_mesh)
	{
		TriangleBvh bvh;
		bvh.Build(a_mesh.vertices, a_mesh.indices);
		return bvh;
	}
}

TEST_CASE("The BVH topology is depth first with boxes that contain their triangles", "[smpphysics][trianglebvh]")
{
	const Mesh mesh = MakeBumpySurface(12, 12);
	TriangleBvh bvh;
	REQUIRE(bvh.Build(mesh.vertices, mesh.indices));
	REQUIRE(bvh.Ready());

	const std::size_t triangleTotal = mesh.indices.size() / 3;
	CHECK(bvh.TriangleCount() == triangleTotal);

	const std::vector<BvhNode>& nodes = bvh.Nodes();
	REQUIRE(!nodes.empty());

	std::vector<std::uint32_t> order = bvh.LeafOrder();
	REQUIRE(order.size() == triangleTotal);
	std::sort(order.begin(), order.end());
	for (std::uint32_t index = 0; index < order.size(); ++index)
		REQUIRE(order[index] == index);

	for (std::size_t index = 0; index < nodes.size(); ++index) {
		CHECK(static_cast<std::size_t>(nodes[index].skip) > index);
		CHECK(static_cast<std::size_t>(nodes[index].skip) <= nodes.size());
	}
	CHECK(nodes[0].skip == nodes.size());

	const auto contains = [](const BvhNode& a_node, const Vec3& a_point) {
		return a_node.low[0] <= a_point.x && a_point.x <= a_node.high[0] && a_node.low[1] <= a_point.y && a_point.y <= a_node.high[1] &&
		       a_node.low[2] <= a_point.z && a_point.z <= a_node.high[2];
	};
	for (std::size_t index = 0; index < nodes.size(); ++index) {
		if (nodes[index].leaf != 0) {
			const std::uint32_t firstSlot = nodes[index].leaf >> 3;
			const std::uint32_t count = nodes[index].leaf & 7u;
			CHECK(count >= 1);
			CHECK(count <= kBvhLeafTriangles);
			REQUIRE(static_cast<std::size_t>(firstSlot) + count <= bvh.LeafOrder().size());
			for (std::uint32_t slot = firstSlot; slot < firstSlot + count; ++slot) {
				const std::uint32_t triangle = bvh.LeafOrder()[slot];
				for (int corner = 0; corner < 3; ++corner)
					CHECK(contains(nodes[index], mesh.vertices[mesh.indices[triangle * 3 + corner]]));
			}
			continue;
		}
		const BvhNode& firstChild = nodes[index + 1];
		const BvhNode& secondChild = nodes[firstChild.skip];
		for (int axis = 0; axis < 3; ++axis) {
			CHECK(nodes[index].low[axis] <= firstChild.low[axis]);
			CHECK(nodes[index].low[axis] <= secondChild.low[axis]);
			CHECK(nodes[index].high[axis] >= firstChild.high[axis]);
			CHECK(nodes[index].high[axis] >= secondChild.high[axis]);
		}
	}

	const std::vector<Vec3> noVertices;
	const std::vector<std::uint32_t> noIndices;
	TriangleBvh empty;
	CHECK_FALSE(empty.Build(noVertices, noIndices));
	CHECK_FALSE(empty.Ready());
}

TEST_CASE("A BVH walk answers every query exactly as the grid does", "[smpphysics][trianglebvh]")
{
	const Mesh mesh = MakeBumpySurface(12, 12);
	const TriangleBvh bvh = BuildBvh(mesh);
	REQUIRE(bvh.Ready());

	TriangleGrid grid;
	grid.Build(mesh.vertices, mesh.indices, 2.0f);

	CheckQueriesMatch(bvh, grid, mesh, NearSurfacePoints(mesh, 300, 20261017u));
}

TEST_CASE("A refit matches a fresh build and the grid after the surface moves", "[smpphysics][trianglebvh]")
{
	Mesh mesh = MakeBumpySurface(12, 8);
	TriangleBvh bvh;
	REQUIRE(bvh.Build(mesh.vertices, mesh.indices));

	TranslateMesh(mesh, { 3.0f, -2.0f, 1.5f });
	RotateRightHalf(mesh, 0.005f);
	bvh.Refit(mesh.vertices, mesh.indices);

	const TriangleBvh fresh = BuildBvh(mesh);
	REQUIRE(fresh.Ready());
	REQUIRE(bvh.Nodes().size() == fresh.Nodes().size());
	for (std::size_t index = 0; index < bvh.Nodes().size(); ++index) {
		for (int axis = 0; axis < 3; ++axis) {
			CHECK(bvh.Nodes()[index].low[axis] == Approx(fresh.Nodes()[index].low[axis]).margin(1e-5f));
			CHECK(bvh.Nodes()[index].high[axis] == Approx(fresh.Nodes()[index].high[axis]).margin(1e-5f));
		}
	}

	TriangleGrid grid;
	grid.Build(mesh.vertices, mesh.indices, 2.0f);
	CheckQueriesMatch(bvh, grid, mesh, NearSurfacePoints(mesh, 300, 424242u));
}

TEST_CASE("Unusable triangles stay in the topology but contribute no box or contact", "[smpphysics][trianglebvh]")
{
	Mesh mesh = MakeBumpySurface(6, 6);
	const std::vector<Vec3> points = NearSurfacePoints(mesh, 100, 7u);

	const std::uint32_t nanVertex = static_cast<std::uint32_t>(mesh.vertices.size());
	mesh.vertices.push_back({ std::numeric_limits<float>::quiet_NaN(), 0.0f, 0.0f });
	const std::uint32_t beyondVertices = static_cast<std::uint32_t>(mesh.vertices.size());
	mesh.indices.insert(mesh.indices.end(), { 0u, 1u, nanVertex, 0u, 1u, beyondVertices });

	TriangleBvh bvh;
	REQUIRE(bvh.Build(mesh.vertices, mesh.indices));
	REQUIRE(bvh.Ready());
	CHECK(bvh.TriangleCount() == mesh.indices.size() / 3);

	for (const BvhNode& node : bvh.Nodes()) {
		for (int axis = 0; axis < 3; ++axis) {
			CHECK(std::isfinite(node.low[axis]));
			CHECK(std::isfinite(node.high[axis]));
		}
	}

	TriangleGrid grid;
	grid.Build(mesh.vertices, mesh.indices, 2.0f);
	CheckQueriesMatch(bvh, grid, mesh, points);
}
