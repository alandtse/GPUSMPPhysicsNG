#include "TriangleGrid.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace SmpPhysics
{
	namespace
	{
		constexpr float kMinimumTriangleArea = 1e-8f;
		constexpr float kMinimumCellSize = 1e-3f;

		int CellOf(float a_value, float a_origin, float a_cellSize, int a_dimension) noexcept
		{
			const float cell = std::floor((a_value - a_origin) / a_cellSize);
			return std::clamp(static_cast<int>(cell), 0, a_dimension - 1);
		}
	}

	void TriangleGrid::Build(const std::vector<Vec3>& a_vertices, const std::vector<std::uint32_t>& a_indices, float a_cellSize)
	{
		m_triangles.clear();
		m_cellStart.clear();
		m_cellItems.clear();
		m_dimensions[0] = m_dimensions[1] = m_dimensions[2] = 0;

		const std::size_t triangleTotal = a_indices.size() / 3;
		m_triangles.reserve(triangleTotal);
		m_boxes.clear();
		m_boxes.reserve(triangleTotal);
		Vec3 low{ std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max() };
		Vec3 high{ std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest() };
		for (std::size_t index = 0; index < triangleTotal; ++index) {
			const std::uint32_t i0 = a_indices[index * 3];
			const std::uint32_t i1 = a_indices[index * 3 + 1];
			const std::uint32_t i2 = a_indices[index * 3 + 2];
			if (i0 >= a_vertices.size() || i1 >= a_vertices.size() || i2 >= a_vertices.size())
				continue;
			const Vec3& a = a_vertices[i0];
			const Vec3& b = a_vertices[i1];
			const Vec3& c = a_vertices[i2];
			if (!IsFiniteVector(a) || !IsFiniteVector(b) || !IsFiniteVector(c))
				continue;
			const Vec3 cross = Cross(b - a, c - a);
			const float twiceArea = Length(cross);
			if (!(twiceArea > kMinimumTriangleArea))
				continue;
			m_triangles.push_back({ a, b, c, cross / twiceArea, twiceArea });
			const Vec3 triLow{ std::min(a.x, std::min(b.x, c.x)), std::min(a.y, std::min(b.y, c.y)), std::min(a.z, std::min(b.z, c.z)) };
			const Vec3 triHigh{ std::max(a.x, std::max(b.x, c.x)), std::max(a.y, std::max(b.y, c.y)), std::max(a.z, std::max(b.z, c.z)) };
			m_boxes.push_back({ triLow, triHigh });
			low = { std::min(low.x, triLow.x), std::min(low.y, triLow.y), std::min(low.z, triLow.z) };
			high = { std::max(high.x, triHigh.x), std::max(high.y, triHigh.y), std::max(high.z, triHigh.z) };
		}
		if (m_triangles.empty())
			return;

		float cell = std::max(a_cellSize, kMinimumCellSize);
		const Vec3 extent = high - low;
		for (;;) {
			const auto cells = [&](float a_extent) { return static_cast<std::size_t>(std::floor(a_extent / cell)) + 1; };
			if (cells(extent.x) * cells(extent.y) * cells(extent.z) <= kMaximumGridCells)
				break;
			cell *= 1.5f;
		}
		m_origin = low;
		m_cellSize = cell;
		m_dimensions[0] = static_cast<int>(std::floor(extent.x / cell)) + 1;
		m_dimensions[1] = static_cast<int>(std::floor(extent.y / cell)) + 1;
		m_dimensions[2] = static_cast<int>(std::floor(extent.z / cell)) + 1;

		const std::size_t cellTotal = static_cast<std::size_t>(m_dimensions[0]) * m_dimensions[1] * m_dimensions[2];
		m_cellStart.assign(cellTotal + 1, 0u);
		m_ranges.resize(m_triangles.size());
		for (std::size_t index = 0; index < m_triangles.size(); ++index) {
			int first[3];
			int last[3];
			if (CellRange(m_boxes[index].low, m_boxes[index].high, first, last))
				m_ranges[index] = { first[0], first[1], first[2], last[0], last[1], last[2] };
			else
				m_ranges[index] = { 0, 0, 0, -1, -1, -1 };
		}
		const auto forEachCell = [&](std::size_t a_triangle, auto&& a_visit) {
			const std::array<int, 6>& range = m_ranges[a_triangle];
			for (int z = range[2]; z <= range[5]; ++z) {
				for (int y = range[1]; y <= range[4]; ++y) {
					for (int x = range[0]; x <= range[3]; ++x)
						a_visit(CellIndex(x, y, z));
				}
			}
		};
		for (std::size_t index = 0; index < m_triangles.size(); ++index)
			forEachCell(index, [&](std::size_t a_cell) { ++m_cellStart[a_cell + 1]; });
		for (std::size_t cellIndex = 0; cellIndex < cellTotal; ++cellIndex)
			m_cellStart[cellIndex + 1] += m_cellStart[cellIndex];
		m_cellItems.resize(m_cellStart[cellTotal]);
		m_fill.assign(m_cellStart.begin(), m_cellStart.end() - 1);
		for (std::size_t index = 0; index < m_triangles.size(); ++index)
			forEachCell(index, [&](std::size_t a_cell) { m_cellItems[m_fill[a_cell]++] = static_cast<std::uint32_t>(index); });
	}

	bool TriangleGrid::CellRange(const Vec3& a_low, const Vec3& a_high, int (&a_first)[3], int (&a_last)[3]) const noexcept
	{
		if (m_dimensions[0] <= 0)
			return false;
		const float lows[3] = { a_low.x, a_low.y, a_low.z };
		const float highs[3] = { a_high.x, a_high.y, a_high.z };
		const float origins[3] = { m_origin.x, m_origin.y, m_origin.z };
		for (int axis = 0; axis < 3; ++axis) {
			if (highs[axis] < origins[axis] || lows[axis] >= origins[axis] + static_cast<float>(m_dimensions[axis]) * m_cellSize)
				return false;
		}
		for (int axis = 0; axis < 3; ++axis) {
			a_first[axis] = CellOf(lows[axis], origins[axis], m_cellSize, m_dimensions[axis]);
			a_last[axis] = CellOf(highs[axis], origins[axis], m_cellSize, m_dimensions[axis]);
		}
		return true;
	}

	std::size_t TriangleGrid::CellIndex(int a_x, int a_y, int a_z) const noexcept
	{
		return (static_cast<std::size_t>(a_z) * m_dimensions[1] + a_y) * m_dimensions[0] + a_x;
	}

	bool TriangleGrid::MayTouch(const Vec3& a_point, float a_reach) const noexcept
	{
		if (m_triangles.empty() || !IsFiniteVector(a_point) || !std::isfinite(a_reach))
			return false;
		const Vec3 span{ a_reach, a_reach, a_reach };
		int first[3];
		int last[3];
		if (!CellRange(a_point - span, a_point + span, first, last))
			return false;
		for (int z = first[2]; z <= last[2]; ++z) {
			for (int y = first[1]; y <= last[1]; ++y) {
				for (int x = first[0]; x <= last[0]; ++x) {
					const std::size_t cell = CellIndex(x, y, z);
					if (m_cellStart[cell + 1] > m_cellStart[cell])
						return true;
				}
			}
		}
		return false;
	}

	namespace
	{
		/** @brief Whether a_point, a_distance from the triangle's plane along a_normal, projects onto the triangle. */
		bool ProjectsInside(const TriangleGrid::Triangle& a_triangle, const Vec3& a_point, const Vec3& a_normal, float a_distance) noexcept
		{
			const Vec3 projected = a_point - a_normal * a_distance;
			const Vec3 toA = a_triangle.a - projected;
			const Vec3 toB = a_triangle.b - projected;
			const Vec3 toC = a_triangle.c - projected;
			const float areaA = Length(Cross(toB, toC));
			const float areaB = Length(Cross(toC, toA));
			const float areaC = Length(Cross(toA, toB));
			return !(areaA + areaB > a_triangle.twiceArea || areaB + areaC > a_triangle.twiceArea || areaC + areaA > a_triangle.twiceArea);
		}
	}

	float TriangleGrid::NearestFaceDistance(const Vec3& a_point, float a_range, Vec3* a_normal) const noexcept
	{
		float nearest = std::numeric_limits<float>::infinity();
		if (m_triangles.empty() || !IsFiniteVector(a_point) || !(a_range > 0.0f))
			return nearest;
		const Vec3 span{ a_range, a_range, a_range };
		int first[3];
		int last[3];
		if (!CellRange(a_point - span, a_point + span, first, last))
			return nearest;
		for (int z = first[2]; z <= last[2]; ++z) {
			for (int y = first[1]; y <= last[1]; ++y) {
				for (int x = first[0]; x <= last[0]; ++x) {
					const std::size_t cell = CellIndex(x, y, z);
					for (std::uint32_t item = m_cellStart[cell]; item < m_cellStart[cell + 1]; ++item) {
						const Triangle& triangle = m_triangles[m_cellItems[item]];
						const float distance = Dot(a_point - triangle.a, triangle.normal);
						if (std::fabs(distance) < std::fabs(nearest) && std::fabs(distance) <= a_range && ProjectsInside(triangle, a_point, triangle.normal, distance)) {
							nearest = distance;
							if (a_normal != nullptr)
								*a_normal = triangle.normal;
						}
					}
				}
			}
		}
		return nearest;
	}

	float TriangleGrid::RadialDepthInside(const Vec3& a_point, float a_reach, float a_allowance, const Vec3& a_axisPoint, Vec3* a_normal) const noexcept
	{
		constexpr float infinity = std::numeric_limits<float>::infinity();
		if (m_triangles.empty() || !IsFiniteVector(a_point) || !(a_reach > 0.0f))
			return infinity;
		Vec3 outward{ a_point.x - a_axisPoint.x, a_point.y - a_axisPoint.y, 0.0f };
		if (!(Length(outward) > 1.0e-4f))
			return infinity;
		outward = Normalized(outward);
		const Vec3 end = a_point + outward * a_reach;
		const Vec3 low{ std::min(a_point.x, end.x), std::min(a_point.y, end.y), a_point.z };
		const Vec3 high{ std::max(a_point.x, end.x), std::max(a_point.y, end.y), a_point.z };
		int first[3];
		int last[3];
		if (!CellRange(low, high, first, last))
			return infinity;
		// An exit through an outward face means the point was inside the shell; an exit within the allowance is the
		// contact's own business, so the walk goes on past it.
		float exit = infinity;
		float entry = infinity;
		Vec3 exitNormal{};
		for (int z = first[2]; z <= last[2]; ++z) {
			for (int y = first[1]; y <= last[1]; ++y) {
				for (int x = first[0]; x <= last[0]; ++x) {
					const std::size_t cell = CellIndex(x, y, z);
					for (std::uint32_t item = m_cellStart[cell]; item < m_cellStart[cell + 1]; ++item) {
						const Triangle& triangle = m_triangles[m_cellItems[item]];
						const float facing = Dot(triangle.normal, outward);
						if (std::fabs(facing) < 1.0e-6f)
							continue;
						const float along = Dot(triangle.a - a_point, triangle.normal) / facing;
						if (!(along > 0.0f) || along > a_reach || !ProjectsInside(triangle, a_point + outward * along, triangle.normal, 0.0f))
							continue;
						if (facing > 0.0f) {
							if (along > a_allowance && along < exit) {
								exit = along;
								exitNormal = triangle.normal;
							}
						} else if (along > a_allowance && along < entry) {
							entry = along;
						}
					}
				}
			}
		}
		if (!std::isfinite(exit) || entry < exit)
			return infinity;
		if (a_normal != nullptr)
			*a_normal = outward;
		return -exit;
	}

	Contact TriangleGrid::Query(const Vec3& a_point, float a_reach, float a_penetration, float a_deepReach) const noexcept
	{
		Contact best;
		std::uint32_t bestIndex = std::numeric_limits<std::uint32_t>::max();
		if (m_triangles.empty() || !IsFiniteVector(a_point) || !std::isfinite(a_reach) || !(a_reach > 0.0f) || !std::isfinite(a_penetration))
			return best;

		const bool flipped = a_penetration < 0.0f;
		const float allowance = std::fabs(a_penetration);
		const float behind = a_penetration > 0.0f ? allowance + a_deepReach : allowance;
		const float extent = std::max(a_reach, behind);
		const Vec3 span{ extent, extent, extent };
		int first[3];
		int last[3];
		if (!CellRange(a_point - span, a_point + span, first, last))
			return best;
		for (int z = first[2]; z <= last[2]; ++z) {
			for (int y = first[1]; y <= last[1]; ++y) {
				for (int x = first[0]; x <= last[0]; ++x) {
					const std::size_t cell = CellIndex(x, y, z);
					for (std::uint32_t item = m_cellStart[cell]; item < m_cellStart[cell + 1]; ++item) {
						const std::uint32_t triangleIndex = m_cellItems[item];
						const Triangle& triangle = m_triangles[triangleIndex];
						Vec3 normal = flipped ? triangle.normal * -1.0f : triangle.normal;
						float distance = Dot(a_point - triangle.a, normal);
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
						// A GPU kernel enumerates the same triangles in another order, so an exact tie breaks on the lower index.
						if (depth < best.depth || (depth == best.depth && triangleIndex > bestIndex))
							continue;
						if (!ProjectsInside(triangle, a_point, normal, distance))
							continue;
						best.depth = depth;
						best.normal = normal;
						bestIndex = triangleIndex;
					}
				}
			}
		}
		return best;
	}
}
