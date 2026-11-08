#include "TriangleBvh.h"

#include <algorithm>
#include <cmath>
#include <numeric>

namespace SmpPhysics
{
	namespace
	{
		bool IsUsable(const std::vector<Vec3>& a_vertices, const std::vector<std::uint32_t>& a_indices, std::uint32_t a_triangle) noexcept
		{
			const std::uint32_t i0 = a_indices[a_triangle * 3];
			const std::uint32_t i1 = a_indices[a_triangle * 3 + 1];
			const std::uint32_t i2 = a_indices[a_triangle * 3 + 2];
			if (i0 >= a_vertices.size() || i1 >= a_vertices.size() || i2 >= a_vertices.size())
				return false;
			return IsFiniteVector(a_vertices[i0]) && IsFiniteVector(a_vertices[i1]) && IsFiniteVector(a_vertices[i2]);
		}

		void AddTriangleBox(const std::vector<Vec3>& a_vertices, const std::vector<std::uint32_t>& a_indices, std::uint32_t a_triangle, Vec3& a_low, Vec3& a_high) noexcept
		{
			if (!IsUsable(a_vertices, a_indices, a_triangle))
				return;
			const Vec3& a = a_vertices[a_indices[a_triangle * 3]];
			const Vec3& b = a_vertices[a_indices[a_triangle * 3 + 1]];
			const Vec3& c = a_vertices[a_indices[a_triangle * 3 + 2]];
			a_low = { std::min(a_low.x, std::min(a.x, std::min(b.x, c.x))),
				std::min(a_low.y, std::min(a.y, std::min(b.y, c.y))),
				std::min(a_low.z, std::min(a.z, std::min(b.z, c.z))) };
			a_high = { std::max(a_high.x, std::max(a.x, std::max(b.x, c.x))),
				std::max(a_high.y, std::max(a.y, std::max(b.y, c.y))),
				std::max(a_high.z, std::max(a.z, std::max(b.z, c.z))) };
		}

		// The empty sentinel is neutral to min and max, so an empty child drops out of the union.
		void AddNodeBox(const BvhNode& a_node, Vec3& a_low, Vec3& a_high) noexcept
		{
			a_low = { std::min(a_low.x, a_node.low[0]), std::min(a_low.y, a_node.low[1]), std::min(a_low.z, a_node.low[2]) };
			a_high = { std::max(a_high.x, a_node.high[0]), std::max(a_high.y, a_node.high[1]), std::max(a_high.z, a_node.high[2]) };
		}

		void FitBoxes(std::vector<BvhNode>& a_nodes, const std::vector<std::uint32_t>& a_leafOrder,
			const std::vector<Vec3>& a_vertices, const std::vector<std::uint32_t>& a_indices)
		{
			for (std::size_t index = a_nodes.size(); index-- > 0;) {
				Vec3 low{ kBvhEmptyLow, kBvhEmptyLow, kBvhEmptyLow };
				Vec3 high{ kBvhEmptyHigh, kBvhEmptyHigh, kBvhEmptyHigh };
				BvhNode& node = a_nodes[index];
				if (node.leaf != 0) {
					const std::uint32_t firstSlot = node.leaf >> kBvhLeafSlotShift;
					const std::uint32_t count = node.leaf & kBvhLeafCountMask;
					for (std::uint32_t slot = firstSlot; slot < firstSlot + count; ++slot)
						AddTriangleBox(a_vertices, a_indices, a_leafOrder[slot], low, high);
				} else {
					const BvhNode& firstChild = a_nodes[index + 1];
					const BvhNode& secondChild = a_nodes[firstChild.skip];
					AddNodeBox(firstChild, low, high);
					AddNodeBox(secondChild, low, high);
				}
				node.low[0] = low.x;
				node.low[1] = low.y;
				node.low[2] = low.z;
				node.high[0] = high.x;
				node.high[1] = high.y;
				node.high[2] = high.z;
			}
		}
	}

	bool TriangleBvh::Build(const std::vector<Vec3>& a_vertices, const std::vector<std::uint32_t>& a_indices)
	{
		m_nodes.clear();
		m_leafOrder.clear();
		if (a_indices.size() < 3)
			return false;

		const std::size_t triangleTotal = a_indices.size() / 3;
		m_leafOrder.resize(triangleTotal);
		std::iota(m_leafOrder.begin(), m_leafOrder.end(), 0u);

		std::vector<Vec3> centres(triangleTotal);
		for (std::size_t triangle = 0; triangle < triangleTotal; ++triangle) {
			const std::uint32_t index = static_cast<std::uint32_t>(triangle);
			if (IsUsable(a_vertices, a_indices, index))
				centres[index] = (a_vertices[a_indices[index * 3]] + a_vertices[a_indices[index * 3 + 1]] + a_vertices[a_indices[index * 3 + 2]]) / 3.0f;
		}

		m_nodes.reserve(triangleTotal * 2);
		const auto emit = [&](auto&& a_self, std::uint32_t a_begin, std::uint32_t a_end) -> void {
			const std::uint32_t node = static_cast<std::uint32_t>(m_nodes.size());
			m_nodes.push_back({});
			if (a_end - a_begin <= kBvhLeafTriangles) {
				m_nodes[node].skip = node + 1;
				m_nodes[node].leaf = (a_begin << kBvhLeafSlotShift) | (a_end - a_begin);
				return;
			}

			Vec3 low{ kBvhEmptyLow, kBvhEmptyLow, kBvhEmptyLow };
			Vec3 high{ kBvhEmptyHigh, kBvhEmptyHigh, kBvhEmptyHigh };
			for (std::uint32_t slot = a_begin; slot < a_end; ++slot) {
				const Vec3& centre = centres[m_leafOrder[slot]];
				low = { std::min(low.x, centre.x), std::min(low.y, centre.y), std::min(low.z, centre.z) };
				high = { std::max(high.x, centre.x), std::max(high.y, centre.y), std::max(high.z, centre.z) };
			}
			const Vec3 extent = high - low;
			const int axis = extent.x >= extent.y && extent.x >= extent.z ? 0 : (extent.y >= extent.z ? 1 : 2);
			const std::uint32_t middle = a_begin + (a_end - a_begin) / 2;
			std::nth_element(m_leafOrder.begin() + a_begin, m_leafOrder.begin() + middle, m_leafOrder.begin() + a_end,
				[&](std::uint32_t a_left, std::uint32_t a_right) {
					const Vec3& left = centres[a_left];
					const Vec3& right = centres[a_right];
					return axis == 0 ? left.x < right.x : (axis == 1 ? left.y < right.y : left.z < right.z);
				});

			a_self(a_self, a_begin, middle);
			a_self(a_self, middle, a_end);
			m_nodes[node].skip = static_cast<std::uint32_t>(m_nodes.size());
		};
		emit(emit, 0, static_cast<std::uint32_t>(triangleTotal));

		FitBoxes(m_nodes, m_leafOrder, a_vertices, a_indices);
		return true;
	}

	void TriangleBvh::Refit(const std::vector<Vec3>& a_vertices, const std::vector<std::uint32_t>& a_indices)
	{
		if (m_nodes.empty() || a_indices.size() / 3 != m_leafOrder.size())
			return;
		FitBoxes(m_nodes, m_leafOrder, a_vertices, a_indices);
	}
}
