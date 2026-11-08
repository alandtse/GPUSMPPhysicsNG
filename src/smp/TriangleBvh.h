#pragma once

#include "Math.h"

#include <cstddef>
#include <cstdint>
#include <vector>

namespace SmpPhysics
{
	/** @brief Most triangles one BVH leaf holds. */
	inline constexpr std::uint32_t kBvhLeafTriangles = 4;
	/** @brief Bit position of a leaf's first slot in BvhNode::leaf; the bits below hold its triangle count. */
	inline constexpr std::uint32_t kBvhLeafSlotShift = 3;
	inline constexpr std::uint32_t kBvhLeafCountMask = 7;
	/** @brief The box of a node with no usable triangle below it: low above high on every axis, neutral to a union. */
	inline constexpr float kBvhEmptyLow = 1e30f;
	inline constexpr float kBvhEmptyHigh = -1e30f;

	/**
	 * @brief One node of a TriangleBvh, laid out as the shader's HairBvhNode reads it.
	 * @details Nodes are stored depth first, so an internal node's first child is the next node. skip is the index
	 *          of the first node after this node's whole subtree, which is where a traversal goes when it rejects
	 *          the node or finishes a leaf. leaf is 0 for an internal node, otherwise (first slot << 3) | count,
	 *          with first slot an index into TriangleBvh::LeafOrder() and count in 1..kBvhLeafTriangles.
	 *          A node with no usable triangle below it has low = +1e30 and high = -1e30 on every axis.
	 */
	struct BvhNode
	{
		float low[3];
		std::uint32_t skip;
		float high[3];
		std::uint32_t leaf;
	};
	static_assert(sizeof(BvhNode) == 32);

	/**
	 * @brief A bounding volume hierarchy over a skinned surface's triangles whose topology is built once and whose
	 *        boxes are refitted whenever the surface is posed again.
	 * @details The topology covers every triangle of the index list, usable or not, so it stays valid as the pose
	 *          changes which triangles are degenerate. A triangle with an index past the vertex list or a non-finite
	 *          corner is placed by the origin and contributes nothing to any box.
	 */
	class TriangleBvh
	{
	public:
		/**
		 * @brief Builds the topology: median split of the triangle centres on the widest axis, leaves of up to
		 *        kBvhLeafTriangles, nodes flattened depth first. Boxes are fitted to the given vertices.
		 * @return false, leaving the hierarchy not ready, when the index list holds no triangle.
		 */
		bool Build(const std::vector<Vec3>& a_vertices, const std::vector<std::uint32_t>& a_indices);

		/**
		 * @brief Refits every box to the given vertices without changing the topology.
		 * @details The index list must be the one Build was given.
		 */
		void Refit(const std::vector<Vec3>& a_vertices, const std::vector<std::uint32_t>& a_indices);

		[[nodiscard]] bool Ready() const noexcept { return !m_nodes.empty(); }
		/** @brief Triangles the topology was built over, usable or not. */
		[[nodiscard]] std::size_t TriangleCount() const noexcept { return m_leafOrder.size(); }
		[[nodiscard]] const std::vector<BvhNode>& Nodes() const noexcept { return m_nodes; }
		/** @brief The source triangle index (position in the index list divided by three) of each leaf slot. */
		[[nodiscard]] const std::vector<std::uint32_t>& LeafOrder() const noexcept { return m_leafOrder; }

	private:
		std::vector<BvhNode> m_nodes;
		std::vector<std::uint32_t> m_leafOrder;
	};
}
