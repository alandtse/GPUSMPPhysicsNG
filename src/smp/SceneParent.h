#pragma once

namespace SmpPhysics
{
	/**
	 * @brief The first node above a_start, going up through a_parentOf, that a_isBone accepts; null when the chain ends first.
	 * @note A rig bone hung from a node hdtSMP64 added (an armor's own copy of a skeleton bone) still has the bone the file names as its first rig ancestor.
	 */
	template <class Node, class ParentOf, class IsBone>
	[[nodiscard]] const Node* NearestBoneAncestor(const Node* a_start, ParentOf a_parentOf, IsBone a_isBone)
	{
		constexpr int kMaxDepth = 256;
		const Node* node = a_start ? a_parentOf(a_start) : nullptr;
		for (int depth = 0; node && depth < kMaxDepth; ++depth, node = a_parentOf(node)) {
			if (a_isBone(node))
				return node;
		}
		return nullptr;
	}
}
