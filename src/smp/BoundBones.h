// SPDX-License-Identifier: GPL-3.0-or-later WITH LicenseRef-Modding-Exception

#pragma once

#include <cstddef>
#include <string>
#include <string_view>
#include <unordered_map>

namespace SmpPhysics
{
	/**
	 * @brief The bones a claim's drawn meshes are skinned to, by name, so a rig bone resolves to the node the mesh follows.
	 * @details A name can exist twice below one skeleton: when the game severs a head it clones the head bone and every bone below it into a
	 *          new limb node and binds the head's meshes to the clones. The skeleton's own bones stay with the body, so a rig resolved by
	 *          name from the skeleton would move bones no mesh is drawn from.
	 */
	/**
	 * @brief The top of the subtree a bound bone hangs in: its ancestor, or itself, whose parent is above the skeleton.
	 * @details That is the skeleton for an ordinary bone and the limb node for a severed one. Null when the bone is not below a parent of the skeleton.
	 * @param a_parentOf Callable (Node*) -> Node*: a node's parent, or null at the top.
	 */
	template <class Node, class ParentOf>
	[[nodiscard]] Node* SubtreeRoot(Node* a_node, Node* a_skeleton, ParentOf&& a_parentOf)
	{
		for (Node* current = a_node; current; current = a_parentOf(current)) {
			Node* parent = a_parentOf(current);
			for (Node* above = a_skeleton ? a_parentOf(a_skeleton) : nullptr; parent && above; above = a_parentOf(above)) {
				if (above == parent)
					return current;
			}
		}
		return nullptr;
	}

	template <class Node>
	class BoundBones
	{
	public:
		/** @brief Records a bone a mesh is skinned to; the first node seen under a name is kept. */
		void Add(std::string_view a_name, Node* a_node)
		{
			if (a_node && !a_name.empty())
				m_byName.try_emplace(std::string(a_name), a_node);
		}

		/**
		 * @brief The subtree most of the bound bones hang in, or a_skeleton when none is bound.
		 * @details A rig bone no mesh is skinned to is looked up there, so the whole rig lands in one copy of the bones.
		 */
		template <class ParentOf>
		[[nodiscard]] Node* CommonRoot(Node* a_skeleton, ParentOf&& a_parentOf) const
		{
			std::unordered_map<Node*, std::size_t> counts;
			Node* best = a_skeleton;
			std::size_t bestCount = 0;
			for (const auto& entry : m_byName) {
				Node* root = SubtreeRoot(entry.second, a_skeleton, a_parentOf);
				if (!root)
					continue;
				const std::size_t count = ++counts[root];
				if (count > bestCount || (count == bestCount && root == a_skeleton)) {
					best = root;
					bestCount = count;
				}
			}
			return best;
		}

		/** @brief The node a rig bone drives: the one a mesh is skinned to under that name, else the skeleton's. */
		[[nodiscard]] Node* Resolve(std::string_view a_name, Node* a_skeletonNode) const
		{
			const auto bound = m_byName.find(std::string(a_name));
			return bound != m_byName.end() ? bound->second : a_skeletonNode;
		}

	private:
		std::unordered_map<std::string, Node*> m_byName;
	};
}
