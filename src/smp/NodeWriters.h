// SPDX-License-Identifier: GPL-3.0-or-later WITH LicenseRef-Modding-Exception

#pragma once

#include <cstdint>
#include <unordered_map>
#include <vector>

namespace SmpPhysics
{
	/**
	 * @brief Per simulated node, the claim whose pose it holds: the last to write it in a frame.
	 * @details A claim writes the same nodes every frame it writes, so a frame only names the claims that wrote, in order, and the map
	 *          is folded again only when that list or the claim set differs from the frame before.
	 */
	class NodeWriters
	{
	public:
		/** @brief Notes that a claim wrote its nodes this frame. */
		void Wrote(std::uint64_t a_token) { m_frame.push_back(a_token); }

		/**
		 * @brief Folds this frame's writes into the map and forgets the nodes of claims that are gone.
		 * @param a_claimSet A serial the caller raises whenever a claim is added, replaced or removed.
		 * @param a_alive Callable (token) -> bool: whether the claim still exists.
		 * @param a_forEachNode Callable (token, visit): calls visit(node) for every node the claim writes.
		 * @return Whether any node changed its writer.
		 */
		template <class Alive, class ForEachNode>
		bool EndFrame(std::uint64_t a_claimSet, Alive&& a_alive, ForEachNode&& a_forEachNode)
		{
			if (a_claimSet == m_notedClaimSet && m_frame == m_noted) {
				m_frame.clear();
				return false;
			}
			bool changed = std::erase_if(m_writer, [&](const auto& a_entry) { return !a_alive(a_entry.second); }) != 0;
			for (const std::uint64_t token : m_frame) {
				if (!a_alive(token))
					continue;
				a_forEachNode(token, [&](const void* a_node) {
					const auto [entry, added] = m_writer.try_emplace(a_node, token);
					changed = changed || added || entry->second != token;
					entry->second = token;
				});
			}
			m_noted.swap(m_frame);
			m_frame.clear();
			m_notedClaimSet = a_claimSet;
			return changed;
		}

		/** @brief The claim that last wrote a node, or null when none has. */
		[[nodiscard]] const std::uint64_t* Find(const void* a_node) const
		{
			const auto entry = m_writer.find(a_node);
			return entry == m_writer.end() ? nullptr : &entry->second;
		}

	private:
		std::unordered_map<const void*, std::uint64_t> m_writer;
		std::vector<std::uint64_t> m_frame;  ///< this frame's writers, in order
		std::vector<std::uint64_t> m_noted;  ///< the writers m_writer was last folded from
		std::uint64_t m_notedClaimSet{};
	};
}
