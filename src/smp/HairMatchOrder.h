// SPDX-License-Identifier: GPL-3.0-or-later WITH LicenseRef-Modding-Exception

#pragma once

#include <cstddef>
#include <span>
#include <string>
#include <vector>

namespace SmpPhysics
{
	/** @brief Distances within this of the closest one are a tie: the same style at the precision the shape comparison has. */
	inline constexpr float kHairTieBand = 0.15f;

	/** @brief How far one list item's hair is from the target's: its index in the list and the shape distance. */
	struct HairRank
	{
		std::size_t index{};
		float distance{};
	};

	/** @brief A list shown closest first: the item indexes in display order and a badge per item, indexed by item. */
	struct HairOrdering
	{
		std::vector<int> order;
		std::vector<std::string> badges;
		std::size_t closest{};  ///< How many items at the front of order are in the tie with the closest.
	};

	/**
	 * @brief Puts the ranked items first, closest to farthest, then every other item in its own order.
	 * @details An index at or past a_count, a non-finite distance and a repeated index are ignored. The badge of an item in the closest tie says so
	 *          and gives its distance, the badge of another ranked item is its distance, and an unranked item has none.
	 */
	[[nodiscard]] HairOrdering OrderByHairMatch(std::span<const HairRank> a_ranked, std::size_t a_count);
}
