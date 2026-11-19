// SPDX-License-Identifier: GPL-3.0-or-later WITH LicenseRef-Modding-Exception

#include "smp/HairMatchOrder.h"

#include <algorithm>
#include <cmath>
#include <format>

namespace SmpPhysics
{
	HairOrdering OrderByHairMatch(std::span<const HairRank> a_ranked, std::size_t a_count)
	{
		std::vector<HairRank> usable;
		std::vector<bool> ranked(a_count, false);
		std::vector<float> distance(a_count, 0.0f);
		for (const HairRank& rank : a_ranked) {
			if (rank.index >= a_count || ranked[rank.index] || !std::isfinite(rank.distance))
				continue;
			ranked[rank.index] = true;
			distance[rank.index] = rank.distance;
			usable.push_back(rank);
		}
		std::ranges::stable_sort(usable, [](const HairRank& a_left, const HairRank& a_right) { return a_left.distance < a_right.distance; });

		HairOrdering result;
		result.badges.resize(a_count);
		result.order.reserve(a_count);
		for (const HairRank& rank : usable) {
			const bool tied = rank.distance <= usable.front().distance + kHairTieBand;
			result.closest += tied ? 1 : 0;
			result.badges[rank.index] = tied ? std::format("closest {:.2f}", rank.distance) : std::format("{:.2f}", rank.distance);
			result.order.push_back(static_cast<int>(rank.index));
		}
		for (std::size_t index = 0; index < a_count; ++index) {
			if (!ranked[index])
				result.order.push_back(static_cast<int>(index));
		}
		return result;
	}
}
