#include "CrowdBases.h"

#include <algorithm>
#include <random>

namespace TestBench
{
	std::vector<std::uint32_t> PickFromIds(const std::vector<std::uint32_t>& a_ids, std::size_t a_count, bool a_randomOrder, std::uint32_t a_seed)
	{
		std::vector<std::uint32_t> picked;
		if (a_ids.empty())
			return picked;
		picked.reserve(a_count);
		const std::size_t size = a_ids.size();
		if (!a_randomOrder) {
			for (std::size_t index = 0; index < a_count; ++index)
				picked.push_back(a_ids[index % size]);
			return picked;
		}
		std::mt19937 generator{ a_seed };
		std::vector<std::uint32_t> deck;
		while (picked.size() < a_count) {
			deck = a_ids;
			std::shuffle(deck.begin(), deck.end(), generator);
			const std::size_t take = std::min(size, a_count - picked.size());
			picked.insert(picked.end(), deck.begin(), deck.begin() + static_cast<std::ptrdiff_t>(take));
		}
		return picked;
	}
}
