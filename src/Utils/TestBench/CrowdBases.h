#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

namespace TestBench
{
	/**
	 * @brief Picks the bases for a_count characters from a list of NPC base form IDs.
	 * @details Without random order the bases are taken in turn, repeating from the start. With it they are dealt like a
	 *          shuffled deck: every block of the bases is a fresh permutation, so no base repeats until the set runs out.
	 * @param a_randomOrder Whether to shuffle instead of taking the bases in order.
	 * @param a_seed Seed of the shuffle, so the same seed gives the same pick.
	 * @return Exactly a_count form IDs, or empty when a_ids is empty.
	 */
	[[nodiscard]] std::vector<std::uint32_t> PickFromIds(const std::vector<std::uint32_t>& a_ids, std::size_t a_count, bool a_randomOrder, std::uint32_t a_seed);
}
