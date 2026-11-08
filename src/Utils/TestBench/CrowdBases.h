#pragma once

#include <cstddef>
#include <cstdint>
#include <string_view>
#include <vector>

namespace TestBench
{
	/** @brief An NPC base a test character can be made from, and the name the menu shows for it. */
	struct BaseChoice
	{
		std::uint32_t formId;
		std::string_view name;
	};

	/** @brief A set of NPC bases the crowd panel can make its characters from, by name. */
	struct BasePreset
	{
		std::string_view name;
		std::vector<BaseChoice> bases;
	};

	/** @brief The base sets the crowd panel offers: "Lydia only", then "Mixed women". */
	[[nodiscard]] const std::vector<BasePreset>& BasePresets();

	/**
	 * @brief Picks the bases for a_count characters from a preset.
	 * @details Without random order the preset's bases are taken in turn, repeating from the start. With it they are dealt like a
	 *          shuffled deck: every block of the preset's bases is a fresh permutation, so no base repeats until the set runs out.
	 * @param a_preset The set to pick from.
	 * @param a_count How many characters to pick bases for.
	 * @param a_randomOrder Whether to shuffle instead of taking the bases in order.
	 * @param a_seed Seed of the shuffle, so the same seed gives the same pick.
	 * @return Exactly a_count form IDs, or empty when the preset has no bases.
	 */
	[[nodiscard]] std::vector<std::uint32_t> PickBases(const BasePreset& a_preset, std::size_t a_count, bool a_randomOrder, std::uint32_t a_seed);

	/**
	 * @brief Picks the bases for a_count characters from a list of NPC base form IDs, the way PickBases does for a preset.
	 * @return Exactly a_count form IDs, or empty when a_ids is empty.
	 */
	[[nodiscard]] std::vector<std::uint32_t> PickFromIds(const std::vector<std::uint32_t>& a_ids, std::size_t a_count, bool a_randomOrder, std::uint32_t a_seed);
}
