#pragma once

#include "Utils/TestBench/CrowdFilter.h"

#include <cstddef>
#include <cstdint>
#include <vector>

namespace TestBench
{
	/** @brief How a spawn picks the NPC bases of its characters. */
	struct SpawnChoice
	{
		std::size_t count = 10;                 ///< How many characters to pick bases for.
		std::vector<std::uint32_t> candidates;  ///< Base form IDs to pick from; ignored when anyAdultWoman is set.
		bool anyAdultWoman = false;             ///< Pick from every named, non-templated female NPC of a humanoid race the game loaded that is not hostile to the player.
		bool randomOrder = true;                ///< Shuffle the picks instead of taking the candidates in order.
		std::uint32_t seed = 0;                 ///< Seed of the shuffle.
		CharacterFilter filter;                 ///< Which kinds of character to leave out.
	};

	/** @brief Every named adult woman the game loaded who is in the potential follower faction and not hostile to the player, in form order. Call it on the game thread. */
	[[nodiscard]] std::vector<std::uint32_t> CollectFemaleFollowers();

	/**
	 * @brief Picks the bases of a spawn, leaving out the characters the filter excludes. Call it on the game thread.
	 * @details Logs how many candidates there were and how many children and elderly the filter left out, with the races it matched.
	 * @return The picked base form IDs, or empty when no candidate passes the filter.
	 */
	[[nodiscard]] std::vector<std::uint32_t> ChooseBases(const SpawnChoice& a_choice);

	/**
	 * @brief Spawns the test crowd with bases picked by ChooseBases, on the game thread. Safe to call from the UI thread.
	 * @details When no candidate passes the filter nothing spawns and the crowd report says so.
	 */
	void SpawnCrowd(SpawnChoice a_choice);
}
