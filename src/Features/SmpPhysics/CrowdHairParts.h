// SPDX-License-Identifier: GPL-3.0-or-later WITH LicenseRef-Modding-Exception

#pragma once

#include "Utils/TestBench/TestCrowd.h"

#include <atomic>
#include <string>
#include <unordered_map>
#include <vector>

namespace SmpPhysics
{
	/** @brief Swaps the hairstyle (the hair head part) of the test crowd's bases or of one character, and puts each back. */
	class CrowdHairParts
	{
	public:
		/** @brief Every hair head part a hairstyle can be picked from, sorted by name; built once the game's forms are loaded. */
		[[nodiscard]] static const std::vector<RE::BGSHeadPart*>& Parts();
		/** @brief The names behind Parts(), in the same order. */
		[[nodiscard]] static const std::vector<std::string>& Names();
		/** @brief The entry of Parts() the last pick used, or -1 while none was. */
		[[nodiscard]] int WornIndex() const noexcept { return wornIndex.load(std::memory_order_relaxed); }

		/** @brief Gives the part at a_index of Parts() to every base the crowd is made from that it suits, then refreshes the characters standing. Game thread only; the outcome goes to the crowd's report. */
		void SetNow(TestBench::TestCrowd& a_crowd, int a_index);
		/** @brief Puts back the hair every base had before SetNow or SetActorNow changed it, then refreshes the characters standing. Game thread only. */
		void RestoreNow(TestBench::TestCrowd& a_crowd);
		/** @brief SetNow for a_actor's base alone. Game thread only. */
		void SetActorNow(TestBench::TestCrowd& a_crowd, RE::Actor& a_actor, int a_index);
		/** @brief Puts back the hair a_actor's base had before it was changed. Game thread only. */
		void RestoreActorNow(TestBench::TestCrowd& a_crowd, RE::Actor& a_actor);

	private:
		/** @brief Gives a_base a_part, remembering the hair it had; false when a_part does not suit it. */
		bool Give(RE::TESNPC& a_base, RE::BGSHeadPart& a_part);

		std::atomic<int> wornIndex{ -1 };
		std::unordered_map<RE::FormID, RE::FormID> naturalParts;  ///< Each base's own hair part, recorded before the first change. Game thread only.
	};
}
