#pragma once

#include "CrowdWigs.h"
#include "smp/HairMatchOrder.h"
#include "smp/HairTargetPick.h"

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

namespace SmpPhysics
{
	/** @brief The kinds of item the target can wear. */
	enum class TargetItem
	{
		Armor,
		Cloak,
		Wig
	};

	/** @brief What the UI shows of the hair target: who it is, how far the ranking of the wigs by their match to its hair has got, and that ranking. */
	struct HairTargetView
	{
		std::uint32_t formID{};  ///< 0 for the player.
		std::string text;
		std::string status;
		std::vector<int> order;
		std::vector<std::string> badges;
		std::pair<std::size_t, std::size_t> progress{};
		bool hasConsole{};
		bool hasCrosshair{};
		bool pinned{};
	};

	/**
	 * @brief The character whose hair the user matches and swaps, found from the console selection and the crosshair, and the wigs ranked by
	 *        how closely their shape matches that character's hair.
	 * @details Tick runs on the game thread; the other members are safe from any thread.
	 */
	class HairTarget
	{
	public:
		/**
		 * @brief Samples the console selection and the crosshair, and ranks the wigs against a new target a little each frame until done.
		 * @details A target that wears a wig is matched by that wig, not by the hair head part the wig hides; a_wigs tells which wig is its own.
		 */
		void Tick(bool a_menuOpen, const CrowdWigs& a_wigs);
		/** @brief Pins a_source as the target; false when it has no character. */
		bool Pin(HairTargetSource a_source);
		/** @brief Lets the console selection set the target again. */
		void Unpin();
		[[nodiscard]] HairTargetView View() const;
		/** @brief The target's form ID, 0 for the player. */
		[[nodiscard]] std::uint32_t FormID() const;

	private:
		void Rank(std::uint32_t a_formID, RE::FormID a_wig);

		mutable std::mutex mutex;
		HairTargetPick pick;
		HairTargetView view;
		std::uint32_t rankedFor{ UINT32_MAX };  ///< The target the view's ranking is for, or is being made for.
		RE::FormID rankedWig{};                 ///< The wig that stood for the target's hair in that ranking, 0 for none.
		bool ranking{};
		std::chrono::steady_clock::time_point sampledAt{};
	};
}
