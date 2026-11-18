#pragma once

#include "CrowdStyles.h"

#include "Utils/TestBench/TestCrowd.h"

#include <atomic>
#include <cstddef>
#include <string>
#include <unordered_map>
#include <vector>

namespace SmpPhysics
{
	/** @brief One node a character's baked hair draws, with what the actor's 3D has of it. */
	struct BakedHairNode
	{
		std::string editorId;  ///< The head part's editor ID, or "(unnamed)" for a part without one.
		bool found = false;    ///< Whether the actor's 3D has a node with that editor ID.
		bool hidden = false;   ///< Whether that node has the hidden flag set.
	};

	/** @brief The nodes of an actor's baked hair: every hair head part of its base and each part's extras, in traversal order. */
	[[nodiscard]] std::vector<BakedHairNode> BakedHairNodes(RE::Actor& a_actor);

	/**
	 * @brief What GPU SMP Physics adds to the test crowd: each character wears a different hair-slot wig, its own hair is hidden under it,
	 *        and the crowd's hair colour can be changed.
	 */
	class CrowdWigs final : public TestBench::CrowdHooks
	{
	public:
		void BeforeSpawn(std::size_t a_count) override;
		void Dress(RE::Actor& a_actor, std::size_t a_index) override;
		[[nodiscard]] bool KeepWhenStripped(const RE::TESBoundObject& a_object) const override;
		void AfterRefresh(RE::Actor& a_actor) override;
		/**
		 * @brief Hides the baked hair of every character still being watched after Dress, once per frame.
		 * @details A new character has no 3D yet and a wig rebuilds it a few frames later, so hiding at Dress is not enough. Game thread only.
		 */
		void Tick();
		[[nodiscard]] std::string DescribeSpawn() const override;

		/** @brief Every wig the game has, by name then form ID: what the wig pickers list. Safe from any thread once the game has loaded. */
		[[nodiscard]] static const std::vector<RE::TESObjectARMO*>& WigForms();
		/** @brief The names behind WigForms(), in the same order, telling apart wigs that share one. */
		[[nodiscard]] static const std::vector<std::string>& WigNames();
		/** @brief The entry of WigForms() the last WearNow put on, or -1 while none was. */
		[[nodiscard]] int WornIndex() const noexcept { return wornIndex.load(std::memory_order_relaxed); }
		/** @brief Puts the wig at a_index of WigForms() on every character the crowd outfits, the player included while it is. Game thread only. */
		void WearNow(TestBench::TestCrowd& a_crowd, int a_index);

		/**
		 * @brief Sets the hair colour of every base the crowd is made from, then refreshes the characters standing now.
		 * @param a_color A name from TestCrowdHairColors(), or a BGSColorForm ID written as a decimal or 0x-prefixed number.
		 * @details Game thread only. The outcome goes to the crowd's report.
		 */
		void SetHairColorNow(TestBench::TestCrowd& a_crowd, const std::string& a_color);

		/**
		 * @brief Puts back the natural hair colour of every base SetHairColorNow changed, then refreshes the characters standing now.
		 * @details Game thread only. The outcome goes to the crowd's report.
		 */
		void RestoreNaturalColorsNow(TestBench::TestCrowd& a_crowd);

	private:
		/** @brief Puts a_wig on a_actor, hides the hair it was born with and watches that it stays hidden. */
		void Wear(RE::Actor& a_actor, RE::TESObjectARMO& a_wig);
		/** @brief A wig wearer watched for its own hair showing: frames left to watch and whether any hair was ever hidden. */
		struct BakedHairWatch
		{
			RE::ObjectRefHandle handle;
			int framesLeft = 0;
			bool everHidden = false;
		};
		std::vector<RE::TESObjectARMO*> chosen;                           ///< The wigs picked for the last spawn, one per distinct style.
		std::atomic<int> wornIndex{ -1 };                                 ///< Into WigForms() for the wig last worn, -1 for none. Written on the game thread.
		std::size_t requested = 0;                                        ///< How many characters the last spawn asked for.
		std::vector<BakedHairWatch> watches;                              ///< Game thread only.
		std::unordered_map<RE::FormID, RE::BGSColorForm*> naturalColors;  ///< Each base's own hair colour, recorded before the first change. Game thread only.
	};
}
