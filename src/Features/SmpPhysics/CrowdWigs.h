#pragma once

#include "CrowdStyles.h"

#include "Utils/TestBench/TestCrowd.h"
#include "smp/OwnWig.h"

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
		void Restore(RE::Actor& a_actor) override;
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

		/** @brief Puts the wig at a_index of WigForms() on a_actor alone, leaving the crowd's own choice as it is; false when there is no such wig. Game thread only. */
		bool WearOn(RE::Actor& a_actor, int a_index);
		/** @brief Takes the wig a_actor wears off and shows its own hair again, or puts back the wig it wore before WearOn first changed it. Game thread only. */
		void TakeOff(RE::Actor& a_actor);
		/** @brief Takes the wig off every character the crowd outfits and lets later spawns wear their own again. Game thread only. */
		void TakeOffNow(TestBench::TestCrowd& a_crowd);

		/** @brief The wig a_actor wears now, or null. Game thread only. */
		[[nodiscard]] static RE::TESObjectARMO* WornWig(RE::Actor& a_actor);
		/** @brief The form ID of the wig that is a_actor's own look: the one it wore before WearOn changed it while WearOn's wig is still on, else the one it wears now; 0 for none. Game thread only. */
		[[nodiscard]] RE::FormID NaturalWig(RE::Actor& a_actor) const;

		/**
		 * @brief Sets the hair colour of every base the crowd is made from, then refreshes the characters standing now.
		 * @param a_color A name from TestCrowdHairColors(), or a BGSColorForm ID written as a decimal or 0x-prefixed number.
		 * @details Game thread only. The outcome goes to the crowd's report.
		 */
		void SetHairColorNow(TestBench::TestCrowd& a_crowd, const std::string& a_color);

		/**
		 * @brief Sets a_actor's own base to a_color, an empty one putting its natural colour back, and rebuilds its 3D so a worn wig takes it.
		 * @details Every character made from the same base changes too. Game thread only. The outcome goes to the crowd's report.
		 */
		void SetActorHairColorNow(TestBench::TestCrowd& a_crowd, RE::Actor& a_actor, const std::string& a_color);

		/**
		 * @brief Puts back the natural hair colour of every base SetHairColorNow changed, then refreshes the characters standing now.
		 * @details Game thread only. The outcome goes to the crowd's report.
		 */
		void RestoreNaturalColorsNow(TestBench::TestCrowd& a_crowd);

	private:
		/** @brief Stops watching a_actor and shows the baked hair that was hidden. */
		void ShowBakedHair(RE::Actor& a_actor);
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
		std::unordered_map<RE::FormID, OwnWig> naturalWigs;               ///< Each actor WearOn changed. Game thread only.
		std::unordered_map<RE::FormID, RE::BGSColorForm*> naturalColors;  ///< Each base's own hair colour, recorded before the first change. Game thread only.
	};
}
