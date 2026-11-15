#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "IdleCache.h"

#include "RE/Skyrim.h"

namespace TestBench
{
	/**
	 * @brief What a feature adds to the test crowd: its own dressing of each character, and what survives stripping.
	 * @details Features register one with TestCrowd::AddHooks (GPU SMP Physics gives each character a wig, for example). Hooks run on the
	 *          game thread, except KeepWhenStripped, which may also run on a devbench thread and must depend only on the object.
	 */
	struct CrowdHooks
	{
		virtual ~CrowdHooks() = default;
		/** @brief A spawn of a_count characters is about to place them. */
		virtual void BeforeSpawn(std::size_t a_count) { (void)a_count; }
		/** @brief The a_index-th character is placed and its base is right; add what this feature wants it to wear or have. */
		virtual void Dress(RE::Actor& a_actor, std::size_t a_index)
		{
			(void)a_actor;
			(void)a_index;
		}
		/** @return Whether stripping armor and weapons leaves this worn object on. */
		[[nodiscard]] virtual bool KeepWhenStripped(const RE::TESBoundObject& a_object) const
		{
			(void)a_object;
			return false;
		}
		/** @brief A character's 3D was just rebuilt; redo anything that rebuild undoes. */
		virtual void AfterRefresh(RE::Actor& a_actor) { (void)a_actor; }
		/** @return A sentence about what this feature did in the last spawn, added to the report, or empty. */
		[[nodiscard]] virtual std::string DescribeSpawn() const { return {}; }
	};

	/**
	 * @brief Developer tool: characters standing in front of the player, made from NPC bases, for testing how a feature renders characters.
	 * @details Spawn and Clear queue their work for the game thread, so they are safe to call from the menu or a devbench command. A feature
	 *          adds its own touches through CrowdHooks; the crowd itself knows nothing about hair, fur or weather.
	 */
	class TestCrowd
	{
	public:
		TestCrowd();
		~TestCrowd();
		TestCrowd(const TestCrowd&) = delete;
		TestCrowd& operator=(const TestCrowd&) = delete;

		/**
		 * @brief Spawns a_count characters in rows ahead of the player, reusing the characters of an earlier spawn that are made from the right base.
		 * @param a_bases Form IDs of the NPC bases to make them from, taken in turn; empty makes them all from one generic female character.
		 */
		void Spawn(std::size_t a_count, std::vector<std::uint32_t> a_bases = {});
		/** @brief Removes every character this tool spawned. */
		void Clear();
		/** @brief Takes armor and weapons off the characters standing now, except what hooks keep,, or puts it back; characters spawned later start stripped while it is on. */
		void SetStripped(bool a_stripped);
		/**
		 * @brief Makes the outfit controls (stripping, armor and cloak) act on the player as well as the spawned characters, or not.
		 * @details Turning it on dresses the player in what the crowd wears now; turning it off puts the player's own outfit back and takes the cloak off.
		 */
		void SetIncludePlayer(bool a_include);
		/** @brief Dresses every character in the next (a_step 1) or previous body armor of ArmorNames(), with weapons and other gear off and what hooks keep left on. */
		void CycleArmor(int a_step);
		/** @brief Dresses every character in the body armor at a_index of ArmorNames(), as CycleArmor does. */
		void SelectArmor(std::size_t a_index);
		/**
		 * @brief The playable body armors cycling and picking step through, by name.
		 * @details Enchanted armors and armors that look like an earlier one (same models) are left out, so every entry looks different. Built once; safe from any thread.
		 */
		[[nodiscard]] static const std::vector<std::string>& ArmorNames();
		/** @brief The armors behind ArmorNames(), in the same order. Safe from any thread once the game has loaded. */
		[[nodiscard]] static std::vector<RE::TESObjectARMO*> ArmorForms();
		/** @brief The cloaks behind CloakNames(), in the same order. */
		[[nodiscard]] static std::vector<RE::TESObjectARMO*> CloakForms();
		/** @brief The entry of ArmorNames() the crowd wears now, or -1 while it wears none of them. */
		[[nodiscard]] int WornArmorIndex() const noexcept { return armorIndex.load(std::memory_order_relaxed); }
		/**
		 * @brief Puts the cloak or cape at a_index of CloakNames() on every character, and on each one spawned later; -1 takes it off.
		 * @details The cloak is a layer of its own: it stays on through stripping and armor changes, until it is taken off.
		 */
		void SelectCloak(int a_index);
		/**
		 * @brief The cloaks and capes the crowd can wear, by name: armors that do not cover the body and whose models sit in a cloak or cape folder or file.
		 * @details Enchanted cloaks and cloaks built from the same addons as an earlier one are left out. Built once; safe from any thread.
		 */
		[[nodiscard]] static const std::vector<std::string>& CloakNames();
		/** @brief The entry of CloakNames() the crowd wears now, or -1 while it wears none. */
		[[nodiscard]] int WornCloakIndex() const noexcept { return cloakIndex.load(std::memory_order_relaxed); }
		/** @brief Rebuilds the 3D of every character standing now, then lets the hooks redo what the rebuild undoes. */
		void Refresh();
		/** @brief Registers hooks a feature wants run for this crowd; they live as long as the feature. Call from the game thread at load. */
		void AddHooks(CrowdHooks* a_hooks);
		/** @return How many characters were refreshed. Game thread only. */
		std::size_t RefreshNow();
		/** @brief The distinct NPC bases of the last spawn, or the default base before any spawn. Game thread only. */
		[[nodiscard]] std::vector<RE::FormID> Bases() const;
		/**
		 * @brief Turns a test character to face the player rotated by a_degrees, so one camera position shows it from the front, side or back.
		 * @param a_index Which character, in spawn order; -1 turns them all.
		 */
		void Turn(int a_index, float a_degrees);
		/** @brief Plays the idle with this editor ID on every character standing now, and on each character spawned later; empty plays the default. */
		void PlayAnimation(std::string a_editorId);
		/** @brief Whether characters spawn animating, which keeps their AI running; off leaves them frozen, so a feature can be tested on its own. */
		void SetAnimateOnSpawn(bool a_animate) { animateOnSpawn.store(a_animate, std::memory_order_relaxed); }
		/** @brief Whether a character plays its idle again whenever the animation ends, so a dance repeats indefinitely. */
		void SetLoopAnimation(bool a_loop);
		/** @brief The editor ID of the idle characters play when spawned; a Cicero idle until one is chosen. */
		[[nodiscard]] std::string Animation() const;
		/** @brief What one test character wears now: its form ID and the names of its worn armor and weapons, and of what hooks keep on it. */
		struct WornSummary
		{
			RE::FormID actor{};
			std::vector<std::string> worn;
			std::vector<std::string> kept;
		};
		/** @brief What every test character wears now; safe to call from any thread. */
		[[nodiscard]] std::vector<WornSummary> Worn() const;
		/** @brief Replaces the sentence the menu and testCrowdReport show. */
		void SetReport(std::string a_text);
		/** @brief The characters standing now, as handles. Safe to call from any thread. */
		[[nodiscard]] std::vector<RE::ObjectRefHandle> Standing() const;
		/** @brief Characters standing now. */
		[[nodiscard]] std::size_t Count() const noexcept { return count.load(std::memory_order_relaxed); }
		/** @brief What the last Spawn or Clear did, in a sentence for the menu. */
		[[nodiscard]] std::string Report() const;

	private:
		/** @brief What a character was wearing before it was stripped, and the armor cycled onto it. */
		struct Outfit
		{
			std::vector<RE::TESBoundObject*> taken;
			RE::TESObjectARMO* cycled = nullptr;
		};
		/** @return Whether a hook keeps this worn object on through stripping. */
		[[nodiscard]] bool KeptByHooks(const RE::TESBoundObject& a_object) const;
		void StripActor(RE::Actor& a_actor);
		void RestoreActor(RE::Actor& a_actor);
		void SetStrippedNow(bool a_stripped);
		void SetIncludePlayerNow(bool a_include);
		/** @brief The spawned characters with 3D loaded, then the player when included. Game thread only. */
		[[nodiscard]] std::vector<RE::Actor*> OutfitTargets();
		void CycleArmorNow(int a_step);
		void SelectCloakNow(int a_index);
		/** @brief Puts the chosen cloak on a character that does not wear it; does nothing while none is chosen. */
		void DressCloak(RE::Actor& a_actor);
		void DressArmorNow(std::size_t a_index);
		void PlayAnimationNow(const std::string& a_editorId);
		void SpawnNow(std::size_t a_count, const std::vector<std::uint32_t>& a_bases);
		void TurnNow(int a_index, float a_degrees);
		void ClearNow();
		/** @brief Disables the next few characters ClearNow queued and schedules the rest for the next frame. Game thread only. */
		void DisableBatchNow();
		/** @brief Disables every character still queued, so a spawn or another clear starts from a settled world. Game thread only. */
		void FinishDisableNow();
		/** @brief Starts listening to the character's animation graph so a finished idle can be played again. Game thread only. */
		void WatchForIdleEnd(RE::Actor& a_actor);
		struct IdleEndSink;

		mutable std::mutex mutex;
		std::vector<RE::ObjectRefHandle> spawned;  ///< Written on the game thread only, under mutex; read there freely and elsewhere under mutex.
		std::string report;                        ///< Guarded by mutex.
		std::string animation;                     ///< The chosen idle's editor ID, or empty for the default. Guarded by mutex.
		std::atomic<std::size_t> count{ 0 };
		std::atomic<bool> stripped{ false };
		std::atomic<bool> includePlayer{ false };
		std::atomic<bool> animateOnSpawn{ false };
		std::atomic<bool> loopAnimation{ false };
		std::unique_ptr<IdleEndSink> idleEndSink;
		std::vector<CrowdHooks*> hooks;                   ///< Registered by features at load; read on the game thread and by Worn.
		std::vector<RE::FormID> crowdBases;               ///< The distinct bases of the last spawn, for hooks that change a base. Game thread only.
		std::unordered_set<RE::FormID> watched;           ///< Characters whose graph idleEndSink listens to. Game thread only.
		std::unordered_map<RE::FormID, Outfit> outfits;   ///< Touched on the game thread only.
		std::vector<RE::ObjectRefHandle> pendingDisable;  ///< Characters ClearNow has not disabled yet. Game thread only.
		std::size_t clearTotal = 0;                       ///< How many characters the clear in progress removes. Game thread only.
		std::atomic<int> armorIndex{ -1 };                ///< Into ArmorNames() for the armor the crowd wears, -1 for none. Written on the game thread.
		std::atomic<int> cloakIndex{ -1 };                ///< Into CloakNames() for the cloak the crowd wears, -1 for none. Written on the game thread.
		RE::TESObjectARMO* cloak = nullptr;               ///< The cloak behind cloakIndex. Game thread only.
	};

	/** @brief The process-wide test crowd. */
	[[nodiscard]] TestCrowd& GetCrowd();
}
