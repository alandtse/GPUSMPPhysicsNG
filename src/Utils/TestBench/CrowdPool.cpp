#include "CrowdPool.h"

#include "Utils/TestBench/CrowdBases.h"
#include "Utils/TestBench/TestCrowd.h"

#include <algorithm>
#include <format>
#include <map>
#include <string>

namespace TestBench
{
	namespace
	{
		constexpr const char* kHumanoidKeyword = "ActorTypeNPC";
		constexpr RE::FormID kPlayerBaseFormId = 0x7;
		constexpr RE::FormID kPlayerFactionFormId = 0xDB1;
		constexpr RE::FormID kPotentialFollowerFactionFormId = 0x5C84D;

		CharacterTraits TraitsOf(const RE::TESNPC& a_npc)
		{
			CharacterTraits traits;
			if (a_npc.race) {
				traits.childRace = a_npc.race->IsChildRace();
				traits.raceEditorId = a_npc.race->GetFormEditorID();
			}
			return traits;
		}

		bool ReactsAsEnemy(RE::TESFaction& a_faction, const RE::TESForm* a_other)
		{
			for (const RE::GROUP_REACTION* reaction : a_faction.reactions) {
				if (reaction && reaction->form == a_other && reaction->fightReaction == RE::FIGHT_REACTION::kEnemy)
					return true;
			}
			return false;
		}

		/** @brief Whether a character would fight the player: very aggressive, or in a faction the player faction treats as an enemy or that treats it so. */
		bool IsHostileToPlayer(const RE::TESNPC& a_npc)
		{
			if (a_npc.GetAggressionLevel() >= RE::ACTOR_AGGRESSION::kVeryAggressive)
				return true;
			auto* playerFaction = RE::TESForm::LookupByID<RE::TESFaction>(kPlayerFactionFormId);
			if (!playerFaction)
				return false;
			for (const RE::FACTION_RANK& membership : a_npc.factions) {
				if (membership.faction && (ReactsAsEnemy(*membership.faction, playerFaction) || ReactsAsEnemy(*playerFaction, membership.faction)))
					return true;
			}
			return false;
		}

		bool IsAdultWomanCandidate(const RE::TESNPC& a_npc)
		{
			if (a_npc.GetFormID() == kPlayerBaseFormId || a_npc.IsDeleted() || !a_npc.IsFemale() || a_npc.baseTemplateForm || IsHostileToPlayer(a_npc))
				return false;
			const char* name = a_npc.GetName();
			return name && *name && a_npc.race && a_npc.race->HasKeywordString(kHumanoidKeyword);
		}

		std::vector<std::uint32_t> CollectAdultWomen()
		{
			std::vector<std::uint32_t> ids;
			if (auto* handler = RE::TESDataHandler::GetSingleton()) {
				for (const RE::TESNPC* npc : handler->GetFormArray<RE::TESNPC>()) {
					if (npc && IsAdultWomanCandidate(*npc))
						ids.push_back(npc->GetFormID());
				}
			}
			return ids;
		}
	}

	std::vector<std::uint32_t> CollectFemaleFollowers()
	{
		std::vector<std::uint32_t> ids;
		auto* followers = RE::TESForm::LookupByID<RE::TESFaction>(kPotentialFollowerFactionFormId);
		auto* handler = RE::TESDataHandler::GetSingleton();
		if (!followers || !handler)
			return ids;
		for (const RE::TESNPC* npc : handler->GetFormArray<RE::TESNPC>()) {
			if (npc && IsAdultWomanCandidate(*npc) && std::ranges::any_of(npc->factions, [followers](const RE::FACTION_RANK& a_membership) { return a_membership.faction == followers; }))
				ids.push_back(npc->GetFormID());
		}
		return ids;
	}

	std::vector<std::uint32_t> ChooseBases(const SpawnChoice& a_choice)
	{
		const std::vector<std::uint32_t> candidates = a_choice.anyAdultWoman ? CollectAdultWomen() : a_choice.candidates;
		std::vector<std::uint32_t> passing;
		std::size_t children = 0;
		std::size_t elderly = 0;
		std::map<std::string, std::size_t> leftOutRaces;
		for (const std::uint32_t formId : candidates) {
			const auto* npc = RE::TESForm::LookupByID<RE::TESNPC>(formId);
			if (!npc)
				continue;
			const CharacterTraits traits = TraitsOf(*npc);
			if (PassesFilter(traits, a_choice.filter)) {
				passing.push_back(formId);
				continue;
			}
			++(traits.childRace ? children : elderly);
			++leftOutRaces[std::string(traits.raceEditorId)];
		}
		std::string races;
		for (const auto& [race, count] : leftOutRaces)
			races += std::format("{}{} x{}", races.empty() ? "" : ", ", race.empty() ? "(no editor ID)" : race, count);
		logger::info("[Test Crowd] Character pool: {} candidates, {} pass the filter; left out {} children and {} elderly{}{}", candidates.size(), passing.size(), children, elderly, races.empty() ? "" : ": ", races);
		return PickFromIds(passing, a_choice.count, a_choice.randomOrder, a_choice.seed);
	}

	void SpawnCrowd(SpawnChoice a_choice)
	{
		if (auto* task = SKSE::GetTaskInterface())
			task->AddTask([choice = std::move(a_choice)] {
				std::vector<std::uint32_t> bases = ChooseBases(choice);
				if (bases.empty())
					GetCrowd().SetReport("No character passes the filter, so nothing spawned.");
				else
					GetCrowd().Spawn(choice.count, std::move(bases));
			});
	}
}
