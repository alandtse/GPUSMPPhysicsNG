#include "HairTarget.h"

#include "CrowdWigs.h"
#include "engine/HairLook.h"

#include <algorithm>
#include <format>
#include <unordered_map>

namespace SmpPhysics
{
	namespace
	{
		constexpr auto kSamplePeriod = std::chrono::milliseconds(250);
		constexpr double kRankBudgetMs = 6.0;
		constexpr std::uint32_t kPlayerFormID = 0x14;
		constexpr std::size_t kMaxRanked = 500;

		/** @brief The form ID of the actor behind a_handle, 0 for none or for the player. */
		std::uint32_t ActorFormID(const RE::ObjectRefHandle& a_handle)
		{
			const auto reference = a_handle.get();
			const auto* actor = reference ? reference->As<RE::Actor>() : nullptr;
			return actor && !actor->IsPlayerRef() ? actor->GetFormID() : 0;
		}

		std::string NameOf(std::uint32_t a_formID)
		{
			const auto* actor = RE::TESForm::LookupByID<RE::Actor>(a_formID ? a_formID : kPlayerFormID);
			const char* name = actor ? actor->GetName() : nullptr;
			return name && *name ? name : "unnamed";
		}

		const char* SourceName(HairTargetSource a_source)
		{
			switch (a_source) {
			case HairTargetSource::Console:
				return "console selection";
			case HairTargetSource::Crosshair:
				return "crosshair";
			default:
				return "you";
			}
		}
	}

	void HairTarget::Tick(bool a_menuOpen, const CrowdWigs& a_wigs)
	{
		const auto now = std::chrono::steady_clock::now();
		{
			const std::scoped_lock lock(mutex);
			if (!ranking && rankedFor == pick.formID && now - sampledAt < kSamplePeriod)
				return;
		}
		sampledAt = now;

		HairTargetSamples samples;
		samples.menuOpen = a_menuOpen;
		samples.console = ActorFormID(RE::Console::GetSelectedRefHandle());
		if (!a_menuOpen) {
			if (const auto* picker = RE::CrosshairPickData::GetSingleton())
				samples.crosshair = ActorFormID(picker->GetActiveTarget());
		}
		std::uint32_t target;
		RE::FormID wig;
		bool rank;
		{
			const std::scoped_lock lock(mutex);
			for (const std::uint32_t id : { pick.console, pick.crosshair, pick.formID }) {
				if (id && !RE::TESForm::LookupByID<RE::Actor>(id))
					ForgetPick(pick, id);
			}
			UpdatePick(pick, samples);
			view.hasConsole = pick.console != 0;
			view.hasCrosshair = pick.crosshair != 0;
			view.pinned = pick.pinned;
			target = pick.formID;
			view.formID = target;
			auto* actor = RE::TESForm::LookupByID<RE::Actor>(target ? target : kPlayerFormID);
			wig = actor ? a_wigs.NaturalWig(*actor) : 0;
			const auto* base = actor ? actor->GetActorBase() : nullptr;
			const auto* hairPart = base ? const_cast<RE::TESNPC*>(base)->GetCurrentHeadPartByType(RE::BGSHeadPart::HeadPartType::kHair) : nullptr;
			const RE::FormID hair = hairPart ? hairPart->GetFormID() : 0;
			if (rankedFor != target || rankedWig != wig || rankedHair != hair) {
				rankedFor = target;
				rankedWig = wig;
				rankedHair = hair;
				ranking = true;
				view.order.clear();
				view.badges.clear();
				view.progress = {};
				view.status = "Ranking hair shapes";
				view.text = std::format("Target: {} ({})", NameOf(target), SourceName(pick.source));
			}
			rank = ranking;
		}
		if (rank)
			Rank(target, wig);
	}

	void HairTarget::Rank(std::uint32_t a_formID, RE::FormID a_wig)
	{
		HairLook::Rank({ { "actorFormID", a_formID ? a_formID : kPlayerFormID }, { "wigFormID", a_wig }, { "top", kMaxRanked }, { "budgetMs", kRankBudgetMs }, { "physicsOnly", false } });
		const json result = HairLook::Result();
		const std::scoped_lock lock(mutex);
		if (rankedFor != a_formID || rankedWig != a_wig)
			return;
		if (result.contains("error")) {
			view.status = std::format("No hair match: {}", result["error"].get<std::string>());
			ranking = false;
			return;
		}
		const std::size_t considered = result.value("considered", std::size_t{});
		const std::size_t pending = result.value("pending", std::size_t{});
		view.progress = { considered - std::min(considered, pending), considered };
		const std::string hair = result.contains("target") ? result["target"].value("name", std::string{}) : std::string{};
		const bool wearsWig = result.contains("target") && result["target"].value("wig", false);
		view.text = std::format("Target: {} ({}), {} {}", NameOf(a_formID), SourceName(pick.source), wearsWig ? "wearing" : "hair", hair.empty() ? "unknown" : hair);
		if (pending)
			return;

		const std::vector<RE::TESObjectARMO*>& wigs = CrowdWigs::WigForms();
		std::unordered_map<std::uint32_t, std::size_t> indexOf;
		for (std::size_t index = 0; index < wigs.size(); ++index)
			indexOf.emplace(wigs[index]->GetFormID(), index);
		std::vector<HairRank> ranks;
		for (const json& match : result.value("matches", json::array())) {
			if (const auto found = indexOf.find(match.value("formId", 0u)); found != indexOf.end())
				ranks.push_back({ found->second, match.value("chamfer", 0.0f) });
		}
		HairOrdering ordering = OrderByHairMatch(ranks, wigs.size());
		view.order = std::move(ordering.order);
		view.badges = std::move(ordering.badges);
		view.status.clear();
		ranking = false;
	}

	bool HairTarget::Pin(HairTargetSource a_source)
	{
		const std::scoped_lock lock(mutex);
		return PinPick(pick, a_source);
	}

	void HairTarget::Unpin()
	{
		const std::scoped_lock lock(mutex);
		UnpinPick(pick);
	}

	HairTargetView HairTarget::View() const
	{
		const std::scoped_lock lock(mutex);
		HairTargetView copy = view;
		if (copy.text.empty())
			copy.text = "Target: none yet";
		return copy;
	}

	std::uint32_t HairTarget::FormID() const
	{
		const std::scoped_lock lock(mutex);
		return pick.formID;
	}
}
