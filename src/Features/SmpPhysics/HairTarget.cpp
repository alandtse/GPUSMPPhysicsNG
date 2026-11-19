#include "HairTarget.h"

#include "CrowdWigs.h"
#include "engine/HairLook.h"
#include "smp/HairPartModel.h"

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
		constexpr auto kViewedWindow = std::chrono::seconds(1);

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

	HairTargetDetail HairWearingOf(RE::Actor* a_actor)
	{
		using Slot = RE::BGSBipedObjectForm::BipedObjectSlot;
		HairTargetDetail detail;
		if (!a_actor)
			return detail;
		auto* base = const_cast<RE::TESNPC*>(a_actor->GetActorBase());
		if (const RE::BGSHeadPart* part = base ? base->GetCurrentHeadPartByType(RE::BGSHeadPart::HeadPartType::kHair) : nullptr) {
			std::vector<std::string> extras;
			for (const RE::BGSHeadPart* extra : part->extraParts)
				extras.emplace_back(extra && extra->GetModel() ? extra->GetModel() : "");
			detail.hairPart = std::string(part->GetFormEditorID());
			detail.hairPartModel = HairPartModel(part->GetModel() ? part->GetModel() : "", extras);
			detail.extraParts = extras.size();
		}
		const auto worn = a_actor->GetInventory([](RE::TESBoundObject& a_object) {
			const auto* armor = a_object.As<RE::TESObjectARMO>();
			return armor && (armor->HasPartOf(Slot::kHair) || armor->HasPartOf(Slot::kLongHair) || armor->HasPartOf(Slot::kHead));
		});
		for (const auto& [object, entry] : worn) {
			const auto* armor = object ? object->As<RE::TESObjectARMO>() : nullptr;
			if (armor && entry.second && entry.second->IsWorn())
				detail.worn.push_back({ armor->GetName(), armor->HasPartOf(Slot::kHair) || armor->HasPartOf(Slot::kLongHair), armor->HasPartOf(Slot::kHead) });
		}
		return detail;
	}

	void HairTarget::Tick(const CrowdWigs& a_wigs)
	{
		const auto now = std::chrono::steady_clock::now();
		if (now.time_since_epoch().count() - viewedAt.load(std::memory_order_relaxed) >= std::chrono::duration_cast<std::chrono::steady_clock::duration>(kViewedWindow).count())
			return;
		{
			const std::scoped_lock lock(mutex);
			if (!ranking && rankedFor == pick.formID && now - sampledAt < kSamplePeriod)
				return;
		}
		sampledAt = now;

		const std::uint32_t console = ActorFormID(RE::Console::GetSelectedRefHandle());
		std::uint32_t target;
		RE::FormID wig;
		bool rank;
		{
			const std::scoped_lock lock(mutex);
			for (const std::uint32_t id : { pick.console, pick.crosshair, pick.formID }) {
				if (id && !RE::TESForm::LookupByID<RE::Actor>(id))
					ForgetPick(pick, id);
			}
			UpdatePick(pick, console);
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
			if (now - detailAt >= kSamplePeriod || detailFor != target) {
				detailAt = now;
				detailFor = target;
				wearing = HairWearingOf(actor);
			}
			HairTargetDetail detail = wearing;
			detail.matchedName = matched.matchedName;
			detail.matchedByWig = matched.matchedByWig;
			detail.matchedModel = matched.matchedModel;
			detail.triangles = matched.triangles;
			view.detail = FormatHairTargetDetail(detail);
			view.matchedModel = matched.matchedModel;
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
		matched = {};
		if (result.contains("target")) {
			matched.matchedName = hair;
			matched.matchedByWig = wearsWig;
			matched.matchedModel = result["target"].value("model", std::string{});
			matched.triangles = result["target"].contains("shape") ? result["target"]["shape"].value("triangles", std::size_t{}) : 0;
		}
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

	void HairTarget::NoteCrosshair(std::uint32_t a_formID)
	{
		const std::scoped_lock lock(mutex);
		SmpPhysics::NoteCrosshair(pick, a_formID);
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
		viewedAt.store(std::chrono::steady_clock::now().time_since_epoch().count(), std::memory_order_relaxed);
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
