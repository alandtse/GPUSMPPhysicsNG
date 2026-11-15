#include "CrowdWigs.h"

#include "Utils/StringUtils.h"

#include <algorithm>
#include <format>
#include <optional>
#include <span>

namespace SmpPhysics
{
	namespace
	{
		/** @brief A hair-slot armor that is not a hood or helmet, with the model path of its worn female model. */
		struct Wig
		{
			RE::TESObjectARMO* armor = nullptr;
			WigCandidate candidate;
		};

		bool NameContains(const std::string& a_text, const char* a_part)
		{
			return Util::ToLowerAscii(a_text).find(a_part) != std::string::npos;
		}

		bool IsWig(const RE::TESBoundObject& a_object)
		{
			using Slot = RE::BGSBipedObjectForm::BipedObjectSlot;
			const auto* armor = a_object.As<RE::TESObjectARMO>();
			return armor && (armor->HasPartOf(Slot::kHair) || armor->HasPartOf(Slot::kLongHair)) && !armor->HasPartOf(Slot::kHead);
		}

		constexpr const char* kUnnamedHeadPart = "(unnamed)";  ///< How a head part without an editor ID is reported.

		constexpr int kBakedHairWatchFrames = 1200;  ///< How long a wig wearer is watched for its own hair showing, since a new character has no 3D yet and equipping a wig rebuilds it a few frames later.

		/**
		 * @brief Hides the character's own hair meshes; the engine leaves them showing under a wig whose second addon sits in another slot.
		 * @return How many hair nodes this call newly hid; zero also when the character has no 3D yet.
		 */
		std::size_t HideBakedHair(RE::Actor& a_actor)
		{
			std::vector<BakedHairNode> nodes = BakedHairNodes(a_actor);
			auto* root = a_actor.Get3D();
			std::size_t newlyHidden = 0;
			for (BakedHairNode& node : nodes) {
				if (!node.found)
					continue;
				if (auto* object = root->GetObjectByName(node.editorId)) {
					if (!node.hidden)
						++newlyHidden;
					object->GetFlags().set(RE::NiAVObject::Flag::kHidden);
					node.hidden = true;
				}
			}
			if (newlyHidden == 0)
				return 0;
			std::string missing;
			for (const BakedHairNode& node : nodes) {
				if (node.found)
					continue;
				missing += missing.empty() ? " (not found: " : ", ";
				missing += node.editorId;
			}
			if (!missing.empty())
				missing += ")";
			const std::size_t hiddenCount = static_cast<std::size_t>(std::count_if(nodes.begin(), nodes.end(), [](const BakedHairNode& a_node) { return a_node.hidden; }));
			const char* name = a_actor.GetName();
			logger::info("[Test Crowd] {} ({:08X}): baked hair hidden on {} of {} hair nodes{}", name && *name ? name : "unnamed", a_actor.GetFormID(), hiddenCount, nodes.size(), missing);
			return newlyHidden;
		}

		std::vector<Wig> CollectWigs()
		{
			std::vector<Wig> wigs;
			auto* handler = RE::TESDataHandler::GetSingleton();
			if (!handler)
				return wigs;
			using Slot = RE::BGSBipedObjectForm::BipedObjectSlot;
			for (auto* armor : handler->GetFormArray<RE::TESObjectARMO>()) {
				if (!armor || armor->armorAddons.empty())
					continue;
				if (!armor->HasPartOf(Slot::kHair) && !armor->HasPartOf(Slot::kLongHair))
					continue;
				if (armor->HasPartOf(Slot::kHead))
					continue;
				const char* name = armor->GetName();
				if (!name || !*name)
					continue;
				std::string model;
				for (auto* addon : armor->armorAddons) {
					if (!addon)
						continue;
					const char* path = addon->bipedModels[RE::SEXES::kFemale].GetModel();
					if (path && *path) {
						model = path;
						break;
					}
				}
				if (model.empty() || (!NameContains(model, "hair") && !NameContains(model, "wig")))
					continue;
				wigs.push_back({ armor, { name, std::move(model) } });
			}
			return wigs;
		}
	}

	std::vector<BakedHairNode> BakedHairNodes(RE::Actor& a_actor)
	{
		std::vector<BakedHairNode> nodes;
		auto* npc = a_actor.GetActorBase();
		auto* root = a_actor.Get3D();
		if (!npc || !npc->headParts || !root)
			return nodes;
		const auto add = [root, &nodes](const RE::BGSHeadPart& a_part) {
			BakedHairNode node;
			node.editorId = a_part.formEditorID.empty() ? kUnnamedHeadPart : a_part.formEditorID.c_str();
			if (!a_part.formEditorID.empty()) {
				if (auto* object = root->GetObjectByName(a_part.formEditorID)) {
					node.found = true;
					node.hidden = object->GetFlags().any(RE::NiAVObject::Flag::kHidden);
				}
			}
			nodes.push_back(std::move(node));
		};
		for (const RE::BGSHeadPart* part : std::span(npc->headParts, npc->numHeadParts)) {
			if (!part || part->type != RE::BGSHeadPart::HeadPartType::kHair)
				continue;
			add(*part);
			for (const RE::BGSHeadPart* extra : part->extraParts) {
				if (extra)
					add(*extra);
			}
		}
		return nodes;
	}

	void CrowdWigs::BeforeSpawn(std::size_t a_count)
	{
		requested = a_count;
		const std::vector<Wig> wigs = CollectWigs();
		std::vector<WigCandidate> candidates;
		candidates.reserve(wigs.size());
		for (const Wig& wig : wigs)
			candidates.push_back(wig.candidate);
		chosen.clear();
		for (const std::size_t index : PickDistinctWigs(candidates, a_count))
			chosen.push_back(wigs[index].armor);
	}

	void CrowdWigs::Dress(RE::Actor& a_actor, std::size_t a_index)
	{
		if (chosen.empty())
			return;
		auto* armor = chosen[a_index % chosen.size()];
		a_actor.AddObjectToContainer(armor, nullptr, 1, nullptr);
		RE::ActorEquipManager::GetSingleton()->EquipObject(&a_actor, armor, nullptr, 1, nullptr, false, true, false, true);
		const char* actorName = a_actor.GetName();
		const char* wigName = armor->GetName();
		logger::info("[Test Crowd] {} ({:08X}) wears {} ({:08X})", actorName && *actorName ? actorName : "unnamed", a_actor.GetFormID(), wigName && *wigName ? wigName : "unnamed", armor->GetFormID());
		HideBakedHair(a_actor);
		const RE::ObjectRefHandle handle = a_actor.GetHandle();
		std::erase_if(watches, [&](const BakedHairWatch& a_watch) { return a_watch.handle == handle; });
		watches.push_back({ handle, kBakedHairWatchFrames, false });
	}

	void CrowdWigs::Tick()
	{
		for (BakedHairWatch& watch : watches) {
			auto reference = watch.handle.get();
			auto* actor = reference ? reference->As<RE::Actor>() : nullptr;
			if (!actor) {
				watch.framesLeft = 0;
				continue;
			}
			if (actor->Is3DLoaded() && HideBakedHair(*actor) > 0)
				watch.everHidden = true;
			if (--watch.framesLeft <= 0 && !watch.everHidden) {
				const char* name = actor->GetName();
				logger::info("[Test Crowd] {} ({:08X}): no baked hair was hidden in {} frames", name && *name ? name : "unnamed", actor->GetFormID(), kBakedHairWatchFrames);
			}
		}
		std::erase_if(watches, [](const BakedHairWatch& a_watch) { return a_watch.framesLeft <= 0; });
	}

	bool CrowdWigs::KeepWhenStripped(const RE::TESBoundObject& a_object) const
	{
		return IsWig(a_object);
	}

	void CrowdWigs::AfterRefresh(RE::Actor& a_actor)
	{
		if (!chosen.empty())
			HideBakedHair(a_actor);
	}

	std::string CrowdWigs::DescribeSpawn() const
	{
		if (chosen.empty())
			return {};
		return std::format("They wear {} different hair styles{}.", chosen.size(), chosen.size() < requested ? ", so styles repeat" : "");
	}

	void CrowdWigs::SetHairColorNow(TestBench::TestCrowd& a_crowd, const std::string& a_color)
	{
		std::optional<std::uint32_t> formId = ResolveHairColor(a_color);
		if (!formId) {
			try {
				formId = static_cast<std::uint32_t>(std::stoul(a_color, nullptr, 0));
			} catch (const std::exception&) {
				formId = std::nullopt;
			}
		}
		auto* color = formId ? RE::TESForm::LookupByID<RE::BGSColorForm>(*formId) : nullptr;
		if (!color) {
			a_crowd.SetReport(std::format("'{}' is not a hair colour: use a name from testCrowdHairColors or a hair colour form ID.", a_color));
			return;
		}
		for (const RE::FormID baseId : a_crowd.Bases()) {
			auto* base = RE::TESForm::LookupByID<RE::TESNPC>(baseId);
			if (!base)
				continue;
			if (base->headRelatedData && base->headRelatedData->hairColor)
				naturalColors.try_emplace(baseId, base->headRelatedData->hairColor);
			base->SetHairColor(color);
		}
		const std::size_t refreshed = a_crowd.RefreshNow();
		a_crowd.SetReport(std::format("Set the crowd's hair colour to {:08X} and refreshed {} characters.", color->GetFormID(), refreshed));
	}

	void CrowdWigs::RestoreNaturalColorsNow(TestBench::TestCrowd& a_crowd)
	{
		std::size_t restored = 0;
		for (const auto& [baseId, color] : naturalColors) {
			auto* base = RE::TESForm::LookupByID<RE::TESNPC>(baseId);
			if (!base || !color)
				continue;
			base->SetHairColor(color);
			++restored;
		}
		naturalColors.clear();
		const std::size_t refreshed = a_crowd.RefreshNow();
		a_crowd.SetReport(std::format("Restored the natural hair colours of {} characters' bases and refreshed {} characters.", restored, refreshed));
	}
}
