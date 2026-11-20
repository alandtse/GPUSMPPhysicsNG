// SPDX-License-Identifier: GPL-3.0-or-later WITH LicenseRef-Modding-Exception

#include "CrowdHairParts.h"

#include "smp/HairPartFit.h"

#include <algorithm>
#include <cstring>
#include <format>
#include <mutex>

namespace SmpPhysics
{
	namespace
	{
		bool SuitsRace(const RE::BGSHeadPart& a_part, const RE::TESNPC& a_base)
		{
			return !a_part.validRaces || (a_base.race && a_part.validRaces->HasForm(a_base.race));
		}

		std::string NameOf(const RE::BGSHeadPart& a_part)
		{
			const char* editorId = a_part.GetFormEditorID();
			return editorId && *editorId ? std::string(editorId) : std::format("{:08X}", a_part.GetFormID());
		}
	}

	const std::vector<RE::BGSHeadPart*>& CrowdHairParts::Parts()
	{
		static std::vector<RE::BGSHeadPart*> parts;
		static std::mutex mutex;
		const std::scoped_lock lock(mutex);
		if (parts.empty()) {
			if (auto* handler = RE::TESDataHandler::GetSingleton()) {
				for (RE::BGSHeadPart* part : handler->GetFormArray<RE::BGSHeadPart>()) {
					if (!part || part->type != RE::BGSHeadPart::HeadPartType::kHair)
						continue;
					const char* model = part->GetModel();
					if (model && *model && !part->IsExtraPart())
						parts.push_back(part);
				}
			}
			std::ranges::sort(parts, [](const RE::BGSHeadPart* a_left, const RE::BGSHeadPart* a_right) {
				const int byName = NameOf(*a_left).compare(NameOf(*a_right));
				return byName != 0 ? byName < 0 : a_left->GetFormID() < a_right->GetFormID();
			});
		}
		return parts;
	}

	const std::vector<std::string>& CrowdHairParts::Names()
	{
		static std::vector<std::string> names;
		static std::mutex mutex;
		const std::vector<RE::BGSHeadPart*>& parts = Parts();
		const std::scoped_lock lock(mutex);
		if (names.size() != parts.size()) {
			names.clear();
			for (const RE::BGSHeadPart* part : parts)
				names.push_back(NameOf(*part));
		}
		return names;
	}

	bool CrowdHairParts::Give(RE::TESNPC& a_base, RE::BGSHeadPart& a_part)
	{
		if (!HairPartFits(static_cast<std::uint32_t>(a_part.flags.underlying()), a_base.IsFemale()) || !SuitsRace(a_part, a_base))
			return false;
		if (auto* own = a_base.GetHeadPartByType(RE::BGSHeadPart::HeadPartType::kHair))
			naturalParts.try_emplace(a_base.GetFormID(), own->GetFormID());
		else
			naturalParts.try_emplace(a_base.GetFormID(), 0);
		a_base.ChangeHeadPart(&a_part);
		return true;
	}

	void CrowdHairParts::SetNow(TestBench::TestCrowd& a_crowd, int a_index)
	{
		const std::vector<RE::BGSHeadPart*>& parts = Parts();
		if (a_index < 0 || static_cast<std::size_t>(a_index) >= parts.size()) {
			a_crowd.SetReport(parts.empty() ? "No hair part is loaded." : std::format("There is no hair part {}; pick one of 0 to {}.", a_index, parts.size() - 1));
			return;
		}
		RE::BGSHeadPart& part = *parts[static_cast<std::size_t>(a_index)];
		std::size_t changed = 0;
		const std::vector<RE::FormID> bases = a_crowd.Bases();
		for (const RE::FormID baseId : bases) {
			if (auto* base = RE::TESForm::LookupByID<RE::TESNPC>(baseId); base && Give(*base, part))
				++changed;
		}
		wornIndex.store(a_index, std::memory_order_relaxed);
		const std::size_t refreshed = a_crowd.RefreshNow();
		a_crowd.SetReport(std::format("Gave {} to {} of {} bases (the rest do not suit it) and refreshed {} characters.", NameOf(part), changed, bases.size(), refreshed));
	}

	void CrowdHairParts::RestoreNow(TestBench::TestCrowd& a_crowd)
	{
		std::size_t restored = 0;
		for (const auto& [baseId, partId] : naturalParts) {
			auto* base = RE::TESForm::LookupByID<RE::TESNPC>(baseId);
			auto* part = partId ? RE::TESForm::LookupByID<RE::BGSHeadPart>(partId) : nullptr;
			if (base && part) {
				base->ChangeHeadPart(part);
				++restored;
			}
		}
		naturalParts.clear();
		wornIndex.store(-1, std::memory_order_relaxed);
		const std::size_t refreshed = a_crowd.RefreshNow();
		a_crowd.SetReport(std::format("Put back the hair of {} bases and refreshed {} characters.", restored, refreshed));
	}

	void CrowdHairParts::SetActorNow(TestBench::TestCrowd& a_crowd, RE::Actor& a_actor, int a_index)
	{
		const std::vector<RE::BGSHeadPart*>& parts = Parts();
		auto* base = const_cast<RE::TESNPC*>(a_actor.GetActorBase());
		if (!base || a_index < 0 || static_cast<std::size_t>(a_index) >= parts.size())
			return;
		RE::BGSHeadPart& part = *parts[static_cast<std::size_t>(a_index)];
		if (!Give(*base, part)) {
			a_crowd.SetReport(std::format("{} does not suit the target.", NameOf(part)));
			return;
		}
		wornIndex.store(a_index, std::memory_order_relaxed);
		a_actor.DoReset3D(true);
		a_crowd.SetReport(std::format("Gave the target {}.", NameOf(part)));
	}

	void CrowdHairParts::RestoreActorNow(TestBench::TestCrowd& a_crowd, RE::Actor& a_actor)
	{
		auto* base = const_cast<RE::TESNPC*>(a_actor.GetActorBase());
		const auto own = base ? naturalParts.find(base->GetFormID()) : naturalParts.end();
		if (own == naturalParts.end())
			return;
		if (auto* part = own->second ? RE::TESForm::LookupByID<RE::BGSHeadPart>(own->second) : nullptr)
			base->ChangeHeadPart(part);
		naturalParts.erase(own);
		a_actor.DoReset3D(true);
		a_crowd.SetReport("Put back the target's own hair.");
	}
}
