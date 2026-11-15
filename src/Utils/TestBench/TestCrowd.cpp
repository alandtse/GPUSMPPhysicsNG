#include "TestCrowd.h"

#include "CrowdFilter.h"
#include "TestCrowdLayout.h"

#include "Globals.h"
#include "Utils/ActorUtils.h"
#include "Utils/MathUtils.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <format>
#include <mutex>
#include <numbers>
#include <optional>
#include <string>
#include <unordered_map>

namespace TestBench
{
	namespace
	{
		/** A generic female character from Skyrim.esm. It has no prebaked face, so head parts and armor added to it change what is drawn. */
		constexpr RE::FormID kTestCharacterBase = 0x000A2C8E;
		constexpr float kStandHeightOffset = 1.0f;
		constexpr const char* kResetIdle = "ResetRoot";      ///< The idle that returns a character to its neutral stance, which Stop plays before freezing it.
		constexpr std::uint32_t kNonPlayableFlag = 1u << 2;  ///< The record flag of an armor the player cannot wear.
		constexpr float kMaxTurnDegrees = 720.0f;            ///< Largest turn a test command may ask for.
		constexpr float kGroundProbeAbove = 200.0f;          ///< How far above the player's feet the ground ray starts, so a slope that rises ahead is still found.
		constexpr float kGroundProbeBelow = 1500.0f;         ///< How far below the player's feet it reaches, so a drop ahead is still found.
		constexpr float kGroundProbeSkip = 2.0f;             ///< How far under a character the ray restarts when it meets one.

		/** @brief The height of the ground under a point: the first collision a ray from above meets that is not a character. */
		std::optional<float> GroundHeightAt(RE::Actor& a_player, const RE::NiPoint3& a_around)
		{
			RE::NiPoint3 from{ a_around.x, a_around.y, a_around.z + kGroundProbeAbove };
			const float bottom = a_around.z - kGroundProbeBelow;
			for (int attempt = 0; attempt < 4; ++attempt) {
				const auto hit = Util::FirstOccluder(a_player, from, RE::NiPoint3{ from.x, from.y, bottom });
				if (!hit)
					return std::nullopt;
				const float height = from.z - hit->distance;
				if (hit->reference == 0 || !RE::TESForm::LookupByID<RE::Actor>(hit->reference))
					return height;
				from.z = height - kGroundProbeSkip;
			}
			return std::nullopt;
		}

		/** @brief Plays an idle on a character; PlayIdle only animates while the character's AI runs, so it holds a do-nothing package to stay put. */
		bool PlayIdleOn(RE::Actor& a_actor, RE::TESIdleForm& a_idle)
		{
			a_actor.EnableAI(true);
			a_actor.InitiateDoNothingPackage();
			auto* process = a_actor.GetActorRuntimeData().currentProcess;
			return process && process->PlayIdle(&a_actor, &a_idle, nullptr);
		}

		void Equip(RE::Actor& a_actor, RE::TESBoundObject& a_object)
		{
			RE::ActorEquipManager::GetSingleton()->EquipObject(&a_actor, &a_object, nullptr, 1, nullptr, false, true, false, true);
		}

		void Unequip(RE::Actor& a_actor, RE::TESBoundObject& a_object)
		{
			RE::ActorEquipManager::GetSingleton()->UnequipObject(&a_actor, &a_object, nullptr, 1, nullptr, false, true, false, true);
		}

		/** @brief The playable armors that cover the body, by name, so cycling steps through them in a stable order. */
		const std::vector<RE::TESObjectARMO*>& BodyArmors()
		{
			static std::vector<RE::TESObjectARMO*> armors;
			static std::mutex mutex;
			const std::scoped_lock lock(mutex);
			if (!armors.empty())
				return armors;
			auto* handler = RE::TESDataHandler::GetSingleton();
			if (!handler)
				return armors;
			std::vector<RE::TESObjectARMO*> candidates;
			for (auto* armor : handler->GetFormArray<RE::TESObjectARMO>()) {
				if (!armor || armor->armorAddons.empty() || (armor->formFlags & kNonPlayableFlag) != 0 || !armor->HasPartOf(RE::BGSBipedObjectForm::BipedObjectSlot::kBody))
					continue;
				// An enchanted armor looks like the one it was made from.
				if (armor->formEnchanting != nullptr)
					continue;
				const char* name = armor->GetName();
				if (name && *name)
					candidates.push_back(armor);
			}
			std::sort(candidates.begin(), candidates.end(), [](const RE::TESObjectARMO* a_left, const RE::TESObjectARMO* a_right) {
				const int byName = std::strcmp(a_left->GetName(), a_right->GetName());
				return byName != 0 ? byName < 0 : a_left->GetFormID() < a_right->GetFormID();
			});
			// Armors built from the same addons draw the same, so the first of each is enough.
			std::vector<std::vector<RE::TESObjectARMA*>> seen;
			for (RE::TESObjectARMO* armor : candidates) {
				std::vector<RE::TESObjectARMA*> addons(armor->armorAddons.begin(), armor->armorAddons.end());
				std::sort(addons.begin(), addons.end());
				if (std::find(seen.begin(), seen.end(), addons) != seen.end())
					continue;
				seen.push_back(std::move(addons));
				armors.push_back(armor);
			}
			return armors;
		}

		bool IsCloak(const RE::TESObjectARMO& a_armor)
		{
			return std::any_of(a_armor.armorAddons.begin(), a_armor.armorAddons.end(), [](const RE::TESObjectARMA* a_addon) {
				if (!a_addon)
					return false;
				for (const auto& model : a_addon->bipedModels) {
					const char* path = model.GetModel();
					if (path && IsCloakModelPath(path))
						return true;
				}
				return false;
			});
		}

		/** @brief The cloaks and capes by name, so choosing steps through them in a stable order. */
		const std::vector<RE::TESObjectARMO*>& CloakArmors()
		{
			static std::vector<RE::TESObjectARMO*> cloaks;
			static std::mutex mutex;
			const std::scoped_lock lock(mutex);
			if (!cloaks.empty())
				return cloaks;
			auto* handler = RE::TESDataHandler::GetSingleton();
			if (!handler)
				return cloaks;
			std::vector<RE::TESObjectARMO*> candidates;
			for (auto* armor : handler->GetFormArray<RE::TESObjectARMO>()) {
				if (!armor || armor->armorAddons.empty() || armor->formEnchanting != nullptr || armor->HasPartOf(RE::BGSBipedObjectForm::BipedObjectSlot::kBody) || !IsCloak(*armor))
					continue;
				const char* name = armor->GetName();
				if (name && *name)
					candidates.push_back(armor);
			}
			std::sort(candidates.begin(), candidates.end(), [](const RE::TESObjectARMO* a_left, const RE::TESObjectARMO* a_right) {
				const int byName = std::strcmp(a_left->GetName(), a_right->GetName());
				return byName != 0 ? byName < 0 : a_left->GetFormID() < a_right->GetFormID();
			});
			std::vector<std::vector<RE::TESObjectARMA*>> seen;
			for (RE::TESObjectARMO* armor : candidates) {
				std::vector<RE::TESObjectARMA*> addons(armor->armorAddons.begin(), armor->armorAddons.end());
				std::sort(addons.begin(), addons.end());
				if (std::find(seen.begin(), seen.end(), addons) != seen.end())
					continue;
				seen.push_back(std::move(addons));
				cloaks.push_back(armor);
			}
			return cloaks;
		}
	}

	/** @brief Plays a character's idle again when its animation graph reports the idle ended, while the crowd is set to loop. */
	struct TestCrowd::IdleEndSink : RE::BSTEventSink<RE::BSAnimationGraphEvent>
	{
		explicit IdleEndSink(TestCrowd& a_owner) :
			owner(a_owner) {}

		RE::BSEventNotifyControl ProcessEvent(const RE::BSAnimationGraphEvent* a_event, RE::BSTEventSource<RE::BSAnimationGraphEvent>*) override
		{
			if (!a_event || !a_event->holder || !owner.loopAnimation.load(std::memory_order_relaxed))
				return RE::BSEventNotifyControl::kContinue;
			const char* tag = a_event->tag.c_str();
			if (!tag || !IsIdleEndEvent(tag))
				return RE::BSEventNotifyControl::kContinue;
			if (auto* task = SKSE::GetTaskInterface()) {
				task->AddTask([this, handle = a_event->holder->GetHandle()] {
					auto reference = handle.get();
					auto* actor = reference ? reference->As<RE::Actor>() : nullptr;
					const std::string name = owner.Animation();
					auto* idle = name == kResetIdle ? nullptr : IdleCache::Find(name);
					if (actor && idle && actor->Is3DLoaded() && owner.loopAnimation.load(std::memory_order_relaxed) && PlayIdleOn(*actor, *idle))
						logger::debug("[Test Crowd] test character {:08X} looped {}", actor->GetFormID(), name);
				});
			}
			return RE::BSEventNotifyControl::kContinue;
		}

		TestCrowd& owner;
	};

	TestCrowd::TestCrowd() :
		idleEndSink(std::make_unique<IdleEndSink>(*this)) {}

	TestCrowd::~TestCrowd() = default;

	void TestCrowd::WatchForIdleEnd(RE::Actor& a_actor)
	{
		if (watched.insert(a_actor.GetFormID()).second)
			a_actor.AddAnimationGraphEventSink(idleEndSink.get());
	}

	void TestCrowd::SetLoopAnimation(bool a_loop)
	{
		loopAnimation.store(a_loop, std::memory_order_relaxed);
	}

	void TestCrowd::Spawn(std::size_t a_count, std::vector<std::uint32_t> a_bases)
	{
		const std::size_t requested = std::clamp<std::size_t>(a_count, 1, kMaxTestCrowd);
		if (auto* task = SKSE::GetTaskInterface())
			task->AddTask([this, requested, bases = std::move(a_bases)] { SpawnNow(requested, bases); });
	}

	void TestCrowd::Clear()
	{
		if (auto* task = SKSE::GetTaskInterface())
			task->AddTask([this] { ClearNow(); });
	}

	std::string TestCrowd::Report() const
	{
		const std::scoped_lock lock(mutex);
		return report;
	}

	void TestCrowd::SetReport(std::string a_text)
	{
		const std::scoped_lock lock(mutex);
		report = std::move(a_text);
	}

	std::string TestCrowd::Animation() const
	{
		{
			const std::scoped_lock lock(mutex);
			if (!animation.empty())
				return animation;
		}
		const std::vector<IdleChoice> idles = IdleCache::Snapshot();
		std::vector<std::string> editorIds;
		editorIds.reserve(idles.size());
		for (const IdleChoice& idle : idles)
			editorIds.push_back(idle.editorId);
		const auto pick = PickDefaultIdle(editorIds);
		return pick ? idles[*pick].editorId : std::string{};
	}

	void TestCrowd::PlayAnimation(std::string a_editorId)
	{
		{
			const std::scoped_lock lock(mutex);
			animation = a_editorId;
		}
		if (auto* task = SKSE::GetTaskInterface())
			task->AddTask([this, editorId = std::move(a_editorId)] { PlayAnimationNow(editorId); });
	}

	void TestCrowd::PlayAnimationNow(const std::string& a_editorId)
	{
		const std::string name = a_editorId.empty() ? Animation() : a_editorId;
		RE::TESIdleForm* idle = IdleCache::Find(name);
		if (!idle) {
			SetReport(std::format("No idle animation named '{}' is loaded.", name));
			return;
		}
		const bool stopping = name == kResetIdle;
		std::size_t played = 0;
		for (const RE::ObjectRefHandle& handle : spawned) {
			auto reference = handle.get();
			auto* actor = reference ? reference->As<RE::Actor>() : nullptr;
			if (actor && actor->Is3DLoaded() && PlayIdleOn(*actor, *idle)) {
				WatchForIdleEnd(*actor);
				++played;
				if (stopping)
					actor->EnableAI(false);
			}
		}
		SetReport(std::format("Played {} on {} of {} test characters.", name, played, spawned.size()));
	}

	std::vector<RE::ObjectRefHandle> TestCrowd::Standing() const
	{
		const std::scoped_lock lock(mutex);
		return spawned;
	}

	std::vector<TestCrowd::WornSummary> TestCrowd::Worn() const
	{
		std::vector<WornSummary> summaries;
		for (const RE::ObjectRefHandle& handle : Standing()) {
			auto reference = handle.get();
			auto* actor = reference ? reference->As<RE::Actor>() : nullptr;
			if (!actor)
				continue;
			WornSummary summary{ actor->GetFormID(), {}, {} };
			const auto inventory = actor->GetInventory([](RE::TESBoundObject& a_object) { return a_object.Is(RE::FormType::Armor, RE::FormType::Weapon); });
			for (const auto& [object, entry] : inventory) {
				if (!object || !entry.second || !entry.second->IsWorn())
					continue;
				const char* name = object->GetName();
				(KeptByHooks(*object) ? summary.kept : summary.worn).push_back(name && *name ? name : "unnamed");
			}
			summaries.push_back(std::move(summary));
		}
		return summaries;
	}

	void TestCrowd::Turn(int a_index, float a_degrees)
	{
		if (auto* task = SKSE::GetTaskInterface())
			task->AddTask([this, a_index, a_degrees] { TurnNow(a_index, a_degrees); });
	}

	void TestCrowd::TurnNow(int a_index, float a_degrees)
	{
		auto* player = globals::game::player;
		if (!player)
			return;
		const float heading = player->GetAngleZ() + std::numbers::pi_v<float> + Util::ClampFinite(a_degrees, -kMaxTurnDegrees, kMaxTurnDegrees, 0.0f) * (std::numbers::pi_v<float> / 180.0f);
		std::size_t turned = 0;
		for (std::size_t index = 0; index < spawned.size(); ++index) {
			if (a_index >= 0 && static_cast<std::size_t>(a_index) != index)
				continue;
			auto reference = spawned[index].get();
			auto* actor = reference ? reference->As<RE::Actor>() : nullptr;
			if (!actor || !actor->Is3DLoaded())
				continue;
			actor->data.angle.z = heading;
			actor->Update3DPosition(true);
			++turned;
		}
		SetReport(std::format("Turned {} characters to {} degrees from facing the player.", turned, a_degrees));
	}

	std::vector<RE::FormID> TestCrowd::Bases() const
	{
		return crowdBases.empty() ? std::vector<RE::FormID>{ kTestCharacterBase } : crowdBases;
	}

	void TestCrowd::AddHooks(CrowdHooks* a_hooks)
	{
		if (a_hooks && std::find(hooks.begin(), hooks.end(), a_hooks) == hooks.end())
			hooks.push_back(a_hooks);
	}

	void TestCrowd::Refresh()
	{
		if (auto* task = SKSE::GetTaskInterface())
			task->AddTask([this] { SetReport(std::format("Refreshed {} characters.", RefreshNow())); });
	}

	std::size_t TestCrowd::RefreshNow()
	{
		std::size_t refreshed = 0;
		for (const RE::ObjectRefHandle& handle : spawned) {
			auto reference = handle.get();
			auto* actor = reference ? reference->As<RE::Actor>() : nullptr;
			if (!actor || !actor->Is3DLoaded())
				continue;
			actor->DoReset3D(true);
			for (CrowdHooks* hook : hooks)
				hook->AfterRefresh(*actor);
			++refreshed;
		}
		return refreshed;
	}

	void TestCrowd::SetStripped(bool a_stripped)
	{
		if (auto* task = SKSE::GetTaskInterface())
			task->AddTask([this, a_stripped] { SetStrippedNow(a_stripped); });
	}

	void TestCrowd::CycleArmor(int a_step)
	{
		if (auto* task = SKSE::GetTaskInterface())
			task->AddTask([this, a_step] { CycleArmorNow(a_step); });
	}

	bool TestCrowd::KeptByHooks(const RE::TESBoundObject& a_object) const
	{
		return std::any_of(hooks.begin(), hooks.end(), [&](const CrowdHooks* a_hook) { return a_hook->KeepWhenStripped(a_object); });
	}

	void TestCrowd::StripActor(RE::Actor& a_actor)
	{
		Outfit& outfit = outfits[a_actor.GetFormID()];
		const auto inventory = a_actor.GetInventory([](RE::TESBoundObject& a_object) { return a_object.Is(RE::FormType::Armor, RE::FormType::Weapon); });
		for (const auto& [object, entry] : inventory) {
			if (!object || !entry.second || !entry.second->IsWorn() || object == cloak || KeptByHooks(*object))
				continue;
			if (std::find(outfit.taken.begin(), outfit.taken.end(), object) == outfit.taken.end())
				outfit.taken.push_back(object);
			Unequip(a_actor, *object);
		}
	}

	void TestCrowd::RestoreActor(RE::Actor& a_actor)
	{
		const auto found = outfits.find(a_actor.GetFormID());
		if (found == outfits.end())
			return;
		if (found->second.cycled)
			Unequip(a_actor, *found->second.cycled);
		for (RE::TESBoundObject* object : found->second.taken)
			Equip(a_actor, *object);
		outfits.erase(found);
	}

	std::vector<RE::Actor*> TestCrowd::OutfitTargets()
	{
		std::vector<RE::Actor*> targets;
		for (const RE::ObjectRefHandle& handle : spawned) {
			auto reference = handle.get();
			auto* actor = reference ? reference->As<RE::Actor>() : nullptr;
			if (actor && actor->Is3DLoaded())
				targets.push_back(actor);
		}
		auto* player = globals::game::player;
		if (includePlayer.load(std::memory_order_relaxed) && player && player->Is3DLoaded())
			targets.push_back(player);
		return targets;
	}

	void TestCrowd::SetIncludePlayer(bool a_include)
	{
		if (auto* task = SKSE::GetTaskInterface())
			task->AddTask([this, a_include] { SetIncludePlayerNow(a_include); });
	}

	void TestCrowd::SetIncludePlayerNow(bool a_include)
	{
		auto* player = globals::game::player;
		if (!player || !player->Is3DLoaded()) {
			SetReport("Load a save first: there is no player to dress.");
			return;
		}
		includePlayer.store(a_include, std::memory_order_relaxed);
		if (a_include) {
			const int worn = armorIndex.load(std::memory_order_relaxed);
			if (worn >= 0)
				DressArmorNow(static_cast<std::size_t>(worn));
			else if (stripped.load(std::memory_order_relaxed))
				StripActor(*player);
			DressCloak(*player);
			SetReport("The outfit controls now act on the player too.");
			return;
		}
		RestoreActor(*player);
		if (cloak)
			Unequip(*player, *cloak);
		SetReport("The outfit controls leave the player alone again; the player's own outfit is back on.");
	}

	void TestCrowd::SetStrippedNow(bool a_stripped)
	{
		stripped.store(a_stripped, std::memory_order_relaxed);
		armorIndex.store(-1, std::memory_order_relaxed);
		for (RE::Actor* actor : OutfitTargets()) {
			if (a_stripped)
				StripActor(*actor);
			else
				RestoreActor(*actor);
		}
		SetReport(a_stripped ? "Took armor and weapons off the test characters." : "Put the test characters' outfits back.");
	}

	const std::vector<std::string>& TestCrowd::ArmorNames()
	{
		static std::vector<std::string> names;
		static std::mutex mutex;
		const std::vector<RE::TESObjectARMO*>& armors = BodyArmors();
		const std::scoped_lock lock(mutex);
		if (names.size() != armors.size()) {
			names.clear();
			std::unordered_map<std::string, std::size_t> uses;
			for (const RE::TESObjectARMO* armor : armors)
				++uses[armor->GetName()];
			for (const RE::TESObjectARMO* armor : armors)
				names.emplace_back(uses[armor->GetName()] > 1 ? std::format("{} [{:08X}]", armor->GetName(), armor->GetFormID()) : std::string(armor->GetName()));
		}
		return names;
	}

	std::vector<RE::TESObjectARMO*> TestCrowd::ArmorForms()
	{
		return BodyArmors();
	}

	std::vector<RE::TESObjectARMO*> TestCrowd::CloakForms()
	{
		return CloakArmors();
	}

	const std::vector<std::string>& TestCrowd::CloakNames()
	{
		static std::vector<std::string> names;
		static std::mutex mutex;
		const std::vector<RE::TESObjectARMO*>& cloaks = CloakArmors();
		const std::scoped_lock lock(mutex);
		if (names.size() != cloaks.size()) {
			names.clear();
			std::unordered_map<std::string, std::size_t> uses;
			for (const RE::TESObjectARMO* armor : cloaks)
				++uses[armor->GetName()];
			for (const RE::TESObjectARMO* armor : cloaks)
				names.emplace_back(uses[armor->GetName()] > 1 ? std::format("{} [{:08X}]", armor->GetName(), armor->GetFormID()) : std::string(armor->GetName()));
		}
		return names;
	}

	void TestCrowd::SelectCloak(int a_index)
	{
		if (auto* task = SKSE::GetTaskInterface())
			task->AddTask([this, a_index] { SelectCloakNow(a_index); });
	}

	void TestCrowd::DressCloak(RE::Actor& a_actor)
	{
		if (!cloak)
			return;
		if (!a_actor.GetInventoryCounts([this](RE::TESBoundObject& a_object) { return &a_object == cloak; }).contains(cloak))
			a_actor.AddObjectToContainer(cloak, nullptr, 1, nullptr);
		Equip(a_actor, *cloak);
	}

	void TestCrowd::SelectCloakNow(int a_index)
	{
		const std::vector<RE::TESObjectARMO*>& cloaks = CloakArmors();
		RE::TESObjectARMO* chosen = a_index >= 0 && static_cast<std::size_t>(a_index) < cloaks.size() ? cloaks[static_cast<std::size_t>(a_index)] : nullptr;
		RE::TESObjectARMO* previous = cloak;
		cloak = chosen;
		cloakIndex.store(chosen ? a_index : -1, std::memory_order_relaxed);
		for (RE::Actor* actor : OutfitTargets()) {
			if (previous && previous != chosen)
				Unequip(*actor, *previous);
			DressCloak(*actor);
		}
		SetReport(chosen ? std::format("Test characters wear the cloak {} ({} of {}).", chosen->GetName(), a_index + 1, cloaks.size()) : (cloaks.empty() ? "No cloak or cape is loaded." : "Took the cloak off the test characters."));
	}

	void TestCrowd::SelectArmor(std::size_t a_index)
	{
		if (auto* task = SKSE::GetTaskInterface())
			task->AddTask([this, a_index] { DressArmorNow(a_index); });
	}

	void TestCrowd::CycleArmorNow(int a_step)
	{
		const auto size = static_cast<int>(BodyArmors().size());
		if (size == 0) {
			SetReport("No playable body armor is loaded to cycle through.");
			return;
		}
		const int worn = armorIndex.load(std::memory_order_relaxed);
		const int from = worn >= 0 ? worn : (a_step > 0 ? -1 : 0);
		DressArmorNow(static_cast<std::size_t>(((from + a_step % size) % size + size) % size));
	}

	void TestCrowd::DressArmorNow(std::size_t a_index)
	{
		const std::vector<RE::TESObjectARMO*>& armors = BodyArmors();
		if (a_index >= armors.size()) {
			SetReport("No playable body armor is loaded to cycle through.");
			return;
		}
		armorIndex.store(static_cast<int>(a_index), std::memory_order_relaxed);
		RE::TESObjectARMO* armor = armors[a_index];
		stripped.store(true, std::memory_order_relaxed);
		for (RE::Actor* actor : OutfitTargets()) {
			Outfit& outfit = outfits[actor->GetFormID()];
			if (outfit.cycled)
				Unequip(*actor, *outfit.cycled);
			StripActor(*actor);
			actor->AddObjectToContainer(armor, nullptr, 1, nullptr);
			Equip(*actor, *armor);
			outfits[actor->GetFormID()].cycled = armor;
		}
		SetReport(std::format("Test characters wear {} ({} of {}).", armor->GetName(), a_index + 1, armors.size()));
	}

	void TestCrowd::DisableBatchNow()
	{
		// A few a frame: disabling many at once has the engine's deferred release queue destroy their BipedAnims together.
		constexpr std::size_t kBatch = 4;
		const std::size_t batchSize = std::min(kBatch, pendingDisable.size());
		for (std::size_t index = 0; index < batchSize; ++index) {
			// Disabling is enough: marking an actor deleted while the engine's job threads still update it is unsafe.
			if (auto reference = pendingDisable[index].get()) {
				if (auto* actor = reference->As<RE::Actor>())
					actor->EnableAI(true);
				reference->Disable();
			}
		}
		pendingDisable.erase(pendingDisable.begin(), pendingDisable.begin() + static_cast<std::ptrdiff_t>(batchSize));
		if (!pendingDisable.empty()) {
			if (auto* task = SKSE::GetTaskInterface())
				task->AddTask([this] { DisableBatchNow(); });
			return;
		}
		SetReport(std::format("Removed {} test characters.", clearTotal));
	}

	void TestCrowd::FinishDisableNow()
	{
		while (!pendingDisable.empty())
			DisableBatchNow();
	}

	void TestCrowd::ClearNow()
	{
		FinishDisableNow();
		const std::size_t removed = spawned.size();
		clearTotal = removed;
		pendingDisable = spawned;
		for (const RE::ObjectRefHandle& handle : spawned) {
			if (auto reference = handle.get()) {
				if (auto* actor = reference->As<RE::Actor>())
					actor->RemoveAnimationGraphEventSink(idleEndSink.get());
			}
		}
		watched.clear();
		{
			const std::scoped_lock lock(mutex);
			spawned.clear();
		}
		outfits.clear();
		armorIndex.store(-1, std::memory_order_relaxed);
		count.store(0, std::memory_order_relaxed);
		SetReport(std::format("Removing {} test characters.", removed));
		DisableBatchNow();
	}

	void TestCrowd::SpawnNow(std::size_t a_count, const std::vector<std::uint32_t>& a_bases)
	{
		FinishDisableNow();
		auto* player = globals::game::player;
		if (!player || !player->Is3DLoaded()) {
			SetReport("Load a save first: there is no player in the world to spawn beside.");
			return;
		}
		std::vector<RE::TESNPC*> baseForSlot;
		for (std::size_t index = 0; index < a_count; ++index) {
			const std::uint32_t baseId = BaseForSlot(index, a_bases, kTestCharacterBase);
			auto* npc = RE::TESForm::LookupByID<RE::TESNPC>(baseId);
			if (!npc) {
				SetReport(std::format("{:08X} is not a character base, so nobody was spawned.", baseId));
				return;
			}
			baseForSlot.push_back(npc);
		}
		crowdBases.clear();
		for (RE::TESNPC* npc : baseForSlot) {
			if (std::find(crowdBases.begin(), crowdBases.end(), npc->GetFormID()) == crowdBases.end())
				crowdBases.push_back(npc->GetFormID());
		}

		RE::TESIdleForm* const idle = animateOnSpawn.load(std::memory_order_relaxed) ? IdleCache::Find(Animation()) : nullptr;
		for (CrowdHooks* hook : hooks)
			hook->BeforeSpawn(a_count);

		const float heading = player->GetAngleZ();
		const RE::NiPoint3 origin = player->GetPosition();
		const float forwardX = std::sin(heading);
		const float forwardY = std::cos(heading);
		const float rightX = forwardY;
		const float rightY = -forwardX;

		// Characters from an earlier spawn stand again in their new slots with their new looks; only the ones beyond the new
		// count are disabled, because recreating actors while the engine's job threads update them is unsafe.
		std::vector<RE::ObjectRefHandle> standing;
		for (std::size_t index = 0; index < a_count; ++index) {
			RE::NiPointer<RE::TESObjectREFR> reference;
			if (index < spawned.size())
				reference = spawned[index].get();
			auto* actor = reference ? reference->As<RE::Actor>() : nullptr;
			if (actor && actor->GetActorBase() != baseForSlot[index]) {
				actor->Disable();
				actor = nullptr;
			}
			if (!actor || !actor->Is3DLoaded() || actor->IsDisabled()) {
				reference = player->PlaceObjectAtMe(baseForSlot[index], false);
				actor = reference ? reference->As<RE::Actor>() : nullptr;
			}
			if (!actor)
				continue;
			for (CrowdHooks* hook : hooks)
				hook->Dress(*actor, index);
			if (stripped.load(std::memory_order_relaxed))
				StripActor(*actor);
			DressCloak(*actor);
			const TestCrowdSlot slot = TestCrowdSlotFor(index);
			const RE::NiPoint3 spot{ origin.x + forwardX * slot.forward + rightX * slot.right, origin.y + forwardY * slot.forward + rightY * slot.right, origin.z };
			const std::optional<float> ground = GroundHeightAt(*player, spot);
			actor->SetPosition(RE::NiPoint3{ spot.x, spot.y, (ground ? *ground : spot.z) + kStandHeightOffset }, true);
			actor->data.angle.z = heading + std::numbers::pi_v<float>;
			actor->Update3DPosition(true);
			if (idle && PlayIdleOn(*actor, *idle))
				WatchForIdleEnd(*actor);
			else if (!idle)
				actor->EnableAI(false);
			standing.push_back(actor->GetHandle());
		}
		for (std::size_t index = a_count; index < spawned.size(); ++index) {
			if (auto reference = spawned[index].get())
				reference->Disable();
		}
		{
			const std::scoped_lock lock(mutex);
			spawned = std::move(standing);
		}
		count.store(spawned.size(), std::memory_order_relaxed);
		std::string description = std::format("Spawned {} test characters.", spawned.size());
		for (const CrowdHooks* hook : hooks) {
			if (const std::string extra = hook->DescribeSpawn(); !extra.empty())
				description += " " + extra;
		}
		SetReport(std::move(description));
	}

	TestCrowd& GetCrowd()
	{
		static TestCrowd crowd;
		return crowd;
	}
}
