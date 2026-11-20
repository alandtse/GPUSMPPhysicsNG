#include "pch.h"

#include "plugin/UiModel.h"

#include "Globals.h"
#include "Utils/D3D.h"
#include "Utils/DevBenchUx.h"
#include "Utils/ShaderFailure.h"
#include "Utils/TestBench/IdleCache.h"
#include "Utils/TestBench/TestCrowd.h"
#include "Utils/TestBench/TestCrowdActions.h"
#include "Utils/TestBench/TestCrowdLayout.h"
#include "engine/NifProbe.h"
#include "plugin/Instance.h"
#include "plugin/Settings.h"
#include "ui/StatusText.h"
#include "ui/UiModel.h"

#include <chrono>
#include <tuple>

namespace
{
	using SettingsUi::Row;

	Row Check(const char* a_label, const char* a_tooltip, bool* a_value, std::function<void()> a_changed = {})
	{
		Row row;
		row.kind = Row::Kind::Checkbox;
		row.label = a_label;
		row.tooltip = a_tooltip;
		row.boolValue = a_value;
		row.onChanged = std::move(a_changed);
		return row;
	}

	Row Slide(const char* a_label, const char* a_tooltip, float* a_value, float a_min, float a_max, const char* a_format)
	{
		Row row;
		row.kind = Row::Kind::Slider;
		row.label = a_label;
		row.tooltip = a_tooltip;
		row.floatValue = a_value;
		row.minimum = a_min;
		row.maximum = a_max;
		row.format = a_format;
		return row;
	}

	Row SlideInt(const char* a_label, const char* a_tooltip, int* a_value, int a_min, int a_max)
	{
		Row row;
		row.kind = Row::Kind::IntSlider;
		row.label = a_label;
		row.tooltip = a_tooltip;
		row.intValue = a_value;
		row.minimum = static_cast<float>(a_min);
		row.maximum = static_cast<float>(a_max);
		row.format = "%d";
		return row;
	}

	Row Header(const char* a_label)
	{
		Row row;
		row.kind = Row::Kind::Header;
		row.label = a_label;
		return row;
	}

	Row Note(const char* a_label)
	{
		Row row;
		row.kind = Row::Kind::Note;
		row.label = a_label;
		return row;
	}

	Row Button(const char* a_label, const char* a_tooltip, std::function<void()> a_click, bool a_sameLine = false)
	{
		Row row;
		row.kind = Row::Kind::Button;
		row.label = a_label;
		row.tooltip = a_tooltip;
		row.onChanged = std::move(a_click);
		row.sameLine = a_sameLine;
		return row;
	}

	Row Pick(const char* a_label, const char* a_tooltip, int* a_value, std::function<std::vector<std::string>()> a_items, std::function<void()> a_changed = {})
	{
		Row row;
		row.kind = Row::Kind::Combo;
		row.label = a_label;
		row.tooltip = a_tooltip;
		row.intValue = a_value;
		row.itemsFn = std::move(a_items);
		row.onChanged = std::move(a_changed);
		return row;
	}

	Row Browse(const char* a_label, const char* a_tooltip, int* a_value, std::function<std::vector<std::string>()> a_items, std::function<int()> a_selected, std::function<void()> a_picked)
	{
		Row row;
		row.kind = Row::Kind::List;
		row.label = a_label;
		row.tooltip = a_tooltip;
		row.intValue = a_value;
		row.itemsFn = std::move(a_items);
		row.selectedFn = std::move(a_selected);
		row.onChanged = std::move(a_picked);
		return row;
	}

	/** @brief Greys a row out on flat Skyrim, for controls that only act on VR hands. */
	Row VrOnly(Row a_row)
	{
		a_row.enabled = [] { return REL::Module::IsVR(); };
		a_row.tooltip += " Needs VR.";
		return a_row;
	}

	/** @brief Greys a row out while developer mode is off, since the debug overlay only draws with it on. */
	Row NeedsDeveloperMode(Row a_row)
	{
		a_row.enabled = [inner = std::move(a_row.enabled)] { return globals::state->IsDeveloperMode() && (!inner || inner()); };
		a_row.tooltip += " Needs developer mode.";
		return a_row;
	}

	Row Text(std::function<std::string()> a_text)
	{
		Row row;
		row.kind = Row::Kind::Note;
		row.textFn = std::move(a_text);
		return row;
	}

	/** @brief Runs a registered TestCrowd command with a_args, as devbench's invokeCommand would. */
	void RunCrowdCommand(const char* a_name, const json& a_args = json::object())
	{
		if (const auto* entry = Util::DevBenchUx::Registry::GetSingleton().FindToolCommand(TestBench::kOwnerName, a_name))
			entry->fn(a_args);
	}

	std::vector<std::string> WithFirst(const char* a_first, const std::vector<std::string>& a_rest)
	{
		std::vector<std::string> items{ a_first };
		items.insert(items.end(), a_rest.begin(), a_rest.end());
		return items;
	}

	/** @brief The physics files the last scan found for each item of a list, re-read when a scan finishes. */
	struct ScannedFiles
	{
		std::function<std::vector<RE::TESObjectARMO*>()> forms;
		std::uint32_t generation{};
		std::vector<std::vector<std::string>> files;

		const std::vector<std::vector<std::string>>& Get()
		{
			const std::uint32_t current = NifProbe::Generation();
			if (current != generation) {
				files.clear();
				if (current) {
					for (const RE::TESObjectARMO* item : forms())
						files.push_back(NifProbe::XmlPaths(item->GetFormID()));
				}
				generation = current;
			}
			return files;
		}
	};

	/** @brief Colours a list by how its items' physics files stand once a scan has found them; a_skip leading entries (such as "None") have no item. */
	std::function<std::vector<int>()> HealthColours(std::shared_ptr<ScannedFiles> a_files, std::size_t a_skip)
	{
		struct Cache
		{
			std::uint32_t generation{};
			std::uint32_t revision{};
			std::vector<int> values;
		};
		return [scanned = std::move(a_files), a_skip, cache = std::make_shared<Cache>()] {
			const std::uint32_t generation = NifProbe::Generation();
			if (!generation)
				return std::vector<int>{};
			const std::uint32_t revision = Instance().HealthRevision();
			if (cache->generation == generation && cache->revision == revision)
				return cache->values;
			const std::vector<SmpPhysics::Health> health = Instance().ItemHealths(scanned->Get());
			cache->values.assign(a_skip, 0);
			for (const SmpPhysics::Health state : health)
				cache->values.push_back(static_cast<int>(state));
			cache->generation = generation;
			cache->revision = revision;
			return cache->values;
		};
	}

	/** @brief Each item's physics bone count for the bone sorts; empty before a scan. a_skip leading entries have no item. */
	std::function<std::vector<int>()> ItemBones(std::shared_ptr<ScannedFiles> a_files, std::size_t a_skip)
	{
		struct Cache
		{
			std::uint32_t generation{};
			std::vector<int> values;
		};
		return [scanned = std::move(a_files), a_skip, cache = std::make_shared<Cache>()] {
			const std::uint32_t generation = NifProbe::Generation();
			if (!generation)
				return std::vector<int>{};
			if (cache->generation != generation) {
				cache->values.assign(a_skip, 0);
				for (const int bones : Instance().ItemBoneCounts(scanned->Get()))
					cache->values.push_back(bones);
				cache->generation = generation;
			}
			return cache->values;
		};
	}

	/** @brief Each item's physics file paths, joined, for a list's filter to match; empty before a scan. a_skip leading entries have no item. */
	std::function<std::vector<std::string>()> ItemFiles(std::shared_ptr<ScannedFiles> a_files, std::size_t a_skip)
	{
		struct Cache
		{
			std::uint32_t generation{};
			std::vector<std::string> values;
		};
		return [scanned = std::move(a_files), a_skip, cache = std::make_shared<Cache>()] {
			const std::uint32_t generation = NifProbe::Generation();
			if (!generation)
				return std::vector<std::string>{};
			if (cache->generation != generation) {
				cache->values.assign(a_skip, std::string{});
				for (const std::vector<std::string>& paths : scanned->Get()) {
					std::string joined;
					for (const std::string& path : paths)
						joined += path + '\n';
					cache->values.push_back(std::move(joined));
				}
				cache->generation = generation;
			}
			return cache->values;
		};
	}

	/** @brief The report for the item a list has selected, refreshed at most once a second; empty before a scan or when it has no physics. a_skip leading entries have no item. */
	std::function<std::string()> SelectedItemReport(std::function<std::vector<RE::TESObjectARMO*>()> a_forms, const int* a_index, int a_skip)
	{
		struct Cache
		{
			int index = -2;
			std::uint32_t generation{};
			std::chrono::steady_clock::time_point at{};
			std::string text;
		};
		return [forms = std::move(a_forms), a_index, a_skip, cache = std::make_shared<Cache>()] {
			const std::uint32_t generation = NifProbe::Generation();
			const int index = *a_index - a_skip;
			if (!generation || index < 0)
				return std::string();
			const auto now = std::chrono::steady_clock::now();
			if (cache->index == index && cache->generation == generation && now - cache->at < std::chrono::seconds(1))
				return cache->text;
			const std::vector<RE::TESObjectARMO*> items = forms();
			cache->text = static_cast<std::size_t>(index) < items.size() ? Instance().ItemReportText(items[static_cast<std::size_t>(index)]->GetFormID()) : std::string();
			cache->index = index;
			cache->generation = generation;
			cache->at = now;
			return cache->text;
		};
	}

	/** @brief The hover tip of a list of items: each item's physics files, their health and size, and what the last claim kept, refreshed at most once a second per item. */
	std::function<std::string(int)> ItemTip(std::function<std::vector<RE::TESObjectARMO*>()> a_forms, int a_skip)
	{
		struct Cache
		{
			int index = -2;
			std::uint32_t generation{};
			std::chrono::steady_clock::time_point at{};
			std::string text;
		};
		return [forms = std::move(a_forms), a_skip, cache = std::make_shared<Cache>()](int a_index) {
			const int index = a_index - a_skip;
			const std::uint32_t generation = NifProbe::Generation();
			if (index < 0)
				return std::string();
			if (!generation)
				return std::string("Scan for physics to see this item's files.");
			const auto now = std::chrono::steady_clock::now();
			if (cache->index == index && cache->generation == generation && now - cache->at < std::chrono::seconds(1))
				return cache->text;
			const std::vector<RE::TESObjectARMO*> items = forms();
			cache->text = static_cast<std::size_t>(index) < items.size() ? Instance().ItemStatsText(items[static_cast<std::size_t>(index)]->GetFormID()) : std::string();
			cache->index = index;
			cache->generation = generation;
			cache->at = now;
			return cache->text;
		};
	}

	/** @brief Shows a list closest to the hair target's hair first, with each item's distance after its name. */
	Row WithHairMatch(Row a_row)
	{
		a_row.sorts.push_back({ "Closest to hair target", [] { return Instance().HairTargetState(true).order; }, [] { return Instance().HairTargetState(true).badges; } });
		return a_row;
	}

	/**
	 * @brief A kind of wearable item the pickers list: its names and forms, what the crowd wears now, and how to put one on.
	 * @details Indexes are the list's, which has skip leading entries (a "None") before the first form. Devbench reaches the same lists through
	 *          testCrowdArmors, testCrowdCloaks and testCrowdWigs, and the same wearing through setTestCrowdArmor, setTestCrowdCloak and setTestCrowdWig.
	 */
	struct ItemKind
	{
		std::function<std::vector<std::string>()> names;
		std::function<std::vector<RE::TESObjectARMO*>()> forms;
		std::function<int()> worn;
		std::function<void(int)> wear;
		int skip{};
		SmpPhysics::TargetItem item{};     ///< What Put on target and Take off target act on.
		const char* noun{};                ///< Names one of this kind in the buttons.
		std::function<Row(Row)> decorate;  ///< Adds what only this kind's list has, when set.
	};

	const ItemKind& ArmorKind()
	{
		static const ItemKind kind{
			[] { return TestBench::TestCrowd::ArmorNames(); },
			[] { return TestBench::TestCrowd::ArmorForms(); },
			[] { return TestBench::GetCrowd().WornArmorIndex(); },
			[](int a_index) { TestBench::GetCrowd().SelectArmor(static_cast<std::size_t>(a_index)); },
			0,
			SmpPhysics::TargetItem::Armor,
			"armor"
		};
		return kind;
	}

	const ItemKind& CloakKind()
	{
		static const ItemKind kind{
			[] { return WithFirst("None", TestBench::TestCrowd::CloakNames()); },
			[] { return TestBench::TestCrowd::CloakForms(); },
			[] { return TestBench::GetCrowd().WornCloakIndex() + 1; },
			[](int a_index) { RunCrowdCommand("setTestCrowdCloak", { { "index", a_index - 1 } }); },
			1,
			SmpPhysics::TargetItem::Cloak,
			"cloak"
		};
		return kind;
	}

	const ItemKind& WigKind()
	{
		static const ItemKind kind{
			[] { return SmpPhysics::CrowdWigs::WigNames(); },
			[] { return SmpPhysics::CrowdWigs::WigForms(); },
			[] { return Instance().WornWigIndex(); },
			[](int a_index) { Instance().WearWig(a_index); },
			0,
			SmpPhysics::TargetItem::Wig,
			"wig",
			WithHairMatch
		};
		return kind;
	}

	/** @brief What the test characters wear: armor, cloak and wig by name, for the readout and for copying a combination. */
	std::string WornCombo()
	{
		const auto part = [](const ItemKind& a_kind) {
			const int worn = a_kind.worn();
			const std::vector<std::string> names = a_kind.names();
			return worn >= 0 && worn < static_cast<int>(names.size()) ? names[static_cast<std::size_t>(worn)] : std::string("none");
		};
		return std::format("Wearing: {} / {} / {}", part(ArmorKind()), part(CloakKind()), part(WigKind()));
	}

	/** @brief A button that copies WornCombo, so a combination can be written down or sent. */
	Row CopyCombo()
	{
		Row copy = Button("Copy combo", "Copies the armor, cloak and wig the test characters wear.", [] {}, true);
		copy.clipboardFn = WornCombo;
		return copy;
	}

	/** @brief A button that copies the selected item's full report for its mod author. */
	Row ItemCopy(const char* a_label, std::function<std::vector<RE::TESObjectARMO*>()> a_forms, const int* a_index, int a_skip)
	{
		Row copy = Button(a_label, "Copies the item's name, plugin, physics files and what is wrong with each, to send to its mod author. Scan first.", [] {});
		copy.clipboardFn = SelectedItemReport(std::move(a_forms), a_index, a_skip);
		return copy;
	}

	Row WithHealth(Row a_row, std::function<std::vector<RE::TESObjectARMO*>()> a_forms, std::size_t a_skip)
	{
		const auto scanned = std::make_shared<ScannedFiles>(ScannedFiles{ std::move(a_forms), 0, {} });
		a_row.itemFilesFn = ItemFiles(scanned, a_skip);
		a_row.progressFn = [] { return NifProbe::Running() ? NifProbe::Progress() : std::pair<std::size_t, std::size_t>{}; };
		a_row.itemStatusFn = HealthColours(scanned, a_skip);
		a_row.itemBonesFn = ItemBones(scanned, a_skip);
		a_row.itemTipFn = ItemTip(scanned->forms, static_cast<int>(a_skip));
		a_row.onShowFiltered = [] {
			if (!NifProbe::Generation() && !NifProbe::Running())
				NifProbe::Start();
		};
		return a_row;
	}

	/** @brief A coloured list of one kind of item; picking selects it. */
	Row ItemList(const ItemKind& a_kind, const char* a_label, const char* a_tooltip, int* a_index)
	{
		Row list = WithHealth(Browse(a_label, a_tooltip, a_index, a_kind.names, {}, {}), a_kind.forms, static_cast<std::size_t>(a_kind.skip));
		return a_kind.decorate ? a_kind.decorate(std::move(list)) : list;
	}

	/** @brief Choosing whose hair the wig list matches and where a wig goes: the target, what it is, and the sources to pick it from. */
	/** @brief A colour box that applies its pick at once: a name from the hair colours, Wig default to leave a worn wig as made, or Character default to give it the character's own colour. */
	Row HairColourRow(int* a_index, const char* a_tooltip, std::function<void(const std::string&)> a_apply)
	{
		return Pick(
			"Hair colour", a_tooltip, a_index, [] {
				std::vector<std::string> names{ "Wig default", "Character default" };
				for (const SmpPhysics::HairColorChoice& color : SmpPhysics::TestCrowdHairColors())
					names.emplace_back(color.name);
				return names; },
			[a_index, apply = std::move(a_apply)] {
				const auto& colors = SmpPhysics::TestCrowdHairColors();
				const bool picked = *a_index > 1 && *a_index <= static_cast<int>(colors.size()) + 1;
				apply(picked ? std::string(colors[static_cast<std::size_t>(*a_index - 2)].name) : *a_index == 1 ? std::string(SmpPhysics::kCharacterColor) :
																												  std::string{});
			});
	}

	int targetHairColor = 0;

	/** @brief The target's line; hovering it shows which hair was matched and what the target wears on its head. */
	Row TargetLine()
	{
		Row row = Text([] {
			const SmpPhysics::HairTargetView view = Instance().HairTargetState();
			return view.status.empty() ? view.text : std::format("{}  ({}: {} of {})", view.text, view.status, view.progress.first, view.progress.second);
		});
		row.tooltipFn = [cache = std::make_shared<std::pair<std::chrono::steady_clock::time_point, std::string>>()] {
			const auto now = std::chrono::steady_clock::now();
			if (now - cache->first < std::chrono::seconds(1))
				return cache->second;
			const SmpPhysics::HairTargetView view = Instance().HairTargetState();
			std::string text = view.detail;
			if (const std::string files = view.matchedModel.empty() ? std::string() : Instance().FilesStatsText(NifProbe::XmlPathsOfModel(view.matchedModel)); !files.empty())
				text += "\nPhysics of that mesh:\n" + files;
			*cache = { now, std::move(text) };
			return cache->second;
		};
		return row;
	}

	std::vector<Row> HairTargetRows()
	{
		const auto pin = [](SmpPhysics::HairTargetSource a_source) { return [a_source] { Instance().PinHairTarget(a_source); }; };
		Row console = Button("Console selection", "Matches hair to and dresses the reference selected with prid in the console. A new selection is taken at once unless you picked a source here.", pin(SmpPhysics::HairTargetSource::Console));
		console.enabled = [] { return Instance().HairTargetState().hasConsole; };
		Row crosshair = Button("Crosshair target", "Matches hair to and dresses the character you last looked at before opening this menu.", pin(SmpPhysics::HairTargetSource::Crosshair), true);
		crosshair.enabled = [] { return Instance().HairTargetState().hasCrosshair; };
		std::vector<Row> rows{
			Header("Target"),
			TargetLine(),
			std::move(console),
			std::move(crosshair),
			Button("Me", "Matches hair to and dresses you.", pin(SmpPhysics::HairTargetSource::Player), true),
			Button("Auto", "Lets the console selection choose again; with none, it is you.", [] { Instance().UnpinHairTarget(); }, true), Text([] { return std::string("Select someone with prid in the console, or look at them and open this menu. Wigs are listed closest first to their hair, or to the wig they wear."); })
		};
		return rows;
	}

	/** @brief Buttons that put the picked item on the target alone or take it off, and one that puts it on the test characters. */
	std::vector<Row> WearRows(const ItemKind& a_kind, const int* a_index)
	{
		const auto picked = [&a_kind, a_index] { return *a_index - a_kind.skip >= 0; };
		Row wear = Button("Put on target", std::format("Puts the picked {} on the target alone. An item stays in a character's inventory in your save until you take it off.", a_kind.noun).c_str(),
			[&a_kind, a_index] { Instance().WearOnTarget(a_kind.item, *a_index - a_kind.skip); });
		wear.enabled = picked;
		Row take = Button("Take off target", std::format("Takes the {} off the target and puts back what it wore; a wig shows their own hair again.", a_kind.noun).c_str(), [&a_kind] { Instance().RestoreTarget(a_kind.item); }, true);
		Row crowd = Button("Put on test characters", std::format("Puts the picked {} on every test character, and on you while Include the player is on.", a_kind.noun).c_str(), [&a_kind, a_index] { a_kind.wear(*a_index); }, true);
		crowd.enabled = picked;
		return { std::move(wear), std::move(take), std::move(crowd) };
	}

	/** @brief One kind of item in a picker: its list, a button to copy the picked item's report and the wear buttons, shown while a_shown holds. */
	std::vector<Row> KindRows(const ItemKind& a_kind, const char* a_label, const char* a_copyLabel, const char* a_tooltip, int* a_index, std::function<bool()> a_shown)
	{
		std::vector<Row> wear = WearRows(a_kind, a_index);
		Row list = ItemList(a_kind, a_label, a_tooltip, a_index);
		list.hideLabel = true;
		list.tooltip += " Double-click an item, or step with Previous and Next, to put it on the target.";
		list.onActivated = [enabled = wear.front().enabled, wearNow = wear.front().onChanged] {
			if (enabled())
				wearNow();
		};
		std::vector<Row> rows{ std::move(list), ItemCopy(a_copyLabel, a_kind.forms, a_index, a_kind.skip) };
		rows.insert(rows.end(), std::make_move_iterator(wear.begin()), std::make_move_iterator(wear.end()));
		for (Row& row : rows)
			row.shown = a_shown;
		return rows;
	}

	std::vector<Row> Join(std::vector<Row> a_first, std::vector<Row> a_second)
	{
		a_first.insert(a_first.end(), std::make_move_iterator(a_second.begin()), std::make_move_iterator(a_second.end()));
		return a_first;
	}

	std::uint64_t ClaimLineId(const SmpPhysicsFeature::ClaimRow& a_row)
	{
		return a_row.token ? a_row.token : std::hash<std::string>{}(std::format("{:08X}{}", a_row.actorFormID, a_row.file));
	}

	SettingsUi::Model model;
	SettingsUi::Model claimsModel;
	SettingsUi::Model reportModel;
	SettingsUi::Model physicsModel;
	SettingsUi::Model crowdModel;
	SettingsUi::Model developerModel;

	/** @brief What the Test crowd page edits; the buttons turn it into the same commands devbench sends. */
	struct CrowdChoices
	{
		int count = static_cast<int>(TestBench::kDefaultTestCrowd);
		bool anyAdultWoman = true;
		bool randomOrder = false;
		bool stripped = false;
		bool includePlayer = false;
		bool animateOnSpawn = false;
		bool loopAnimation = false;
		int idle = 0;
		int hairColor = 0;
	} crowdChoices;

	/** @brief What the item pickers on the Armor physics and Test crowd pages have selected: the kind shown and each kind's picked entry. */
	struct PickerChoices
	{
		int kind = 0;
		int armor = -1;
		int cloak = 0;
		int wig = -1;
	} pickerChoices;

	/** @brief Armors, cloaks and wigs in one list, the kind picked from a box; each kind keeps its own pick. */
	std::vector<Row> ItemPicker()
	{
		const auto showing = [](int a_kind) { return [a_kind] { return pickerChoices.kind == a_kind; }; };
		std::vector<Row> rows{ Pick("Item type", "Which kind of item the list below shows. Each kind keeps its own pick.", &pickerChoices.kind, [] { return std::vector<std::string>{ "Armors", "Cloaks", "Wigs" }; }) };
		const auto add = [&rows](std::vector<Row> a_more) { rows = Join(std::move(rows), std::move(a_more)); };
		add(KindRows(ArmorKind(), "Armors", "Copy report for the selected armor", "Every playable body armor. Click one to see its physics and copy a report for its mod author.", &pickerChoices.armor, showing(0)));
		add(KindRows(CloakKind(), "Cloaks", "Copy report for the selected cloak", "Every cloak and cape. Click one to see its physics and copy a report for its mod author.", &pickerChoices.cloak, showing(1)));
		add(KindRows(WigKind(), "Wigs", "Copy report for the selected wig", "Every wig, the hair-slot armors hair mods add. Click one to see its physics and copy a report for its mod author.", &pickerChoices.wig, showing(2)));
		Row colour = HairColourRow(&targetHairColor, "Sets the target's hair colour at once; a worn wig takes it. Every character made from the same base changes too. Wig default leaves a wig as made; Character default gives it the character's own colour.", [](const std::string& a_color) { Instance().ApplyTargetHairColor(a_color); });
		colour.shown = showing(2);
		rows.push_back(std::move(colour));
		return rows;
	}

	/** @brief The crowd's switch for dressing the player with it, on the Test crowd page. */
	Row IncludePlayerCheck()
	{
		return Check("Include the player", "Strip, armor and cloak choices also apply to you; off, your own outfit is put back and what was added to your inventory is removed.", &crowdChoices.includePlayer, [] { RunCrowdCommand("setTestCrowdIncludePlayer", { { "enabled", crowdChoices.includePlayer } }); });
	}
}

void PluginUi::Register()
{
	using namespace SmpPhysicsLimits;
	auto& feature = Instance();
	auto& settings = feature.settings;
	const auto reclaim = [&feature] { feature.ReevaluateClaimedActors(); };

	model.title = "Settings";
	model.rows = {
		Header("Simulation"),
		Check("Enable GPU SMP physics", "While enabled, meshes whose physics file describes a bone chain are simulated here instead of by hdtSMP64. Every other mesh stays with hdtSMP64.", &settings.Enabled, [&feature, &settings] {
			if (settings.Enabled)
				feature.ReevaluateAllActors();
			else
				feature.ReevaluateClaimedActors();
		}),
		Check("Collide with the body", "Hair is kept out of the head, neck and shoulders and off the ground.", &settings.CollisionEnabled, reclaim),
		Check("Simulate head parts", "Hair, wigs and other head parts.", &settings.ClaimHeadParts, reclaim),
		Check("Simulate armor", "Armor and clothing with physics.", &settings.ClaimArmor, reclaim),
		Check("React to nearby bodies and hands", "Other characters, the player's VR hands and held weapons or objects push the hair.", &settings.BodyContacts),
		VrOnly(Check("Hold hair with a closed hand", "Closing a hand on a strand of hair holds it to the hand until the hand opens.", &settings.GrabStrands)),
		Header("Capes and cloaks"),
		Check("Capes rest on the floor", "A cape stops at the floor under the character instead of hanging through it.", &settings.ClothFloor),
		Check("Capes react to loose objects", "A cape is pushed by loose objects the physics world moves.", &settings.ClothObjects),
		Check("Drape capes on the body", "A cape authored to hang over armor is turned in toward the body it is worn on.", &settings.DrapeCloth),
		Check("Collide hair with capes and cloth", "Hair rests on top of a cape simulated on the same character.", &settings.ClothCollision),
		Header("Physics"),
		Slide("Gravity scale", "Multiplies the mod author's value; 1 leaves it as authored. Press Re-evaluate actors to apply it to hair that is already simulated.", &settings.GravityScale, kMinScale, kMaxScale, "%.2f"),
		Slide("Stiffness scale", "Multiplies the mod author's value; 1 leaves it as authored. Press Re-evaluate actors to apply it to hair that is already simulated.", &settings.StiffnessScale, kMinScale, kMaxScale, "%.2f"),
		Slide("Damping scale", "Multiplies the mod author's value; 1 leaves it as authored. Press Re-evaluate actors to apply it to hair that is already simulated.", &settings.DampingScale, kMinScale, kMaxScale, "%.2f"),
		Slide("Wind strength", "How hard the weather's wind pushes hair and cloth outdoors; 0 switches wind off, the default is 6. Its scale differs from hdtSMP64's.", &settings.WindStrength, 0.0f, kMaxWindStrength, "%.1f"),
		Header("Limits and culling"),
		Slide("Simulation distance", "Game units from the camera beyond which claims are not stepped.", &settings.SimulationDistance, kMinSimulationDistance, kMaxSimulationDistance, "%.0f"),
		SlideInt("Maximum claims", "Meshes beyond this count keep being simulated by hdtSMP64.", &settings.MaxClaims, kMinClaims, kMaxClaims),
		SlideInt("Simulation budget", "The most physics files simulated each frame, nearest to the camera first.", &settings.MaxSimulatedClaims, kMinClaims, kMaxClaims),
		Slide("Minimum screen size (%)", "Stops simulating hair that covers less than this share of the screen height for a while; 0 switches it off.", &settings.MinScreenSizePercent, 0.0f, kMaxScreenSizePercent, "%.1f"),
		Check("Skip hair that is out of view", "Stops simulating hair that has stayed out of view for half a second and is not close to the camera.", &settings.CullOffscreen),
		Note("Hair that was skipped restarts from rest when it comes back, which can show as a brief pop."),
		Header("Performance"),
		Check("Skin from the GPU solver", "Draws hair the GPU solver simulates from its current result. Off, hair follows the result a few frames late.", &settings.GpuSkinning),
		Check("Batch the skin palette", "Builds every skinned mesh's matrices once per frame instead of once per draw.", &settings.BatchSkinPalette),
	};
	model.status = [&feature] {
		const auto status = feature.GetUiStatus();
		const auto shaders = Util::GetShaderPrewarmStatus();
		return SettingsUi::StatusText(status.linked, status.linkStatus, status.claimCount, status.stepMilliseconds, shaders.running, shaders.done, shaders.total);
	};
	model.onSave = [&feature] { PluginSettings::Save(feature); };
	model.onReevaluate = [&feature] { feature.ReevaluateAllActors(); };
	model.onReset = [&feature] { feature.ResetSettings(); };

	claimsModel.title = "Claims";
	claimsModel.showFooter = false;
	claimsModel.status = model.status;
	claimsModel.claims = [&feature] {
		std::vector<SettingsUi::ClaimLine> lines;
		auto rows = feature.GetClaimRows();
		std::ranges::sort(rows, [](const auto& a_left, const auto& a_right) { return std::tie(a_left.actorFormID, a_left.file) < std::tie(a_right.actorFormID, a_right.file); });
		for (const auto& row : rows) {
			const auto* actor = RE::TESForm::LookupByID<RE::Actor>(row.actorFormID);
			const char* name = actor ? actor->GetName() : nullptr;
			lines.push_back({ std::format("{} ({:08X})  {}  {} bones", name && *name ? name : "?", row.actorFormID, row.file, row.boneCount), row.enabled ? 1 : 0, ClaimLineId(row), row.state ? row.state : "", feature.CanRememberBaseForm(row.actorFormID) });
		}
		for (const std::string& declined : feature.GetDeclinedLines())
			lines.push_back({ "Declined: " + declined, -1, 0 });
		return lines;
	};
	claimsModel.onRemember = [&feature](std::uint64_t a_id, bool a_baseForm) {
		for (const auto& row : feature.GetClaimRows()) {
			if (ClaimLineId(row) == a_id) {
				feature.AddOverride(row.actorFormID, row.file, a_baseForm ? SmpPhysicsFeature::OverrideScope::BaseForm : SmpPhysicsFeature::OverrideScope::PhysicsFile);
				PluginSettings::SaveOverrides(feature);
				return;
			}
		}
	};
	claimsModel.rules = [&feature] { return feature.GetOverrideLines(); };
	claimsModel.onRemoveRule = [&feature](std::size_t a_index) {
		if (feature.RemoveOverride(a_index))
			PluginSettings::SaveOverrides(feature);
	};
	claimsModel.onEnabled = [&feature](std::uint64_t a_id, bool a_enabled) {
		for (const auto& row : feature.GetClaimRows()) {
			if (ClaimLineId(row) == a_id) {
				feature.SetClaimEnabled(row.actorFormID, row.file, a_enabled);
				return;
			}
		}
	};

	reportModel.title = "Physics file report";
	reportModel.showFooter = false;
	reportModel.rows = { Text([] { return std::string("Problems found in physics files this session and what was done about them, as text to paste to each file's author."); }) };
	reportModel.reports = [&feature] {
		std::vector<SettingsUi::ReportBlock> blocks;
		for (auto& [file, text] : feature.GetRepairReports())
			blocks.push_back({ file, std::move(text) });
		const json audit = NifProbe::XmlAuditResult();
		for (const auto& [key, title] : { std::pair{ "orphans", "Physics files no model uses" }, std::pair{ "dangling", "Physics files a model names that are missing" } }) {
			const json& list = audit[key];
			if (list.empty())
				continue;
			std::string text = std::string(title) + " (loose files only):\n";
			for (const auto& path : list)
				text += path.get<std::string>() + '\n';
			blocks.push_back({ std::format("{} ({})", title, list.size()), std::move(text) });
		}
		return blocks;
	};

	physicsModel.title = "Armor physics";
	physicsModel.showFooter = false;
	physicsModel.rows = Join(
		{ Text([] { return std::string("Finds which armors and cloaks carry physics and whether their files work, without dressing anyone."); }),
			Button("Rescan physics", "Reads every armor's, cloak's and head part's models for physics data and checks each physics file, then lists physics files nothing uses. A scan runs when a list first opens; this runs it again after you change your mods.", [] { NifProbe::Start(); }),
			Text([] {
				const auto [done, total] = NifProbe::Progress();
				if (NifProbe::Running())
					return std::format("Scanning: {} of {}", done, total);
				return NifProbe::Generation() ? std::string("Green [OK]: no file problems detected. Amber [REPAIRED]: repaired and used. Red [FAULT]: file fault. Plain: no physics found.") : std::string("Scanning starts when a list opens."); }) },
		Join(HairTargetRows(), ItemPicker()));

	auto& choices = crowdChoices;
	crowdModel.title = "Test crowd";
	crowdModel.showFooter = false;
	crowdModel.rows = Join(Join(Join(std::vector<Row>{
										 Header("Spawn"),
										 SlideInt("Test characters", "How many characters to spawn in rows ahead of you.", &choices.count, 1, static_cast<int>(TestBench::kMaxTestCrowd)),
										 Check("Any adult woman", "Picks from every named adult female NPC that is friendly or neutral to you; off, one generic female character is used.", &choices.anyAdultWoman),
										 Check("Random order", "Shuffles which characters appear.", &choices.randomOrder),
										 Button("Spawn test characters", "Spawns the characters standing still, each wearing a different wig.", [&choices] {
											 RunCrowdCommand("spawnTestCrowd", { { "count", choices.count }, { "anyAdultWoman", choices.anyAdultWoman }, { "randomOrder", choices.randomOrder } });
										 }),
										 Button("Remove them", "Removes the characters spawned here.", [] { RunCrowdCommand("clearTestCrowd"); }, true),
										 Button("Refresh", "Rebuilds their 3D and redoes what a rebuild undoes.", [] { RunCrowdCommand("refreshTestCrowd"); }, true),
										 Text([] { return std::format("{}  ({} standing)", TestBench::GetCrowd().Report(), TestBench::GetCrowd().Count()); }),
										 Header("Outfit"),
										 IncludePlayerCheck(),
										 Check("Strip armor and weapons", "Takes armor and weapons off, except the wigs; later spawns start stripped while this is on.", &choices.stripped, [&choices] { RunCrowdCommand("stripTestCrowd", { { "enabled", choices.stripped } }); }),
										 Text(WornCombo),
										 CopyCombo(),
									 },
									HairTargetRows()),
							   ItemPicker()),
		Join(std::vector<Row>{
				 Header("Animation"),
				 Check("Animate on spawn (characters' AI runs)", "Off, test characters spawn frozen.", &choices.animateOnSpawn, [&choices] { TestBench::GetCrowd().SetAnimateOnSpawn(choices.animateOnSpawn); }),
				 Check("Loop the animation", "Plays the animation again each time it ends.", &choices.loopAnimation, [&choices] { RunCrowdCommand("setTestCrowdLoop", { { "enabled", choices.loopAnimation } }); }),
				 Browse("Idle", "The idle animation to play on every test character.", &choices.idle, [] {
			const auto idles = TestBench::IdleCache::Snapshot();
			if (idles.empty())
				return std::vector<std::string>{};
			std::vector<std::string> names{ "Default (Cicero)" };
			for (const TestBench::IdleChoice& idle : idles)
				names.push_back(idle.editorId);
			return names; }, {}, {}),
				 Button("Play on test characters", "Plays the chosen idle on everyone standing, and on each one spawned later.", [&choices] {
			const auto idles = TestBench::IdleCache::Snapshot();
			const bool picked = choices.idle > 0 && choices.idle <= static_cast<int>(idles.size());
			RunCrowdCommand("playTestCrowdAnimation", { { "idle", picked ? idles[static_cast<std::size_t>(choices.idle - 1)].editorId : std::string{} } }); }),
				 Button("Stop and freeze", "Stops the animation.", [] { RunCrowdCommand("playTestCrowdAnimation", { { "idle", "ResetRoot" } }); }, true),
				 Header("Hair colour"),
			 },
			std::vector<Row>{ HairColourRow(&choices.hairColor, "Sets the hair colour of every base the crowd is made from, and refreshes the characters standing.", [&feature](const std::string& a_color) { feature.ApplyCrowdHairColor(a_color); }) }));

	developerModel.title = "Developer";
	developerModel.onSave = model.onSave;
	developerModel.onReset = model.onReset;
	developerModel.rows = {
		Header("Debug overlay"),
		Check("Developer mode", "Allows the debug overlay to draw. Turning it off also takes back what the test crowd put on you.", &globals::state->developerMode, [] {
			if (!globals::state->developerMode) {
				crowdChoices.includePlayer = false;
				RunCrowdCommand("setTestCrowdIncludePlayer", { { "enabled", false } });
			}
		}),
		NeedsDeveloperMode(VrOnly(Check("Show grab debug", "Draws the strands hands hold, those in reach, and the hair hands push.", &settings.ShowGrabDebug))),
		NeedsDeveloperMode(Check("Show collider debug", "Outlines the head, body and armor proxies and other bodies' capsules that hair collides with.", &settings.ShowColliderDebug)),
		NeedsDeveloperMode(Check("Show joint debug", "Draws each simulated bone from its parent, coloured by who simulates it: lime is the GPU, amber is claimed but not stepped, grey is inactive, purple is hdtSMP64 on the CPU. Orange marks stretched or squeezed joints.", &settings.ShowJointDebug)),
		NeedsDeveloperMode(Check("Show skeleton debug", "Draws the skeletons of nearby actors and the player's hands.", &settings.ShowSkeletonDebug)),
		NeedsDeveloperMode(SlideInt("Overlay actors", "Most actors the overlays draw, nearest first.", &settings.DebugMaxActors, 1, kMaxDebugActors)),
		Header("Shaders"),
		Button("Reload shaders", "Recompiles the skinning and debug line shaders from the shader folder on the next frame.", [&feature] { feature.ReloadVisualShaders(); }),
	};

	const auto shaderAlerts = [] { return Util::ShaderAlertLines(Util::GetShaderFailures()); };
	for (SettingsUi::Model* page : { &model, &claimsModel, &physicsModel, &reportModel, &crowdModel, &developerModel })
		page->alerts = shaderAlerts;
	crowdModel.lockedReason = [] { return globals::state->IsDeveloperMode() ? std::string() : std::string("Developer tool. Turn on Developer mode on the Developer page to use the test crowd."); };
	physicsModel.lockedReason = [] { return globals::state->IsDeveloperMode() ? std::string() : std::string("Developer tool. Turn on Developer mode on the Developer page to browse armor physics and try items on."); };

	SettingsUi::Register({ &model, &claimsModel, &reportModel, &developerModel, &physicsModel, &crowdModel });
}
