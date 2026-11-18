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

	/** @brief Colours a list by how its items' physics files stand once a scan has found them; a_skip leading entries (such as "None") have no item. */
	std::function<std::vector<int>()> HealthColours(std::function<std::vector<RE::TESObjectARMO*>()> a_forms, std::size_t a_skip)
	{
		struct Cache
		{
			std::uint32_t generation{};
			std::chrono::steady_clock::time_point at{};
			std::vector<int> values;
		};
		return [forms = std::move(a_forms), a_skip, cache = std::make_shared<Cache>()] {
			const std::uint32_t generation = NifProbe::Generation();
			if (!generation)
				return std::vector<int>{};
			const auto now = std::chrono::steady_clock::now();
			if (cache->generation == generation && now - cache->at < std::chrono::seconds(1))
				return cache->values;
			const std::vector<RE::TESObjectARMO*> items = forms();
			std::vector<std::vector<std::string>> files;
			files.reserve(items.size());
			for (const RE::TESObjectARMO* item : items)
				files.push_back(NifProbe::XmlPaths(item->GetFormID()));
			const std::vector<SmpPhysics::Health> health = Instance().ItemHealths(files);
			cache->values.assign(a_skip, 0);
			for (const SmpPhysics::Health state : health)
				cache->values.push_back(static_cast<int>(state));
			cache->generation = generation;
			cache->at = now;
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
	};

	const ItemKind& ArmorKind()
	{
		static const ItemKind kind{
			[] { return TestBench::TestCrowd::ArmorNames(); },
			[] { return TestBench::TestCrowd::ArmorForms(); },
			[] { return TestBench::GetCrowd().WornArmorIndex(); },
			[](int a_index) { TestBench::GetCrowd().SelectArmor(static_cast<std::size_t>(a_index)); },
			0
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
			1
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
			0
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

	/** @brief A line about the selected item's physics. */
	Row ItemNote(std::function<std::vector<RE::TESObjectARMO*>()> a_forms, const int* a_index, int a_skip)
	{
		return Text([report = SelectedItemReport(std::move(a_forms), a_index, a_skip)] {
			const std::string text = report();
			if (text.empty())
				return std::string("Select an item that has physics to see its report.");
			const std::size_t first = text.find('\n');
			return text.substr(0, text.find('\n', first == std::string::npos ? 0 : first + 1));
		});
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
		a_row.itemStatusFn = HealthColours(std::move(a_forms), a_skip);
		a_row.onShowFiltered = [] {
			if (!NifProbe::Generation() && !NifProbe::Running())
				NifProbe::Start();
		};
		return a_row;
	}

	/** @brief A coloured list of one kind of item; picking selects it and, with a_wearOnPick, puts it on the test characters at once. */
	Row ItemList(const ItemKind& a_kind, const char* a_label, const char* a_tooltip, int* a_index, bool a_wearOnPick)
	{
		return WithHealth(Browse(a_label, a_tooltip, a_index, a_kind.names, a_wearOnPick ? a_kind.worn : std::function<int()>{}, a_wearOnPick ? [&a_kind, a_index] { a_kind.wear(*a_index); } : std::function<void()>{}), a_kind.forms, static_cast<std::size_t>(a_kind.skip));
	}

	/** @brief A list of one kind of item with the selected one's physics state, a button to copy its report and one to put it on; picking only selects. */
	std::vector<Row> ItemBrowser(const ItemKind& a_kind, const char* a_label, const char* a_copyLabel, const char* a_wearLabel, const char* a_tooltip, int* a_index)
	{
		Row wear = Button(a_wearLabel, "Puts the selected item on the test characters, and on you while Include the player is on. Spawn them on the Test crowd page first, or include the player.", [&a_kind, a_index] { a_kind.wear(*a_index); }, true);
		wear.enabled = [a_index] { return *a_index >= 0; };
		wear = NeedsDeveloperMode(std::move(wear));
		Row list = ItemList(a_kind, a_label, a_tooltip, a_index, false);
		list.tooltip += " Double-click an item to put it on, as the button below does.";
		list.onActivated = [enabled = wear.enabled, wearNow = wear.onChanged] {
			if (enabled())
				wearNow();
		};
		return { std::move(list), ItemNote(a_kind.forms, a_index, a_kind.skip), ItemCopy(a_copyLabel, a_kind.forms, a_index, a_kind.skip), std::move(wear) };
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
		int armor = -1;
		int cloak = 0;
		int wig = -1;
		int idle = 0;
		int hairColor = 0;
	} crowdChoices;

	/** @brief What the Armor physics page has selected; separate from the crowd's, since picking there dresses nobody. */
	struct BrowseChoices
	{
		int armor = -1;
		int cloak = 0;
		int wig = -1;
	} browseChoices;

	/** @brief The crowd's switch for dressing the player with it; the Armor physics and Test crowd pages share it. */
	Row IncludePlayerCheck()
	{
		return Check("Include the player", "Strip, armor and cloak choices also apply to you; off, your own outfit is put back and the cloak comes off.", &crowdChoices.includePlayer, [] { RunCrowdCommand("setTestCrowdIncludePlayer", { { "enabled", crowdChoices.includePlayer } }); });
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
			Button("Scan for physics", "Reads every armor's, cloak's and head part's models for physics data and checks each physics file, then lists physics files nothing uses. Takes a few seconds.", [] { NifProbe::Start(); }),
			Text([] {
				const auto [done, total] = NifProbe::Progress();
				if (NifProbe::Running())
					return std::format("Scanning: {} of {}", done, total);
				return NifProbe::Generation() ? std::string("Green [OK]: no file problems detected. Amber [REPAIRED]: repaired and used. Red [FAULT]: file fault. Plain: no physics found.") : std::string("Not scanned yet."); }),
			NeedsDeveloperMode(IncludePlayerCheck()) },
		Join(ItemBrowser(ArmorKind(), "Armors", "Copy report for the selected armor", "Dress test characters in it", "Every playable body armor. Click one to see its physics and copy a report for its mod author.", &browseChoices.armor),
			Join(ItemBrowser(CloakKind(), "Cloaks", "Copy report for the selected cloak", "Put it on test characters", "Every cloak and cape. Click one to see its physics and copy a report for its mod author.", &browseChoices.cloak),
				ItemBrowser(WigKind(), "Wigs", "Copy report for the selected wig", "Put it on test characters", "Every wig, the hair-slot armors hair mods add. Click one to see its physics and copy a report for its mod author.", &browseChoices.wig))));

	auto& choices = crowdChoices;
	crowdModel.title = "Test crowd";
	crowdModel.showFooter = false;
	crowdModel.rows = {
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
		Button("Previous armor", "Dresses everyone in the previous body armor.", [] { RunCrowdCommand("cycleTestCrowdArmor", { { "step", -1 } }); }),
		Button("Next armor", "Dresses everyone in the next body armor.", [] { RunCrowdCommand("cycleTestCrowdArmor", { { "step", 1 } }); }, true),
		Text(WornCombo),
		CopyCombo(),
		ItemList(ArmorKind(), "Armor list", "Every playable body armor; click one to dress the test characters in it. Coloured by the scan on the Armor physics page.", &choices.armor, true),
		ItemList(CloakKind(), "Cloak", "A cloak or cape every test character wears, on top of armor or bare body.", &choices.cloak, true),
		ItemList(WigKind(), "Wig", "A wig every test character wears, in place of the one it spawned with.", &choices.wig, true),
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
		Pick("Colour", "Sets the hair colour of every base the crowd is made from.", &choices.hairColor, [] {
			std::vector<std::string> names{ "Natural" };
			for (const SmpPhysics::HairColorChoice& color : SmpPhysics::TestCrowdHairColors())
				names.emplace_back(color.name);
			return names; }),
		Button("Apply", "Applies the chosen colour and refreshes the characters standing.", [&feature, &choices] {
			const auto& colors = SmpPhysics::TestCrowdHairColors();
			const bool picked = choices.hairColor > 0 && choices.hairColor <= static_cast<int>(colors.size());
			const std::string color = picked ? std::string(colors[static_cast<std::size_t>(choices.hairColor - 1)].name) : std::string{};
			feature.ApplyCrowdHairColor(color); }),
	};

	developerModel.title = "Developer";
	developerModel.onSave = model.onSave;
	developerModel.onReset = model.onReset;
	developerModel.rows = {
		Header("Debug overlay"),
		Check("Developer mode", "Allows the debug overlay to draw.", &globals::state->developerMode),
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

	SettingsUi::Register({ &model, &claimsModel, &physicsModel, &reportModel, &developerModel, &crowdModel });
}
