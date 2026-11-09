#include "pch.h"

#include "plugin/UiModel.h"

#include "Globals.h"
#include "Utils/DevBenchUx.h"
#include "Utils/TestBench/IdleCache.h"
#include "Utils/TestBench/TestCrowd.h"
#include "Utils/TestBench/TestCrowdActions.h"
#include "Utils/TestBench/TestCrowdLayout.h"
#include "plugin/Instance.h"
#include "plugin/Settings.h"
#include "ui/UiModel.h"

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

	std::uint64_t ClaimLineId(const SmpPhysicsFeature::ClaimRow& a_row)
	{
		return a_row.token ? a_row.token : std::hash<std::string>{}(std::format("{:08X}{}", a_row.actorFormID, a_row.file));
	}

	SettingsUi::Model model;
	SettingsUi::Model claimsModel;
	SettingsUi::Model crowdModel;
	SettingsUi::Model developerModel;

	/** @brief What the Test crowd page edits; the buttons turn it into the same commands devbench sends. */
	struct CrowdChoices
	{
		int count = 10;
		bool anyAdultWoman = true;
		bool randomOrder = false;
		bool stripped = false;
		bool animateOnSpawn = false;
		bool loopAnimation = false;
		int cloak = 0;
		int idle = 0;
		int hairColor = 0;
	} crowdChoices;
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
		Check("Hold hair with a closed hand", "In VR, closing a hand on a strand of hair holds it to the hand until the hand opens.", &settings.GrabStrands),
		Header("Capes and cloaks"),
		Check("Capes rest on the floor", "A cape stops at the floor under the character instead of hanging through it.", &settings.ClothFloor),
		Check("Capes react to loose objects", "A cape is pushed by loose objects the physics world moves.", &settings.ClothObjects),
		Check("Drape capes on the body", "A cape authored to hang over armor is turned in toward the body it is worn on.", &settings.DrapeCloth),
		Check("Collide hair with capes and cloth", "Hair rests on top of a cape simulated on the same character.", &settings.ClothCollision),
		Header("Physics"),
		Slide("Gravity scale", "Applies to claims made from now on.", &settings.GravityScale, kMinScale, kMaxScale, "%.2f"),
		Slide("Stiffness scale", "Applies to claims made from now on.", &settings.StiffnessScale, kMinScale, kMaxScale, "%.2f"),
		Slide("Damping scale", "Applies to claims made from now on.", &settings.DampingScale, kMinScale, kMaxScale, "%.2f"),
		Slide("Wind strength", "How hard the weather's wind pushes hair and cloth outdoors; 0 switches wind off.", &settings.WindStrength, 0.0f, kMaxWindStrength, "%.1f"),
		Header("Limits and culling"),
		Slide("Simulation distance", "Game units from the camera beyond which claims are not stepped.", &settings.SimulationDistance, kMinSimulationDistance, kMaxSimulationDistance, "%.0f"),
		SlideInt("Maximum claims", "Meshes beyond this count keep being simulated by hdtSMP64.", &settings.MaxClaims, kMinClaims, kMaxClaims),
		SlideInt("Simulation budget", "Claims simulated each frame, nearest to the camera first.", &settings.MaxSimulatedClaims, kMinClaims, kMaxClaims),
		Slide("Minimum screen size (%)", "Stops simulating hair that covers less than this share of the screen height for a while; 0 switches it off.", &settings.MinScreenSizePercent, 0.0f, kMaxScreenSizePercent, "%.1f"),
		Check("Skip hair that is out of view", "Stops simulating hair that has stayed out of view for half a second and is not close to the camera.", &settings.CullOffscreen),
		Note("Hair that was skipped restarts from rest when it comes back, which can show as a brief pop."),
		Header("Performance"),
		Check("Skin from the GPU solver", "Draws hair the GPU solver simulates from its current result. Off, hair follows the result a few frames late.", &settings.GpuSkinning),
		Check("Batch the skin palette", "Builds every skinned mesh's matrices once per frame instead of once per draw.", &settings.BatchSkinPalette),
	};
	model.status = [&feature] {
		const auto status = feature.GetUiStatus();
		return std::format("{}\nActive claims: {}   Simulation time per frame: {:.3f} ms",
			status.linked ? status.linkStatus : std::string("Not linked to hdtSMP64. Install it and load it before this plugin."),
			status.claimCount, status.stepMilliseconds);
	};
	model.onSave = [&feature] { PluginSettings::Save(feature); };
	model.onReevaluate = [&feature] { feature.ReevaluateAllActors(); };

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
			lines.push_back({ std::format("{} ({:08X})  {}  {} bones", name && *name ? name : "?", row.actorFormID, row.file, row.boneCount), row.enabled ? 1 : 0, ClaimLineId(row) });
		}
		for (const std::string& declined : feature.GetDeclinedLines())
			lines.push_back({ "Declined: " + declined, -1, 0 });
		return lines;
	};
	claimsModel.onEnabled = [&feature](std::uint64_t a_id, bool a_enabled) {
		for (const auto& row : feature.GetClaimRows()) {
			if (ClaimLineId(row) == a_id) {
				feature.SetClaimEnabled(row.actorFormID, row.file, a_enabled);
				return;
			}
		}
	};

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
		Check("Strip armor and weapons", "Takes armor and weapons off, except the wigs; later spawns start stripped while this is on.", &choices.stripped, [&choices] { RunCrowdCommand("stripTestCrowd", { { "enabled", choices.stripped } }); }),
		Button("Previous armor", "Dresses everyone in the previous body armor.", [] { RunCrowdCommand("cycleTestCrowdArmor", { { "step", -1 } }); }),
		Button("Next armor", "Dresses everyone in the next body armor.", [] { RunCrowdCommand("cycleTestCrowdArmor", { { "step", 1 } }); }, true),
		Text([] {
			const int worn = TestBench::GetCrowd().WornArmorIndex();
			const auto& names = TestBench::TestCrowd::ArmorNames();
			return "Wearing: " + (worn >= 0 && worn < static_cast<int>(names.size()) ? names[static_cast<std::size_t>(worn)] : std::string("none"));
		}),
		Pick("Cloak", "A cloak or cape every test character wears, on top of armor or bare body.", &choices.cloak, [] { return WithFirst("None", TestBench::TestCrowd::CloakNames()); }, [&choices] { RunCrowdCommand("setTestCrowdCloak", { { "index", choices.cloak - 1 } }); }),
		Header("Animation"),
		Check("Animate on spawn (characters' AI runs)", "Off, test characters spawn frozen.", &choices.animateOnSpawn, [&choices] { TestBench::GetCrowd().SetAnimateOnSpawn(choices.animateOnSpawn); }),
		Check("Loop the animation", "Plays the animation again each time it ends.", &choices.loopAnimation, [&choices] { RunCrowdCommand("setTestCrowdLoop", { { "enabled", choices.loopAnimation } }); }),
		Pick("Idle", "The idle animation to play on every test character.", &choices.idle, [] {
			std::vector<std::string> names{ "Default (Cicero)" };
			for (const TestBench::IdleChoice& idle : TestBench::IdleCache::Snapshot())
				names.push_back(idle.editorId);
			return names;
		}),
		Button("Play on test characters", "Plays the chosen idle on everyone standing, and on each one spawned later.", [&choices] {
			const auto idles = TestBench::IdleCache::Snapshot();
			const bool picked = choices.idle > 0 && choices.idle <= static_cast<int>(idles.size());
			RunCrowdCommand("playTestCrowdAnimation", { { "idle", picked ? idles[static_cast<std::size_t>(choices.idle - 1)].editorId : std::string{} } });
		}),
		Button("Stop and freeze", "Stops the animation.", [] { RunCrowdCommand("playTestCrowdAnimation", { { "idle", "ResetRoot" } }); }, true),
		Header("Hair colour"),
		Pick("Colour", "Sets the hair colour of every base the crowd is made from.", &choices.hairColor, [] {
			std::vector<std::string> names{ "Natural" };
			for (const SmpPhysics::HairColorChoice& color : SmpPhysics::TestCrowdHairColors())
				names.emplace_back(color.name);
			return names;
		}),
		Button("Apply", "Applies the chosen colour and refreshes the characters standing.", [&feature, &choices] {
			const auto& colors = SmpPhysics::TestCrowdHairColors();
			const bool picked = choices.hairColor > 0 && choices.hairColor <= static_cast<int>(colors.size());
			const std::string color = picked ? std::string(colors[static_cast<std::size_t>(choices.hairColor - 1)].name) : std::string{};
			feature.ApplyCrowdHairColor(color);
		}),
	};

	developerModel.title = "Developer";
	developerModel.onSave = model.onSave;
	developerModel.rows = {
		Header("Debug overlay"),
		Check("Developer mode", "Allows the debug overlay to draw.", &globals::state->developerMode),
		Check("Show grab debug", "Draws the strands hands hold, those in reach, and the hair hands push.", &settings.ShowGrabDebug),
		Check("Show collider debug", "Outlines the head, body and armor proxies and other bodies' capsules that hair collides with.", &settings.ShowColliderDebug),
		Check("Show joint debug", "Draws each simulated bone from its parent and flags those stretched or squeezed from rest length.", &settings.ShowJointDebug),
		Check("Show skeleton debug", "Draws the skeletons of nearby actors and the player's hands.", &settings.ShowSkeletonDebug),
		SlideInt("Overlay actors", "Most actors the overlays draw, nearest first.", &settings.DebugMaxActors, 1, 8),
		Header("Shaders"),
		Button("Reload shaders", "Recompiles the skinning and debug line shaders from the shader folder on the next frame.", [&feature] { feature.ReloadVisualShaders(); }),
	};

	SettingsUi::Register({ &model, &claimsModel, &crowdModel, &developerModel });
}
