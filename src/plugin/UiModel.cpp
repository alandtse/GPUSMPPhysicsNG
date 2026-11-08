#include "pch.h"

#include "plugin/UiModel.h"

#include "plugin/Instance.h"
#include "plugin/Settings.h"
#include "ui/UiModel.h"

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

	SettingsUi::Model model;
}

void PluginUi::Register()
{
	using namespace SmpPhysicsLimits;
	auto& feature = Instance();
	auto& settings = feature.settings;
	const auto reclaim = [&feature] { feature.ReevaluateClaimedActors(); };

	model.rows = {
		Header("Simulation"),
		Check("Enable GPU SMP physics", "While enabled, meshes whose physics file describes a bone chain are simulated here instead of by hdtSMP64. Every other mesh stays with hdtSMP64.", &settings.Enabled, [&feature, &settings] {
			if (settings.Enabled)
				feature.ReevaluateAllActors();
			else
				feature.ReevaluateClaimedActors();
		}),
		Check("Rigid-body solver", "Reproduces hdtSMP64's constraints. Off, the faster chain approximation steps the hair.", &settings.RigidSolver, [&feature, &settings] { feature.SetDefaultSolver(settings.RigidSolver); }),
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
		Slide("Wind strength", "How hard the weather's wind pushes the hair outdoors; 0 switches wind off.", &settings.WindStrength, 0.0f, kMaxWindStrength, "%.1f"),
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
	SettingsUi::Register(&model);
}
