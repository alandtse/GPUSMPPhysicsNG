#include "Features/SmpPhysics.h"
#include "engine/NifProbe.h"

#include "Features/Wind/Wind.h"
#include "Features/Wind/WindShelterQuery.h"
#include "Globals.h"
#include "GpuPassTimer.h"
#include "Utils/ActorUtils.h"
#include "Utils/BodyCapsules.h"
#include "Utils/D3D.h"
#include "Utils/DevBenchUx.h"
#include "Utils/Game.h"
#include "Utils/HavokObjects.h"
#include "Utils/HiggsInterface.h"
#include "Utils/MathUtils.h"
#include "Utils/StringUtils.h"
#include "Utils/TestBench/TestCrowd.h"
#include "d3d/TracyGpu.h"
#include "engine/ReadyWeaponGate.h"
#include "smp/CardVertices.h"
#include "smp/GpuViews.h"
#include "smp/SceneParent.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <exception>
#include <format>
#include <functional>
#include <numbers>
#include <set>
#include <string>
#include <utility>

#define I18N_KEY_PREFIX "feature.smp_physics."

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(
	SmpPhysicsFeature::Settings,
	Enabled,
	ClaimHeadParts,
	ClaimArmor,
	CollisionEnabled,
	SimulationDistance,
	MaxClaims,
	MaxSimulatedClaims,
	MinScreenSizePercent,
	CullOffscreen,
	GravityScale,
	WindStrength,
	RigidContacts,
	StiffnessScale,
	DampingScale,
	GpuSkinning,
	RetargetRest,
	BodyContacts,
	ClothCollision,
	ClothFloor,
	ClothObjects,
	DrapeCloth,
	GrabStrands,
	ShowGrabDebug,
	ShowColliderDebug,
	ShowJointDebug,
	DebugMaxActors,
	ShowSkeletonDebug,
	BatchSkinPalette,
	FlushAfterSubmit)

using namespace SmpPhysicsLimits;

namespace
{
	constexpr std::uint32_t kMaxPhysicsFileBytes = 4u * 1024u * 1024u;
	/** @brief Seconds between reads of the weather, as in hdtSMP64. */
	constexpr float kWindTick = 0.5f;
	constexpr float kWindExposureEasing = 0.05f;
	constexpr float kWindExposureSnap = 0.01f;  ///< The exposure jumps to its target once this near it, so the easing ends.
	constexpr float kDegreesPerRadian = 180.0f / std::numbers::pi_v<float>;
	constexpr float kDefaultHandPinOffset = 15.0f;  ///< Game units along the hand's x axis a test pin sits from the hand.
	constexpr float kMaxHandPinOffset = 100.0f;
	constexpr double kMicrosecondsPerMillisecond = 1000.0;
	constexpr float kHeadHeightFallback = 100.0f;  ///< Height above the feet used for wind tests when a skeleton has no head node.
	constexpr int kDefaultRecordFrames = 300;
	constexpr int kMaxRecordFrames = 3000;
	constexpr std::array<float, 4> kDefaultProbeElevations{ 0.0f, 6.0f, 14.0f, 25.0f };
	constexpr std::size_t kMaxProbeElevations = 16;
	constexpr float kMaxProbeElevation = 90.0f;
	constexpr float kMaxProbeAzimuth = 360.0f;
	constexpr float kDefaultProbeReach = 3000.0f;
	constexpr float kMinProbeReach = 10.0f;
	constexpr float kMaxProbeReach = 20000.0f;
	/** @brief Hair points tested against the wearer's own body, and the offsets that spread the claims' shelter tests across a tick. */
	constexpr std::size_t kWindShelterSamples = 8;
	constexpr std::uint64_t kWindProbeSlots = 8;
	/** @brief Worldspaces hdtSMP64 treats as interiors even though they have a worldspace. */
	constexpr std::array<std::uint32_t, 20> kNotExteriorWorlds{ 0x69857u, 0x1EE62u, 0x20DCBu, 0x1FAE2u, 0x34240u,
		0x50015u, 0x2C965u, 0x29AB7u, 0x4F838u, 0x3A9D6u, 0x243DEu, 0xC97EBu, 0xC350Du, 0x1CDD3u, 0x1CDD9u,
		0x21EDBu, 0x1E49Du, 0x2B101u, 0x2A9D8u, 0x20BFEu };
	/** Translation jump above which the kinematic root is moved by teleport, so the solver is reset instead of dragged after it. */
	constexpr float kTeleportDistance = 150.0f;
	/** A bone's reach from its kinematic root is its rest strand length times this plus kBoneReachMargin; a solver pose past it restarts the claim. */
	constexpr float kBoneReachSlack = 2.0f;
	constexpr float kBoneReachMargin = 5.0f;
	/** @brief Consecutive frames a claim's actor may be missing before the claim is dropped. */
	constexpr std::uint32_t kStaleClaimFrames = 180;
	constexpr float kOffscreenKeepDistance = 500.0f;       ///< Hair this close to the camera is always simulated, as hdtSMP64's minCullingDistance does.
	constexpr float kOffscreenMargin = 600.0f;             ///< A claim farther outside the view than this counts as out of view.
	constexpr float kOffscreenResumeMargin = 300.0f;       ///< A culled claim resumes within this of the view; at or above kOffscreenMargin it would resume and re-cull in turn.
	constexpr float kHairScreenRadius = 80.0f;             ///< The reach of a claim's hair from its first kinematic bone, in game units, for the screen size test.
	constexpr float kBodyContactRange = 1000.0f;           ///< Bodies and hands farther than this from the camera are not tracked for hair.
	constexpr float kBodyContactReach = 8.0f;              ///< How far past a claim's hair a body capsule still matters, in game units.
	constexpr float kReleasedHandClearance = 1.0f;         ///< How far a hand that let go must be from the hair it held before it pushes it again, in game units.
	constexpr std::uint32_t kReleasedHandMaxFrames = 600;  ///< The longest a hand that let go is kept from pushing the hair, so a hand that stays in the hair does not hold it for good.
	constexpr float kGrabNearClaimMargin = 10.0f;          ///< How far past a claim's hair bound a hand may be and still have its strands tested for a grab.
	constexpr RE::FormID kPlayerFormID = 0x14;
	constexpr float kSurfaceGridCell = 8.0f;                   ///< Cell size of the grid the collider overlay uses to find the surface nearest a hair sphere, in game units; at least the farthest a sphere looks.
	constexpr std::uint32_t kActorlessClaimKey = 0x80000000u;  ///< Marks the overlay's stand-in actor key for a claim that has no actor.
	constexpr std::size_t kSphereRingMax = 150;                ///< The most rings the collider overlay draws per claim for hair spheres resting on a surface.
	constexpr float kSphereTouchTolerance = 0.3f;              ///< A hair sphere this close to a surface, past its radius and the surface margin, counts as resting on it in the collider overlay, in game units.
	constexpr float kSphereReportGap = 2.0f;                   ///< Hair spheres with more clear space than this to the nearest surface are left out of the collider overlay, in game units.
	constexpr float kDebugDetailDistance = 400.0f;             ///< Claims farther than this draw no collision mesh edges or sphere rings, in game units.
	constexpr int kDebugMaxActorsLimit = 16;                   ///< Upper end of the debug overlay actor slider.
	constexpr float kParallelAxisEpsilon = 1.0e-3f;            ///< A cross product shorter than this means the two directions are parallel.
	constexpr float kDebugMaxDistance = 1500.0f;               ///< Claims and actors farther from the camera than this are left out of the debug overlays, in game units.
	constexpr int kSkeletonMaxDepth = 32;                      ///< Deepest node nesting the skeleton overlay follows.
	constexpr float kSkeletonMinLength = 0.01f;                ///< Skeleton nodes closer than this to their parent draw no line, in game units.
	constexpr float kJointLengthTolerance = 0.1f;              ///< How far a joint's length may drift from rest, as a fraction, before the joint overlay flags it.
	constexpr float kJointMinimumRestLength = 0.1f;            ///< Joints shorter than this at rest are not judged for stretch, in game units.
	constexpr float kJointCrossSize = 0.6f;                    ///< Half-length of the cross the joint overlay draws at a bone origin, in game units.
	constexpr float kSurfaceDebugReach = 3.5f;                 ///< How close to a hair bone a collision-mesh vertex must be for the collider overlay to draw its edges, in game units.
	constexpr std::size_t kSurfaceDebugMaxLines = 2000;        ///< The most collision-mesh edges the collider overlay draws per claim.
	constexpr float kDebugDuplicateDistance = 0.5f;            ///< Two capsule ends this close count as one capsule in the collider overlay, in game units.
	constexpr float kDebugInfluenceMargin = 2.0f;              ///< How far outside a hand capsule a hair segment still counts as pushed by it in the grab overlay, in game units.
	constexpr float kTipSegmentExtension = 0.5f;               ///< How far past a tip bone's origin its strand reaches, as a share of the segment that arrives at it.
	constexpr float kHairBoundMargin = 6.0f;                   ///< Added to the farthest bone from the hair's centre, so the strands around the bones are inside the gather sphere.
	constexpr float kMaxHairBoundRadius = 300.0f;              ///< Largest gather sphere a claim may ask for, so a runaway rig cannot gather the whole cell.
	constexpr float kScreenSizeResumeFactor = 1.25f;           ///< A claim culled for being small resumes once it is this much larger than the limit.
	constexpr std::uint32_t kOffscreenCullFrames = 30;         ///< Consecutive frames out of view before a claim stops being simulated.
	constexpr std::uint32_t kRegistrationsPerFrame = 1;        ///< Claims registered with the GPU solver per frame; each registration rebuilds solver buffers.

	/** @brief The wearer's head position, or a point above the feet where the skeleton has no head node. */
	RE::NiPoint3 HeadOrigin(RE::Actor& a_actor)
	{
		const auto* head = a_actor.GetNodeByName(SmpPhysics::kHeadBoneName);
		return head ? head->world.translate : a_actor.GetPosition() + RE::NiPoint3{ 0.0f, 0.0f, kHeadHeightFallback };
	}
	constexpr float kMinFrameDelta = 0.0f;
	constexpr float kMaxFrameDelta = 0.1f;
	constexpr int kRequiredInterfaceMajor = 2;
	constexpr int kRequiredInterfaceMinor = 2;
	constexpr std::size_t kMaxDeclinedFiles = 64;
	constexpr std::size_t kMaxDeclinedRows = 20;
	thread_local std::string pendingDeclineReason;  ///< What RecordDecline last noted during the claim on this thread.
	constexpr const char* kStartupSender = "hdtsmp64";
	constexpr std::uint32_t kStartupMessageType = static_cast<std::uint32_t>(hdt::PluginInterface::MSG_STARTUP);
	using SmpPhysics::kLogPrefix;

	/** @brief Appends every skin instance below a_model that is skinned to at least one of a_bones. */
	void CollectClaimedSkins(RE::NiAVObject* a_model, const std::vector<RE::NiPointer<RE::NiAVObject>>& a_bones,
		std::vector<RE::NiPointer<RE::NiSkinInstance>>& a_out, std::vector<RE::NiPointer<RE::NiAVObject>>& a_geometries)
	{
		const std::function<void(RE::NiAVObject*)> visit = [&](RE::NiAVObject* a_node) {
			if (!a_node)
				return;
			if (auto* geometry = a_node->AsGeometry()) {
				RE::NiSkinInstance* skin = geometry->GetGeometryRuntimeData().skinInstance.get();
				if (skin && skin->skinData && skin->bones) {
					const std::uint32_t count = skin->skinData->GetBoneCount();
					bool usesRigBone = false;
					for (std::uint32_t index = 0; index < count && !usesRigBone; ++index) {
						usesRigBone = std::any_of(a_bones.begin(), a_bones.end(),
							[bone = skin->bones[index]](const RE::NiPointer<RE::NiAVObject>& a_rigBone) { return a_rigBone.get() == bone; });
					}
					if (usesRigBone) {
						a_out.emplace_back(skin);
						a_geometries.emplace_back(geometry);
					}
				}
			}
			if (auto* node = a_node->AsNode()) {
				for (auto& child : node->GetChildren())
					visit(child.get());
			}
		};
		visit(a_model);
	}

	SmpPhysics::ParseResult ReadPhysicsFile(const std::string& a_path)
	{
		SmpPhysics::ParseResult result;

		RE::BSResourceNiBinaryStream stream(a_path);
		if (!stream.good()) {
			result.reason = "the physics file could not be opened";
			return result;
		}

		const std::uint32_t size = stream.stream ? stream.stream->totalSize : 0;
		if (size == 0) {
			result.reason = "the physics file is empty";
			return result;
		}
		if (size > kMaxPhysicsFileBytes) {
			result.reason = std::format("the physics file is larger than {} bytes", kMaxPhysicsFileBytes);
			return result;
		}

		std::string text(size, '\0');
		if (!stream.read(text.data(), size)) {
			result.reason = "the physics file could not be read in full";
			return result;
		}
		return SmpPhysics::ParseSmpXml(text);
	}

	json SurfacePrepJson(const SmpPhysics::GpuSolver::SurfacePrepInfo& a_info)
	{
		return { { "active", a_info.active }, { "surfaces", a_info.surfaces }, { "vertices", a_info.vertices },
			{ "triangles", a_info.triangles }, { "nodes", a_info.nodes }, { "refitLevels", a_info.refitLevels } };
	}
}

std::pair<std::string, std::vector<std::string>> SmpPhysicsFeature::GetFeatureSummary()
{
	return { T(TKEY("description"), "Simulates hair and cloth meshes with the {brand} rigid-body solver, taking them over from hdtSMP64."),
		{ T(TKEY("key_feature_1"), "Runs its own pendulum solver on the meshes whose physics file describes a bone chain"),
			T(TKEY("key_feature_2"), "Claims only what the settings cover: head parts, armor, distance from the camera and a claim limit"),
			T(TKEY("key_feature_3"), "Hands meshes back to hdtSMP64 the moment the settings stop covering them, with no restart"),
			T(TKEY("key_feature_4"), "Works alongside hdtSMP64, which keeps simulating every mesh this feature declines") } };
}

void SmpPhysicsFeature::DataLoaded()
{
	TestBench::GetCrowd().AddHooks(&crowdWigs);
	SmpPhysics::ReadyWeaponGate::Install();
}

void SmpPhysicsFeature::OnPluginMessage(const SKSE::MessagingInterface::Message* a_message)
{
	if (a_message->type != kStartupMessageType || !a_message->data || !a_message->sender || _stricmp(a_message->sender, kStartupSender) != 0)
		return;

	auto* smpInterface = static_cast<hdt::PluginInterface*>(a_message->data);

	bool accepted = false;
	std::string status;
	{
		const std::scoped_lock lock(stateMutex);
		if (seenInterface == smpInterface)
			return;
		seenInterface = smpInterface;

		const auto smpVersion = smpInterface->getVersionInfo().interfaceVersion;
		interfaceVersion = smpVersion;
		if (smpVersion.major != kRequiredInterfaceMajor || smpVersion.minor < kRequiredInterfaceMinor) {
			linkStatus = std::format("hdtSMP64 plugin interface {}.{}.{} is older than the required {}.{}.0",
				smpVersion.major, smpVersion.minor, smpVersion.patch, kRequiredInterfaceMajor, kRequiredInterfaceMinor);
		} else {
			linkStatus = std::format("linked to hdtSMP64 plugin interface {}.{}.{}", smpVersion.major, smpVersion.minor, smpVersion.patch);
			accepted = true;
		}
		status = linkStatus;
	}

	if (!accepted) {
		logger::warn("{} {}", kLogPrefix, status);
		return;
	}

	// Registered outside stateMutex: these calls take hdtSMP64's own lock, while claim() below is
	// invoked under that lock and takes stateMutex, so holding both here would invert the order.
	smpInterface->addMeshOwner(&meshOwner, hdt::MeshOwnerInfo{ "GPUSMPPhysicsNG", 0 });
	smpInterface->addFrameListener(&frameListener);
	smpInterface->addFrameSyncListener(&frameSyncListener);
	{
		const std::scoped_lock lock(stateMutex);
		linkedInterface = smpInterface;
	}
	linked.store(true, std::memory_order_release);
	logger::info("{} {}", kLogPrefix, status);
}

void SmpPhysicsFeature::LoadSettings(json& a_json)
{
	settings = a_json;
	settings.SimulationDistance = Util::ClampFinite(settings.SimulationDistance, kMinSimulationDistance, kMaxSimulationDistance, Settings{}.SimulationDistance);
	settings.MaxClaims = std::clamp(settings.MaxClaims, kMinClaims, kMaxClaims);
	settings.MaxSimulatedClaims = std::clamp(settings.MaxSimulatedClaims, kMinClaims, kMaxClaims);
	settings.MinScreenSizePercent = Util::ClampFinite(settings.MinScreenSizePercent, 0.0f, kMaxScreenSizePercent, Settings{}.MinScreenSizePercent);
	settings.GravityScale = Util::ClampFinite(settings.GravityScale, kMinScale, kMaxScale, Settings{}.GravityScale);
	settings.WindStrength = Util::ClampFinite(settings.WindStrength, 0.0f, kMaxWindStrength, Settings{}.WindStrength);
	settings.StiffnessScale = Util::ClampFinite(settings.StiffnessScale, kMinScale, kMaxScale, Settings{}.StiffnessScale);
	settings.DampingScale = Util::ClampFinite(settings.DampingScale, kMinScale, kMaxScale, Settings{}.DampingScale);
}

void SmpPhysicsFeature::SaveSettings(json& a_json)
{
	a_json = settings;
}

void SmpPhysicsFeature::RestoreDefaultSettings()
{
	settings = {};
}

/** @brief World pose of named nodes below an actor's 3D, so the motion of a bone can be compared whoever simulates it. */
static json NodeWorldJson(const json& a_args)
{
	const auto* actor = RE::TESForm::LookupByID<RE::Actor>(a_args.value("actorFormID", 0u));
	RE::NiAVObject* root = actor ? actor->Get3D() : nullptr;
	if (!root)
		return { { "error", "the actor has no 3D loaded" } };
	const std::string match = Util::ToLowerAscii(a_args.value("contains", std::string{}));
	json rows = json::array();
	std::function<void(RE::NiAVObject*)> visit = [&](RE::NiAVObject* a_node) {
		if (!a_node)
			return;
		if (!match.empty() && Util::ToLowerAscii(a_node->name.c_str()).find(match) != std::string::npos) {
			const RE::NiTransform& world = a_node->world;
			json rotation = json::array();
			for (int row = 0; row < 3; ++row)
				rotation.push_back({ world.rotate.entry[row][0], world.rotate.entry[row][1], world.rotate.entry[row][2] });
			rows.push_back({ { "name", a_node->name.c_str() }, { "position", { world.translate.x, world.translate.y, world.translate.z } }, { "rotation", rotation } });
		}
		if (auto* node = a_node->AsNode()) {
			for (auto& child : node->GetChildren())
				visit(child.get());
		}
	};
	visit(root);
	return { { "nodes", rows } };
}

void SmpPhysicsFeature::ClearShaderCache()
{
	gpuSolver.InvalidateShaders();
	gpuSkinning.InvalidateShader();
	debugLines.InvalidateShaders();
}

void SmpPhysicsFeature::ReloadVisualShaders()
{
	Util::ClearShaderFailures();
	gpuSkinning.InvalidateShader();
	debugLines.InvalidateShaders();
}

void SmpPhysicsFeature::ReloadSolverShaders()
{
	const std::scoped_lock lock(stateMutex);
	Util::ClearShaderFailures();
	gpuSolver.InvalidateShaders();
}

void SmpPhysicsFeature::RegisterUxActions()
{
	FEATURE_QUERY("settings", "The plugin's settings as Settings.json holds them, with the values in force now.",
		[](const Feature* a_self, const json&) -> json { return static_cast<const SmpPhysicsFeature*>(a_self)->settings; });
	FEATURE_COMMAND("setSettings", "Change settings for this session without saving them. Args: any keys of the settings query, for example Enabled, ClaimArmor or WindStrength, and developerMode (bool) for the debug overlay; values are clamped as on load. A change to Enabled, ClaimHeadParts, ClaimArmor or CollisionEnabled makes hdtSMP64 offer every actor's meshes again.",
		[](Feature* a_self, const json& a_args) {
			auto* self = static_cast<SmpPhysicsFeature*>(a_self);
			const auto before = self->settings;
			json merged = self->settings;
			merged.update(a_args, true);
			if (a_args.contains("developerMode") && a_args["developerMode"].is_boolean())
				globals::state->developerMode = a_args["developerMode"].get<bool>();
			self->LoadSettings(merged);
			if (before.Enabled != self->settings.Enabled || before.ClaimHeadParts != self->settings.ClaimHeadParts || before.ClaimArmor != self->settings.ClaimArmor || before.CollisionEnabled != self->settings.CollisionEnabled)
				self->ReevaluateAllActors();
		});
	FEATURE_QUERY("nodeWorld", "World position and rotation (three rows) of named nodes below an actor's 3D, whether hdtSMP64 or this feature simulates them, so a bone's motion can be compared between the two. Args: actorFormID (number), contains (string, case-insensitive part of the node name; SMP renames a cloak's bones per armor, so \"CB 01 6\" matches where an exact name does not).",
		[](const Feature*, const json& a_args) -> json { return NodeWorldJson(a_args); });
	FEATURE_QUERY("status", "Report the hdtSMP64 link source, whether the GPU solver is available and how many claims it holds, the live claims with their bone and collision counts, step cost, wind exposure and whether the GPU solver took them, the weather wind they are pushed by, and the physics files this feature declined. In VR it also reports the strand grip (grip grab: HIGGS, per hand whether the grip is squeezed, strands in reach and held, and whether HIGGS is kept from the hand; controller pulses sent; whether the player's weapon is drawn) and the debug line counts of the strand renderer.",
		[](const Feature* a_self, const json&) -> json { return static_cast<const SmpPhysicsFeature*>(a_self)->BuildStatusJson(); });
	FEATURE_COMMAND("gpuPassTiming", "Measure the GPU time of every SmpPhysics pass with timestamp queries, for comparing builds. Args: enabled (bool, default true), reset (bool, clear the totals first). Read the result with gpuPassStats.",
		[](Feature*, const json& a_args) {
			if (a_args.value("reset", false))
				GpuPassTimer::Reset();
			GpuPassTimer::SetEnabled(a_args.value("enabled", true));
		});
	FEATURE_QUERY("gpuPassStats", "Average GPU milliseconds and calls per frame of each pass since gpuPassTiming last reset, with the number of frames measured. Nested passes (ContactPasses holds ContactCull, ContactSpheres and ContactReduce) are counted in both.",
		[](const Feature*, const json&) -> json {
			const GpuPassTimer::Stats stats = GpuPassTimer::GetStats();
			json passes = json::array();
			for (const GpuPassTimer::PassStat& pass : stats.passes)
				passes.push_back({ { "name", pass.name }, { "msPerFrame", pass.millisecondsPerFrame }, { "callsPerFrame", pass.callsPerFrame } });
			return json({ { "enabled", stats.enabled }, { "frames", stats.frames }, { "passes", passes } });
		});
	FEATURE_QUERY("solverBones", "Read, per claim, the pose the GPU solver last returned for each simulated bone in its kinematic root's frame: finite flag, local position, distance from the root, the reach limit that distance is checked against, and exceedsReach. A claim whose pose passes its reach is restarted; divergenceRestarts counts how often. Use it to see why hair is stretched. Args: actorFormID (number, optional; all claims when absent).",
		[](const Feature* a_self, const json& a_args) -> json { return static_cast<const SmpPhysicsFeature*>(a_self)->SolverBonesJson(a_args); });
	FEATURE_COMMAND("windProbe", "Queue rays from an actor's head and read their hits with windProbeResult, to see what the wind shelter test would meet. Args: actorFormID (number), elevations (array of degrees above the horizontal, negative aims down, default the shelter set), azimuth (degrees, world heading of the rays, default toward the wind), reach (units, default 3000). The rays run on the next frame.",
		[](Feature* a_self, const json& a_args) { static_cast<SmpPhysicsFeature*>(a_self)->QueueWindProbe(a_args); });
	FEATURE_QUERY("windProbeResult", "The hits of the last windProbe: per ray its elevation, how far it travelled (negative when clear), and the reference, form type and name it hit (reference 0 is terrain or other unowned geometry), or pending until it has run.",
		[](const Feature* a_self, const json&) -> json { return static_cast<const SmpPhysicsFeature*>(a_self)->WindProbeResult(); });
	FEATURE_QUERY("sampleBones", "Read the pose of every node of an actor whose name starts with a prefix, in the frame of a reference node (default the head), so SMP-driven and claimed hair can be compared. Args: actorFormID (number), prefix (string), reference (string, optional).",
		[](const Feature*, const json& a_args) -> json { return SampleBonesJson(a_args); });
	FEATURE_COMMAND("reloadShaders", "Drop the compiled skinning and debug line shaders so they recompile from Data/Shaders on the next frame, for iterating on them without restarting the game.",
		[](Feature* a_self, const json&) { static_cast<SmpPhysicsFeature*>(a_self)->ReloadVisualShaders(); });
	FEATURE_COMMAND("reloadSolverShaders", "Drop the solver's compute shaders so they recompile from Data/Shaders on the next step, for comparing two versions of a kernel in one session.",
		[](Feature* a_self, const json&) { static_cast<SmpPhysicsFeature*>(a_self)->ReloadSolverShaders(); });
	FEATURE_COMMAND("recordBones", "Capture, once per frame right after hair is stepped, the head-frame pose of an actor's nodes whose names start with a prefix. Args: actorFormID (number), prefix (string), frames (number, default 300), world (bool, also record each node's world position and rotation matrix, for the tuning bench), extraNodes (array of exact node names to record as well, e.g. the head), joints (bool, also record each simulated joint's angles, Euler XYZ in radians in the joint frame the solver limits, of the claim whose physics file contains jointFilter, a string, default the first claim of the actor).",
		[](Feature* a_self, const json& a_args) { static_cast<SmpPhysicsFeature*>(a_self)->StartBoneRecording(a_args); });
	FEATURE_QUERY("recordedBones", "Return the frames captured by recordBones: active flag, node names, and per frame the time, head world position and node head-frame positions; with joints, jointInfo (the claim's file, each joint's child and parent bone and its lower and upper limits) and per frame the joint angles in the same order.",
		[](const Feature* a_self, const json&) -> json { return static_cast<const SmpPhysicsFeature*>(a_self)->RecordedBonesJson(); });
	FEATURE_COMMAND("probeBodyContact", "Hang a test capsule below a claimed actor's first kinematic hair bone, as if another body stood there, so hair contact with other bodies can be checked without arranging two actors. Args: actorFormID (number), enabled (bool, default true), offset ([x, y, z] game units from the bone, default zero), length (number, how far the capsule reaches straight down, default 40), radius (number, default 6). The claim status lists the capsules in use under externalCapsules.",
		[](Feature* a_self, const json& a_args) { static_cast<SmpPhysicsFeature*>(a_self)->SetBodyContactProbe(a_args); });
	FEATURE_COMMAND("setTestCrowdHairColor", "Set the hair colour on every base the test crowd (see spawnTestCrowd) is made from, which overrides their natural colour, and refresh the characters standing now, so every wig can be checked in each colour; later spawns wear it too. Args: color (string, a name from testCrowdHairColors, or a hair colour form ID as a number or 0x string). Read the outcome from testCrowdReport.",
		[](Feature* a_self, const json& a_args) {
			const json& color = a_args.contains("color") ? a_args["color"] : json();
			std::string text = color.is_number_integer() ? std::to_string(color.get<std::uint64_t>()) : color.is_string() ? color.get<std::string>() :
			                                                                                                                std::string{};
			if (auto* task = SKSE::GetTaskInterface()) {
				task->AddTask([self = static_cast<SmpPhysicsFeature*>(a_self), text = std::move(text)] {
					self->crowdWigs.SetHairColorNow(TestBench::GetCrowd(), text);
				});
			}
		});
	FEATURE_QUERY("testCrowdHairColors", "The vanilla hair colours setTestCrowdHairColor knows by name, darkest first, with each colour's form ID.",
		[](const Feature*, const json&) -> json {
			json rows = json::array();
			for (const SmpPhysics::HairColorChoice& color : SmpPhysics::TestCrowdHairColors())
				rows.push_back({ { "name", std::string(color.name) }, { "formId", color.formId } });
			return rows;
		});
	FEATURE_COMMAND("restoreTestCrowdHairColors", "Put back the natural hair colour of every base setTestCrowdHairColor changed, so each test character can be compared with its own colour again, and refresh the characters standing now. Read the outcome from testCrowdReport.",
		[](Feature* a_self, const json&) {
			if (auto* task = SKSE::GetTaskInterface()) {
				task->AddTask([self = static_cast<SmpPhysicsFeature*>(a_self)] {
					self->crowdWigs.RestoreNaturalColorsNow(TestBench::GetCrowd());
				});
			}
		});
	FEATURE_QUERY("testCrowdBakedHair", "Report, per standing test character, each hair head-part node of its base: the editor ID, whether the actor's 3D has that node, and whether the node is hidden. Use it to check that baked hair is hidden under the wigs; a head part without an editor ID is listed as (unnamed).",
		[](const Feature*, const json&) -> json {
			json rows = json::array();
			for (const RE::ObjectRefHandle& handle : TestBench::GetCrowd().Standing()) {
				auto reference = handle.get();
				auto* actor = reference ? reference->As<RE::Actor>() : nullptr;
				if (!actor)
					continue;
				json nodes = json::array();
				for (const SmpPhysics::BakedHairNode& node : SmpPhysics::BakedHairNodes(*actor))
					nodes.push_back({ { "editorId", node.editorId }, { "found", node.found }, { "hidden", node.hidden } });
				const char* name = actor->GetName();
				rows.push_back({ { "actorFormID", actor->GetFormID() }, { "name", name && *name ? name : "unnamed" }, { "nodes", std::move(nodes) } });
			}
			return rows;
		});
	FEATURE_QUERY("poseCheck", "Per claim, how far the simulated hair bones are from where the animation alone would put them (the bone's rest pose carried by its tree's current root): maxSolvedFromRest and meanSolvedFromRest in game units, zero while the hair hangs at rest, maxSolvedFromSag and meanSolvedFromSag, the same against where the claim predicted the rest would settle under gravity (see collisionAudit's retargetSagMaxMove), and maxSolvedFromNode, how far the solver pose is from the engine node, which is the animation pose until the solver writes back. Args: actorFormID (number, only that actor's claims).",
		[](const Feature* a_self, const json& a_args) -> json { return static_cast<const SmpPhysicsFeature*>(a_self)->PoseCheckJson(a_args); });
	FEATURE_COMMAND("placeCrowdHairAtHand", "Move a test character so one strand of its claimed hair lies in the middle of the cast HIGGS makes from a player hand's palm along the palm normal, so a grip squeezed on that hand (for example through an injected OpenVR controller state) has hair to take. Needs the VR hand nodes. Args: hand (\"left\" or \"right\", default right), index (which test character, default 0), away (number, game units to hold the hair off the cast axis, sideways, default 0). Read the outcome from testCrowdReport.",
		[](Feature* a_self, const json& a_args) { static_cast<SmpPhysicsFeature*>(a_self)->PlaceCrowdHairAtHand(a_args); });
	FEATURE_QUERY("skinProbe", "Report a skinned mesh's NiSkinInstance (matrix counts, frame id, the engine's CPU copies of the bone matrices) next to the matrices the skinning shader would build from the bones' world transforms and skinToBone, for the first few bones. Args: actorFormID (number), mesh (string), bones (number, default 3).",
		[](const Feature*, const json& a_args) -> json { return SmpPhysicsFeature::SkinProbeJson(a_args); });
	FEATURE_COMMAND("holdStrand", "Hold one strand of a claimed actor's hair toward a fixed world point, as a closed hand would, so the pin can be checked on the CPU and GPU solvers without a headset. Args: actorFormID (number), count (number of chains to hold, default 1), bone (rig bone index to hold first; otherwise the dynamic bones farthest from the bone their chain hangs from, one per chain), offset (array of three numbers, world units from that bone's current position, default 15 along +X), enabled (bool, false releases). Read heldStrands in the status query.",
		[](Feature* a_self, const json& a_args) { static_cast<SmpPhysicsFeature*>(a_self)->SetStrandProbe(a_args); });
	FEATURE_QUERY("claimTally", "How many meshes hdtSMP64 offered this feature since the last claimTallyReset, by mesh kind (attempts, accepts, declines), how many distinct (actor, file) pairs ended taken or declined, and the decline reasons ranked by the pairs they hold back, each with its first full text, its split by kind and a few example files, plus every physics file's own counts. Quoted names and numbers in a reason are folded together so one cause is one row; pairs are counted by their latest outcome, so a retry does not inflate them.",
		[](const Feature* a_self, const json&) -> json { return static_cast<const SmpPhysicsFeature*>(a_self)->ClaimTallyJson(); });
	FEATURE_COMMAND("setCollisionOverride", "Debugging switch: stop one piece of a rig colliding with a surface or with the body proxies, then run reevaluatePlayer (or reevaluate) to rebuild the claims with it. Args: piece (string, a triangle shape's name such as Skirtpart1, whose bones' spheres are affected), target (string, a surface's name, or \"colliders\" for every body proxy), off (bool, default true; false restores the pair). The active switches are listed under collisionOverrides in status.",
		[](Feature* a_self, const json& a_args) { static_cast<SmpPhysicsFeature*>(a_self)->SetCollisionOverride(a_args.value("piece", std::string{}), a_args.value("target", std::string{}), a_args.value("off", true)); });
	FEATURE_COMMAND("clearCollisionOverrides", "Remove every setCollisionOverride switch; run reevaluatePlayer afterwards.",
		[](Feature* a_self, const json&) { static_cast<SmpPhysicsFeature*>(a_self)->SetCollisionOverride({}, {}, false); });
	FEATURE_COMMAND("nifPhysicsProbe", "Prototype: read the leading bytes of every body armor's and cloak's models on a worker thread and look for hdtSMP64's physics extra data, to time a scan that does not dress anyone. Read the outcome from nifPhysicsProbeResult.",
		[](Feature*, const json&) { NifProbe::Start(); });
	FEATURE_QUERY("nifPhysicsProbeResult", "The nifPhysicsProbe outcome: running, armors done of total, models read and missing, bytes read, elapsed milliseconds and milliseconds per model, and the models whose header names the physics extra data with the xml paths it holds.",
		[](const Feature*, const json&) -> json { return NifProbe::Result(); });
	FEATURE_QUERY("xmlAudit", "After nifPhysicsProbe has finished: the loose physics xml files that no model or defaultBBPs.xml entry names (orphans) and the names with no loose file (dangling; a file packed in an archive shows here too).",
		[](const Feature*, const json&) -> json { return NifProbe::XmlAuditResult(); });
	FEATURE_QUERY("islandSizes", "How the rigid solver's islands split by constraint count in the last buffer build: islands and constraints per size bucket (1, 2-4, 5-8, 9-16, 17-32, 33-64, 65 or more), the island total, the largest island's constraints and the most colour classes one takes. Shows which solve kernel suits the islands.",
		[](const Feature* a_self, const json&) -> json { return static_cast<const SmpPhysicsFeature*>(a_self)->IslandSizesJson(); });
	FEATURE_QUERY("physicsHealth", "After nifPhysicsProbe has finished: each armor and cloak the scan found physics files for, with the worst health among them (working, repaired or broken) and the files; what colours the Test crowd lists.",
		[](const Feature* a_self, const json&) -> json { return const_cast<SmpPhysicsFeature*>(static_cast<const SmpPhysicsFeature*>(a_self))->PhysicsHealthJson(); });
	FEATURE_COMMAND("claimTallyReset", "Clear the claim tally, then run reevaluate to count every actor's meshes again.",
		[](Feature* a_self, const json&) { static_cast<SmpPhysicsFeature*>(a_self)->ResetClaimTally(); });
	FEATURE_COMMAND("reevaluate", "Ask hdtSMP64 to re-run claiming for every loaded actor, so the current settings take effect.",
		[](Feature* a_self, const json&) { static_cast<SmpPhysicsFeature*>(a_self)->ReevaluateAllActors(); });
	FEATURE_COMMAND("setClaimEnabled", "Switch one actor's physics file off or on for testing: off, this feature declines the file for that actor and hdtSMP64 is asked to simulate it, so the same hair can be compared in both solvers; the settings menu lists the same switches. Args: actorFormID (number), file (string, the physics file as status lists it, compared without regard to case), enabled (bool, default true).",
		[](Feature* a_self, const json& a_args) { static_cast<SmpPhysicsFeature*>(a_self)->SetClaimEnabled(a_args.value("actorFormID", 0u), a_args.value("file", std::string{}), a_args.value("enabled", true)); });
	FEATURE_COMMAND("reevaluatePlayer", "Ask hdtSMP64 to re-run claiming for the player only.",
		[](Feature* a_self, const json&) { static_cast<SmpPhysicsFeature*>(a_self)->ReevaluatePlayer(); });
	FEATURE_QUERY("kinematicCheck", "Measure, for a claimed mesh, how far the engine's skinning matrices for the bones the solver does not simulate are from matrices rebuilt from the world poses recorded when the solver stepped: engineVsStepPoseMax near float rounding means those bones could be built once per frame after the solver step. Call with arm true, move the actor or camera for a few seconds, then call with arm false to read samples, staticBones, engineVsBindPoseMax, engineVsStepPoseMax and stepVsBindTranslationMax. Args: actorFormID (number), mesh (string), arm (boolean).",
		[](const Feature* a_self, const json& a_args) -> json { return static_cast<const SmpPhysicsFeature*>(a_self)->KinematicCheckJson(a_args); });
	FEATURE_QUERY("skinPaletteCheck", "Compare the skinning matrices bound for a claimed mesh with the engine's own, bone by bone: bones the solver does not simulate must match exactly, simulated bones should differ only by the solver's newer pose. Each call requests a fresh capture and answers with the one the previous call requested. Args: actorFormID (number), mesh (string).",
		[](const Feature* a_self, const json& a_args) -> json { return static_cast<const SmpPhysicsFeature*>(a_self)->SkinPaletteCheckJson(a_args); });
	FEATURE_QUERY("collisionAudit", "Report, on demand, the collision proxies this feature built for an actor: per claim, how many spheres it has and how many of those start inside a collider and are exempt from it, then every collider's type, its driver node, its radius, whether it was fitted to the worn meshes or left at a default, and how many skinned body vertices the fit used; per claim, spheresBehindFace counts the spheres whose centre sits behind the nearest one-sided armor face by more than the allowance and skin, deepestBehindFace is the deepest of them, retargetBehindSpheres, retargetResidualSpheres, retargetedJoints, retargetKeptAuthored, retargetMaxAngle, retargetMaxBoneMove and retargetSagMaxMove say what the claim-time rest retarget did about spheres that settle behind a worn surface once the rest sags under gravity, and spheresInsideShell (with insideShellBones per hair bone) counts the spheres now inside a worn shell by the retarget's own measure, the distance straight out from the body axis to the face the hair would exit through, so the two agree when the hair settled where the retarget predicted; spheresBehindOutwardFace how many of those faces point away from the body axis (an outer plate rather than an inner lining), behindFaceDepthBuckets counts them at up to 1, 2, 4, 8, 16 and more units behind, behindFaceBones counts them per hair bone, behindFaceSurfaces counts them per surface as [outward-facing, inward-facing] lists over the same depth buckets, and behindBySphere lists each sphere's distance behind its nearest face, so a run before and after a pull shows which spheres ended up under the armor; every posed collision surface adds its triangle count, margin, penetration allowances and world-space bounds next to the head driver. Args: actorFormID (number), optionally with hairMesh and targetMesh together, which also measures how deep the hair mesh sinks into the target mesh, skinning both with the nodes' current world transforms so hdtSMP64-driven and claimed hair can be compared, with the GPU solver's pose for the bones it moves unless pose is \"engine\"; overlaps true, with neither mesh given, measures every ordered pair of the claim's own shapes whose tags let them collide and lists, per pair, how many vertices of the first sit behind the second and how deep. When any hair vertex lies behind the target it also reports insideBounds (low/high corners in the head frame), insideByHeight (counts per 10 unit bucket of head-frame height) and insideByBone (the hair bones owning most of those vertices, with their mean depth).",
		[](const Feature* a_self, const json& a_args) -> json { return static_cast<const SmpPhysicsFeature*>(a_self)->CollisionAuditJson(a_args); });
	FEATURE_QUERY("surfaceCheck", "Copy back a submitted frame's GPU collision-surface pass and compare it with the CPU reference for the same driver poses, answering with the check the previous call requested (call twice, a frame apart), reporting how many triangle and node records were compared, the largest component error of each and how many records, node boxes and node topologies differ; no mismatches and a zero error means the GPU pass reproduces the reference exactly up to float rounding, and the check needs a submitted frame with the GPU backend active.",
		[](const Feature* a_self, const json&) -> json { return static_cast<const SmpPhysicsFeature*>(a_self)->SurfaceCheckJson(); });
}

void SmpPhysicsFeature::OnDeferredGeometryEnd()
{
	if (!settings.Enabled || !linked.load(std::memory_order_acquire))
		return;

	debugLines.Render();
}

void SmpPhysicsFeature::OnBoneMatricesUploaded(RE::NiSkinInstance* a_skin)
{
	gpuSkinning.Apply(a_skin);
}

namespace
{
	struct PoseSample
	{
		std::string name;
		RE::NiPoint3 local;
		float bendDeg;
		RE::NiPoint3 world;
		float rotation[9];
	};

	/** @return False when the actor or the reference node is not loaded; a_referenceWorld is the reference node's world position. */
	bool CollectPose(RE::FormID a_formID, const std::string& a_prefix, const std::string& a_referenceName, RE::NiPoint3& a_referenceWorld, std::vector<PoseSample>& a_out, const std::vector<std::string>& a_extraNames = {})
	{
		auto* actor = a_formID ? RE::TESForm::LookupByID<RE::Actor>(a_formID) : nullptr;
		auto* root = actor ? actor->Get3D() : nullptr;
		auto* reference = root ? root->GetObjectByName(RE::BSFixedString(a_referenceName)) : nullptr;
		if (!reference)
			return false;

		const auto& r = reference->world.rotate.entry;
		a_referenceWorld = reference->world.translate;
		std::function<void(RE::NiAVObject*)> visit = [&](RE::NiAVObject* a_node) {
			if (!a_node)
				return;
			const bool extra = std::ranges::any_of(a_extraNames, [&](const std::string& a_name) { return Util::IEquals(a_node->name.c_str(), a_name); });
			if (extra || (!a_prefix.empty() && a_node->name.size() >= a_prefix.size() && _strnicmp(a_node->name.c_str(), a_prefix.c_str(), a_prefix.size()) == 0)) {
				const auto& c = a_node->world.rotate.entry;
				float trace = 0.0f;
				for (int i = 0; i < 3; ++i)
					for (int k = 0; k < 3; ++k)
						trace += r[k][i] * c[k][i];
				const RE::NiPoint3 d = a_node->world.translate - a_referenceWorld;
				a_out.push_back({ a_node->name.c_str(),
					{ r[0][0] * d.x + r[0][1] * d.y + r[0][2] * d.z, r[1][0] * d.x + r[1][1] * d.y + r[1][2] * d.z, r[2][0] * d.x + r[2][1] * d.y + r[2][2] * d.z },
					std::acos(std::clamp((trace - 1.0f) * 0.5f, -1.0f, 1.0f)) * kDegreesPerRadian,
					a_node->world.translate,
					{ c[0][0], c[0][1], c[0][2], c[1][0], c[1][1], c[1][2], c[2][0], c[2][1], c[2][2] } });
			}
			if (auto* node = a_node->AsNode()) {
				for (auto& child : node->GetChildren())
					visit(child.get());
			}
		};
		visit(root);
		return true;
	}
}

json SmpPhysicsFeature::SampleBonesJson(const json& a_args)
{
	RE::NiPoint3 referenceWorld;
	std::vector<PoseSample> samples;
	if (!CollectPose(a_args.value("actorFormID", 0u), a_args.value("prefix", std::string{}), a_args.value("reference", std::string(SmpPhysics::kHeadBoneName)), referenceWorld, samples))
		return { { "error", "actor or reference node not loaded" } };

	json bones = json::array();
	for (const auto& sample : samples)
		bones.push_back({ { "name", sample.name }, { "bendDeg", sample.bendDeg }, { "x", sample.local.x }, { "y", sample.local.y }, { "z", sample.local.z } });
	return { { "bones", bones } };
}

void SmpPhysicsFeature::StartBoneRecording(const json& a_args)
{
	const std::scoped_lock lock(recordingMutex);
	recording = {};
	recording.actorFormID = a_args.value("actorFormID", 0u);
	recording.prefix = a_args.value("prefix", std::string{});
	recording.framesRemaining = std::clamp(a_args.value("frames", kDefaultRecordFrames), 1, kMaxRecordFrames);
	recording.world = a_args.value("world", false);
	recording.joints = a_args.value("joints", false);
	recording.jointFilter = a_args.value("jointFilter", std::string{});
	recording.extraNodes = a_args.value("extraNodes", std::vector<std::string>{});
	recording.active = true;
}

void SmpPhysicsFeature::RecordBoneFrame()
{
	const std::scoped_lock lock(recordingMutex);
	if (!recording.active)
		return;

	RE::NiPoint3 referenceWorld;
	std::vector<PoseSample> samples;
	if (CollectPose(recording.actorFormID, recording.prefix, SmpPhysics::kHeadBoneName, referenceWorld, samples, recording.extraNodes)) {
		if (recording.names.empty()) {
			for (const auto& sample : samples)
				recording.names.push_back(sample.name);
		}
		const float frameDelta = globals::game::deltaTime ? *globals::game::deltaTime : RE::GetSecondsSinceLastFrame();
		recording.elapsedSeconds += frameDelta;
		json bones = json::array();
		for (const auto& sample : samples)
			bones.push_back({ sample.local.x, sample.local.y, sample.local.z });
		json frame = { { "t", recording.elapsedSeconds }, { "dt", frameDelta }, { "head", { referenceWorld.x, referenceWorld.y, referenceWorld.z } }, { "bones", bones } };
		if (recording.world) {
			json world = json::array();
			for (const auto& sample : samples) {
				world.push_back({ sample.world.x, sample.world.y, sample.world.z });
				for (const float value : sample.rotation)
					world.back().push_back(value);
			}
			frame["world"] = std::move(world);
		}
		if (recording.joints)
			RecordJointFrame(frame);
		recording.frames.push_back(std::move(frame));
	}
	if (--recording.framesRemaining <= 0)
		recording.active = false;
}

void SmpPhysicsFeature::RecordJointFrame(json& a_frame)
{
	const std::scoped_lock lock(stateMutex);
	const Claim* claim = nullptr;
	for (const auto& [token, candidate] : claims) {
		if (candidate.actorFormID == recording.actorFormID && candidate.parsed && candidate.restPose.size() == candidate.nodes.size() &&
			candidate.physicsFile.find(recording.jointFilter) != std::string::npos && candidate.dynamicBoneCount > 0) {
			claim = &candidate;
			break;
		}
	}
	if (!claim)
		return;
	const SmpPhysics::SmpRig& rig = claim->parsed->rig;
	if (recording.jointToken != claim->token) {
		recording.jointToken = claim->token;
		recording.jointFrames.clear();
		json names = json::array(), parents = json::array(), lower = json::array(), upper = json::array();
		for (std::size_t index = 0; index < rig.joints.size(); ++index) {
			const SmpPhysics::RigJoint& joint = rig.joints[index];
			if (rig.bones[static_cast<std::size_t>(joint.child)].kinematic)
				continue;
			recording.jointFrames.emplace_back(index, SmpPhysics::ResolveJointFrames(joint, claim->restPose[static_cast<std::size_t>(joint.child)], claim->restPose[static_cast<std::size_t>(joint.parent)]));
			names.push_back(rig.bones[static_cast<std::size_t>(joint.child)].name);
			parents.push_back(rig.bones[static_cast<std::size_t>(joint.parent)].name);
			lower.push_back({ joint.angularLower.x, joint.angularLower.y, joint.angularLower.z });
			upper.push_back({ joint.angularUpper.x, joint.angularUpper.y, joint.angularUpper.z });
		}
		recording.jointInfo = { { "file", claim->physicsFile }, { "child", names }, { "parent", parents }, { "lower", lower }, { "upper", upper } };
	}
	json angles = json::array();
	for (const auto& [index, frames] : recording.jointFrames) {
		const SmpPhysics::RigJoint& joint = rig.joints[index];
		const SmpPhysics::Transform parent = claim->BoneWorld(static_cast<std::size_t>(joint.parent));
		const SmpPhysics::Transform child = claim->BoneWorld(static_cast<std::size_t>(joint.child));
		const SmpPhysics::Quat jointRotation = parent.rotation * frames.frameB.rotation;
		const SmpPhysics::Vec3 euler = SmpPhysics::ToEulerXYZ(SmpPhysics::Conjugate(jointRotation) * child.rotation * frames.frameA.rotation);
		angles.push_back({ euler.x, euler.y, euler.z });
	}
	a_frame["joints"] = std::move(angles);
}

json SmpPhysicsFeature::RecordedBonesJson() const
{
	const std::scoped_lock lock(recordingMutex);
	return { { "active", recording.active }, { "frameCount", recording.frames.size() }, { "names", recording.names }, { "jointInfo", recording.jointInfo }, { "frames", recording.frames } };
}

json SmpPhysicsFeature::SolverBonesJson(const json& a_args) const
{
	const RE::FormID only = a_args.value("actorFormID", 0u);
	json claimRows = json::array();
	const std::scoped_lock lock(stateMutex);
	for (const auto& entry : claims) {
		const Claim& claim = entry.second;
		if (!claim.parsed || (only != 0 && claim.actorFormID != only))
			continue;
		const SmpPhysics::SmpRig& rig = claim.parsed->rig;
		json bones = json::array();
		for (std::size_t index = 0; index < rig.bones.size(); ++index) {
			if (rig.bones[index].kinematic)
				continue;
			json row = { { "bone", index }, { "name", rig.bones[index].name } };
			if (index < claim.gpuLastBones.size()) {
				const SmpPhysics::Transform& pose = claim.gpuLastBones[index];
				const bool finite = SmpPhysics::IsFiniteTransform(pose);
				const float distance = SmpPhysics::Length(pose.translation);
				row["finite"] = finite;
				row["local"] = { pose.translation.x, pose.translation.y, pose.translation.z };
				row["distance"] = distance;
				if (index < claim.boneReach.size())
					row["exceedsReach"] = !finite || distance > claim.boneReach[index];
			}
			if (index < claim.boneReach.size())
				row["reach"] = claim.boneReach[index];
			bones.push_back(std::move(row));
		}
		claimRows.push_back({ { "token", entry.first }, { "actorFormID", claim.actorFormID }, { "file", claim.physicsFile },
			{ "gpuRegistered", claim.gpuRegistered }, { "hasPose", claim.gpuLastBones.size() == rig.bones.size() },
			{ "divergenceRestarts", claim.divergenceRestarts }, { "bones", std::move(bones) } });
	}
	return { { "claims", std::move(claimRows) } };
}

json SmpPhysicsFeature::BuildStatusJson() const
{
	json result;
	{
		const std::scoped_lock lock(stateMutex);
		result["linked"] = linked.load(std::memory_order_relaxed);
		result["interfaceVersion"] = { interfaceVersion.major, interfaceVersion.minor, interfaceVersion.patch };
		result["enabled"] = settings.Enabled;
		result["claimCount"] = claims.size();
		json overrides = json::array();
		for (const SmpPhysics::CollisionOverride& override : collisionOverrides)
			overrides.push_back({ { "piece", override.piece }, { "target", override.target } });
		result["collisionOverrides"] = std::move(overrides);
		result["simulationBudget"] = settings.MaxSimulatedClaims;
		result["culledByBudget"] = std::count_if(claims.begin(), claims.end(), [](const auto& a_entry) { return a_entry.second.culledByBudget; });
		result["culledOffscreen"] = std::count_if(claims.begin(), claims.end(), [](const auto& a_entry) { return a_entry.second.culledOffscreen; });
		result["culledSmall"] = std::count_if(claims.begin(), claims.end(), [](const auto& a_entry) { return a_entry.second.culledSmall; });
		result["totalStepMicroseconds"] = totalStepMicroseconds;
		result["gpuAvailable"] = gpuSolver.Available();
		result["gpuClaimCount"] = gpuSolver.ClaimCount();
		result["gpuRigidClaimCount"] = gpuSolver.Rigid().ClaimCount();
		result["gpuRigidAvailable"] = gpuSolver.Rigid().Available();
		result["heldStrands"] = HeldStrandsJson();
		result["grip"] = GripReportJson();
		result["gpuSurfacePrep"] = SurfacePrepJson(gpuSolver.SurfacePrep());
		if (!gpuSolver.Available())
			result["gpuUnavailableReason"] = gpuSolver.UnavailableReason();
		result["wind"] = { { "outdoors", windOutdoors }, { "strength", settings.WindStrength }, { "clock", windClock } };
		{
			const SmpPhysics::DebugLines::Stats lineStats = debugLines.GetStats();
			result["debugLines"] = { { "requested", lineStats.requested }, { "drawn", lineStats.drawn }, { "stateFailures", lineStats.stateFailures }, { "failed", lineStats.failed },
				{ "developerMode", globals::state->IsDeveloperMode() }, { "shaderCacheEnabled", false } };
		}
		result["gpuSkinning"] = { { "enabled", settings.GpuSkinning }, { "skinInstances", gpuSkinning.SkinCount() }, { "appliedSinceLastStatus", gpuSkinning.TakeAppliedCount() }, { "batched", settings.BatchSkinPalette }, { "batchedSkinInstances", gpuSkinning.BatchedSkinCount() } };

		result["claims"] = json::array();
		gpuSolver.RequestContactStats();
		const auto contactStats = gpuSolver.ContactStats();
		for (const auto& entry : claims) {
			const Claim& claim = entry.second;
			json contacts = { { "touching", 0 }, { "meanDepth", 0.0f }, { "maxDepth", 0.0f } };
			for (const auto& stat : contactStats) {
				if (stat.token == entry.first)
					contacts = { { "touching", stat.touching }, { "meanDepth", stat.meanDepth }, { "maxDepth", stat.maxDepth }, { "bones", stat.bones },
						{ "detail", [&stat]() { json rows = json::array(); for (std::size_t i = 0; i < stat.contacts.size(); ++i) { const auto& c = stat.contacts[i]; rows.push_back({ { "bone", stat.bones[i] }, { "depth", c.depth }, { "deepest", c.deepest }, { "collider", c.collider }, { "normal", { c.normal[0], c.normal[1], c.normal[2] } }, { "position", { c.position[0], c.position[1], c.position[2] } } }); } return rows; }() } };
			}
			result["claims"].push_back({ { "token", entry.first },
				{ "contacts", contacts },
				{ "boneSpheres", [&claim]() { json counts = json::array(); for (const auto& list : claim.spheres) counts.push_back(list.size()); return counts; }() },
				{ "actorFormID", claim.actorFormID },
				{ "file", claim.physicsFile },
				{ "boneCount", claim.boneCount },
				{ "dynamicBoneCount", claim.dynamicBoneCount },
				{ "additionalJointCount", claim.additionalJointCount },
				{ "sphereCount", claim.sphereCount },
				{ "colliderCount", claim.colliderCount },
				{ "surfaceCount", claim.surfaces.size() },
				{ "gpuRegistered", claim.gpuRegistered },
				{ "divergenceRestarts", claim.divergenceRestarts },
				{ "stepMicroseconds", claim.stepMicroseconds },
				{ "windFactor", claim.windFactor },
				{ "windVector", { claim.wind.x, claim.wind.y, claim.wind.z } },
				{ "windMagnitude", SmpPhysics::Length(claim.wind) },
				{ "windTarget", claim.windTarget },
				{ "windWorldExposure", claim.windWorldExposure },
				{ "windHit", [&claim]() {
					 json hit = { { "distance", claim.windHitDistance }, { "reference", claim.windHitReference } };
					 if (auto* reference = claim.windHitReference ? RE::TESForm::LookupByID<RE::TESObjectREFR>(claim.windHitReference) : nullptr) {
						 hit["name"] = reference->GetDisplayFullName();
						 hit["baseEditorId"] = reference->GetBaseObject() ? reference->GetBaseObject()->GetFormEditorID() : "";
						 hit["formType"] = static_cast<int>(reference->GetFormType());
					 }
					 return hit;
				 }() },
				{ "windSelfExposure", claim.windSelfExposure }, { "windRayHits", claim.windRayHits }, { "actorFound", RE::TESForm::LookupByID<RE::Actor>(claim.actorFormID) != nullptr }, { "actor3DLoaded", [&claim]() { const auto* actor = RE::TESForm::LookupByID<RE::Actor>(claim.actorFormID); return actor != nullptr && actor->Is3DLoaded(); }() }, { "cameraDistance", claim.lastCameraDistance }, { "steppedFrames", claim.steppedFrames }, { "skippedByDistanceFrames", claim.skippedByDistanceFrames }, { "skippedNotReadyFrames", claim.skippedNotReadyFrames }, { "inactive", claim.inactive }, { "culledByBudget", claim.culledByBudget }, { "culledOffscreen", claim.culledOffscreen }, { "culledSmall", claim.culledSmall }, { "externalColliders", claim.externalCount }, { "externalCapsules", [&claim]() { json rows = json::array(); for (std::size_t i = 0; i < claim.externalColliders.size(); ++i) { const auto& c = claim.externalColliders[i]; rows.push_back({ c.segmentA.x, c.segmentA.y, c.segmentA.z, c.segmentB.x, c.segmentB.y, c.segmentB.z, c.radius, claim.externalOwners[i] }); } return rows; }() }, { "externalCentre", { claim.externalCentre.x, claim.externalCentre.y, claim.externalCentre.z } } });
		}

		result["declined"] = json::array();
		for (const auto& entry : declinedFiles)
			result["declined"].push_back({ { "file", entry.first }, { "reason", entry.second } });
	}
	return result;
}

SmpPhysicsFeature::UiStatus SmpPhysicsFeature::GetUiStatus() const
{
	const std::scoped_lock lock(stateMutex);
	return { linked.load(std::memory_order_relaxed), linkStatus, claims.size(), totalStepMicroseconds / kMicrosecondsPerMillisecond };
}

void SmpPhysicsFeature::ApplyCrowdHairColor(std::string a_color)
{
	if (auto* task = SKSE::GetTaskInterface())
		task->AddTask([this, color = std::move(a_color)] {
			if (color.empty())
				crowdWigs.RestoreNaturalColorsNow(TestBench::GetCrowd());
			else
				crowdWigs.SetHairColorNow(TestBench::GetCrowd(), color);
		});
}

std::vector<SmpPhysicsFeature::ClaimRow> SmpPhysicsFeature::GetClaimRows() const
{
	const std::scoped_lock lock(stateMutex);
	std::vector<ClaimRow> rows;
	for (const auto& entry : claims)
		rows.push_back({ entry.second.actorFormID, entry.second.physicsFile, entry.second.boneCount, entry.second.gpuRegistered, true, entry.first });
	for (const auto& entry : disabledClaims)
		rows.push_back(entry.second);
	return rows;
}

std::vector<std::string> SmpPhysicsFeature::GetDeclinedLines() const
{
	const std::scoped_lock lock(stateMutex);
	std::vector<std::string> lines;
	for (const auto& entry : declinedFiles) {
		if (lines.size() >= kMaxDeclinedRows)
			break;
		lines.push_back(entry.first + ": " + entry.second);
	}
	return lines;
}

std::vector<std::pair<std::string, std::string>> SmpPhysicsFeature::GetRepairReports() const
{
	std::vector<std::pair<std::string, std::string>> reports;
	for (const SmpPhysics::RepairLog::FileReport& report : repairLog.Take())
		reports.emplace_back(report.file, SmpPhysics::RepairLog::Format(report));
	return reports;
}

std::vector<SmpPhysics::Health> SmpPhysicsFeature::ItemHealths(const std::vector<std::vector<std::string>>& a_itemFiles)
{
	const auto key = [](const std::string& a_path) {
		std::string normal = Util::ToLowerAscii(a_path);
		std::ranges::replace(normal, '/', '\\');
		return normal;
	};
	std::set<std::string> repaired;
	for (const SmpPhysics::RepairLog::FileReport& report : repairLog.Take())
		repaired.insert(key(report.file));
	std::map<std::string, std::string> declined;
	{
		const std::scoped_lock lock(stateMutex);
		for (const auto& entry : declinedFiles)
			declined.emplace(key(entry.first), entry.second);
	}

	std::map<std::string, SmpPhysics::Health> byFile;
	std::vector<SmpPhysics::Health> result;
	result.reserve(a_itemFiles.size());
	for (const std::vector<std::string>& files : a_itemFiles) {
		SmpPhysics::Health worst = SmpPhysics::Health::None;
		for (const std::string& file : files) {
			const std::string fileKey = key(file);
			auto found = byFile.find(fileKey);
			if (found == byFile.end()) {
				const auto parsed = GetParsedRig(file);
				const auto reason = declined.find(fileKey);
				found = byFile.emplace(fileKey, SmpPhysics::ClassifyFile(parsed->status == SmpPhysics::ParseStatus::Ok, parsed->reason, repaired.contains(fileKey) || !parsed->warnings.empty(), reason != declined.end() ? std::string_view(reason->second) : std::string_view{})).first;
			}
			worst = SmpPhysics::Worse(worst, found->second);
		}
		result.push_back(worst);
	}
	return result;
}

std::string SmpPhysicsFeature::ItemReportText(RE::FormID a_armor)
{
	const auto* armor = RE::TESForm::LookupByID<RE::TESObjectARMO>(a_armor);
	const std::vector<std::string> files = NifProbe::XmlPaths(a_armor);
	if (!armor || files.empty())
		return {};
	static constexpr std::array<const char*, 4> kNames = { "no physics found", "works", "repaired and used", "broken" };
	const SmpPhysics::Health health = ItemHealths({ files }).front();
	const RE::TESFile* file = armor->GetFile(0);
	std::string text = std::format("{} [{:08X}] from {}\nPhysics: {}\n", armor->GetName(), a_armor, file ? file->GetFilename() : std::string_view("?"), kNames[static_cast<std::size_t>(health)]);
	const auto key = [](const std::string& a_path) {
		std::string normal = Util::ToLowerAscii(a_path);
		std::ranges::replace(normal, '/', '\\');
		return normal;
	};
	const std::vector<SmpPhysics::RepairLog::FileReport> reports = repairLog.Take();
	for (const std::string& path : files) {
		text += "File: " + path + '\n';
		for (const SmpPhysics::RepairLog::FileReport& report : reports) {
			if (key(report.file) == key(path))
				text += SmpPhysics::RepairLog::Format(report) + '\n';
		}
		const std::scoped_lock lock(stateMutex);
		for (const auto& declined : declinedFiles) {
			if (key(declined.first) == key(path))
				text += "Declined: " + declined.second + '\n';
		}
	}
	return text;
}

json SmpPhysicsFeature::IslandSizesJson() const
{
	const SmpPhysics::RigidGpu::IslandSizes& sizes = gpuSolver.Rigid().IslandSizeSummary();
	return { { "buckets", json::array({ "1", "2-4", "5-8", "9-16", "17-32", "33-64", "65+" }) }, { "islands", sizes.islands }, { "constraints", sizes.constraints },
		{ "total", sizes.total }, { "largest", sizes.largest }, { "mostColors", sizes.mostColors } };
}

json SmpPhysicsFeature::PhysicsHealthJson()
{
	static constexpr std::array<const char*, 4> kNames = { "none", "working", "repaired", "broken" };
	json rows = json::array();
	for (const auto& list : { TestBench::TestCrowd::ArmorForms(), TestBench::TestCrowd::CloakForms() }) {
		std::vector<std::vector<std::string>> files;
		for (const RE::TESObjectARMO* armor : list)
			files.push_back(NifProbe::XmlPaths(armor->GetFormID()));
		const std::vector<SmpPhysics::Health> health = ItemHealths(files);
		for (std::size_t index = 0; index < list.size(); ++index) {
			if (health[index] != SmpPhysics::Health::None)
				rows.push_back({ { "name", list[index]->GetName() }, { "formId", list[index]->GetFormID() }, { "health", kNames[static_cast<std::size_t>(health[index])] }, { "files", files[index] } });
		}
	}
	return { { "items", std::move(rows) } };
}

json SmpPhysicsFeature::SurfaceCheckJson() const
{
	const SmpPhysics::GpuSolver::SurfacePrepCheckResult check = gpuSolver.LastSurfacePrepCheck();
	gpuSolver.RequestSurfacePrepCheck();
	json result = { { "ran", check.ran },
		{ "triangles", check.triangles },
		{ "nodes", check.nodes },
		{ "maxTriangleError", check.maxTriangleError },
		{ "maxNormalError", check.maxNormalError },
		{ "triangleMismatches", check.triangleMismatches },
		{ "maxNodeError", check.maxNodeError },
		{ "nodeMismatches", check.nodeMismatches },
		{ "topologyMismatches", check.topologyMismatches },
		{ "gpuSurfacePrep", SurfacePrepJson(gpuSolver.SurfacePrep()) } };
	if (!check.ran)
		result["reason"] = check.reason;
	return result;
}

void SmpPhysicsFeature::ReevaluateActor(RE::Actor* a_actor)
{
	if (!a_actor)
		return;

	hdt::PluginInterface* smpInterface = nullptr;
	{
		const std::scoped_lock lock(stateMutex);
		smpInterface = linkedInterface;
	}
	// Called with stateMutex released: hdtSMP64 queues the reload under its own lock and every
	// mesh then asks the owners again, which re-enters claim() and takes stateMutex.
	if (smpInterface)
		smpInterface->reloadActorPhysics(a_actor);
}

void SmpPhysicsFeature::ReevaluatePlayer()
{
	ReevaluateActor(globals::game::player);
}

void SmpPhysicsFeature::ResetSettings()
{
	{
		const std::scoped_lock lock(stateMutex);
		settings = Settings{};
	}
	ReevaluateAllActors();
}

void SmpPhysicsFeature::ReevaluateAllActors()
{
	// The settings page calls this from the menu thread; hdtSMP64 must rebuild an actor's physics on the game thread.
	auto* task = SKSE::GetTaskInterface();
	if (!task)
		return;
	task->AddTask([this] {
		hdt::PluginInterface* smpInterface = nullptr;
		{
			const std::scoped_lock lock(stateMutex);
			smpInterface = linkedInterface;
		}
		if (!smpInterface)
			return;
		Util::ForEachLoadedActor([smpInterface](RE::Actor* a_actor) {
			if (a_actor)
				smpInterface->reloadActorPhysics(a_actor);
		});
	});
}

void SmpPhysicsFeature::ReevaluateClaimedActors()
{
	auto* task = SKSE::GetTaskInterface();
	if (!task)
		return;
	task->AddTask([this] {
		hdt::PluginInterface* smpInterface = nullptr;
		std::vector<RE::FormID> actorFormIDs;
		{
			const std::scoped_lock lock(stateMutex);
			smpInterface = linkedInterface;
			actorFormIDs.reserve(claims.size());
			for (const auto& entry : claims) {
				const RE::FormID formID = entry.second.actorFormID;
				if (formID != 0 && std::find(actorFormIDs.begin(), actorFormIDs.end(), formID) == actorFormIDs.end())
					actorFormIDs.push_back(formID);
			}
		}
		if (!smpInterface)
			return;
		for (const RE::FormID formID : actorFormIDs) {
			if (auto* actor = RE::TESForm::LookupByID<RE::Actor>(formID))
				smpInterface->reloadActorPhysics(actor);
		}
	});
}

void SmpPhysicsFeature::SetClaimEnabled(RE::FormID a_actorFormID, const std::string& a_file, bool a_enabled)
{
	hdt::PluginInterface* smpInterface = nullptr;
	{
		const std::scoped_lock lock(stateMutex);
		const auto key = std::make_pair(a_actorFormID, Util::ToLowerAscii(a_file));
		if (a_enabled) {
			disabledClaims.erase(key);
		} else {
			int boneCount = 0;
			for (const auto& entry : claims) {
				if (entry.second.actorFormID == a_actorFormID && Util::ToLowerAscii(entry.second.physicsFile) == key.second)
					boneCount = entry.second.boneCount;
			}
			disabledClaims.insert_or_assign(key, ClaimRow{ a_actorFormID, a_file, boneCount, false, false });
		}
		smpInterface = linkedInterface;
	}
	if (!smpInterface)
		return;
	if (auto* actor = RE::TESForm::LookupByID<RE::Actor>(a_actorFormID))
		smpInterface->reloadActorPhysics(actor);
}

bool SmpPhysicsFeature::MeshOwner::claim(const hdt::IMeshClaimContext& a_context, std::uint64_t a_token)
{
	pendingDeclineReason.clear();
	bool accepted = false;
	try {
		accepted = feature.ClaimMesh(a_context, a_token);
	} catch (const std::exception& e) {
		try {
			if (const char* path = a_context.getPhysicsFile(); path && *path)
				feature.RecordDecline(path, std::string("claiming threw: ") + e.what());
		} catch (...) {
		}
	} catch (...) {
	}
	try {
		feature.TallyClaim(a_context, accepted);
	} catch (...) {
	}
	return accepted;
}

void SmpPhysicsFeature::MeshOwner::release(std::uint64_t a_token)
{
	try {
		feature.ReleaseMesh(a_token);
	} catch (...) {
	}
}

RE::BSEventNotifyControl SmpPhysicsFeature::FrameListener::ProcessEvent(
	const hdt::PluginFrameEvent* a_event, RE::BSTEventSource<hdt::PluginFrameEvent>*)
{
	if (a_event)
		feature.gamePaused.store(a_event->gamePaused, std::memory_order_relaxed);
	return RE::BSEventNotifyControl::kContinue;
}

RE::BSEventNotifyControl SmpPhysicsFeature::FrameSyncListener::ProcessEvent(
	const hdt::PluginFrameSyncEvent*, RE::BSTEventSource<hdt::PluginFrameSyncEvent>*)
{
	GpuPassTimer::NextFrame();
	TracyGpu::NextFrame();
	try {
		feature.crowdWigs.Tick();
		feature.StepClaims();
		feature.RecordBoneFrame();
	} catch (const std::exception& error) {
		logger::error("{} the frame step failed: {}", kLogPrefix, error.what());
	}
	return RE::BSEventNotifyControl::kContinue;
}

bool SmpPhysicsFeature::PassesClaimGate(const hdt::IMeshClaimContext& a_context, const std::string& a_path)
{
	const char* declineReason = nullptr;
	{
		const std::scoped_lock lock(stateMutex);
		if (!linked.load(std::memory_order_relaxed) || !linkedInterface || !settings.Enabled)
			return false;
		if (a_context.getKind() == hdt::MeshKind::HeadPart && !settings.ClaimHeadParts)
			declineReason = "head part claiming is disabled";
		else if (a_context.getKind() == hdt::MeshKind::Armor && !settings.ClaimArmor)
			declineReason = "armor claiming is disabled";
		else if (std::count_if(claims.begin(), claims.end(), [](const auto& a_entry) { return !a_entry.second.inactive; }) >= settings.MaxClaims)
			declineReason = "the claim limit is reached";
	}
	if (declineReason) {
		RecordDecline(a_path, declineReason);
		return false;
	}
	return true;
}

/** @brief The rig without the bones the skeleton lacks and the joints that name them, as SMP skips both. */
static SmpPhysics::SmpRig DropMissingBones(const hdt::IMeshClaimContext& a_context, RE::NiNode* a_skeleton, const SmpPhysics::SmpRig& a_rig)
{
	std::vector<bool> missing(a_rig.bones.size(), false);
	for (std::size_t index = 0; index < a_rig.bones.size(); ++index) {
		const std::string& boneName = a_rig.bones[index].name;
		const char* mergedName = a_context.getMergedBoneName(boneName.c_str());
		missing[index] = a_skeleton->GetObjectByName(RE::BSFixedString(mergedName && *mergedName ? mergedName : boneName.c_str())) == nullptr;
	}
	return SmpPhysics::DropBones(a_rig, missing);
}

bool SmpPhysicsFeature::ResolveRigNodes(const hdt::IMeshClaimContext& a_context, RE::NiNode* a_skeleton, const SmpPhysics::SmpRig& a_rig,
	const std::string& a_path, std::vector<RE::NiPointer<RE::NiAVObject>>& a_nodes)
{
	a_nodes.assign(a_rig.bones.size(), nullptr);
	for (std::size_t index = 0; index < a_rig.bones.size(); ++index) {
		const std::string& boneName = a_rig.bones[index].name;
		const char* mergedName = a_context.getMergedBoneName(boneName.c_str());
		const char* lookupName = mergedName && *mergedName ? mergedName : boneName.c_str();
		RE::NiAVObject* node = a_skeleton->GetObjectByName(RE::BSFixedString(lookupName));
		if (!node) {
			RecordDecline(a_path, std::format("bone '{}' has no node below the skeleton", boneName));
			return false;
		}
		a_nodes[index].reset(node);
	}

	for (const SmpPhysics::RigJoint& joint : a_rig.joints) {
		if (joint.reversed)
			continue;
		const auto* parentNode = a_nodes[static_cast<std::size_t>(joint.parent)]->AsNode();
		const RE::NiAVObject* childNode = a_nodes[static_cast<std::size_t>(joint.child)].get();
		const RE::NiAVObject* sceneParent = SmpPhysics::NearestBoneAncestor(
			childNode, [](const RE::NiAVObject* a_node) -> const RE::NiAVObject* { return a_node->parent; },
			[&a_nodes](const RE::NiAVObject* a_node) { return std::any_of(a_nodes.begin(), a_nodes.end(), [a_node](const auto& a_bone) { return a_bone.get() == a_node; }); });
		if (!parentNode || sceneParent != parentNode) {
			const std::string parentName = a_rig.bones[static_cast<std::size_t>(joint.parent)].name;
			const std::string childName = a_rig.bones[static_cast<std::size_t>(joint.child)].name;
			RecordDecline(a_path, std::format("the nearest rig bone above '{}' is '{}', not the physics parent '{}'", childName,
									  sceneParent ? sceneParent->name.c_str() : "nothing", parentName));
			return false;
		}
		if (childNode->parent != parentNode) {
			repairLog.Record(a_path, std::format("'{}' hangs under '{}', a node hdtSMP64 added when it merged the armor, not directly under '{}' as the file says", a_rig.bones[static_cast<std::size_t>(joint.child)].name, childNode->parent ? childNode->parent->name.c_str() : "nothing", a_rig.bones[static_cast<std::size_t>(joint.parent)].name),
				"accepted it and simulated the bone from where the scene graph puts it");
		}
	}
	return true;
}

bool SmpPhysicsFeature::ResolveRestPose(const SmpPhysics::SmpRig& a_rig, const std::vector<RE::NiPointer<RE::NiAVObject>>& a_nodes,
	const std::string& a_path, std::vector<SmpPhysics::Transform>& a_restPose, std::size_t& a_dynamicCount, float& a_bodyScale)
{
	// A dynamic bone's rest pose has to come from local, which no simulation writes, because its
	// world transform is already overwritten by hdtSMP64 before the first frame this feature sees.
	const std::size_t boneCount = a_rig.bones.size();
	std::vector<RE::NiTransform> restTransforms(boneCount);
	std::vector<bool> resolved(boneCount, false);
	a_dynamicCount = 0;
	for (std::size_t index = 0; index < boneCount; ++index) {
		if (a_rig.bones[index].kinematic) {
			restTransforms[index] = a_nodes[index]->world;
			resolved[index] = true;
		} else {
			++a_dynamicCount;
		}
	}

	// The scene graph, not the joints, says where a bone rests: a joint NormalizeRigTopology turned around hangs a bone
	// from its physics child.
	std::unordered_map<const RE::NiAVObject*, std::size_t> boneOfNode;
	for (std::size_t index = 0; index < boneCount; ++index)
		boneOfNode.emplace(a_nodes[index].get(), index);
	std::size_t pending = a_dynamicCount;
	for (std::size_t pass = 0; pass <= boneCount && pending > 0; ++pass) {
		for (std::size_t index = 0; index < boneCount; ++index) {
			if (resolved[index])
				continue;
			const RE::NiAVObject* sceneParent = a_nodes[index]->parent;
			if (!sceneParent)
				continue;
			const auto parentBone = boneOfNode.find(sceneParent);
			if (parentBone != boneOfNode.end() && !resolved[parentBone->second])
				continue;
			const RE::NiTransform& parentWorld = parentBone != boneOfNode.end() ? restTransforms[parentBone->second] : sceneParent->world;
			restTransforms[index] = parentWorld * a_nodes[index]->local;
			resolved[index] = true;
			--pending;
		}
	}
	if (pending > 0) {
		RecordDecline(a_path, "a dynamic bone's rest pose does not resolve from the scene graph");
		return false;
	}

	a_restPose.resize(boneCount);
	a_bodyScale = 1.0f;
	bool scaleRead = false;
	for (std::size_t index = 0; index < boneCount; ++index) {
		a_restPose[index] = ToSolverTransform(restTransforms[index]);
		if (!scaleRead && !a_rig.bones[index].kinematic) {
			a_bodyScale = restTransforms[index].scale;
			scaleRead = true;
		}
	}
	return true;
}

void SmpPhysicsFeature::ReadViewRules(Claim& a_claim)
{
	std::ifstream configStream("Data\\SKSE\\Plugins\\hdtSkinnedMeshConfigs\\configs.json");
	if (!configStream)
		return;
	const json config = json::parse(configStream, nullptr, false, true);
	if (config.is_object() && config.contains("smp") && config["smp"].is_object()) {
		a_claim.hideUnderWig = config["smp"].value("disableSMPHairWhenWigEquipped", false);
		a_claim.firstPersonOff = config["smp"].value("disable1stPersonViewPhysics", false);
	}
}

void SmpPhysicsFeature::InstallClaimCollision(Claim& a_claim, ClaimCollision& a_collision)
{
	std::vector<SmpPhysics::Collider> solverColliders = a_collision.colliders;
	solverColliders.resize(solverColliders.size() + SmpPhysics::kExternalColliderSlots);
	a_claim.externalDriver = static_cast<int>(a_collision.drivers.size());
	a_collision.drivers.emplace_back();
	a_claim.driverWorld.assign(a_collision.drivers.size(), SmpPhysics::Transform{});
	a_claim.gpuCollision = SmpPhysics::GpuRig::PackCollision(a_collision.spheres, solverColliders,
		static_cast<std::uint32_t>(a_collision.drivers.size()), a_collision.boneSurfaceMask);
	a_claim.spheres = std::move(a_collision.spheres);
	a_claim.retargetBehind = a_collision.retargetBehind;
	a_claim.retargetSagMaxMove = a_collision.retargetSagMaxMove;
	a_claim.sagPose = std::move(a_collision.sagPose);
	a_claim.retargetResidual = a_collision.retargetResidual;
	a_claim.retargetedJoints = a_collision.retargetedJoints;
	a_claim.retargetKeptAuthored = a_collision.retargetKeptAuthored;
	a_claim.retargetMaxAngle = a_collision.retargetMaxAngle;
	a_claim.retargetMaxBoneMove = a_collision.retargetMaxBoneMove;
	a_claim.colliders = std::move(a_collision.colliders);
	a_claim.colliderFits = std::move(a_collision.colliderFits);
	a_claim.surfaces = std::move(a_collision.surfaces);
	a_claim.collisionDrivers = std::move(a_collision.drivers);
	a_claim.hairMeshNames = std::move(a_collision.hairMeshNames);
	a_claim.bodySignature = a_collision.bodySignature;
	a_claim.collisionActive = true;
}

bool SmpPhysicsFeature::ClaimMesh(const hdt::IMeshClaimContext& a_context, std::uint64_t a_token)
{
	ZoneScopedN("SmpPhysics::ClaimMesh");
	const char* rawPath = a_context.getPhysicsFile();
	const std::string path = rawPath && *rawPath ? std::string(rawPath) : std::string();
	if (path.empty()) {
		RecordDecline("<no physics file>", "the mesh has no physics file");
		return false;
	}
	if (const RE::Actor* claimActor = a_context.getActor()) {
		const std::scoped_lock lock(stateMutex);
		if (disabledClaims.contains({ claimActor->GetFormID(), Util::ToLowerAscii(path) }))
			return false;
	}
	if (!PassesClaimGate(a_context, path))
		return false;

	RE::NiNode* skeleton = a_context.getSkeleton();
	if (!skeleton) {
		RecordDecline(path, "the mesh has no skeleton node");
		return false;
	}

	const auto parsed = GetParsedRig(path);
	if (!parsed || parsed->status != SmpPhysics::ParseStatus::Ok) {
		RecordDecline(path, parsed && !parsed->reason.empty() ? parsed->reason : std::string("the physics file is not supported"));
		return false;
	}

	SmpPhysics::SmpRig rig = DropMissingBones(a_context, skeleton, parsed->rig);
	std::string topologyReason;
	if (!SmpPhysics::NormalizeRigTopology(rig, topologyReason)) {
		RecordDecline(path, topologyReason);
		return false;
	}
	const auto isKinematic = [](const SmpPhysics::RigBone& a_bone) { return a_bone.kinematic; };
	if (std::all_of(rig.bones.begin(), rig.bones.end(), isKinematic) && !std::all_of(parsed->rig.bones.begin(), parsed->rig.bones.end(), isKinematic)) {
		RecordDecline(path, "no dynamic bone is left once the bones the skeleton lacks and the groups no joint holds are dropped");
		return false;
	}
	const std::size_t boneCount = rig.bones.size();

	std::vector<RE::NiPointer<RE::NiAVObject>> nodes;
	std::vector<SmpPhysics::Transform> restPose;
	std::size_t dynamicCount = 0;
	float bodyScale = 1.0f;
	if (!ResolveRigNodes(a_context, skeleton, rig, path, nodes) || !ResolveRestPose(rig, nodes, path, restPose, dynamicCount, bodyScale))
		return false;

	const bool hasCollisionShapes = !rig.collision.vertexShapes.empty() || !rig.collision.triangleShapes.empty();
	if (hasCollisionShapes && !settings.CollisionEnabled) {
		RecordDecline(path, "collision is switched off in the settings");
		return false;
	}

	ClaimCollision collision;
	if (hasCollisionShapes) {
		std::string collisionReason;
		if (!BuildClaimCollision(a_context, rig, nodes, restPose, collision, collisionReason)) {
			RegisterBodyShapes(a_context, path, collision.bodyShapes);
			RecordDecline(path, collisionReason);
			return false;
		}
	}

	SmpPhysics::SolverParams params;
	params.gravityScale = settings.GravityScale;
	params.stiffnessScale = settings.StiffnessScale;
	params.dampingScale = settings.DampingScale;
	params.bodyScale = bodyScale;

	if (!gpuSolver.Available()) {
		RecordDecline(path, "the GPU solver is not available: " + gpuSolver.UnavailableReason());
		return false;
	}
	if (!gpuSolver.Rigid().Available()) {
		RecordDecline(path, "the rigid-body solver is not available");
		return false;
	}
	std::string rigReason;
	if (!SmpPhysics::ValidateRig(rig, rigReason) || restPose.size() != boneCount ||
		!std::all_of(restPose.begin(), restPose.end(), [](const SmpPhysics::Transform& a_pose) { return SmpPhysics::IsFiniteTransform(a_pose); })) {
		RecordDecline(path, "the rest pose is not usable by the solver");
		return false;
	}

	Claim claim;
	claim.solverParams = SmpPhysics::SanitizeParams(params);

	for (std::size_t index = 0; index < boneCount; ++index) {
		if (rig.bones[index].kinematic) {
			claim.firstKinematicBone = static_cast<int>(index);
			break;
		}
	}

	std::vector<RE::NiPointer<RE::NiSkinInstance>> skins;
	std::vector<RE::NiPointer<RE::NiAVObject>> cardGeometries;
	CollectClaimedSkins(a_context.getModel(), nodes, skins, cardGeometries);

	auto* actor = a_context.getActor();
	claim.token = a_token;
	claim.headPart = a_context.getKind() == hdt::MeshKind::HeadPart;
	claim.cloth = IsClothClaim(a_context);
	ReadViewRules(claim);
	claim.actorFormID = actor ? actor->GetFormID() : 0;
	claim.physicsFile = path;
	const bool rigChanged = rig.bones.size() != parsed->rig.bones.size() || rig.joints.size() != parsed->rig.joints.size() ||
	                        rig.additionalJoints.size() != parsed->rig.additionalJoints.size() ||
	                        std::any_of(rig.joints.begin(), rig.joints.end(), [](const SmpPhysics::RigJoint& a_joint) { return a_joint.reversed; });
	if (rigChanged) {
		auto claimRig = std::make_shared<SmpPhysics::ParseResult>(*parsed);
		claimRig->rig = rig;
		claim.parsed = std::move(claimRig);
	} else {
		claim.parsed = parsed;
	}
	claim.nodes = std::move(nodes);
	claim.skins = std::move(skins);
	claim.cardGeometries = std::move(cardGeometries);
	claim.kinematicInput.assign(boneCount, SmpPhysics::Transform{});
	claim.previousKinematic = restPose;
	claim.restPose = std::move(restPose);
	claim.treeKinematicParent = SmpPhysics::GpuRig::BuildTreeKinematicParents(rig);
	claim.boneReach = SmpPhysics::GpuRig::BuildBoneReach(rig, claim.restPose, kBoneReachSlack, kBoneReachMargin, bodyScale);
	claim.boneCount = static_cast<int>(boneCount);
	claim.dynamicBoneCount = static_cast<int>(dynamicCount);
	claim.additionalJointCount = static_cast<int>(rig.additionalJoints.size());
	claim.sphereCount = collision.sphereCount;
	claim.exemptSphereCount = collision.exemptSpheres;
	claim.colliderCount = static_cast<int>(collision.colliders.size());
	if (hasCollisionShapes)
		InstallClaimCollision(claim, collision);

	const int additionalJoints = claim.additionalJointCount;
	const int sphereCount = claim.sphereCount;
	const int exemptSpheres = claim.exemptSphereCount;
	const int colliderCount = claim.colliderCount;
	{
		const std::scoped_lock lock(stateMutex);
		// A token hdtSMP64 reuses must not leave the GPU backend holding the previous rig for it.
		gpuSolver.RemoveClaim(a_token);
		gpuSkinning.RemoveClaim(a_token);
		if (claim.cloth)
			RegisterClothMeshes(claim.actorFormID, claim.cardGeometries);
		claims.insert_or_assign(a_token, std::move(claim));
	}
	logger::info("{} Claimed '{}' for token {} ({} bones, {} dynamic, {} joints dropped, {} collision spheres, {} of {} start inside a collider and are exempt, {} colliders, {} fitted, {} defaulted)",
		kLogPrefix, path, a_token, boneCount, dynamicCount, additionalJoints, sphereCount,
		exemptSpheres, sphereCount, colliderCount, collision.fitted, collision.defaulted);
	if (collision.retargetBehind > 0) {
		logger::info("{} '{}': {} spheres settle inside a worn surface (gravity sags the rest up to {:.1f} units); {} joints turned up to {:.2f} rad, moving a bone at most {:.1f} units, {} spheres left inside{}",
			kLogPrefix, path, collision.retargetBehind, collision.retargetSagMaxMove, collision.retargetedJoints, collision.retargetMaxAngle, collision.retargetMaxBoneMove, collision.retargetResidual,
			collision.retargetKeptAuthored ? "; too few could be freed, so the authored rest is kept" : "");
	}
	return true;
}

std::shared_ptr<const SmpPhysics::ParseResult> SmpPhysicsFeature::GetParsedRig(const std::string& a_path)
{
	{
		const std::scoped_lock lock(stateMutex);
		if (const auto found = parsedRigs.find(a_path); found != parsedRigs.end())
			return found->second;
	}

	auto parsed = std::make_shared<SmpPhysics::ParseResult>(ReadPhysicsFile(a_path));
	for (const std::string& warning : parsed->warnings) {
		logger::warn("{} '{}': {}", kLogPrefix, a_path, warning);
		repairLog.Record(a_path, warning, "repaired it and used the file");
	}

	const std::scoped_lock lock(stateMutex);
	const auto entry = parsedRigs.try_emplace(a_path, std::move(parsed));
	return entry.first->second;
}

void SmpPhysicsFeature::RegisterBodyShapes(const hdt::IMeshClaimContext& a_context, const std::string& a_file,
	const std::vector<SmpPhysics::RigShape>& a_shapes)
{
	const RE::Actor* actor = a_context.getActor();
	if (!actor || a_shapes.empty())
		return;
	const RE::FormID formID = actor->GetFormID();
	const std::scoped_lock lock(stateMutex);
	std::vector<BodyShape>& known = bodyShapes[formID];
	bool added = false;
	for (const SmpPhysics::RigShape& shape : a_shapes) {
		const bool present = std::any_of(known.begin(), known.end(),
			[&](const BodyShape& body) { return body.file == a_file && body.shape.name == shape.name; });
		if (!present) {
			known.push_back({ a_file, shape });
			added = true;
		}
	}
	if (added && std::find(bodyShapeActors.begin(), bodyShapeActors.end(), formID) == bodyShapeActors.end())
		bodyShapeActors.push_back(formID);
}

void SmpPhysicsFeature::TallyClaim(const hdt::IMeshClaimContext& a_context, bool a_accepted)
{
	const char* path = a_context.getPhysicsFile();
	const RE::Actor* actor = a_context.getActor();
	const auto kind = a_context.getKind() == hdt::MeshKind::HeadPart ? SmpPhysics::ClaimTally::Kind::HeadPart : SmpPhysics::ClaimTally::Kind::Armor;
	const std::string_view reason = pendingDeclineReason.empty() ? std::string_view("declined without a reason (switched off for this actor, or refused by the claim gate)") : std::string_view(pendingDeclineReason);
	claimTally.Record(kind, actor ? actor->GetFormID() : 0, path && *path ? path : "<no physics file>", a_accepted, a_accepted ? std::string_view{} : reason);
}

json SmpPhysicsFeature::ClaimTallyJson() const
{
	const SmpPhysics::ClaimTally::Snapshot snapshot = claimTally.Take();
	json result = { { "armor", { { "attempts", snapshot.kinds[0].attempts }, { "accepts", snapshot.kinds[0].accepts }, { "declines", snapshot.kinds[0].declines } } },
		{ "headPart", { { "attempts", snapshot.kinds[1].attempts }, { "accepts", snapshot.kinds[1].accepts }, { "declines", snapshot.kinds[1].declines } } },
		{ "pairsAccepted", snapshot.pairsAccepted }, { "pairsDeclined", snapshot.pairsDeclined }, { "pairsTruncated", snapshot.pairsTruncated } };
	result["reasons"] = json::array();
	for (const auto& row : snapshot.reasons)
		result["reasons"].push_back({ { "reason", row.reason }, { "example", row.example }, { "declines", row.declines }, { "pairs", row.pairs }, { "armor", row.byKind[0] }, { "headPart", row.byKind[1] }, { "sampleFiles", row.sampleFiles } });
	result["files"] = json::array();
	for (const auto& row : snapshot.files)
		result["files"].push_back({ { "file", row.file }, { "attempts", row.attempts }, { "accepts", row.accepts }, { "declines", row.declines } });
	return result;
}

void SmpPhysicsFeature::SetCollisionOverride(const std::string& a_piece, const std::string& a_target, bool a_off)
{
	const std::scoped_lock lock(stateMutex);
	if (a_piece.empty() && a_target.empty()) {
		collisionOverrides.clear();
		return;
	}
	std::erase_if(collisionOverrides, [&](const SmpPhysics::CollisionOverride& a_override) { return a_override.piece == a_piece && a_override.target == a_target; });
	if (a_off)
		collisionOverrides.push_back({ a_piece, a_target });
}

void SmpPhysicsFeature::ResetClaimTally()
{
	claimTally.Reset();
}

void SmpPhysicsFeature::RecordDecline(const std::string& a_file, const std::string& a_reason)
{
	pendingDeclineReason = a_reason;
	bool shouldLog = false;
	{
		const std::scoped_lock lock(stateMutex);
		const auto existing = declinedFiles.find(a_file);
		if (existing != declinedFiles.end()) {
			if (existing->second != a_reason) {
				existing->second = a_reason;
				shouldLog = true;
			}
		} else if (declinedFiles.size() < kMaxDeclinedFiles) {
			declinedFiles.emplace(a_file, a_reason);
			shouldLog = true;
		}
	}
	if (shouldLog)
		logger::info("{} Declined '{}': {}", kLogPrefix, a_file, a_reason);
	if (SmpPhysics::RepairLog::IsFileFault(a_reason))
		repairLog.Record(a_file, a_reason, "declined the file, so hdtSMP64 simulates it");
}

void SmpPhysicsFeature::ReleaseMesh(std::uint64_t a_token)
{
	const std::scoped_lock lock(stateMutex);
	gpuSolver.RemoveClaim(a_token);
	gpuSkinning.RemoveClaim(a_token);
	claims.erase(a_token);
}

json SmpPhysicsFeature::QueueWindProbe(const json& a_args)
{
	WindProbeRequest request;
	request.actor = a_args.value("actorFormID", 0u);
	if (a_args.contains("elevations") && a_args["elevations"].is_array()) {
		for (const auto& value : a_args["elevations"]) {
			if (value.is_number() && request.elevations.size() < kMaxProbeElevations)
				request.elevations.push_back(Util::ClampFinite(value.get<float>(), -kMaxProbeElevation, kMaxProbeElevation, 0.0f));
		}
	}
	if (request.elevations.empty())
		request.elevations.assign(kDefaultProbeElevations.begin(), kDefaultProbeElevations.end());
	if (a_args.contains("azimuth") && a_args["azimuth"].is_number())
		request.azimuthDegrees = Util::ClampFinite(a_args["azimuth"].get<float>(), -kMaxProbeAzimuth, kMaxProbeAzimuth, 0.0f);
	request.reach = std::clamp(a_args.value("reach", kDefaultProbeReach), kMinProbeReach, kMaxProbeReach);
	const std::scoped_lock lock(windProbeMutex);
	windProbeRequest = std::move(request);
	windProbeResult = { { "pending", true } };
	return windProbeResult;
}

json SmpPhysicsFeature::WindProbeResult() const
{
	const std::scoped_lock lock(windProbeMutex);
	return windProbeResult;
}

void SmpPhysicsFeature::RunWindProbe()
{
	std::optional<WindProbeRequest> request;
	{
		const std::scoped_lock lock(windProbeMutex);
		request.swap(windProbeRequest);
	}
	if (!request)
		return;
	json result = { { "pending", false } };
	auto* actor = RE::TESForm::LookupByID<RE::Actor>(request->actor);
	if (!actor || !actor->Is3DLoaded()) {
		result["error"] = "the actor is not loaded";
	} else {
		const RE::NiPoint3 origin = HeadOrigin(*actor);
		float azimuth = request->azimuthDegrees;
		if (!std::isfinite(azimuth)) {
			SmpPhysics::Vec3 wind{};
			{
				const std::scoped_lock lock(stateMutex);
				for (const auto& entry : claims) {
					if (entry.second.actorFormID == request->actor) {
						wind = entry.second.wind;
						break;
					}
				}
			}
			azimuth = SmpPhysics::Length(wind) > 0.0f ? std::atan2(-wind.y, -wind.x) * kDegreesPerRadian : 0.0f;
		}
		const float heading = azimuth / kDegreesPerRadian;
		result["origin"] = { origin.x, origin.y, origin.z };
		result["azimuth"] = azimuth;
		result["rays"] = json::array();
		for (const float elevationDegrees : request->elevations) {
			const float elevation = elevationDegrees / kDegreesPerRadian;
			const RE::NiPoint3 direction{ std::cos(elevation) * std::cos(heading), std::cos(elevation) * std::sin(heading), std::sin(elevation) };
			json ray = { { "elevation", elevationDegrees }, { "distance", -1.0f }, { "reference", 0 } };
			if (const auto hit = Util::FirstOccluder(*actor, origin, origin + direction * request->reach)) {
				ray["distance"] = hit->distance;
				ray["reference"] = hit->reference;
				if (auto* reference = hit->reference ? RE::TESForm::LookupByID<RE::TESObjectREFR>(hit->reference) : nullptr) {
					ray["formType"] = static_cast<int>(reference->GetFormType());
					ray["name"] = reference->GetDisplayFullName();
					ray["baseFormId"] = reference->GetBaseObject() ? reference->GetBaseObject()->GetFormID() : 0u;
				}
			}
			result["rays"].push_back(std::move(ray));
		}
	}
	const std::scoped_lock lock(windProbeMutex);
	windProbeResult = std::move(result);
}

void SmpPhysicsFeature::UpdateWind(float a_frameDelta)
{
	windClock = std::fmod(windClock + a_frameDelta, SmpPhysics::kWindClockWrap);
	windCooldown -= a_frameDelta;
	if (windCooldown <= 0.0f) {
		windCooldown = kWindTick;
		const auto* player = globals::game::player;
		const auto* cell = player ? player->GetParentCell() : nullptr;
		const auto* worldSpace = cell ? cell->GetRuntimeData().worldSpace : nullptr;
		windOutdoors = worldSpace != nullptr &&
		               std::find(kNotExteriorWorlds.begin(), kNotExteriorWorlds.end(), worldSpace->formID) == kNotExteriorWorlds.end();
	}

	std::vector<float3> positions;
	std::vector<Claim*> owners;
	for (auto& entry : claims) {
		Claim& claim = entry.second;
		claim.wind = {};
		if (!windOutdoors || !(settings.WindStrength > 0.0f) || claim.firstKinematicBone < 0 || claim.nodes.size() <= static_cast<std::size_t>(claim.firstKinematicBone))
			continue;
		const auto* node = claim.nodes[static_cast<std::size_t>(claim.firstKinematicBone)].get();
		if (!node)
			continue;
		positions.push_back({ node->world.translate.x, node->world.translate.y, node->world.translate.z });
		owners.push_back(&claim);
	}
	if (positions.empty())
		return;
	std::vector<Wind::PublishedWindSample> samples(positions.size());
	if (!globals::features::wind.SamplePublishedWind(positions, samples))
		return;
	const float scale = settings.WindStrength * SmpPhysics::kWindSpeedPerStrength;
	for (std::size_t index = 0; index < owners.size(); ++index) {
		const float3& field = samples[index].windExcludingHavokImpulses;
		SmpPhysics::Vec3 wind{ field.x * scale, field.y * scale, field.z * scale };
		const float magnitude = SmpPhysics::Length(wind);
		if (!std::isfinite(magnitude))
			continue;
		owners[index]->wind = magnitude > SmpPhysics::kMaxWindSpeed ? wind * (SmpPhysics::kMaxWindSpeed / magnitude) : wind;
	}
}

void SmpPhysicsFeature::UpdateClaimWindExposure(Claim& a_claim, float a_frameDelta)
{
	if (a_claim.steppedFrames <= 1)
		a_claim.windProbeTimer = kWindTick * static_cast<float>(a_claim.token % kWindProbeSlots) / static_cast<float>(kWindProbeSlots);
	a_claim.windProbeTimer -= a_frameDelta;
	if (a_claim.windProbeTimer <= 0.0f) {
		a_claim.windProbeTimer = kWindTick;
		a_claim.windWorldExposure = 1.0f;
		a_claim.windSelfExposure = 1.0f;
		const float magnitude = SmpPhysics::Length(a_claim.wind);
		auto* actor = a_claim.actorFormID != 0 ? RE::TESForm::LookupByID<RE::Actor>(a_claim.actorFormID) : nullptr;
		if (actor && magnitude > 0.0f) {
			const RE::NiPoint3 origin = HeadOrigin(*actor);
			const RE::NiPoint3 towardSource{ -a_claim.wind.x / magnitude, -a_claim.wind.y / magnitude, -a_claim.wind.z / magnitude };
			const WindShelter::Estimate shelter = WindShelter::EstimateShelter(*actor, origin, RE::NiPoint3{ a_claim.wind.x, a_claim.wind.y, a_claim.wind.z });
			a_claim.windWorldExposure = shelter.exposure;
			a_claim.windRayHits = shelter.hitDistance;
			a_claim.windHitDistance = shelter.blockerDistance;
			a_claim.windHitReference = shelter.blockerReference;

			std::vector<SmpPhysics::Vec3> samples;
			const std::size_t stride = std::max<std::size_t>(1, a_claim.nodes.size() / kWindShelterSamples);
			for (std::size_t index = 0; index < a_claim.nodes.size(); index += stride) {
				const auto* node = a_claim.nodes[index].get();
				if (node && !a_claim.parsed->rig.bones[index].kinematic)
					samples.push_back({ node->world.translate.x, node->world.translate.y, node->world.translate.z });
			}
			const SmpPhysics::Vec3 toward{ towardSource.x, towardSource.y, towardSource.z };
			a_claim.windSelfExposure = SmpPhysics::SelfWindExposure(a_claim.colliders, a_claim.surfaces, a_claim.driverWorld, samples, toward, SmpPhysics::kSelfShelterReach);
		}
		a_claim.windTarget = a_claim.windWorldExposure * a_claim.windSelfExposure;
	}
	const float next = a_claim.windFactor + (a_claim.windTarget - a_claim.windFactor) * kWindExposureEasing;
	a_claim.windFactor = std::abs(a_claim.windTarget - next) < kWindExposureSnap ? a_claim.windTarget : next;
}

void SmpPhysicsFeature::ApplyCulling()
{
	const RE::NiPoint3 eye = Util::GetAverageEyePosition();
	constexpr float kKeepBias = 1.1f;  // A claim already inside the budget keeps its place against one about as near, so two never trade places every frame.
	struct Ranked
	{
		Claim* claim;
		float key;
	};
	std::vector<Ranked> live;
	live.reserve(claims.size());
	for (auto& entry : claims) {
		Claim& claim = entry.second;
		const bool wasCulled = claim.culledByBudget;
		claim.culledByBudget = false;
		if (claim.inactive || claim.firstKinematicBone < 0)
			continue;
		const auto* node = claim.nodes[static_cast<std::size_t>(claim.firstKinematicBone)].get();
		if (!node)
			continue;
		claim.lastCameraDistance = node->world.translate.GetDistance(eye);
		if (settings.CullOffscreen && claim.lastCameraDistance > kOffscreenKeepDistance &&
			!Util::SphereInViewFrustum(node->world.translate, claim.culledOffscreen ? kOffscreenResumeMargin : kOffscreenMargin))
			++claim.offscreenFrames;
		else
			claim.offscreenFrames = 0;
		claim.culledOffscreen = claim.offscreenFrames >= kOffscreenCullFrames;
		if (settings.MinScreenSizePercent > 0.0f && claim.lastCameraDistance > kOffscreenKeepDistance &&
			Util::SphereScreenHeightPercent(node->world.translate, kHairScreenRadius) <
				settings.MinScreenSizePercent * (claim.culledSmall ? kScreenSizeResumeFactor : 1.0f))
			++claim.smallFrames;
		else
			claim.smallFrames = 0;
		claim.culledSmall = claim.smallFrames >= kOffscreenCullFrames;
		if (claim.culledOffscreen || claim.culledSmall)
			continue;
		live.push_back({ &claim, claim.lastCameraDistance * (wasCulled ? kKeepBias : 1.0f) });
	}
	const auto budget = static_cast<std::size_t>(settings.MaxSimulatedClaims);
	if (live.size() <= budget)
		return;
	std::sort(live.begin(), live.end(), [](const Ranked& a_left, const Ranked& a_right) { return a_left.key < a_right.key; });
	for (std::size_t index = budget; index < live.size(); ++index)
		live[index].claim->culledByBudget = true;
}

void SmpPhysicsFeature::SetBodyContactProbe(const json& a_args)
{
	using Probe = BodyContactProbe;
	const std::scoped_lock lock(stateMutex);
	bodyContactProbe.active = a_args.value("enabled", true);
	bodyContactProbe.actorFormID = a_args.value("actorFormID", 0u);
	bodyContactProbe.length = Util::ClampFinite(a_args.value("length", Probe::kDefaultLength), 0.0f, Probe::kMaxExtent, Probe::kDefaultLength);
	bodyContactProbe.radius = Util::ClampFinite(a_args.value("radius", Probe::kDefaultRadius), 0.1f, Probe::kMaxExtent, Probe::kDefaultRadius);
	bodyContactProbe.offset = {};
	if (a_args.contains("offset") && a_args["offset"].is_array() && a_args["offset"].size() == 3) {
		float offset[3]{};
		for (std::size_t axis = 0; axis < 3; ++axis) {
			const json& value = a_args["offset"][axis];
			offset[axis] = value.is_number() ? Util::ClampFinite(value.get<float>(), -Probe::kMaxExtent, Probe::kMaxExtent, 0.0f) : 0.0f;
		}
		bodyContactProbe.offset = { offset[0], offset[1], offset[2] };
	}
}

/** @brief A sphere around every bone of a claim: the centre of its nodes and how far the farthest one lies from it, plus a margin. */
struct HairBound
{
	RE::NiPoint3 center;
	float radius = 0.0f;
};

SmpPhysics::Transform SmpPhysicsFeature::Claim::BoneWorld(std::size_t a_bone) const
{
	if (a_bone >= nodes.size() || !nodes[a_bone] || !parsed)
		return {};
	if (!parsed->rig.bones[a_bone].kinematic && a_bone < treeKinematicParent.size()) {
		const int root = treeKinematicParent[a_bone];
		if (gpuRegistered && gpuLastBones.size() == nodes.size() && root >= 0 && static_cast<std::size_t>(root) < nodes.size() && nodes[static_cast<std::size_t>(root)])
			return ToSolverTransform(nodes[static_cast<std::size_t>(root)]->world) * gpuLastBones[a_bone];
	}
	return ToSolverTransform(nodes[a_bone]->world);
}

std::vector<std::optional<SmpPhysics::Vec3>> SmpPhysicsFeature::Claim::BonePositions() const
{
	std::vector<std::optional<SmpPhysics::Vec3>> positions(nodes.size());
	for (std::size_t bone = 0; bone < nodes.size(); ++bone) {
		if (nodes[bone])
			positions[bone] = BoneWorld(bone).translation;
	}
	return positions;
}

static HairBound MeasureHairBound(const std::vector<RE::NiPointer<RE::NiAVObject>>& a_nodes, const RE::NiPoint3& a_fallbackCenter)
{
	RE::NiPoint3 sum{};
	std::size_t count = 0;
	for (const auto& node : a_nodes) {
		if (!node)
			continue;
		sum += node->world.translate;
		++count;
	}
	if (count == 0)
		return { a_fallbackCenter, kHairScreenRadius };
	const RE::NiPoint3 center = sum / static_cast<float>(count);
	float farthest = 0.0f;
	for (const auto& node : a_nodes) {
		if (node)
			farthest = std::max(farthest, (node->world.translate - center).Length());
	}
	return { center, std::min(farthest + kHairBoundMargin, kMaxHairBoundRadius) };
}

/** @brief The stretch of hair a bone carries: from its origin to its first dynamic child's, or past its origin by half the last segment for a tip. */
static bool StrandSegmentOf(const std::vector<std::optional<SmpPhysics::Vec3>>& a_positions, const SmpPhysics::SmpRig& a_rig, std::size_t a_bone,
	SmpPhysics::Vec3& a_start, SmpPhysics::Vec3& a_end)
{
	const auto position = [&a_positions](std::size_t a_index, SmpPhysics::Vec3& a_out) {
		if (a_index >= a_positions.size() || !a_positions[a_index])
			return false;
		a_out = *a_positions[a_index];
		return true;
	};
	if (!position(a_bone, a_start))
		return false;
	for (const SmpPhysics::RigJoint& joint : a_rig.joints) {
		if (joint.parent == static_cast<int>(a_bone) && joint.child >= 0 && !a_rig.bones[static_cast<std::size_t>(joint.child)].kinematic)
			return position(static_cast<std::size_t>(joint.child), a_end);
	}
	for (const SmpPhysics::RigJoint& joint : a_rig.joints) {
		SmpPhysics::Vec3 parent;
		if (joint.child == static_cast<int>(a_bone) && joint.parent >= 0 && position(static_cast<std::size_t>(joint.parent), parent)) {
			a_end = a_start + (a_start - parent) * kTipSegmentExtension;
			return true;
		}
	}
	return false;
}

/** @brief The position a strand hangs from and how long it is from there to the held bone and point at rest; false when the rig does not say. */
static bool StrandReachOf(const SmpPhysics::SmpRig& a_rig, const std::vector<SmpPhysics::Transform>& a_restPose, std::size_t a_bone,
	const SmpPhysics::Vec3& a_heldLocal, std::size_t& a_rootBone, float& a_length)
{
	if (a_bone >= a_rig.bones.size() || a_bone >= a_restPose.size())
		return false;
	std::vector<int> parentOf(a_rig.bones.size(), -1);
	for (const SmpPhysics::RigJoint& joint : a_rig.joints) {
		if (joint.child >= 0 && static_cast<std::size_t>(joint.child) < parentOf.size())
			parentOf[static_cast<std::size_t>(joint.child)] = joint.parent;
	}
	float length = SmpPhysics::Length(a_heldLocal);
	std::size_t bone = a_bone;
	for (std::size_t guard = 0; guard < a_rig.bones.size(); ++guard) {
		const int parent = parentOf[bone];
		if (parent < 0 || static_cast<std::size_t>(parent) >= a_restPose.size())
			return false;
		length += SmpPhysics::Length(a_restPose[bone].translation - a_restPose[static_cast<std::size_t>(parent)].translation);
		if (a_rig.bones[static_cast<std::size_t>(parent)].kinematic) {
			a_rootBone = static_cast<std::size_t>(parent);
			a_length = length;
			return true;
		}
		bone = static_cast<std::size_t>(parent);
	}
	return false;
}

/** @brief The first dynamic bone of the chain a bone belongs to, so every stretch of one hair card counts as one strand. */
static std::uint32_t StrandGroupOf(const SmpPhysics::SmpRig& a_rig, const std::vector<int>& a_parentOf, std::size_t a_bone)
{
	std::size_t bone = a_bone;
	for (std::size_t guard = 0; guard < a_rig.bones.size(); ++guard) {
		const int parent = a_parentOf[bone];
		if (parent < 0 || a_rig.bones[static_cast<std::size_t>(parent)].kinematic)
			break;
		bone = static_cast<std::size_t>(parent);
	}
	return static_cast<std::uint32_t>(bone);
}

/** @brief The sphere cast HIGGS makes from a hand's palm to pick what it will grab: where it starts and which way the palm faces. */
struct PalmCastFrame
{
	SmpPhysics::Vec3 origin;
	SmpPhysics::Vec3 normal;
	SmpPhysics::Vec3 fingers;  ///< Unit vector along the open fingers.
};

static PalmCastFrame PalmCastOf(const RE::NiAVObject& a_hand, bool a_left)
{
	SmpPhysics::Vec3 vector = SmpPhysics::kHiggsPalmVector;
	SmpPhysics::Vec3 offset = SmpPhysics::kHiggsPalmPosition;
	if (a_left) {
		vector.x = -vector.x;
		offset.x = -offset.x;
	}
	const RE::NiMatrix3& rotation = a_hand.world.rotate;
	const RE::NiPoint3 normal = rotation * RE::NiPoint3{ vector.x, vector.y, vector.z };
	const RE::NiPoint3 origin = a_hand.world.translate + rotation * (RE::NiPoint3{ offset.x, offset.y, offset.z } * a_hand.world.scale);
	const RE::NiPoint3 fingers = rotation * RE::NiPoint3{ 0.0f, 0.0f, 1.0f };
	return { { origin.x, origin.y, origin.z }, SmpPhysics::Normalized({ normal.x, normal.y, normal.z }), SmpPhysics::Normalized({ fingers.x, fingers.y, fingers.z }) };
}

void SmpPhysicsFeature::ReleaseGrabState()
{
	auto* higgs = Util::Higgs::GetInterface();
	for (std::size_t side = 0; side < heldPins.size(); ++side) {
		heldPins[side].clear();
		if (higgs && higgsHandDisabled[side])
			higgs->EnableHand(side == 0);
		higgsHandDisabled[side] = false;
	}
	SmpPhysics::ReadyWeaponGate::SetHeldHands(0);
}

/** @brief A controller pulse on a grab, then a gentle one now and then while a held strand is pulled taut; a_side is 0 for the left hand. */
static std::atomic<std::uint32_t> grabHapticPulses{ 0 };

static void PulseGrabHaptics(std::size_t a_side, bool a_justGrabbed, float a_tension)
{
	static std::array<std::chrono::steady_clock::time_point, 2> lastPulse{};
	auto* openvr = RE::BSOpenVR::GetSingleton();
	if (!openvr)
		return;
	const auto now = std::chrono::steady_clock::now();
	const float duration = SmpPhysics::HapticPulseDuration(a_justGrabbed, a_tension, std::chrono::duration<float>(now - lastPulse[a_side]).count());
	if (duration <= 0.0f)
		return;
	lastPulse[a_side] = now;
	grabHapticPulses.fetch_add(1, std::memory_order_relaxed);
	openvr->TriggerHapticPulse(a_side != 0, duration);
}

void SmpPhysicsFeature::UpdateStrandGrabs(float a_frameDelta)
{
	debugReachSegments.clear();
	debugGrabBones.clear();
	for (auto& entry : claims) {
		entry.second.grabs.clear();
		entry.second.grabSides = 0;
	}
	if (strandProbe.active) {
		const auto probed = claims.find(strandProbe.token);
		if (probed != claims.end() && !probed->second.inactive)
			probed->second.grabs.insert(probed->second.grabs.end(), strandProbe.grabs.begin(), strandProbe.grabs.end());
		else
			strandProbe.active = false;
	}
	const auto release = [this] { ReleaseGrabState(); };
	// A hand that lets go with its fingers in the hair would hold that hair up on them, so it pushes nothing until it is clear of it.
	const auto markReleased = [this](std::size_t a_side, const std::vector<HeldPin>& a_pins) {
		for (const HeldPin& pin : a_pins) {
			if (const auto found = claims.find(pin.token); found != claims.end()) {
				found->second.releasedSides |= 1 << a_side;
				found->second.releasedFrames[a_side] = 0;
			}
		}
	};
	if (!settings.GrabStrands || !settings.BodyContacts || !globals::game::isVR) {
		release();
		return;
	}
	auto* higgs = Util::Higgs::GetInterface();
	const auto setHiggsHand = [this, higgs](std::size_t a_side, bool a_disabled) {
		if (!higgs || higgsHandDisabled[a_side] == a_disabled)
			return;
		if (a_disabled)
			higgs->DisableHand(a_side == 0);
		else
			higgs->EnableHand(a_side == 0);
		higgsHandDisabled[a_side] = a_disabled;
	};
	gripReport.higgsBuild = higgs ? higgs->GetBuildNumber() : 0;
	gripReport.higgs = higgs != nullptr;

	const auto& tracker = Util::BodyCapsules::GetTracker();
	std::uint32_t gate = 0;
	for (std::size_t side = 0; side < heldPins.size(); ++side) {
		const bool left = side == 0;
		std::vector<HeldPin>& pins = heldPins[side];
		GripDiagnostic before = gripReport.hands[side];
		GripDiagnostic& diag = gripReport.hands[side];
		diag.handNodes = false;
		diag.inReach = 0;
		diag.nearest = -1.0f;
		const RE::NiAVObject* handNode = tracker.GetHandNode(left);
		if (!handNode) {
			pins.clear();
			setHiggsHand(side, false);
			diag.closed = false;
			diag.pins = 0;
			continue;
		}
		diag.handNodes = true;
		diag.closed = SmpPhysics::ReadyWeaponGate::IsGripDown(left);
		const SmpPhysics::Transform hand = ToSolverTransform(handNode->world);
		if (before.closed && !diag.closed) {
			markReleased(side, pins);
			pins.clear();
			diag.dropped = false;
			grabOverstretchSeconds[side] = 0.0f;
			grabOverreachSeconds[side] = 0.0f;
		}

		// The strands within reach of this hand each frame, so the grip that is about to close on one can be kept from drawing the weapon.
		const PalmCastFrame cast = PalmCastOf(*handNode, left);
		const SmpPhysics::Vec3 castEnd = cast.origin + cast.normal * SmpPhysics::kHiggsSelectDistance;
		const SmpPhysics::Vec3 castMiddle = (cast.origin + castEnd) * 0.5f;
		const RE::NiPoint3 middle{ castMiddle.x, castMiddle.y, castMiddle.z };
		std::vector<SmpPhysics::GrabCandidate> candidates;
		for (const auto& entry : claims) {
			const Claim& claim = entry.second;
			if (claim.inactive || claim.culled() || !claim.parsed)
				continue;
			const HairBound bound = MeasureHairBound(claim.nodes, middle);
			if ((bound.center - middle).Length() > bound.radius + SmpPhysics::kGrabReach + kGrabNearClaimMargin)
				continue;
			const SmpPhysics::SmpRig& rig = claim.parsed->rig;
			std::vector<int> parentOf(rig.bones.size(), -1);
			for (const SmpPhysics::RigJoint& joint : rig.joints) {
				if (joint.child >= 0 && static_cast<std::size_t>(joint.child) < parentOf.size())
					parentOf[static_cast<std::size_t>(joint.child)] = joint.parent;
			}
			const auto positions = claim.BonePositions();
			for (std::size_t bone = 0; bone < rig.bones.size() && bone < claim.nodes.size(); ++bone) {
				if (rig.bones[bone].kinematic || !claim.nodes[bone])
					continue;
				SmpPhysics::Vec3 start;
				SmpPhysics::Vec3 end;
				if (StrandSegmentOf(positions, rig, bone, start, end))
					candidates.push_back({ entry.first, StrandGroupOf(rig, parentOf, bone), static_cast<int>(bone), start, end });
			}
		}
		const SmpPhysics::GrabHandShape handShape = SmpPhysics::MakeHandGrabShape(cast.origin, cast.normal, cast.fingers);
		const auto picks = SmpPhysics::PickGrabCandidates(candidates, handShape, SmpPhysics::kGrabReach, SmpPhysics::kMaxGrabPinsPerHand);
		std::optional<float> hairAlong;
		if (!picks.empty())
			hairAlong = std::max(SmpPhysics::Dot(picks.front().point - cast.origin, cast.normal), 0.0f);
		std::optional<float> otherAlong;
		for (const Util::BodyCapsules::Capsule& capsule : tracker.GetCapsules()) {
			if (capsule.owner == kPlayerFormID)
				continue;
			const SmpPhysics::Vec3 start{ capsule.pointA.x, capsule.pointA.y, capsule.pointA.z };
			const SmpPhysics::Vec3 end{ capsule.pointB.x, capsule.pointB.y, capsule.pointB.z };
			if (const auto hit = SmpPhysics::PalmCastHit(cast.origin, cast.normal, start, end, capsule.radius)) {
				const float surface = std::max(*hit - capsule.radius, 0.0f);
				if (!otherAlong || surface < *otherAlong)
					otherAlong = surface;
			}
		}
		diag.target = SmpPhysics::DecidePalmTarget(hairAlong, otherAlong);
		diag.inReach = picks.size();
		if (settings.ShowGrabDebug) {
			for (const SmpPhysics::GrabPick& pick : picks)
				debugReachSegments.push_back({ candidates[pick.index].start, candidates[pick.index].end, SmpPhysics::DebugColor::InReach });
		}
		diag.nearest = picks.empty() ? -1.0f : picks.front().distance;

		if (diag.closed && pins.empty() && !diag.dropped && diag.target != SmpPhysics::PalmTarget::Other && !(higgs && higgs->IsHoldingObject(left))) {
			for (const SmpPhysics::GrabPick& pick : picks) {
				const SmpPhysics::GrabCandidate& candidate = candidates[pick.index];
				const auto owner = claims.find(candidate.claim);
				const SmpPhysics::Transform boneWorld = owner->second.BoneWorld(static_cast<std::size_t>(candidate.bone));
				pins.push_back({ candidate.claim, candidate.bone, boneWorld.Inverse().TransformPoint(pick.point), hand.Inverse().TransformPoint(pick.point) });
			}
		}

		std::erase_if(pins, [this](const HeldPin& a_pin) {
			const auto found = claims.find(a_pin.token);
			return found == claims.end() || found->second.inactive || found->second.culled();
		});
		float grabTension = 0.0f;
		for (const HeldPin& pin : pins) {
			const Claim& claim = claims.find(pin.token)->second;
			grabTension = std::max(grabTension, SmpPhysics::Length(hand.TransformPoint(pin.anchor) - claim.BoneWorld(static_cast<std::size_t>(pin.bone)).TransformPoint(pin.local)));
		}
		float grabOverreach = 0.0f;
		for (const HeldPin& pin : pins) {
			const Claim& claim = claims.find(pin.token)->second;
			std::size_t rootBone = 0;
			float strandLength = 0.0f;
			if (claim.parsed && StrandReachOf(claim.parsed->rig, claim.restPose, static_cast<std::size_t>(pin.bone), pin.local, rootBone, strandLength))
				grabOverreach = std::max(grabOverreach, SmpPhysics::Length(hand.TransformPoint(pin.anchor) - claim.BoneWorld(rootBone).translation) - strandLength);
		}
		const bool slipped = SmpPhysics::ShouldDropGrab(grabTension, a_frameDelta, grabOverstretchSeconds[side]);
		const bool overreached = SmpPhysics::ShouldDropOverreach(grabOverreach, a_frameDelta, grabOverreachSeconds[side]);
		if (!pins.empty() && (slipped || overreached)) {
			logger::info("{} {} hand: dropped {} strand(s) that could no longer follow it ({:.1f} units from the hand, {:.1f} past its own length)", kLogPrefix, left ? "left" : "right", pins.size(), grabTension, grabOverreach);
			markReleased(side, pins);
			pins.clear();
			diag.dropped = true;
			grabTension = 0.0f;
			PulseGrabHaptics(side, true, 0.0f);
		}
		for (const HeldPin& pin : pins) {
			Claim& claim = claims.find(pin.token)->second;
			claim.grabs.push_back({ pin.bone, pin.local, hand.TransformPoint(pin.anchor), SmpPhysics::kGrabStiffness });
			claim.grabSides |= 1 << side;
		}
		diag.pins = pins.size();
		if (settings.ShowGrabDebug || settings.ShowJointDebug) {
			// The cast takes the colour of what the grip would do: white for nothing, green for hair it will take, yellow for hair it holds, orange for a body part HIGGS keeps.
			const SmpPhysics::DebugColor castColor = !pins.empty()                                ? SmpPhysics::DebugColor::Held :
			                                         diag.target == SmpPhysics::PalmTarget::Hair  ? SmpPhysics::DebugColor::InReach :
			                                         diag.target == SmpPhysics::PalmTarget::Other ? SmpPhysics::DebugColor::Stretched :
			                                                                                        SmpPhysics::DebugColor::Hand;
			for (std::size_t segment = 0; segment < handShape.segmentCount; ++segment) {
				debugReachSegments.push_back({ handShape.segments[segment][0], handShape.segments[segment][1], castColor });
				SmpPhysics::AppendRing(debugReachSegments, handShape.segments[segment][1], SmpPhysics::Normalized(handShape.segments[segment][1] - handShape.segments[segment][0]), SmpPhysics::kGrabReach, castColor);
			}
			for (const SmpPhysics::GrabPick& pick : picks)
				debugGrabBones.push_back({ candidates[pick.index].claim, candidates[pick.index].bone, false });
			for (const HeldPin& pin : pins)
				debugGrabBones.push_back({ pin.token, pin.bone, true });
		}
		PulseGrabHaptics(side, pins.size() > before.pins, grabTension);
		const auto gatePicks = SmpPhysics::PickGrabCandidates(candidates, handShape, SmpPhysics::kWeaponGateReach, 1);
		// HIGGS loses the hand only while the palm is pointed at hair rather than at a body part, so its body and object interactions work everywhere else.
		const bool onHair = !pins.empty() || diag.target == SmpPhysics::PalmTarget::Hair;
		if (const auto action = SmpPhysics::DecideHiggsHand(onHair, higgs && higgs->IsHoldingObject(left), higgsHandDisabled[side]); action != SmpPhysics::HiggsHandAction::None)
			setHiggsHand(side, action == SmpPhysics::HiggsHandAction::Disable);
		if (!pins.empty() || !gatePicks.empty())
			gate |= left ? SmpPhysics::ReadyWeaponGate::kLeftHand : SmpPhysics::ReadyWeaponGate::kRightHand;

		if (before.closed != diag.closed || (before.inReach > 0) != (diag.inReach > 0) || before.pins != diag.pins || before.handNodes != diag.handNodes) {
			logger::info("{} {} hand: grip {}, {} strand(s) in reach{}, holding {}", kLogPrefix,
				left ? "left" : "right", diag.closed ? "closed" : "open", diag.inReach, diag.inReach ? std::format(" (nearest {:.1f})", diag.nearest) : std::string{}, diag.pins);
		}
	}
	SmpPhysics::ReadyWeaponGate::SetHeldHands(gate);
}

void SmpPhysicsFeature::AppendJointLines(const Claim& a_claim, std::vector<SmpPhysics::DebugSegment>& a_out, const std::vector<DebugGrabBone>& a_grabBones)
{
	const auto positions = a_claim.BonePositions();
	const auto position = [&positions](int a_bone, SmpPhysics::Vec3& a_at) {
		if (a_bone < 0 || static_cast<std::size_t>(a_bone) >= positions.size() || !positions[static_cast<std::size_t>(a_bone)])
			return false;
		a_at = *positions[static_cast<std::size_t>(a_bone)];
		return true;
	};
	const SmpPhysics::DebugColor ownerColor = a_claim.inactive                           ? SmpPhysics::DebugColor::SolverIdle :
	                                          a_claim.culled() || !a_claim.gpuRegistered ? SmpPhysics::DebugColor::SolverCulled :
	                                                                                       SmpPhysics::DebugColor::SolverGpu;
	for (const SmpPhysics::RigJoint& joint : a_claim.parsed->rig.joints) {
		SmpPhysics::Vec3 parent;
		SmpPhysics::Vec3 child;
		if (!position(joint.parent, parent) || !position(joint.child, child))
			continue;
		bool stretched = false;
		if (static_cast<std::size_t>(std::max(joint.parent, joint.child)) < a_claim.restPose.size()) {
			const float rest = SmpPhysics::Length(a_claim.restPose[static_cast<std::size_t>(joint.child)].translation - a_claim.restPose[static_cast<std::size_t>(joint.parent)].translation);
			const float now = SmpPhysics::Length(child - parent);
			stretched = rest > kJointMinimumRestLength && std::abs(now / rest - 1.0f) > kJointLengthTolerance;
		}
		SmpPhysics::DebugColor color = stretched ? SmpPhysics::DebugColor::Stretched : ownerColor;
		for (const DebugGrabBone& grab : a_grabBones) {
			if (grab.token != a_claim.token || (grab.bone != joint.parent && grab.bone != joint.child))
				continue;
			color = grab.held ? SmpPhysics::DebugColor::Held : SmpPhysics::DebugColor::InReach;
			if (grab.held)
				break;
		}
		a_out.push_back({ parent, child, color });
		SmpPhysics::AppendCross(a_out, child, kJointCrossSize, color);
	}
}

void SmpPhysicsFeature::AppendDeclinedLines(std::vector<SmpPhysics::DebugSegment>& a_out)
{
	// Only as accurate as the decline record: a pair is drawn while its latest outcome is a decline, from the engine's own pose.
	const auto* camera = RE::PlayerCamera::GetSingleton();
	const RE::NiPoint3 eye = camera && camera->cameraRoot ? camera->cameraRoot->world.translate : RE::NiPoint3{};
	std::size_t actorsDrawn = 0;
	std::vector<std::uint32_t> drawn;
	for (const auto& [formID, file] : claimTally.DeclinedPairs()) {
		const auto* actor = RE::TESForm::LookupByID<RE::Actor>(formID);
		RE::NiNode* root = actor && actor->Is3DLoaded() ? actor->Get3D()->AsNode() : nullptr;
		if (!root || root->world.translate.GetDistance(eye) > kDebugMaxDistance)
			continue;
		if (std::find(drawn.begin(), drawn.end(), formID) == drawn.end()) {
			if (actorsDrawn >= static_cast<std::size_t>(std::max(settings.DebugMaxActors, 0)))
				continue;
			drawn.push_back(formID);
			++actorsDrawn;
		}
		// stateMutex is already held, so this cannot go through GetParsedRig.
		auto cached = parsedRigs.find(file);
		if (cached == parsedRigs.end())
			cached = parsedRigs.try_emplace(file, std::make_shared<SmpPhysics::ParseResult>(ReadPhysicsFile(file))).first;
		const auto& parsed = cached->second;
		if (!parsed)
			continue;
		for (const SmpPhysics::RigJoint& joint : parsed->rig.joints) {
			if (joint.parent < 0 || joint.child < 0 || static_cast<std::size_t>(std::max(joint.parent, joint.child)) >= parsed->rig.bones.size())
				continue;
			const auto* from = root->GetObjectByName(RE::BSFixedString(parsed->rig.bones[static_cast<std::size_t>(joint.parent)].name));
			const auto* to = root->GetObjectByName(RE::BSFixedString(parsed->rig.bones[static_cast<std::size_t>(joint.child)].name));
			if (!from || !to)
				continue;
			const SmpPhysics::Vec3 end{ to->world.translate.x, to->world.translate.y, to->world.translate.z };
			a_out.push_back({ { from->world.translate.x, from->world.translate.y, from->world.translate.z }, end, SmpPhysics::DebugColor::SolverHdt });
			SmpPhysics::AppendCross(a_out, end, kJointCrossSize, SmpPhysics::DebugColor::SolverHdt);
		}
	}
}

void SmpPhysicsFeature::AppendSurfaceEdgesNearHair(const Claim& a_claim, std::vector<SmpPhysics::DebugSegment>& a_out)
{
	std::vector<SmpPhysics::Vec3> hair;
	const auto positions = a_claim.BonePositions();
	for (std::size_t bone = 0; bone < positions.size() && bone < a_claim.parsed->rig.bones.size(); ++bone) {
		if (!a_claim.parsed->rig.bones[bone].kinematic && positions[bone])
			hair.push_back(*positions[bone]);
	}
	if (hair.empty())
		return;

	struct NearTriangle
	{
		SmpPhysics::Vec3 a;
		SmpPhysics::Vec3 b;
		SmpPhysics::Vec3 c;
		float margin;
	};
	std::vector<NearTriangle> triangles;
	const std::size_t limit = a_out.size() + kSurfaceDebugMaxLines;
	for (const SmpPhysics::SkinnedSurface& surface : a_claim.surfaces) {
		const std::vector<SmpPhysics::Vec3> posed = SmpPhysics::PoseSurface(surface, a_claim.driverWorld);
		std::vector<char> closeToHair(posed.size(), 0);
		for (std::size_t vertex = 0; vertex < posed.size(); ++vertex) {
			closeToHair[vertex] = std::any_of(hair.begin(), hair.end(), [&](const SmpPhysics::Vec3& a_bone) {
				const SmpPhysics::Vec3 offset = posed[vertex] - a_bone;
				return SmpPhysics::Dot(offset, offset) < kSurfaceDebugReach * kSurfaceDebugReach;
			});
		}
		for (std::size_t corner = 0; corner + 2 < surface.indices.size(); corner += 3) {
			const std::size_t corners[3] = { surface.indices[corner], surface.indices[corner + 1], surface.indices[corner + 2] };
			if (corners[0] >= posed.size() || corners[1] >= posed.size() || corners[2] >= posed.size())
				continue;
			if (closeToHair[corners[0]] || closeToHair[corners[1]] || closeToHair[corners[2]])
				triangles.push_back({ posed[corners[0]], posed[corners[1]], posed[corners[2]], surface.margin });
			// An edge shared by two triangles is drawn by the one that traverses it from the lower index.
			for (std::size_t edge = 0; edge < 3 && a_out.size() < limit; ++edge) {
				const std::size_t from = corners[edge];
				const std::size_t to = corners[(edge + 1) % 3];
				if (from < to && (closeToHair[from] || closeToHair[to]))
					a_out.push_back({ posed[from], posed[to], SmpPhysics::DebugColor::Collider });
			}
		}
	}
	if (triangles.empty())
		return;

	// The triangles go in a grid so each sphere looks only at the ones around it.
	const auto cellOf = [](float a_value) { return static_cast<std::int64_t>(std::floor(a_value / kSurfaceGridCell)); };
	const auto cellKey = [](std::int64_t a_x, std::int64_t a_y, std::int64_t a_z) {
		return static_cast<std::uint64_t>((a_x & 0x1FFFFF) | ((a_y & 0x1FFFFF) << 21) | ((a_z & 0x1FFFFF) << 42));
	};
	std::unordered_map<std::uint64_t, std::vector<std::uint32_t>> grid;
	for (std::size_t index = 0; index < triangles.size(); ++index) {
		const NearTriangle& triangle = triangles[index];
		const SmpPhysics::Vec3 low{ std::min({ triangle.a.x, triangle.b.x, triangle.c.x }), std::min({ triangle.a.y, triangle.b.y, triangle.c.y }), std::min({ triangle.a.z, triangle.b.z, triangle.c.z }) };
		const SmpPhysics::Vec3 high{ std::max({ triangle.a.x, triangle.b.x, triangle.c.x }), std::max({ triangle.a.y, triangle.b.y, triangle.c.y }), std::max({ triangle.a.z, triangle.b.z, triangle.c.z }) };
		for (std::int64_t x = cellOf(low.x); x <= cellOf(high.x); ++x) {
			for (std::int64_t y = cellOf(low.y); y <= cellOf(high.y); ++y) {
				for (std::int64_t z = cellOf(low.z); z <= cellOf(high.z); ++z)
					grid[cellKey(x, y, z)].push_back(static_cast<std::uint32_t>(index));
			}
		}
	}
	std::vector<std::uint32_t> visitedAt(triangles.size(), 0);
	std::uint32_t visitStamp = 0;

	// Each hair collision sphere as a ring facing the camera: teal while it rests on a surface, pink with a line to the surface while there is clear space.
	const auto* camera = globals::game::playerCamera;
	const RE::NiPoint3 eye = camera ? camera->GetRuntimeData2().pos : RE::NiPoint3{};
	const auto& spheres = a_claim.spheres;
	std::size_t rings = 0;
	for (std::size_t bone = 0; bone < spheres.size() && bone < a_claim.nodes.size(); ++bone) {
		if (!a_claim.nodes[bone])
			continue;
		const SmpPhysics::Transform world = a_claim.BoneWorld(bone);
		for (const SmpPhysics::BoneSphere& sphere : spheres[bone]) {
			const SmpPhysics::Vec3 centre = world.TransformPoint(sphere.centerLocal);
			float nearest = std::numeric_limits<float>::max();
			SmpPhysics::Vec3 nearestPoint{};
			float nearestMargin = 0.0f;
			++visitStamp;
			for (std::int64_t x = cellOf(centre.x) - 1; x <= cellOf(centre.x) + 1; ++x) {
				for (std::int64_t y = cellOf(centre.y) - 1; y <= cellOf(centre.y) + 1; ++y) {
					for (std::int64_t z = cellOf(centre.z) - 1; z <= cellOf(centre.z) + 1; ++z) {
						const auto cell = grid.find(cellKey(x, y, z));
						if (cell == grid.end())
							continue;
						for (const std::uint32_t index : cell->second) {
							if (visitedAt[index] == visitStamp)
								continue;
							visitedAt[index] = visitStamp;
							const NearTriangle& triangle = triangles[index];
							SmpPhysics::Vec3 point;
							const float distance = SmpPhysics::DistanceToTriangle(centre, triangle.a, triangle.b, triangle.c, point);
							if (distance < nearest) {
								nearest = distance;
								nearestPoint = point;
								nearestMargin = triangle.margin;
							}
						}
					}
				}
			}
			if (nearest == std::numeric_limits<float>::max())
				continue;
			const float gap = nearest - sphere.radius - nearestMargin;
			if (gap > kSphereReportGap)
				continue;
			// A sphere on a surface gets a ring; one with clear space only the line across it, which is a twelfth of the lines.
			if (gap < kSphereTouchTolerance) {
				if (rings++ < kSphereRingMax)
					SmpPhysics::AppendRing(a_out, centre, centre - SmpPhysics::Vec3{ eye.x, eye.y, eye.z }, sphere.radius, SmpPhysics::DebugColor::SphereTouch);
			} else {
				a_out.push_back({ centre, nearestPoint, SmpPhysics::DebugColor::SphereGap });
			}
		}
	}
}

void SmpPhysicsFeature::AppendColliderOutlines(const Claim& a_claim, std::vector<SmpPhysics::DebugSegment>& a_out,
	std::vector<std::pair<SmpPhysics::Vec3, SmpPhysics::Vec3>>& a_drawnExternal, bool a_detailed, const std::vector<std::uint32_t>& a_shownActors)
{
	for (const SmpPhysics::Collider& collider : a_claim.colliders) {
		if (collider.driverIndex < 0 || static_cast<std::size_t>(collider.driverIndex) >= a_claim.driverWorld.size())
			continue;
		const SmpPhysics::Transform& world = a_claim.driverWorld[static_cast<std::size_t>(collider.driverIndex)];
		if (collider.type == SmpPhysics::Collider::Type::Capsule)
			SmpPhysics::AppendCapsuleOutline(a_out, world.TransformPoint(collider.segmentA), world.TransformPoint(collider.segmentB), collider.radius, SmpPhysics::DebugColor::Collider);
		else if (collider.type == SmpPhysics::Collider::Type::Ellipsoid)
			SmpPhysics::AppendEllipsoidOutline(a_out, world.TransformPoint(collider.center), world.rotation * collider.orientation, collider.halfExtents, SmpPhysics::DebugColor::Collider);
	}
	if (a_detailed)
		AppendSurfaceEdgesNearHair(a_claim, a_out);
	// Other bodies' capsules are already in world space, and the same one reaches every claim near it.
	for (std::size_t index = 0; index < a_claim.externalColliders.size() && index < a_claim.externalOwners.size(); ++index) {
		const SmpPhysics::Collider& other = a_claim.externalColliders[index];
		const RE::FormID owner = a_claim.externalOwners[index];
		if (other.type != SmpPhysics::Collider::Type::Capsule)
			continue;
		if (owner != kPlayerFormID && std::find(a_shownActors.begin(), a_shownActors.end(), owner) == a_shownActors.end())
			continue;
		const bool drawn = std::any_of(a_drawnExternal.begin(), a_drawnExternal.end(), [&other](const auto& a_pair) {
			return SmpPhysics::Length(a_pair.first - other.segmentA) < kDebugDuplicateDistance && SmpPhysics::Length(a_pair.second - other.segmentB) < kDebugDuplicateDistance;
		});
		if (drawn)
			continue;
		a_drawnExternal.emplace_back(other.segmentA, other.segmentB);
		SmpPhysics::AppendCapsuleOutline(a_out, other.segmentA, other.segmentB, other.radius, SmpPhysics::DebugColor::Hand);
	}
}

void SmpPhysicsFeature::UpdateGrabDebug()
{
	std::vector<SmpPhysics::DebugSegment> segments;
	if ((settings.ShowGrabDebug || settings.ShowColliderDebug || settings.ShowJointDebug || settings.ShowSkeletonDebug) && globals::state->IsDeveloperMode()) {
		segments = debugReachSegments;
		const auto push = [&segments](const SmpPhysics::Vec3& a_start, const SmpPhysics::Vec3& a_end, SmpPhysics::DebugColor a_color) {
			segments.push_back({ a_start, a_end, a_color });
		};
		std::vector<std::pair<SmpPhysics::Vec3, SmpPhysics::Vec3>> drawnExternal;
		// Nearest claims first, so the line budget runs out on the far ones. A culled claim still draws: it is only not simulated.
		std::vector<const Claim*> nearClaims;
		for (const auto& entry : claims) {
			if (!entry.second.inactive && entry.second.parsed && entry.second.lastCameraDistance < kDebugMaxDistance)
				nearClaims.push_back(&entry.second);
		}
		std::sort(nearClaims.begin(), nearClaims.end(), [](const Claim* a_left, const Claim* a_right) { return a_left->lastCameraDistance < a_right->lastCameraDistance; });
		// A mesh handed over with no actor counts as an actor of its own, so a scene full of them cannot all pass as one.
		const auto actorKey = [](const Claim& a_claim) { return a_claim.actorFormID != 0 ? a_claim.actorFormID : (kActorlessClaimKey | static_cast<std::uint32_t>(a_claim.token & 0x7FFFFFFFu)); };
		std::vector<SmpPhysics::ActorDistance> claimActors;
		for (const Claim* nearClaim : nearClaims)
			claimActors.push_back({ actorKey(*nearClaim), nearClaim->lastCameraDistance });
		const std::vector<std::uint32_t> shownActors = SmpPhysics::NearestActors(std::move(claimActors), settings.DebugMaxActors);
		std::erase_if(nearClaims, [&](const Claim* a_claim) { return std::find(shownActors.begin(), shownActors.end(), actorKey(*a_claim)) == shownActors.end(); });
		for (const Claim* nearClaim : nearClaims) {
			const Claim& claim = *nearClaim;
			if (settings.ShowColliderDebug)
				AppendColliderOutlines(claim, segments, drawnExternal, claim.lastCameraDistance < kDebugDetailDistance, shownActors);
			if (settings.ShowJointDebug)
				AppendJointLines(claim, segments, debugGrabBones);
			if (!settings.ShowGrabDebug || claim.culled())
				continue;
			const SmpPhysics::SmpRig& rig = claim.parsed->rig;
			std::vector<int> parentOf(rig.bones.size(), -1);
			for (const SmpPhysics::RigJoint& joint : rig.joints) {
				if (joint.child >= 0 && static_cast<std::size_t>(joint.child) < parentOf.size())
					parentOf[static_cast<std::size_t>(joint.child)] = joint.parent;
			}

			// Whole cards a hand holds, and the line from each held point to where the hand wants it.
			std::vector<std::uint32_t> heldGroups;
			for (const SmpPhysics::Grab& grab : claim.grabs) {
				if (grab.bone < 0 || static_cast<std::size_t>(grab.bone) >= claim.nodes.size() || !claim.nodes[static_cast<std::size_t>(grab.bone)])
					continue;
				heldGroups.push_back(StrandGroupOf(rig, parentOf, static_cast<std::size_t>(grab.bone)));
				const SmpPhysics::Vec3 held = claim.BoneWorld(static_cast<std::size_t>(grab.bone)).TransformPoint(grab.local);
				push(held, grab.target, SmpPhysics::DebugColor::Pull);
			}
			const auto positions = claim.BonePositions();
			for (std::size_t bone = 0; bone < rig.bones.size() && bone < claim.nodes.size(); ++bone) {
				if (rig.bones[bone].kinematic || !claim.nodes[bone])
					continue;
				SmpPhysics::Vec3 start;
				SmpPhysics::Vec3 end;
				if (!StrandSegmentOf(positions, rig, bone, start, end))
					continue;
				if (std::find(heldGroups.begin(), heldGroups.end(), StrandGroupOf(rig, parentOf, bone)) != heldGroups.end()) {
					push(start, end, SmpPhysics::DebugColor::Held);
					continue;
				}
				// Hair a hand's capsule is pushing: a bone inside, or just outside, one of the hand capsules the solver was given.
				for (std::size_t index = 0; index < claim.externalColliders.size() && index < claim.externalOwners.size(); ++index) {
					if (claim.externalOwners[index] != kPlayerFormID)
						continue;
					const SmpPhysics::Collider& hand = claim.externalColliders[index];
					const SmpPhysics::Vec3 axis = hand.segmentB - hand.segmentA;
					const float lengthSquared = SmpPhysics::Dot(axis, axis);
					const SmpPhysics::Vec3 middle = (start + end) * 0.5f;
					const float along = lengthSquared > 1.0e-8f ? std::clamp(SmpPhysics::Dot(middle - hand.segmentA, axis) / lengthSquared, 0.0f, 1.0f) : 0.0f;
					if (SmpPhysics::Length(middle - (hand.segmentA + axis * along)) < hand.radius + kDebugInfluenceMargin) {
						push(start, end, SmpPhysics::DebugColor::Influenced);
						break;
					}
				}
			}
			for (std::size_t index = 0; index < claim.externalColliders.size() && index < claim.externalOwners.size(); ++index) {
				if (claim.externalOwners[index] == kPlayerFormID)
					push(claim.externalColliders[index].segmentA, claim.externalColliders[index].segmentB, SmpPhysics::DebugColor::Hand);
			}
		}
		if (settings.ShowJointDebug)
			AppendDeclinedLines(segments);
		if (settings.ShowSkeletonDebug)
			AppendSkeletonLines(segments, settings.DebugMaxActors);
	}
	debugLines.SetSegments(std::move(segments));
}

static void AppendSkeletonOf(const RE::NiAVObject* a_node, SmpPhysics::DebugColor a_color, std::vector<SmpPhysics::DebugSegment>& a_out, int a_depth)
{
	// CommonLib has no const AsNode; nothing here writes to the node.
	const auto* node = a_node ? const_cast<RE::NiAVObject*>(a_node)->AsNode() : nullptr;
	if (!node || a_depth > kSkeletonMaxDepth)
		return;
	const RE::NiPoint3& from = node->world.translate;
	for (const auto& child : node->GetChildren()) {
		const auto* childNode = child ? child->AsNode() : nullptr;
		if (!childNode)
			continue;
		const RE::NiPoint3& to = childNode->world.translate;
		if ((to - from).Length() > kSkeletonMinLength)
			a_out.push_back({ { from.x, from.y, from.z }, { to.x, to.y, to.z }, a_color });
		AppendSkeletonOf(childNode, a_color, a_out, a_depth + 1);
	}
}

void SmpPhysicsFeature::AppendSkeletonLines(std::vector<SmpPhysics::DebugSegment>& a_out, int a_maxActors)
{
	const auto* camera = globals::game::playerCamera;
	const RE::NiPoint3 eye = camera ? camera->GetRuntimeData2().pos : RE::NiPoint3{};
	if (auto* processLists = RE::ProcessLists::GetSingleton()) {
		std::vector<SmpPhysics::ActorDistance> candidates;
		std::vector<RE::NiPointer<RE::Actor>> actors;
		for (const auto& handle : processLists->highActorHandles) {
			const auto actor = handle.get();
			if (!actor || actor->IsPlayerRef() || !actor->Is3DLoaded())
				continue;
			const float distance = (actor->GetPosition() - eye).Length();
			if (distance <= kDebugMaxDistance) {
				candidates.push_back({ actor->GetFormID(), distance });
				actors.push_back(actor);
			}
		}
		const std::vector<std::uint32_t> chosen = SmpPhysics::NearestActors(std::move(candidates), a_maxActors);
		for (const auto& actor : actors) {
			if (std::find(chosen.begin(), chosen.end(), actor->GetFormID()) != chosen.end())
				AppendSkeletonOf(actor->Get3D(), SmpPhysics::DebugColor::Skeleton, a_out, 0);
		}
	}
	const auto& tracker = Util::BodyCapsules::GetTracker();
	for (const bool left : { true, false })
		AppendSkeletonOf(tracker.GetHandNode(left), SmpPhysics::DebugColor::Fingers, a_out, 0);
}

json SmpPhysicsFeature::GripReportJson() const
{
	json result;
	result["higgs"] = gripReport.higgs;
	result["higgsBuild"] = gripReport.higgsBuild;
	result["readyWeaponEventsSwallowed"] = SmpPhysics::ReadyWeaponGate::SwallowedCount();
	result["hapticPulses"] = grabHapticPulses.load(std::memory_order_relaxed);
	result["weaponDrawn"] = globals::game::player && globals::game::player->AsActorState()->IsWeaponDrawn();
	result["hands"] = json::array();
	for (std::size_t side = 0; side < gripReport.hands.size(); ++side) {
		const GripDiagnostic& diag = gripReport.hands[side];
		result["hands"].push_back({ { "hand", side == 0 ? "left" : "right" }, { "handNodes", diag.handNodes }, { "closed", diag.closed }, { "strandsInReach", diag.inReach }, { "nearestDistance", diag.nearest }, { "holding", diag.pins },
			{ "higgsDisabled", higgsHandDisabled[side] }, { "dropped", diag.dropped },
			{ "palmTarget", diag.target == SmpPhysics::PalmTarget::Hair ? "hair" : diag.target == SmpPhysics::PalmTarget::Other ? "other" :
																																  "none" } });
	}
	return result;
}

void SmpPhysicsFeature::PlaceCrowdHairAtHand(const json& a_args)
{
	const bool left = a_args.value("hand", std::string("right")) == "left";
	const auto index = static_cast<std::size_t>(std::max(a_args.value("index", 0), 0));
	const float away = Util::ClampFinite(a_args.value("away", 0.0f), -kMaxHandPinOffset, kMaxHandPinOffset, 0.0f);
	auto* task = SKSE::GetTaskInterface();
	if (!task)
		return;
	task->AddTask([this, left, index, away] {
		const RE::NiAVObject* handNode = Util::BodyCapsules::GetTracker().GetHandNode(left);
		if (!handNode) {
			TestBench::GetCrowd().SetReport("The player's hand nodes were not found, so there is no hand to place hair against.");
			return;
		}
		const PalmCastFrame cast = PalmCastOf(*handNode, left);
		const std::vector<RE::ObjectRefHandle> standing = TestBench::GetCrowd().Standing();
		auto reference = index < standing.size() ? standing[index].get() : nullptr;
		auto* actor = reference ? reference->As<RE::Actor>() : nullptr;
		if (!actor) {
			TestBench::GetCrowd().SetReport(std::format("There is no test character number {}.", index));
			return;
		}
		std::optional<SmpPhysics::Vec3> strand;
		{
			const std::scoped_lock lock(stateMutex);
			for (const auto& entry : claims) {
				const Claim& claim = entry.second;
				if (claim.actorFormID != actor->GetFormID() || !claim.parsed)
					continue;
				const SmpPhysics::SmpRig& rig = claim.parsed->rig;
				const auto positions = claim.BonePositions();
				for (std::size_t bone = 0; bone < rig.bones.size() && !strand; ++bone) {
					SmpPhysics::Vec3 start;
					SmpPhysics::Vec3 end;
					if (!rig.bones[bone].kinematic && StrandSegmentOf(positions, rig, bone, start, end))
						strand = (start + end) * 0.5f;
				}
				if (strand)
					break;
			}
		}
		if (!strand) {
			TestBench::GetCrowd().SetReport(std::format("Test character {} has no claimed hair to place at the hand.", index));
			return;
		}
		const SmpPhysics::Vec3 aim = cast.origin + cast.normal * (SmpPhysics::kHiggsSelectDistance * 0.5f) - *strand;
		RE::NiPoint3 offset{ aim.x, aim.y, aim.z };
		if (away != 0.0f) {
			SmpPhysics::Vec3 side = SmpPhysics::Cross(cast.normal, { 0.0f, 0.0f, 1.0f });
			if (SmpPhysics::Length(side) < kParallelAxisEpsilon)
				side = SmpPhysics::Cross(cast.normal, { 1.0f, 0.0f, 0.0f });
			side = SmpPhysics::Normalized(side) * away;
			offset += RE::NiPoint3{ side.x, side.y, side.z };
		}
		actor->SetPosition(actor->GetPosition() + offset, true);
		actor->Update3DPosition(true);
		TestBench::GetCrowd().SetReport(std::format("Placed test character {}'s hair at the {} hand.", index, left ? "left" : "right"));
	});
}

void SmpPhysicsFeature::SetStrandProbe(const json& a_args)
{
	const std::scoped_lock lock(stateMutex);
	strandProbe = {};
	if (!a_args.value("enabled", true))
		return;
	const RE::FormID actorFormID = a_args.value("actorFormID", 0u);
	const std::size_t wanted = static_cast<std::size_t>(std::clamp(a_args.value("count", 1), 1, static_cast<int>(SmpPhysics::kMaxGrabPinsPerHand)));
	SmpPhysics::Vec3 offset{ kDefaultHandPinOffset, 0.0f, 0.0f };
	if (a_args.contains("offset") && a_args["offset"].is_array() && a_args["offset"].size() == 3) {
		float values[3]{};
		for (std::size_t axis = 0; axis < 3; ++axis)
			values[axis] = a_args["offset"][axis].is_number() ? Util::ClampFinite(a_args["offset"][axis].get<float>(), -kMaxHandPinOffset, kMaxHandPinOffset, 0.0f) : 0.0f;
		offset = { values[0], values[1], values[2] };
	}
	for (const auto& entry : claims) {
		const Claim& claim = entry.second;
		if (claim.actorFormID != actorFormID || !claim.parsed)
			continue;
		const SmpPhysics::SmpRig& rig = claim.parsed->rig;
		std::vector<int> parentOf(rig.bones.size(), -1);
		for (const SmpPhysics::RigJoint& joint : rig.joints) {
			if (joint.child >= 0 && static_cast<std::size_t>(joint.child) < parentOf.size())
				parentOf[static_cast<std::size_t>(joint.child)] = joint.parent;
		}
		struct Reach
		{
			float distance;
			std::size_t bone;
		};
		std::vector<Reach> reach;
		const int requested = a_args.value("bone", -1);
		for (std::size_t bone = 0; bone < rig.bones.size() && bone < claim.nodes.size(); ++bone) {
			if (rig.bones[bone].kinematic || !claim.nodes[bone])
				continue;
			const int rootBone = bone < claim.treeKinematicParent.size() ? claim.treeKinematicParent[bone] : -1;
			if (rootBone < 0 || !claim.nodes[static_cast<std::size_t>(rootBone)])
				continue;
			const float distance = requested == static_cast<int>(bone) ? std::numeric_limits<float>::max() :
			                                                             (claim.nodes[bone]->world.translate - claim.nodes[static_cast<std::size_t>(rootBone)]->world.translate).Length();
			reach.push_back({ distance, bone });
		}
		std::sort(reach.begin(), reach.end(), [](const Reach& a_left, const Reach& a_right) { return a_left.distance > a_right.distance; });
		std::vector<std::uint32_t> taken;
		for (const Reach& candidate : reach) {
			if (strandProbe.grabs.size() >= wanted)
				break;
			const std::uint32_t group = StrandGroupOf(rig, parentOf, candidate.bone);
			if (std::find(taken.begin(), taken.end(), group) != taken.end())
				continue;
			SmpPhysics::Vec3 start;
			SmpPhysics::Vec3 end;
			if (!StrandSegmentOf(claim.BonePositions(), rig, candidate.bone, start, end))
				continue;
			taken.push_back(group);
			const SmpPhysics::Vec3 point = (start + end) * 0.5f;
			const SmpPhysics::Transform boneWorld = claim.BoneWorld(candidate.bone);
			strandProbe.grabs.push_back({ static_cast<int>(candidate.bone), boneWorld.Inverse().TransformPoint(point), point + offset, SmpPhysics::kGrabStiffness });
		}
		if (!strandProbe.grabs.empty()) {
			strandProbe.active = true;
			strandProbe.token = entry.first;
			return;
		}
	}
}

json SmpPhysicsFeature::PoseCheckJson(const json& a_args) const
{
	const std::uint32_t only = a_args.value("actorFormID", 0u);
	json result = json::array();
	const std::scoped_lock lock(stateMutex);
	for (const auto& entry : claims) {
		const Claim& claim = entry.second;
		if (!claim.parsed || claim.nodes.empty() || (only != 0 && claim.actorFormID != only) || claim.restPose.size() != claim.nodes.size())
			continue;
		const SmpPhysics::SmpRig& rig = claim.parsed->rig;
		float fromRest = 0.0f;
		float fromRestSum = 0.0f;
		float fromSag = 0.0f;
		float fromSagSum = 0.0f;
		float fromNode = 0.0f;
		std::size_t counted = 0;
		for (std::size_t bone = 0; bone < rig.bones.size() && bone < claim.nodes.size(); ++bone) {
			if (rig.bones[bone].kinematic || !claim.nodes[bone] || bone >= claim.treeKinematicParent.size())
				continue;
			const auto root = static_cast<std::size_t>(claim.treeKinematicParent[bone]);
			if (root >= claim.nodes.size() || !claim.nodes[root])
				continue;
			const SmpPhysics::Transform solved = claim.BoneWorld(bone);
			const SmpPhysics::Transform animated = ToSolverTransform(claim.nodes[root]->world) * (claim.restPose[root].Inverse() * claim.restPose[bone]);
			const float rest = SmpPhysics::Length(solved.translation - animated.translation);
			fromRest = std::max(fromRest, rest);
			fromRestSum += rest;
			if (bone < claim.sagPose.size() && root < claim.sagPose.size()) {
				const SmpPhysics::Transform settled = ToSolverTransform(claim.nodes[root]->world) * (claim.sagPose[root].Inverse() * claim.sagPose[bone]);
				const float sag = SmpPhysics::Length(solved.translation - settled.translation);
				fromSag = std::max(fromSag, sag);
				fromSagSum += sag;
			}
			const RE::NiPoint3& at = claim.nodes[bone]->world.translate;
			fromNode = std::max(fromNode, SmpPhysics::Length(solved.translation - SmpPhysics::Vec3{ at.x, at.y, at.z }));
			++counted;
		}
		result.push_back({ { "token", entry.first }, { "actorFormID", claim.actorFormID }, { "dynamicBones", counted }, { "maxSolvedFromRest", fromRest },
			{ "meanSolvedFromRest", counted ? fromRestSum / static_cast<float>(counted) : 0.0f }, { "maxSolvedFromNode", fromNode },
			{ "maxSolvedFromSag", fromSag }, { "meanSolvedFromSag", counted ? fromSagSum / static_cast<float>(counted) : 0.0f } });
	}
	return result;
}

json SmpPhysicsFeature::HeldStrandsJson() const
{
	json result = json::array();
	const auto describe = [this](const char* a_hand, std::uint64_t a_token, int a_bone, const SmpPhysics::Vec3& a_local, const SmpPhysics::Vec3& a_target) {
		json entry = { { "hand", a_hand }, { "token", a_token }, { "bone", a_bone }, { "target", { a_target.x, a_target.y, a_target.z } } };
		const auto found = claims.find(a_token);
		if (found != claims.end() && a_bone >= 0 && static_cast<std::size_t>(a_bone) < found->second.nodes.size() && found->second.nodes[static_cast<std::size_t>(a_bone)]) {
			const SmpPhysics::Vec3 at = ToSolverTransform(found->second.nodes[static_cast<std::size_t>(a_bone)]->world).TransformPoint(a_local);
			entry["position"] = { at.x, at.y, at.z };
			entry["distanceToTarget"] = SmpPhysics::Length(at - a_target);
		}
		return entry;
	};
	if (strandProbe.active) {
		for (const SmpPhysics::Grab& grab : strandProbe.grabs)
			result.push_back(describe("probe", strandProbe.token, grab.bone, grab.local, grab.target));
	}
	for (std::size_t side = 0; side < heldPins.size(); ++side) {
		for (const HeldPin& pin : heldPins[side]) {
			SmpPhysics::Vec3 target{};
			const auto found = claims.find(pin.token);
			if (found != claims.end()) {
				for (const SmpPhysics::Grab& grab : found->second.grabs) {
					if (grab.bone == pin.bone)
						target = grab.target;
				}
			}
			result.push_back(describe(side == 0 ? "left" : "right", pin.token, pin.bone, pin.local, target));
		}
	}
	return result;
}

void SmpPhysicsFeature::UpdateExternalColliders(Claim& a_claim)
{
	if (a_claim.externalDriver < 0)
		return;
	a_claim.externalColliders.clear();
	a_claim.externalOwners.clear();
	const auto* node = settings.BodyContacts && a_claim.firstKinematicBone >= 0 ? a_claim.nodes[static_cast<std::size_t>(a_claim.firstKinematicBone)].get() : nullptr;
	if (node) {
		std::array<Util::BodyCapsules::Capsule, Util::BodyCapsules::MaxSelectedCapsules> found;
		float nearestGap = 0.0f;
		const HairBound bound = MeasureHairBound(a_claim.nodes, node->world.translate);
		std::size_t count = Util::BodyCapsules::GetTracker().Select(bound.center, bound.radius, kBodyContactReach,
			a_claim.actorFormID, found, nearestGap, true);
		if (count > SmpPhysics::kExternalColliderSlots) {
			// Slots go to the capsules nearest an actual hair bone: ranking by distance from the hair's middle lets a palm near
			// the face fill every slot and drop the fingertips touching it.
			std::vector<SmpPhysics::Vec3> anchors;
			for (std::size_t bone = 0; bone < a_claim.spheres.size(); ++bone) {
				if (!a_claim.spheres[bone].empty())
					anchors.push_back(a_claim.BoneWorld(bone).TransformPoint(a_claim.spheres[bone].front().centerLocal));
			}
			if (!anchors.empty()) {
				constexpr float kBodyRankPenalty = 1.0e6f;
				std::array<std::pair<float, std::size_t>, Util::BodyCapsules::MaxSelectedCapsules> ranked{};
				for (std::size_t index = 0; index < count; ++index) {
					const Util::BodyCapsules::Capsule& capsule = found[index];
					const SmpPhysics::Vec3 start{ capsule.pointA.x, capsule.pointA.y, capsule.pointA.z };
					const SmpPhysics::Vec3 axis = SmpPhysics::Vec3{ capsule.pointB.x, capsule.pointB.y, capsule.pointB.z } - start;
					const float axisLengthSquared = SmpPhysics::Dot(axis, axis);
					float nearestAnchor = std::numeric_limits<float>::max();
					for (const SmpPhysics::Vec3& anchor : anchors) {
						const float along = axisLengthSquared > 0.0f ? std::clamp(SmpPhysics::Dot(anchor - start, axis) / axisLengthSquared, 0.0f, 1.0f) : 0.0f;
						nearestAnchor = std::min(nearestAnchor, SmpPhysics::Length(anchor - (start + axis * along)));
					}
					ranked[index] = { nearestAnchor - capsule.radius + (capsule.appendage ? 0.0f : kBodyRankPenalty), index };
				}
				std::partial_sort(ranked.begin(), ranked.begin() + SmpPhysics::kExternalColliderSlots, ranked.begin() + count);
				std::array<Util::BodyCapsules::Capsule, SmpPhysics::kExternalColliderSlots> chosen;
				for (std::size_t slot = 0; slot < SmpPhysics::kExternalColliderSlots; ++slot)
					chosen[slot] = found[ranked[slot].second];
				std::copy(chosen.begin(), chosen.end(), found.begin());
				count = SmpPhysics::kExternalColliderSlots;
			}
		}
		const SmpPhysics::Vec3 centre{ node->world.translate.x, node->world.translate.y, node->world.translate.z };
		a_claim.externalCentre = centre;
		if (bodyContactProbe.active && bodyContactProbe.actorFormID == a_claim.actorFormID) {
			SmpPhysics::Collider probe;
			probe.type = SmpPhysics::Collider::Type::Capsule;
			probe.driverIndex = a_claim.externalDriver;
			probe.segmentA = centre + bodyContactProbe.offset;
			probe.segmentB = probe.segmentA - SmpPhysics::Vec3{ 0.0f, 0.0f, bodyContactProbe.length };
			probe.radius = bodyContactProbe.radius;
			a_claim.externalColliders.push_back(probe);
			a_claim.externalOwners.push_back(0);
		}
		int stillTouching = 0;
		const auto capsuleTouchesHair = [&a_claim](const SmpPhysics::Vec3& a_start, const SmpPhysics::Vec3& a_end, float a_radius) {
			const SmpPhysics::Vec3 axis = a_end - a_start;
			const float axisLengthSquared = SmpPhysics::Dot(axis, axis);
			for (std::size_t bone = 0; bone < a_claim.spheres.size(); ++bone) {
				if (a_claim.spheres[bone].empty())
					continue;
				const SmpPhysics::Transform world = a_claim.BoneWorld(bone);
				for (const SmpPhysics::BoneSphere& sphere : a_claim.spheres[bone]) {
					const SmpPhysics::Vec3 centre = world.TransformPoint(sphere.centerLocal);
					const float along = axisLengthSquared > 0.0f ? std::clamp(SmpPhysics::Dot(centre - a_start, axis) / axisLengthSquared, 0.0f, 1.0f) : 0.0f;
					if (SmpPhysics::Length(centre - (a_start + axis * along)) < a_radius + sphere.radius + kReleasedHandClearance)
						return true;
				}
			}
			return false;
		};
		for (std::size_t index = 0; index < count && index < found.size(); ++index) {
			if (found[index].appendage && found[index].side >= 0 && (a_claim.grabSides & (1 << found[index].side)) != 0)
				continue;
			const SmpPhysics::Vec3 segmentA{ found[index].pointA.x, found[index].pointA.y, found[index].pointA.z };
			const SmpPhysics::Vec3 segmentB{ found[index].pointB.x, found[index].pointB.y, found[index].pointB.z };
			if (found[index].appendage && found[index].side >= 0 && (a_claim.releasedSides & (1 << found[index].side)) != 0 &&
				capsuleTouchesHair(segmentA, segmentB, found[index].radius)) {
				stillTouching |= 1 << found[index].side;
				continue;
			}
			SmpPhysics::Collider collider;
			collider.type = SmpPhysics::Collider::Type::Capsule;
			collider.driverIndex = a_claim.externalDriver;
			collider.segmentA = segmentA;
			collider.segmentB = segmentB;
			collider.radius = found[index].radius;
			a_claim.externalColliders.push_back(collider);
			a_claim.externalOwners.push_back(found[index].owner);
		}
		for (std::size_t side = 0; side < a_claim.releasedFrames.size(); ++side) {
			const int bit = 1 << side;
			if ((a_claim.releasedSides & bit) == 0)
				continue;
			if ((stillTouching & bit) == 0 || ++a_claim.releasedFrames[side] > kReleasedHandMaxFrames)
				a_claim.releasedSides &= ~bit;
		}
	}
	if (settings.ClothObjects && a_claim.cloth && node) {
		const HairBound bound = MeasureHairBound(a_claim.nodes, node->world.translate);
		for (const Util::BodyCapsules::Capsule& object : Util::HavokObjects::Near(bound.center, bound.radius, kBodyContactReach)) {
			if (a_claim.externalColliders.size() >= SmpPhysics::kExternalColliderSlots)
				break;
			SmpPhysics::Collider collider;
			collider.type = SmpPhysics::Collider::Type::Capsule;
			collider.driverIndex = a_claim.externalDriver;
			collider.segmentA = { object.pointA.x, object.pointA.y, object.pointA.z };
			collider.segmentB = { object.pointB.x, object.pointB.y, object.pointB.z };
			collider.radius = object.radius;
			a_claim.externalColliders.push_back(collider);
			a_claim.externalOwners.push_back(object.owner);
		}
	}
	a_claim.externalColliders.resize(std::min(a_claim.externalColliders.size(), SmpPhysics::kExternalColliderSlots));
	a_claim.externalOwners.resize(a_claim.externalColliders.size());
	a_claim.externalCount = static_cast<int>(a_claim.externalColliders.size());
	a_claim.externalPacked.assign(SmpPhysics::kExternalColliderSlots, SmpPhysics::GpuRig::PackCollider(SmpPhysics::Collider{}));
	for (std::size_t index = 0; index < a_claim.externalColliders.size(); ++index)
		a_claim.externalPacked[index] = SmpPhysics::GpuRig::PackCollider(a_claim.externalColliders[index]);
}

void SmpPhysicsFeature::StepClaims()
{
	gpuSkinning.SetBatched(settings.BatchSkinPalette);
	gpuSkinning.SetEnabled(linked.load(std::memory_order_acquire) && settings.Enabled && settings.GpuSkinning &&
						   gpuSolver.Available());
	if (!linked.load(std::memory_order_acquire))
		return;
	if (!settings.Enabled) {
		if (!claimsHandedBack) {
			claimsHandedBack = true;
			ReevaluateClaimedActors();
		}
		return;
	}
	if (claimsHandedBack) {
		claimsHandedBack = false;
		ReevaluateAllActors();
	}
	if (gamePaused.load(std::memory_order_relaxed))
		return;
	RunWindProbe();
	{
		const std::scoped_lock lock(stateMutex);
		if (claims.empty()) {
			ReleaseGrabState();
			debugReachSegments.clear();
			debugGrabBones.clear();
			UpdateGrabDebug();
			return;
		}
	}
	if (settings.BodyContacts)
		Util::BodyCapsules::GetTracker().Update(kBodyContactRange);

	ZoneScopedN("SmpPhysics::Step");

	const auto* camera = globals::game::playerCamera;
	const RE::NiPoint3 cameraPosition = camera ? camera->GetRuntimeData2().pos : RE::NiPoint3{};
	const float frameDelta = std::clamp(globals::game::deltaTime ? *globals::game::deltaTime : RE::GetSecondsSinceLastFrame(),
		kMinFrameDelta, kMaxFrameDelta);

	const bool useGpu = gpuSolver.Available();
	std::vector<RE::FormID> refitActors;

	double totalMicroseconds = 0.0;
	{
		const std::scoped_lock lock(stateMutex);
		UpdateWind(frameDelta);
		ApplyCulling();
		UpdateStrandGrabs(frameDelta);
		UpdateGrabDebug();
		std::vector<SmpPhysics::GpuSolver::ClaimResult> results;
		std::unordered_map<std::uint64_t, std::size_t> resultOfToken;
		{
			ZoneScopedN("SmpPhysics::PollResults");
			if (useGpu && gpuSolver.Poll(results)) {
				for (std::size_t index = 0; index < results.size(); ++index)
					resultOfToken.emplace(results[index].token, index);
			}
		}

		registrationsThisFrame = 0u;
		std::vector<SmpPhysics::GpuSolver::FrameClaim> submissions;
		{
			ZoneScopedN("SmpPhysics::StepClaims");
			for (auto& entry : claims) {
				const auto found = resultOfToken.find(entry.first);
				const SmpPhysics::GpuSolver::ClaimResult* result = found == resultOfToken.end() ? nullptr : &results[found->second];
				if (!StepClaim(entry.second, cameraPosition, camera != nullptr, frameDelta, result, submissions))
					continue;
				totalMicroseconds += entry.second.stepMicroseconds;
				const RE::FormID formID = entry.second.actorFormID;
				if (WornMeshesChanged(entry.second, frameDelta) &&
					std::find(refitActors.begin(), refitActors.end(), formID) == refitActors.end())
					refitActors.push_back(formID);
			}
		}

		for (auto claim = claims.begin(); claim != claims.end();) {
			if (claim->second.missingActorFrames < kStaleClaimFrames) {
				++claim;
				continue;
			}
			logger::info("{} Dropping the claim for '{}' (token {}): its actor no longer exists", kLogPrefix, claim->second.physicsFile, claim->first);
			gpuSolver.RemoveClaim(claim->first);
			gpuSkinning.RemoveClaim(claim->first);
			claim = claims.erase(claim);
		}

		for (const RE::FormID formID : bodyShapeActors) {
			if (std::find(refitActors.begin(), refitActors.end(), formID) == refitActors.end())
				refitActors.push_back(formID);
		}
		bodyShapeActors.clear();

		bool submitted = false;
		{
			ZoneScopedN("SmpPhysics::Submit");
			gpuSolver.SetFlushAfterSubmit(settings.FlushAfterSubmit);
			submitted = !submissions.empty() && gpuSolver.SubmitFrame(submissions);
		}
		const auto worldsView = gpuSolver.WorldsView();
		for (auto& entry : claims) {
			const bool poseReady = useGpu && entry.second.gpuRegistered && !entry.second.inactive && !entry.second.culled() &&
			                       entry.second.missingActorFrames == 0 && entry.second.lastCameraDistance <= settings.SimulationDistance &&
			                       gpuSolver.PoseReady(entry.first);
			gpuSkinning.SetClaimState(entry.first, poseReady, worldsView, gpuSolver.Frame());
		}
		gpuSkinning.BuildBatch();
		totalStepMicroseconds = totalMicroseconds;
	}

	// Asking hdtSMP64 to re-run claiming is what rebuilds the proxies from what the actor now wears, and it
	// re-enters claim() and takes stateMutex, so it cannot be done while the lock above is held.
	for (const RE::FormID formID : refitActors)
		ReevaluateActor(RE::TESForm::LookupByID<RE::Actor>(formID));
}

bool SmpPhysicsFeature::RegisterClaimForGpu(Claim& a_claim)
{
	ZoneScopedN("SmpPhysics::RegisterClaim");
	if (!a_claim.parsed)
		return false;
	std::string reason;
	if (!gpuSolver.AddClaim(a_claim.token, a_claim.parsed->rig, a_claim.restPose, a_claim.solverParams,
			a_claim.gpuCollision, a_claim.surfaces, a_claim.windAreas, reason)) {
		logger::info("{} '{}' stays at the engine's pose: {}", kLogPrefix, a_claim.physicsFile, reason);
		return false;
	}
	a_claim.previousKinematic = a_claim.kinematicInput;
	return true;
}

/** @brief Maps a skin instance's bones onto the rig's dynamic bones and copies their bind transforms; the vertices are read separately. */
static SmpPhysics::CardSkin MakeCardSkin(const RE::NiSkinInstance& a_skin, const std::vector<RE::NiPointer<RE::NiAVObject>>& a_nodes,
	const SmpPhysics::SmpRig& a_rig)
{
	SmpPhysics::CardSkin card;
	const std::uint32_t count = a_skin.skinData->GetBoneCount();
	card.rigBone.assign(count, -1);
	card.bind.resize(count);
	for (std::uint32_t bone = 0; bone < count; ++bone) {
		for (std::size_t rigBone = 0; rigBone < a_nodes.size(); ++rigBone) {
			if (a_nodes[rigBone].get() == a_skin.bones[bone] && !a_rig.bones[rigBone].kinematic) {
				card.rigBone[bone] = static_cast<int>(rigBone);
				break;
			}
		}
		const RE::NiTransform transform = a_skin.skinData->GetBoneDataSkinToBone(bone);
		SmpPhysics::BindTransform& bind = card.bind[bone];
		for (int row = 0; row < 3; ++row) {
			for (int column = 0; column < 3; ++column)
				bind.rotation[row][column] = transform.rotate.entry[row][column];
		}
		bind.scale = transform.scale;
		bind.translation = { transform.translate.x, transform.translate.y, transform.translate.z };
	}
	return card;
}

void SmpPhysicsFeature::MeasureWindAreas(Claim& a_claim)
{
	a_claim.windMeasured = true;
	if (!a_claim.parsed)
		return;
	const SmpPhysics::SmpRig& rig = a_claim.parsed->rig;
	std::vector<SmpPhysics::CardSkin> cards;
	for (std::size_t index = 0; index < a_claim.skins.size() && index < a_claim.cardGeometries.size(); ++index) {
		const RE::NiSkinInstance* skin = a_claim.skins[index].get();
		if (!skin || !skin->skinData || !skin->bones)
			continue;
		SmpPhysics::CardSkin card = MakeCardSkin(*skin, a_claim.nodes, rig);
		std::string reason;
		if (SmpPhysics::ReadCardVertices(a_claim.cardGeometries[index].get(), card.vertices, reason) &&
			SmpPhysics::ReadCardTriangles(a_claim.cardGeometries[index].get(), card.triangles, reason))
			cards.push_back(std::move(card));
	}
	if (cards.empty())
		return;
	a_claim.windAreas = SmpPhysics::EstimateBoneWindAreas(rig, cards);
}

void SmpPhysicsFeature::BindClaimSkins(Claim& a_claim)
{
	ZoneScopedN("SmpPhysics::BindClaimSkins");
	a_claim.skinsBound = true;
	const auto offset = gpuSolver.BoneOffset(a_claim.token);
	if (!offset || !a_claim.parsed)
		return;

	const SmpPhysics::SmpRig& rig = a_claim.parsed->rig;
	std::size_t bound = 0;
	for (std::size_t skinIndex = 0; skinIndex < a_claim.skins.size(); ++skinIndex) {
		RE::NiSkinInstance* skin = a_claim.skins[skinIndex].get();
		if (!skin || !skin->skinData || !skin->bones)
			continue;
		const std::uint32_t count = skin->skinData->GetBoneCount();
		std::vector<std::uint32_t> slots(count, SmpPhysics::GpuSkinning::kStaticBone);
		std::vector<SmpPhysics::SkinRows> skinToBone(count);
		bool anyDynamic = false;
		for (std::uint32_t index = 0; index < count; ++index) {
			for (std::size_t bone = 0; bone < a_claim.nodes.size(); ++bone) {
				if (a_claim.nodes[bone].get() == skin->bones[index] && !rig.bones[bone].kinematic) {
					slots[index] = *offset + static_cast<std::uint32_t>(bone);
					anyDynamic = true;
					break;
				}
			}
			const RE::NiTransform transform = skin->skinData->GetBoneDataSkinToBone(index);
			skinToBone[index] = SmpPhysics::PackSkinToBone(transform.rotate.entry, transform.scale,
				transform.translate.x, transform.translate.y, transform.translate.z);
		}
		if (anyDynamic && gpuSkinning.Register(skin, a_claim.token, std::move(slots), std::move(skinToBone)))
			++bound;
	}
	logger::info("{} '{}': {} of {} skinned meshes will be drawn from the GPU solver", kLogPrefix, a_claim.physicsFile, bound, a_claim.skins.size());
}

void SmpPhysicsFeature::RestartClaim(Claim& a_claim)
{
	a_claim.gpuResetRequested = true;
	a_claim.gpuHasPrevious = false;
	a_claim.gpuLastBones.clear();
}

bool SmpPhysicsFeature::ClaimReady(Claim& a_claim, const RE::Actor* a_actor)
{
	if (a_claim.actorFormID != 0) {
		a_claim.missingActorFrames = a_actor ? 0u : a_claim.missingActorFrames + 1u;
		if (!a_actor || !a_actor->Is3DLoaded()) {
			++a_claim.skippedNotReadyFrames;
			return false;
		}
	}

	const SmpPhysics::SmpRig& rig = a_claim.parsed->rig;
	for (std::size_t index = 0; index < rig.bones.size(); ++index) {
		if (!rig.bones[index].kinematic)
			continue;
		const auto* node = a_claim.nodes[index].get();
		if (!node || !node->parent) {
			++a_claim.skippedNotReadyFrames;
			return false;
		}
	}
	return true;
}

bool SmpPhysicsFeature::UpdateClaimInactive(Claim& a_claim, const RE::Actor* a_actor)
{
	if (!a_actor)
		return false;
	bool inactive = false;
	if (a_claim.headPart && a_claim.hideUnderWig) {
		auto* actor = const_cast<RE::Actor*>(a_actor);
		inactive = actor->GetWornArmor(RE::BIPED_MODEL::BipedObjectSlot::kHair) != nullptr ||
		           actor->GetWornArmor(RE::BIPED_MODEL::BipedObjectSlot::kLongHair) != nullptr;
	}
	if (a_claim.firstPersonOff && a_actor == globals::game::player && globals::game::playerCamera &&
		globals::game::playerCamera->currentState == globals::game::playerCamera->GetRuntimeData().cameraStates[0])
		inactive = true;
	a_claim.inactive = inactive;
	if (inactive) {
		RestartClaim(a_claim);
	}
	return inactive;
}

bool SmpPhysicsFeature::ClaimInRange(Claim& a_claim, const RE::NiPoint3& a_cameraPosition, bool a_haveCameraPosition)
{
	if (!a_haveCameraPosition || a_claim.firstKinematicBone < 0)
		return true;
	const auto* node = a_claim.nodes[static_cast<std::size_t>(a_claim.firstKinematicBone)].get();
	if (!node) {
		++a_claim.skippedNotReadyFrames;
		return false;
	}
	a_claim.lastCameraDistance = node->world.translate.GetDistance(a_cameraPosition);
	if (a_claim.lastCameraDistance > settings.SimulationDistance) {
		++a_claim.skippedByDistanceFrames;
		RestartClaim(a_claim);
		return false;
	}
	return true;
}

void SmpPhysicsFeature::GatherSolverInput(Claim& a_claim)
{
	const SmpPhysics::SmpRig& rig = a_claim.parsed->rig;
	for (std::size_t index = 0; index < rig.bones.size(); ++index) {
		if (rig.bones[index].kinematic)
			a_claim.kinematicInput[index] = ToSolverTransform(a_claim.nodes[index]->world);
	}
	gpuSkinning.RecordStepPoses(a_claim.token);
	for (std::size_t index = 0; index < a_claim.collisionDrivers.size(); ++index) {
		const auto* node = a_claim.collisionDrivers[index].get();
		a_claim.driverWorld[index] = node != nullptr ? ToSolverTransform(node->world) : SmpPhysics::Transform{};
	}
	UpdateExternalColliders(a_claim);

	if (!a_claim.windMeasured)
		MeasureWindAreas(a_claim);
}

bool SmpPhysicsFeature::RootTeleported(const Claim& a_claim) const
{
	const SmpPhysics::SmpRig& rig = a_claim.parsed->rig;
	for (std::size_t index = 0; index < rig.bones.size(); ++index) {
		if (!rig.bones[index].kinematic)
			continue;
		const SmpPhysics::Vec3 moved = a_claim.kinematicInput[index].translation - a_claim.previousKinematic[index].translation;
		if (SmpPhysics::Dot(moved, moved) > kTeleportDistance * kTeleportDistance)
			return true;
	}
	return false;
}

void SmpPhysicsFeature::SubmitGpuFrame(Claim& a_claim, float a_frameDelta, std::vector<SmpPhysics::GpuSolver::FrameClaim>& a_submissions)
{
	SmpPhysics::GpuSolver::FrameClaim submission;
	submission.token = a_claim.token;
	submission.kinematicWorld = &a_claim.kinematicInput;
	submission.driverWorld = &a_claim.driverWorld;
	submission.externalColliders = a_claim.externalDriver >= 0 ? &a_claim.externalPacked : nullptr;
	submission.hasPreviousInput = a_claim.gpuHasPrevious;
	submission.resetRequested = a_claim.gpuResetRequested;
	submission.deltaTime = a_frameDelta;
	submission.wind = a_claim.wind;
	submission.windTime = windClock;
	submission.windFactor = a_claim.windFactor;
	submission.rigidContacts = settings.RigidContacts;
	submission.grabs = a_claim.grabs;
	a_submissions.push_back(submission);
	a_claim.gpuHasPrevious = true;
}

void SmpPhysicsFeature::WriteBackPose(Claim& a_claim)
{
	const SmpPhysics::SmpRig& rig = a_claim.parsed->rig;
	for (std::size_t index = 0; index < rig.bones.size(); ++index) {
		if (rig.bones[index].kinematic)
			continue;
		auto* node = a_claim.nodes[index].get();
		if (!node)
			continue;
		const SmpPhysics::Transform world = a_claim.kinematicInput[static_cast<std::size_t>(a_claim.treeKinematicParent[index])] * a_claim.gpuLastBones[index];
		SmpPhysics::ToRotationMatrix(world.rotation, node->world.rotate.entry);
		node->world.translate = RE::NiPoint3{ world.translation.x, world.translation.y, world.translation.z };
	}
}

bool SmpPhysicsFeature::StepClaim(Claim& a_claim, const RE::NiPoint3& a_cameraPosition, bool a_haveCameraPosition, float a_frameDelta,
	const SmpPhysics::GpuSolver::ClaimResult* a_gpuResult, std::vector<SmpPhysics::GpuSolver::FrameClaim>& a_submissions)
{
	if (!a_claim.parsed)
		return false;

	const std::size_t boneCount = a_claim.parsed->rig.bones.size();
	if (a_claim.nodes.size() != boneCount || a_claim.kinematicInput.size() != boneCount || a_claim.previousKinematic.size() != boneCount)
		return false;

	const auto* actor = a_claim.actorFormID != 0 ? RE::TESForm::LookupByID<RE::Actor>(a_claim.actorFormID) : nullptr;
	if (!ClaimReady(a_claim, actor) || UpdateClaimInactive(a_claim, actor))
		return false;
	if (a_claim.culled()) {
		RestartClaim(a_claim);
		return false;
	}
	if (!ClaimInRange(a_claim, a_cameraPosition, a_haveCameraPosition))
		return false;
	const bool useGpu = gpuSolver.Available();
	const bool registering = useGpu && !a_claim.gpuRegistered && !a_claim.gpuRegistrationFailed;
	if (registering && registrationsThisFrame >= kRegistrationsPerFrame)
		return false;
	++a_claim.steppedFrames;

	const auto start = std::chrono::steady_clock::now();
	GatherSolverInput(a_claim);

	if (registering) {
		++registrationsThisFrame;
		a_claim.gpuRegistered = RegisterClaimForGpu(a_claim);
		a_claim.gpuRegistrationFailed = !a_claim.gpuRegistered;
		if (a_claim.gpuRegistered && !a_claim.skinsBound)
			BindClaimSkins(a_claim);
	}

	const bool teleported = RootTeleported(a_claim);
	const bool gpuDrives = useGpu && a_claim.gpuRegistered;
	UpdateClaimWindExposure(a_claim, a_frameDelta);
	if (teleported)
		RestartClaim(a_claim);

	if (gpuDrives)
		SubmitGpuFrame(a_claim, a_frameDelta, a_submissions);
	a_claim.gpuResetRequested = false;

	a_claim.previousKinematic = a_claim.kinematicInput;

	if (a_gpuResult && a_gpuResult->failed)
		RestartClaim(a_claim);

	const bool applyGpu = gpuDrives && a_gpuResult && a_gpuResult->bones.size() == boneCount;
	if (applyGpu && !a_gpuResult->failed) {
		if (SmpPhysics::GpuRig::PoseExceedsReach(a_claim.parsed->rig, a_gpuResult->bones, a_claim.boneReach)) {
			if (a_claim.divergenceRestarts++ == 0)
				logger::warn("{} '{}' (token {}) placed a bone beyond its strand's reach, so the claim restarts from rest", kLogPrefix, a_claim.physicsFile, a_claim.token);
			RestartClaim(a_claim);
		} else {
			a_claim.gpuLastBones = a_gpuResult->bones;
		}
	}
	const bool holdGpu = gpuDrives && a_claim.gpuLastBones.size() == boneCount;

	if (holdGpu)
		WriteBackPose(a_claim);

	a_claim.stepMicroseconds = std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - start).count();
	return true;
}

#undef I18N_KEY_PREFIX
