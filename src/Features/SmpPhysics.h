#pragma once

#include "Features/SmpPhysics/ClaimOverrides.h"
#include "Features/SmpPhysics/CrowdWigs.h"
#include "Features/SmpPhysics/HairTarget.h"

#include "Feature.h"
#include "d3d/GpuSolver.h"
#include "engine/DebugLines.h"
#include "engine/GpuSkinning.h"
#include "smp/ClaimTally.h"
#include "smp/Collision.h"
#include "smp/CollisionOverride.h"
#include "smp/ItemStats.h"
#include "smp/PhysicsHealth.h"
#include "smp/RepairLog.h"
#include "smp/SelfWindShelter.h"
#include "smp/SkinnedSurface.h"
#include "smp/SmpRig.h"
#include "smp/SolverCommon.h"
#include "smp/StrandGrab.h"

#include "hdtSMP64/PluginAPI.h"

#include <array>
#include <atomic>
#include <cstdint>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

namespace Util::BodyCapsules
{
	struct Capsule;
}

/** @brief Bounds of the user-facing settings, shared by the settings page and the feature's own clamping. */
namespace SmpPhysicsLimits
{
	constexpr int kMaxDebugActors = 8;
	constexpr float kMinSimulationDistance = 100.0f;
	constexpr float kMaxSimulationDistance = 20000.0f;
	constexpr int kMinClaims = 1;
	constexpr int kMaxClaims = 256;
	constexpr float kMinScale = 0.0f;
	constexpr float kMaxScale = 10.0f;
	constexpr float kMaxWindStrength = 100.0f;
	constexpr float kMaxScreenSizePercent = 50.0f;
}

/** @brief An engine node transform as the solver sees it. The engine's scale is dropped: a Skyrim actor is not scaled. */
[[nodiscard]] inline SmpPhysics::Transform ToSolverTransform(const RE::NiTransform& a_transform) noexcept
{
	return { SmpPhysics::FromRotationMatrix(a_transform.rotate.entry),
		{ a_transform.translate.x, a_transform.translate.y, a_transform.translate.z } };
}

/**
 * @brief Simulates hair and cloth meshes that hdtSMP64 hands over, on the GPU.
 *
 * The feature stays inert until hdtSMP64's plugin interface is linked and the user enables it: it takes
 * over only the meshes whose SMP XML describes a bone-pendulum rig it can simulate, and asks hdtSMP64 to
 * re-run claiming as soon as the settings stop covering them, so those meshes fall back to SMP.
 * @note The type carries the Feature suffix because the core module it drives owns the SmpPhysics namespace.
 */
struct SmpPhysicsFeature final : Feature
{
	struct Settings
	{
		bool Enabled = true;  ///< On by default: installing this plugin is the opt-in.
		bool ClaimHeadParts = true;
		bool ClaimArmor = true;
		bool CollisionEnabled = true;        ///< Rigs whose file declares collision shapes are only claimed while this is on.
		float SimulationDistance = 2000.0f;  ///< Game units; claims farther from the camera are not stepped.
		int MaxClaims = 128;
		float MinScreenSizePercent = 0.0f;  ///< Stops simulating a claim whose hair covers less than this percentage of the screen height for a while; 0 switches it off.
		bool CullOffscreen = false;         ///< Stops simulating a claim that has stayed outside the view for a while, unless it is close to the camera; it restarts from rest when it comes back into view.
		int MaxSimulatedClaims = 128;       ///< Most claims stepped per frame, nearest to the camera first; the rest are drawn from the engine's pose until they rank inside the budget again.
		float GravityScale = 1.0f;
		bool RigidContacts = true;  ///< Pushes rigid-solver bodies out of the collision proxies after every substep.
		float WindStrength = 6.0f;  ///< Scales the weather wind on every claim (hair and cloth): 300 units/s per unit at full sky wind, 0 switches it off. Not hdtSMP64's scale.
		float StiffnessScale = 1.0f;
		float DampingScale = 1.0f;
		bool GpuSkinning = true;         ///< Draws GPU-solved hair from the solver's own pose instead of the staged readback.
		bool RetargetRest = false;       ///< Turns the rest pose of hair authored inside worn armor out of it when a claim is built; off matches hdtSMP64, which keeps the authored rest.
		bool BodyContacts = true;        ///< Hair is pushed by other bodies, hands and held items that come near it.
		bool ClothFloor = true;          ///< A cape or cloak rests on the floor under the character; hdtSMP64 only does this for rigs that declare a ground shape.
		bool ClothObjects = true;        ///< A cape or cloak is pushed by loose havok objects, such as a thrown barrel or a dropped weapon; hdtSMP64 does not.
		bool DrapeCloth = true;          ///< Turns a cape or cloak authored to hang over armor in toward the body when a claim is built; hdtSMP64 keeps the authored rest.
		bool ClothCollision = true;      ///< Hair collides with the cloth, such as a cape, that other rigs simulate on the same actor; hdtSMP64 does not.
		bool ShowGrabDebug = false;      ///< Developer mode: draw the strands hands hold, those in reach, and the hair hands push.
		bool ShowSkeletonDebug = false;  ///< Developer mode: draw the skeletons of nearby actors and the player's hands.
		bool ShowJointDebug = false;     ///< Developer mode: draw each simulated bone from its parent, and flag those stretched or squeezed from rest length.
		int DebugMaxActors = 2;          ///< Most actors the debug overlays draw, nearest to the camera first.
		bool ShowColliderDebug = false;  ///< Developer mode: outline the head, body and armor proxies and other bodies' capsules that hair collides with.
		bool GrabStrands = true;         ///< In VR, closing a hand on hair holds that strand to the hand.
		bool BatchSkinPalette = true;    ///< Builds every skinned mesh's matrices once per frame after the solver step, instead of one dispatch per draw.
		bool FlushAfterSubmit = false;   ///< Sends the solver's commands to the GPU as soon as they are recorded; otherwise the driver holds them until the game flushes, which left the GPU idle for several milliseconds.
	};

	Settings settings;

	virtual inline std::string GetShortName() override { return "SmpPhysics"; }

	/** @brief Registers the hdtSMP64 startup listener, the only work this feature does before it is enabled. */
	virtual void OnPluginMessage(const SKSE::MessagingInterface::Message* a_message) override;
	virtual void DataLoaded() override;
	/** @brief Drops the compiled skinning and debug line shaders only, leaving the solver running, so edited shader files take effect. */
	void ReloadVisualShaders();
	/** @brief Drops the solver's compute shaders so edited files compile on the next step; claims keep their state. */
	void ReloadSolverShaders();
	/** @brief Fails the GPU solver on purpose so every claim goes back to hdtSMP64, for testing the fallback. */
	void SimulateGpuFailure();
	virtual void RegisterUxActions() override;
	/** @brief Draws the debug overlay's lines into the deferred G-buffer. */
	virtual void OnDeferredGeometryEnd() override;
	/** @brief Rebinds a claimed mesh's skinning matrices to the GPU solver's pose, on the render thread. */
	virtual void OnBoneMatricesUploaded(RE::NiSkinInstance* a_skin) override;
	virtual void LoadSettings(json& a_json) override;
	virtual void SaveSettings(json& a_json) override;

	/** @brief One row of the settings UI's claim table: a claimed physics file, or one switched off for testing. */
	struct ClaimRow
	{
		RE::FormID actorFormID{};
		std::string file;
		int boneCount{};
		bool gpu{};
		bool enabled{};
		std::uint64_t token{};  ///< The live claim's token, zero for a row switched off; makes the row's widget ID unique.
		const char* state{};    ///< Why a live claim is drawn from the engine's pose this frame, or null while it is simulated.
	};

	/** @brief Asks hdtSMP64 to re-run claiming for every loaded actor, so a newly enabled feature picks meshes up. */
	void ReevaluateAllActors();
	/** @brief Puts every setting back to its default and asks hdtSMP64 to offer every actor's meshes again; the saved file is not touched. */
	void ResetSettings();
	/** @brief Asks hdtSMP64 to re-run claiming for the player alone. */
	void ReevaluatePlayer();
	/** @brief Asks hdtSMP64 to re-run claiming for the actors this feature currently holds. */
	void ReevaluateClaimedActors();
	/**
	 * @brief Switches the simulation of one actor's physics file on or off, so a claim can be compared with hdtSMP64 driving the same hair.
	 * @details Off, the file is declined for that actor and hdtSMP64 is asked to offer the actor's meshes again, so it simulates the file itself.
	 * @param a_file The physics file as the claim lists it, compared without regard to case.
	 */
	void SetClaimEnabled(RE::FormID a_actorFormID, const std::string& a_file, bool a_enabled);

	/** @brief How far a saved rule reaches. */
	enum class OverrideScope
	{
		BaseForm,    ///< The file for characters made from the same base form.
		PhysicsFile  ///< The file for every character.
	};
	/**
	 * @brief Saves a rule that keeps one actor's physics file with hdtSMP64, and applies it to the actors that match it now.
	 * @return false when the rule cannot be saved: no file, a duplicate, or a base form scope for an actor made in this session, which has no stable name.
	 */
	bool AddOverride(RE::FormID a_actorFormID, const std::string& a_file, OverrideScope a_scope);
	/** @brief Drops a saved rule and asks hdtSMP64 to offer the meshes it had kept back again. */
	bool RemoveOverride(std::size_t a_index);
	/** @return Each saved rule as a line of text, in the order RemoveOverride counts them. */
	[[nodiscard]] std::vector<std::string> GetOverrideLines() const;
	/** @return Whether a base form rule can name this actor, which an actor made in this session cannot. */
	[[nodiscard]] bool CanRememberBaseForm(RE::FormID a_actorFormID) const;
	void LoadOverrides(const json& a_json);
	void SaveOverrides(json& a_json) const;

	/** @brief What the settings page shows beside the controls. */
	struct UiStatus
	{
		bool linked{};
		std::string linkStatus;
		std::size_t claimCount{};
		double stepMilliseconds{};
	};
	[[nodiscard]] UiStatus GetUiStatus() const;
	/** @brief Sets the test crowd's hair colour from the game thread, or puts back the natural colours when a_color is empty. */
	void ApplyCrowdHairColor(std::string a_color);
	/** @brief The live claims and the files switched off for testing, one row each, for the Claims page. */
	[[nodiscard]] std::vector<ClaimRow> GetClaimRows() const;
	/** @brief Offers, takes and declines by mesh kind, and the decline reasons ranked by the (actor, file) pairs they hold back. */
	[[nodiscard]] json ClaimTallyJson() const;
	void ResetClaimTally();
	/** @brief Adds (a_off) or removes a debugging switch that stops a piece colliding with a surface or the body proxies; an empty piece and target clear them all. */
	void SetCollisionOverride(const std::string& a_piece, const std::string& a_target, bool a_off);
	/** @brief Up to the first few declined physics files, each as "file: reason". */
	[[nodiscard]] std::vector<std::string> GetDeclinedLines() const;
	/** @brief Each physics file with a problem, as text to paste to its author. */
	[[nodiscard]] std::vector<std::pair<std::string, std::string>> GetRepairReports() const;
	/** @brief Text to send an item's mod author: its name, plugin, physics files and what is wrong with each; empty when the scan found no files. Game thread only. */
	[[nodiscard]] std::string ItemReportText(RE::FormID a_armor);
	/** @brief A few short lines for hovering an item: each physics file's health and size, and how many bones its claims kept. Empty before a scan. Game thread only. */
	[[nodiscard]] std::string ItemStatsText(RE::FormID a_armor);
	/** @brief Puts the wig at a_index of CrowdWigs::WigForms() on the crowd and, while it is included, the player; runs on the game thread. */
	void WearWig(int a_index);
	/** @brief The character whose hair is matched and swapped, and the wigs ranked by their match to its hair. */
	[[nodiscard]] SmpPhysics::HairTargetView HairTargetState() const { return hairTarget.View(); }
	/** @brief Pins a_source as the hair target; false when it has no character. */
	bool PinHairTarget(SmpPhysics::HairTargetSource a_source) { return hairTarget.Pin(a_source); }
	/** @brief Lets the console selection set the hair target again. */
	void UnpinHairTarget() { hairTarget.Unpin(); }
	/** @brief Puts the entry a_index of the lists of a_item (CrowdWigs::WigForms(), TestCrowd::ArmorForms(), CloakForms()) on the target alone; runs on the game thread. */
	void WearOnTarget(SmpPhysics::TargetItem a_item, int a_index);
	/** @brief Takes a_item off the target and puts back what it wore; for a wig, shows its own hair; runs on the game thread. */
	void RestoreTarget(SmpPhysics::TargetItem a_item);
	/** @brief The entry of CrowdWigs::WigForms() worn last, or -1 for none. */
	[[nodiscard]] int WornWigIndex() const noexcept { return crowdWigs.WornIndex(); }
	/** @brief The rigid solver's island size summary, for the devbench islandSizes query. */
	[[nodiscard]] json IslandSizesJson() const;
	/** @brief The scan's armors and cloaks with their worst health, for the devbench physicsHealth query. Game thread only. */
	[[nodiscard]] json PhysicsHealthJson();
	/** @brief The worst health among each item's physics files, parsing any not yet read; None for an item with no files. Game thread only. */
	[[nodiscard]] std::vector<SmpPhysics::Health> ItemHealths(const std::vector<std::vector<std::string>>& a_itemFiles);
	/** @brief Each item's physics bones, summed over its files as written; 0 for an item with none. */
	[[nodiscard]] std::vector<int> ItemBoneCounts(const std::vector<std::vector<std::string>>& a_itemFiles);

	/** @brief Link state, live claims and declined physics files, for the devbench status query. */
	[[nodiscard]] json BuildStatusJson() const;
	/** @brief Per claim of the "actorFormID" arg (all claims when absent): each bone's solver pose in its kinematic root's frame, its reach limit and whether it is past it. */
	[[nodiscard]] json SolverBonesJson(const json& a_args) const;
	/** @brief The last frame's GPU collision-surface pass measured against the CPU reference, for the surfaceCheck query. */
	[[nodiscard]] json SurfaceCheckJson() const;
	/** @brief Reports, in the "reference" node frame, the pose of every actor node whose name starts with the "prefix" arg, whoever simulates it. */
	[[nodiscard]] static json SampleBonesJson(const json& a_args);
	/** @brief Queues shelter-style rays from an actor's head for the next frame, since Havok queries belong on the game thread. */
	json QueueWindProbe(const json& a_args);
	/** @brief The last windProbe's hits, or a pending marker until the game thread has run it. */
	[[nodiscard]] json WindProbeResult() const;
	/** @brief Runs a queued windProbe; called from StepClaims on the game thread. */
	void RunWindProbe();
	[[nodiscard]] static json SkinProbeJson(const json& a_args);
	/** @brief Compares the skinning matrices bound for a claimed mesh against the engine's own, bone by bone. Args: actorFormID, mesh. Asks for a fresh capture each call, so the first answer is empty. */
	[[nodiscard]] json SkinPaletteCheckJson(const json& a_args) const;
	/** @brief Measures how well the engine's matrices for the bones the solver does not simulate match ones rebuilt from the world poses recorded at the solver step, which a batched palette would have to use. Args: actorFormID, mesh, arm (true starts a measurement, false reads it). */
	[[nodiscard]] json KinematicCheckJson(const json& a_args) const;
	/** @brief Reports, on demand, the collision proxies this feature built for an actor and how deep its hair mesh sinks into a target mesh. Args: actorFormID, optionally with hairMesh and targetMesh together. */
	[[nodiscard]] json CollisionAuditJson(const json& a_args) const;
	/** @brief Moves a test character so a strand of its hair lies in the cast HIGGS makes from a player hand's palm, for grab tests with injected controllers. */
	void PlaceCrowdHairAtHand(const json& a_args);
	/** @brief Arms a per-frame capture of the solver's poses, taken right after the claims are stepped. Args: actorFormID, prefix, frames. */
	void StartBoneRecording(const json& a_args);
	/** @brief The frames captured so far by StartBoneRecording. */
	[[nodiscard]] json RecordedBonesJson() const;

private:
	/** @brief Takes a mesh over from hdtSMP64, or declines so that hdtSMP64 keeps simulating it. */
	class MeshOwner final : public hdt::IMeshOwner
	{
	public:
		explicit MeshOwner(SmpPhysicsFeature& a_feature) noexcept :
			feature(a_feature) {}

		bool claim(const hdt::IMeshClaimContext& a_context, std::uint64_t a_token) override;
		void release(std::uint64_t a_token) override;

	private:
		SmpPhysicsFeature& feature;
	};

	/** @brief Latches whether hdtSMP64 has the game paused. */
	class FrameListener final : public hdt::IFrameListener
	{
	public:
		explicit FrameListener(SmpPhysicsFeature& a_feature) noexcept :
			feature(a_feature) {}

		RE::BSEventNotifyControl ProcessEvent(const hdt::PluginFrameEvent* a_event, RE::BSTEventSource<hdt::PluginFrameEvent>*) override;

	private:
		SmpPhysicsFeature& feature;
	};

	/** @brief Steps the claims once hdtSMP64 has joined its simulation and written its own bone transforms. */
	class FrameSyncListener final : public hdt::IFrameSyncListener
	{
	public:
		explicit FrameSyncListener(SmpPhysicsFeature& a_feature) noexcept :
			feature(a_feature) {}

		RE::BSEventNotifyControl ProcessEvent(const hdt::PluginFrameSyncEvent*, RE::BSTEventSource<hdt::PluginFrameSyncEvent>*) override;

	private:
		SmpPhysicsFeature& feature;
	};

	/** @brief The collision proxies built for one claim, and how many of them had to fall back on a default. */
	struct ClaimCollision
	{
		/** @brief How one collider was measured, so the audit can tell a fitted proxy from one left at its default. */
		struct ColliderFit
		{
			int vertexCount{};  ///< skinned body vertices the measurement used
			bool fitted{};      ///< false when the collider kept a default radius
		};

		std::vector<std::vector<SmpPhysics::BoneSphere>> spheres;  ///< one list per rig bone
		std::vector<std::uint32_t> boneSurfaceMask;                ///< per rig bone, the first 32 surfaces its spheres may touch; empty when every bone may touch every surface
		std::size_t layerExempted{};                               ///< bone and surface pairs left out because the bone's spheres were authored behind that surface
		std::vector<SmpPhysics::Collider> colliders;
		std::vector<ColliderFit> colliderFits;               ///< one per collider, in the same order as colliders
		std::vector<SmpPhysics::SkinnedSurface> surfaces;    ///< the rig's own collision meshes, posed by the drivers
		std::vector<SmpPhysics::RigShape> bodyShapes;        ///< set when the rig has no hair shape: its triangle shapes, for other rigs on the actor
		std::vector<RE::NiPointer<RE::NiAVObject>> drivers;  ///< the nodes the colliders are expressed in
		std::vector<std::string> hairMeshNames;              ///< the meshes the rig's own shapes name, which are never body
		std::uint64_t bodySignature{};                       ///< hashes the worn meshes the colliders were measured from
		int sphereCount{};
		int exemptSpheres{};  ///< of sphereCount, the ones that start inside a collider
		int fitted{};
		int defaulted{};
		int retargetBehind{};    ///< spheres that settle inside a worn surface's shell when the authored rest sags under gravity
		int retargetResidual{};  ///< of those, the ones the joint budget could not free
		int retargetedJoints{};
		bool retargetKeptAuthored{};  ///< the retarget could not free enough of the hair, so the authored rest was kept
		float retargetMaxAngle{};
		float retargetMaxBoneMove{};
		float retargetSagMaxMove{};                  ///< how far gravity was predicted to carry a bone from its rest, see RestRetarget.h
		std::vector<SmpPhysics::Transform> sagPose;  ///< where the rest is predicted to settle under gravity, per rig bone
	};

	/** @brief One mesh this feature simulates, with the state rebuilt every frame from its scene-graph nodes. */
	struct Claim
	{
		std::uint64_t token{};
		RE::FormID actorFormID{};  ///< Zero when hdtSMP64 handed over a mesh with no actor.
		std::string physicsFile;
		std::shared_ptr<const SmpPhysics::ParseResult> parsed;
		std::vector<RE::NiPointer<RE::NiAVObject>> nodes;             ///< One per rig bone, owned past the claim call.
		std::vector<RE::NiPointer<RE::NiSkinInstance>> skins;         ///< The skin instances of the claimed model that use at least one rig bone.
		std::vector<RE::NiPointer<RE::NiAVObject>> cardGeometries;    ///< The geometry of each entry of skins, in the same order.
		bool skinsBound{};                                            ///< Whether those skin instances were handed to the GPU skinning pass.
		std::vector<SmpPhysics::Transform> kinematicInput;            ///< Rebuilt in place each frame; dynamic entries stay the identity.
		std::vector<SmpPhysics::Transform> previousKinematic;         ///< Last frame's kinematic entries, to detect a teleport.
		std::vector<SmpPhysics::Transform> restPose;                  ///< What Initialize adopted, so the GPU solver can be given the same rest pose.
		std::vector<int> treeKinematicParent;                         ///< Per bone, the kinematic bone the bone's tree hangs from.
		std::vector<RE::NiPointer<RE::NiAVObject>> collisionDrivers;  ///< One per collider driverIndex; rebuilt only at claim time.
		std::vector<SmpPhysics::Collider> colliders;                  ///< The proxies the solver was given, for the audit query.
		std::vector<ClaimCollision::ColliderFit> colliderFits;        ///< How each of them measured, in the same order.
		std::vector<SmpPhysics::SkinnedSurface> surfaces;             ///< The rig's own collision meshes, which boneDriver indexes into collisionDrivers.
		std::vector<std::string> hairMeshNames;                       ///< The meshes the rig's own shapes name, which are never body.
		std::uint64_t bodySignature{};                                ///< Hashes the worn meshes the proxies were measured from.
		double refitCheckElapsed{};                                   ///< Seconds since the worn meshes were last hashed.
		std::vector<SmpPhysics::Transform> driverWorld;               ///< This frame's world transform of each driver.
		SmpPhysics::GpuRig::CollisionData gpuCollision;               ///< The same proxies packed for the compute shader.
		std::vector<std::vector<SmpPhysics::BoneSphere>> spheres;     ///< The collision spheres of each rig bone, in the bone's frame.
		SmpPhysics::SolverParams solverParams;                        ///< The sanitized scales and substep policy the GPU solver was registered with.
		bool collisionActive{};
		int retargetBehind{};    ///< Spheres the claim found settling inside a worn surface's shell under gravity.
		int retargetResidual{};  ///< Of those, the ones still inside once the joints' limit budget was spent.
		int retargetedJoints{};
		bool retargetKeptAuthored{};
		float retargetMaxAngle{};
		float retargetMaxBoneMove{};
		float retargetSagMaxMove{};                  ///< How far gravity was predicted to carry a bone from its rest.
		std::vector<SmpPhysics::Transform> sagPose;  ///< Where the rest was predicted to settle under gravity, so poseCheck can compare the solver with it.

		int externalDriver{ -1 };                                        ///< The world-frame entry of collisionDrivers the reserved proxies sit in, or -1 for a claim with none.
		std::vector<RE::FormID> externalOwners;                          ///< The actor each proxy of externalColliders belongs to.
		SmpPhysics::Vec3 externalCentre{};                               ///< Where this frame's proxies were gathered around.
		int externalCount{};                                             ///< Proxies in use this frame.
		std::vector<SmpPhysics::Collider> externalColliders;             ///< This frame's world-space proxies for other bodies.
		std::vector<SmpPhysics::Grab> grabs;                             ///< The points hands hold this frame, by rig bone, with where each hand wants them.
		int grabSides{ 0 };                                              ///< Bit 0 when the left hand holds a strand of this claim, bit 1 the right.
		int releasedSides{ 0 };                                          ///< Bit 0 when the left hand let go of this claim's hair and still touches it, bit 1 the right; that hand pushes nothing until it is clear.
		std::array<std::uint32_t, 2> releasedFrames{};                   ///< Per hand: frames since it let go, so a hand that never clears the hair is pushed in again.
		std::vector<SmpPhysics::GpuRig::ColliderPacked> externalPacked;  ///< The same, padded to the reserved slots for the compute shader.
		bool gpuRegistered{};
		bool gpuRegistrationFailed{};  ///< A rig the GPU backend cannot pack stays at the engine's pose for good.
		bool gpuResetRequested{};
		float lastCameraDistance{};       ///< Distance from the camera to the claim's first kinematic bone at the last check.
		bool headPart{};                  ///< Offered as a head part, which hdtSMP64 deactivates under a hair-slot item.
		bool cloth{};                     ///< Worn in a cloak-type slot, so the hair of the same actor collides with its mesh; see IsClothClaim.
		bool hideUnderWig{};              ///< hdtSMP64's disableSMPHairWhenWigEquipped when this was claimed.
		bool firstPersonOff{};            ///< hdtSMP64's disable1stPersonViewPhysics when this was claimed.
		bool inactive{};                  ///< hdtSMP64 would not simulate it this frame, so neither is it.
		bool culledByBudget{};            ///< Beyond the nearest MaxSimulatedClaims claims this frame, so it is drawn from the engine's pose.
		bool culledOffscreen{};           ///< Out of view for kOffscreenCullFrames frames and beyond the near range, so it is drawn from the engine's pose.
		std::uint32_t offscreenFrames{};  ///< Consecutive frames outside the view.
		bool culledSmall{};               ///< Smaller on screen than MinScreenSizePercent for kOffscreenCullFrames frames, so it is drawn from the engine's pose.
		std::uint32_t smallFrames{};      ///< Consecutive frames below the screen size limit.
		/** @return Whether the claim is skipped this frame for being over budget or out of view. */
		[[nodiscard]] bool culled() const noexcept { return culledByBudget || culledOffscreen || culledSmall; }
		std::uint32_t missingActorFrames{};       ///< Consecutive frames the claim's actor form did not exist.
		std::uint64_t steppedFrames{};            ///< Frames this claim was stepped.
		std::uint64_t skippedByDistanceFrames{};  ///< Frames skipped because the claim was beyond SimulationDistance.
		std::uint64_t skippedNotReadyFrames{};    ///< Frames skipped because the actor or its nodes were not loaded.
		bool gpuHasPrevious{};                    ///< Whether the GPU has been handed a kinematic input to interpolate from.
		/** @brief Where a bone is this frame: a kinematic bone's animated pose, or a simulated bone's solver pose carried by its tree's current root. */
		[[nodiscard]] SmpPhysics::Transform BoneWorld(std::size_t a_bone) const;
		/** @brief BoneWorld's position of every bone, empty where the bone has no node. */
		[[nodiscard]] std::vector<std::optional<SmpPhysics::Vec3>> BonePositions() const;
		std::vector<SmpPhysics::Transform> gpuLastBones;  ///< The last pose the GPU returned, held while the next readback is not ready.
		std::vector<float> boneReach;                     ///< Per rig bone, how far from its kinematic root the solver may place it; see GpuRig::BuildBoneReach.
		std::uint32_t divergenceRestarts{};               ///< Times a solver pose left boneReach and the claim was restarted.
		int firstKinematicBone{ -1 };
		int boneCount{};
		int dynamicBoneCount{};
		int sphereCount{};
		int exemptSphereCount{};  ///< of sphereCount, the ones that start inside a collider and ignore it
		int colliderCount{};
		int additionalJointCount{};  ///< Joints SMP applies that a single pivot per bone cannot.
		double stepMicroseconds{};
		SmpPhysics::Vec3 wind{};                                  ///< World wind velocity at the claim's head this frame, Skyrim units per second.
		std::vector<float> windAreas;                             ///< Per rig bone, the mesh surface it drives, or zero where none is measured.
		bool windMeasured{};                                      ///< Whether windAreas has been measured.
		float windFactor{ 1.0f };                                 ///< The actor's smoothed wind exposure.
		float windTarget{ 1.0f };                                 ///< What the last shelter tests said it should be: the world's share times the wearer's.
		float windWorldExposure{ 1.0f };                          ///< The world ray's share of it.
		float windHitDistance{ -1.0f };                           ///< How far upwind the world ray found something, or -1 for a clear ray.
		RE::FormID windHitReference{};                            ///< The reference that owns what it hit, or 0.
		std::array<float, 3> windRayHits{ -1.0f, -1.0f, -1.0f };  ///< Per WindShelter::kRays, how far that ray travelled, or negative when clear.
		float windSelfExposure{ 1.0f };                           ///< The wearer's own head, body and armour's share of it.
		float windProbeTimer{};                                   ///< Seconds until the next shelter test.
	};

	/** @brief Whether the feature is on and has room for the mesh; a refusal other than being off is recorded as a decline. */
	[[nodiscard]] bool PassesClaimGate(const hdt::IMeshClaimContext& a_context, const std::string& a_path);
	/** @brief Finds the skeleton node of every rig bone and checks the scene graph matches the rig's joints; a failure is recorded as a decline. */
	[[nodiscard]] bool ResolveRigNodes(const hdt::IMeshClaimContext& a_context, RE::NiNode* a_skeleton, const SmpPhysics::SmpRig& a_rig,
		const std::string& a_path, std::vector<RE::NiPointer<RE::NiAVObject>>& a_nodes);
	/** @brief Rebuilds every bone's rest pose, dynamic bones from their local transforms, and reports the node scale of the first dynamic bone; a failure is recorded as a decline. */
	[[nodiscard]] bool ResolveRestPose(const SmpPhysics::SmpRig& a_rig, const std::vector<RE::NiPointer<RE::NiAVObject>>& a_nodes,
		const std::string& a_path, std::vector<SmpPhysics::Transform>& a_restPose, std::size_t& a_dynamicCount, float& a_bodyScale);
	/** @brief Copies the wig and first-person rules hdtSMP64 is configured with onto the claim. */
	void ReadViewRules(Claim& a_claim);
	/** @brief Hands a claim its collision proxies, reserving the slots other bodies fill each frame. */
	void InstallClaimCollision(Claim& a_claim, ClaimCollision& a_collision);
	[[nodiscard]] bool ClaimMesh(const hdt::IMeshClaimContext& a_context, std::uint64_t a_token);
	/** @brief One entry per claim of the actor: sphere probes, the deepest penetrating spheres and the colliders. */
	[[nodiscard]] json DescribeAuditClaims(RE::FormID a_formID) const;
	struct CollisionBuildState;
	[[nodiscard]] bool ResolveCollisionInputs(CollisionBuildState& s, std::string& a_reason) const;
	[[nodiscard]] bool BuildHairSpheres(CollisionBuildState& s, std::string& a_reason) const;
	void BuildBodyProxies(CollisionBuildState& s) const;
	void BuildRigSurfaces(CollisionBuildState& s) const;
	void BuildArmorSurfaces(CollisionBuildState& s) const;
	/** @brief Adds a two-sided surface for each cloth mesh another rig simulates on the actor, which hair collides with whatever its tags say. */
	void BuildClothSurfaces(CollisionBuildState& s) const;
	/** @brief Whether a claimed mesh is worn as a cloak or cape: armor in a slot that is neither the body's nor one the hair surfaces skip. */
	[[nodiscard]] static bool IsClothClaim(const hdt::IMeshClaimContext& a_context);
	/** @brief Remembers the meshes a cloth claim simulates, which outlive the claim so a reload builds hair with them. Called with stateMutex held. */
	void RegisterClothMeshes(RE::FormID a_actorFormID, const std::vector<RE::NiPointer<RE::NiAVObject>>& a_geometries);
	/** @brief Folds the actor's remembered cloth mesh names into a body signature. Called with stateMutex held. */
	[[nodiscard]] std::uint64_t HashClothMeshes(RE::FormID a_actorFormID, std::uint64_t a_signature) const;
	/** @brief Turns the rest pose out of the worn surfaces it was authored inside, see RetargetRestToSurfaces; a_restPose is replaced when any joint turned. */
	void RetargetRest(CollisionBuildState& s, std::vector<SmpPhysics::Transform>& a_restPose) const;
	/** @brief Tallies the fits and marks the spheres authored inside a collider, once every proxy and surface is in. */
	void FinishCollision(CollisionBuildState& s) const;
	[[nodiscard]] bool BuildClaimCollision(const hdt::IMeshClaimContext& a_context, const SmpPhysics::SmpRig& a_rig,
		const std::vector<RE::NiPointer<RE::NiAVObject>>& a_boneNodes, std::vector<SmpPhysics::Transform>& a_restPose, ClaimCollision& a_collision, std::string& a_reason) const;
	/**
	 * @brief Whether a_claim's proxies no longer cover the meshes its actor is wearing.
	 * @details hdtSMP64 re-runs claiming for the mesh that was re-attached, not for this feature's other claims,
	 *          so an armour swap leaves the proxies measured against the old cuirass until the actor is reloaded.
	 * @return true once per change of the worn set, and never for a claim with no proxies or no actor.
	 * @note Called with stateMutex held, and it must not call into hdtSMP64.
	 */
	[[nodiscard]] bool WornMeshesChanged(Claim& a_claim, float a_frameDelta);
	/** @brief Every proxy a_claim holds, with what drives it and how it was measured. Called with stateMutex held. */
	[[nodiscard]] static json DescribeClaimColliders(const Claim& a_claim);
	void ReleaseMesh(std::uint64_t a_token);
	/** @brief Marks the claims that stay out of view and those beyond the nearest MaxSimulatedClaims as culled for this frame, measured from the camera the frame is rendered with; call before the claims are stepped. */
	void ApplyCulling();
	/** @brief A test capsule standing in for another body, hung below a claim's first kinematic bone; see the probeBodyContact command. */
	struct BodyContactProbe
	{
		bool active{};
		RE::FormID actorFormID{};
		SmpPhysics::Vec3 offset{};  ///< From the first kinematic bone to the capsule's top end.
		static constexpr float kDefaultLength = 40.0f;
		static constexpr float kDefaultRadius = 6.0f;
		static constexpr float kMaxExtent = 200.0f;  ///< Longest length, radius or offset a command may ask for.
		float length{ kDefaultLength };              ///< How far the capsule reaches straight down from there.
		float radius{ kDefaultRadius };
	};
	BodyContactProbe bodyContactProbe;

	/** @brief One strand a closed hand holds: the claim and bone, and the held point in the bone's frame and in the hand's. */
	struct HeldPin
	{
		std::uint64_t token{};
		int bone{ -1 };
		SmpPhysics::Vec3 local{};   ///< The held point in the bone's own frame.
		SmpPhysics::Vec3 anchor{};  ///< The held point in the hand's own frame.
	};
	std::array<float, 2> grabOverreachSeconds{};    ///< Per hand: how long it has been farther from a held strand's root than the strand is long. Guarded by stateMutex.
	std::array<float, 2> grabOverstretchSeconds{};  ///< Per hand: how long its held strands have been too far from it. Guarded by stateMutex.
	std::array<bool, 2> higgsHandDisabled{};        ///< Whether this feature has told HIGGS not to pick things up with a hand, left then right. Guarded by stateMutex.
	/** @brief Lets go of every held strand and gives HIGGS its hands back. Called with stateMutex held. */
	void ReleaseGrabState();
	std::array<std::vector<HeldPin>, 2> heldPins;  ///< Left, then right; empty while the hand holds nothing. Guarded by stateMutex.
	/** @brief What the grip detection saw for one hand last frame, for the status query, the menu and the log. */
	struct GripDiagnostic
	{
		bool handNodes{};                 ///< The player's hand skeleton nodes were found.
		bool closed{};                    ///< The hand's grip button is down.
		std::size_t inReach{};            ///< Strands within reach of the palm to fingertip line.
		float nearest{ -1.0f };           ///< Distance to the nearest of them, or -1 for none.
		SmpPhysics::PalmTarget target{};  ///< What the palm is pointed at: hair, another body part or nothing.
		bool dropped{};                   ///< The strands were let go because they could no longer follow the hand, and stay let go until the grip opens.
		std::size_t pins{};               ///< Strands the hand holds.
	};
	struct GripReport
	{
		bool higgs{};  ///< The HIGGS interface was found, so a held object can be told apart from a grip on hair.
		unsigned int higgsBuild{};
		std::array<GripDiagnostic, 2> hands;  ///< Left, then right.
	};
	GripReport gripReport;  ///< Guarded by stateMutex.
	[[nodiscard]] json GripReportJson() const;
	/** @brief A bone of a strand a hand is about to take or holds, for colouring its joints in the overlay. */
	struct DebugGrabBone
	{
		std::uint64_t token{};
		int bone{ -1 };
		bool held{};
	};
	std::vector<DebugGrabBone> debugGrabBones;                 ///< Rebuilt every frame by UpdateStrandGrabs. Guarded by stateMutex.
	std::vector<SmpPhysics::DebugSegment> debugReachSegments;  ///< The strands within reach of a hand this frame. Guarded by stateMutex.
	/** @brief Collects the lines the grab debug overlay draws from this frame's grabs, reach and hand contacts. Called with stateMutex held. */
	void UpdateGrabDebug();
	/** @brief Appends the skeletons of the actors near the camera, and the player's hand skeletons. */
	static void AppendSkeletonLines(std::vector<SmpPhysics::DebugSegment>& a_out, int a_maxActors);
	/** @brief Draws the meshes hdtSMP64 simulates. Called with stateMutex held. */
	void AppendDeclinedLines(std::vector<SmpPhysics::DebugSegment>& a_out);
	/** @brief Appends each of a claim's joints as a line from its parent bone to the bone, orange where its length is off its rest length. */
	static void AppendJointLines(const Claim& a_claim, std::vector<SmpPhysics::DebugSegment>& a_out, const std::vector<DebugGrabBone>& a_grabBones);
	/** @brief Appends the edges of a claim's collision meshes (worn armor, head) that lie within reach of its hair. */
	static void AppendSurfaceEdgesNearHair(const Claim& a_claim, std::vector<SmpPhysics::DebugSegment>& a_out);
	/** @brief Appends outlines of the proxies a claim's hair collides with; a_drawnExternal keeps a capsule shared by several claims from drawing twice, and a_detailed adds the collision mesh edges and sphere rings. */
	static void AppendColliderOutlines(const Claim& a_claim, std::vector<SmpPhysics::DebugSegment>& a_out,
		std::vector<std::pair<SmpPhysics::Vec3, SmpPhysics::Vec3>>& a_drawnExternal, bool a_detailed, const std::vector<std::uint32_t>& a_shownActors);

	/** @brief Starts, keeps and ends the grabs for this frame and tells each claim what its hand holds. Called with stateMutex held. */
	void UpdateStrandGrabs(float a_frameDelta);
	[[nodiscard]] json HeldStrandsJson() const;
	/** @brief Per claim, how far the simulated hair is from where the animation alone would put it, for checking that the overlays and the grab see the solver's pose. */
	[[nodiscard]] json PoseCheckJson(const json& a_args) const;

	/** @brief A strand held without a hand, for checking the pin from devbench; see the holdStrand command. */
	struct StrandProbe
	{
		bool active{};
		std::uint64_t token{};
		std::vector<SmpPhysics::Grab> grabs;
	};
	StrandProbe strandProbe;  ///< Guarded by stateMutex.
	void SetStrandProbe(const json& a_args);
	/** @brief Picks the nearby body capsules a claim's hair collides with this frame and writes them into the claim. Called with stateMutex held. */
	void UpdateExternalColliders(Claim& a_claim);
	/** @brief Places a stand-in capsule beside an actor so the hand contact path can be checked without a hand. Args: enabled, actorFormID, length, radius, offset. */
	void SetBodyContactProbe(const json& a_args);
	/** @brief Sends a claim back to the pose the engine animates and has the next GPU frame restart it from rest. */
	void RestartClaim(Claim& a_claim);
	/** @brief Whether the claim's actor and kinematic nodes are loaded well enough to step it. */
	[[nodiscard]] bool ClaimReady(Claim& a_claim, const RE::Actor* a_actor);
	/** @brief Applies the wig and first-person rules to a claim; true while it is inactive. */
	[[nodiscard]] bool UpdateClaimInactive(Claim& a_claim, const RE::Actor* a_actor);
	[[nodiscard]] bool ClaimInRange(Claim& a_claim, const RE::NiPoint3& a_cameraPosition, bool a_haveCameraPosition);
	void GatherSolverInput(Claim& a_claim);
	[[nodiscard]] bool RootTeleported(const Claim& a_claim) const;
	void SubmitGpuFrame(Claim& a_claim, float a_frameDelta, std::vector<SmpPhysics::GpuSolver::FrameClaim>& a_submissions);
	/** @brief Writes the last pose the GPU returned into the claim's simulated nodes. */
	void WriteBackPose(Claim& a_claim);
	bool StepClaim(Claim& a_claim, const RE::NiPoint3& a_cameraPosition, bool a_haveCameraPosition, float a_frameDelta,
		const SmpPhysics::GpuSolver::ClaimResult* a_gpuResult, std::vector<SmpPhysics::GpuSolver::FrameClaim>& a_submissions);
	/** @brief Registers a_claim with the GPU solver at its rest pose. */
	[[nodiscard]] bool RegisterClaimForGpu(Claim& a_claim);
	/** @brief Maps a_claim's skin instances onto its solver slots and registers them with the skinning pass. */
	void BindClaimSkins(Claim& a_claim);
	void StepClaims();
	/** @brief Advances the flutter clock and samples the Wind feature's field at each claim's head, outdoors only. */
	void UpdateWind(float a_frameDelta);
	/** @brief Eases a_claim's exposure toward what a ray from its head against the wind says. */
	void UpdateClaimWindExposure(Claim& a_claim, float a_frameDelta);
	/** @brief Measures the mesh surface each of a_claim's bones drives, for the wind drag. */
	void MeasureWindAreas(Claim& a_claim);
	void RecordBoneFrame();
	/** @brief Adds the angles of the recording's claim's simulated joints, in the frames the solver limits them in, to a recorded frame. */
	void RecordJointFrame(json& a_frame);
	/** @brief Reuses a cached parse for a_path, or reads and parses the file once. */
	[[nodiscard]] std::shared_ptr<const SmpPhysics::ParseResult> GetParsedRig(const std::string& a_path);
	/** @brief Counts the offer ClaimMesh just answered, with the reason RecordDecline noted if it was declined. */
	void TallyClaim(const hdt::IMeshClaimContext& a_context, bool a_accepted);
	/** @brief Records a declined physics file and logs it the first time that path and reason are seen. */
	void RecordDecline(const std::string& a_file, const std::string& a_reason);
	/** @brief Remembers a declined rig's triangle shapes for the actor, so its hair rigs collide with them; asks for the actor's hair to be offered again when one is new. */
	void RegisterBodyShapes(const hdt::IMeshClaimContext& a_context, const std::string& a_file, const std::vector<SmpPhysics::RigShape>& a_shapes);
	void ReevaluateActor(RE::Actor* a_actor);

	float windClock{};     ///< Flutter clock, seconds.
	float windCooldown{};  ///< Seconds until the outdoor check is repeated.
	bool windOutdoors{};   ///< Whether the player stands in an exterior worldspace.

	std::atomic<bool> linked{ false };  ///< Read on the frame path, so it stays outside the mutex.
	std::atomic<bool> gamePaused{ false };

	SmpPhysics::GpuSolver gpuSolver;
	mutable SmpPhysics::DebugLines debugLines;    ///< Has its own lock, because the render thread draws it while the frame sync thread fills it.
	mutable SmpPhysics::GpuSkinning gpuSkinning;  ///< Has its own lock, because the render thread reads it while the frame sync thread registers.

	struct WindProbeRequest
	{
		RE::FormID actor{};
		std::vector<float> elevations;                                    ///< Degrees above the horizontal; negative aims down.
		float azimuthDegrees{ std::numeric_limits<float>::quiet_NaN() };  ///< World heading of the rays, or NaN for toward the wind.
		float reach{ 3000.0f };
	};
	mutable std::mutex windProbeMutex;
	std::optional<WindProbeRequest> windProbeRequest;
	json windProbeResult = json::object();
	mutable std::mutex stateMutex;  ///< Guards everything below, and nothing is called into hdtSMP64 while it is held.
	std::string linkStatus;
	hdt::PluginInterface* linkedInterface{};
	hdt::PluginInterface* seenInterface{};  ///< Keeps a repeated startup message from relinking or re-warning.
	hdt::PluginInterface::Version interfaceVersion{};
	std::unordered_map<std::uint64_t, Claim> claims;
	std::unordered_map<std::string, std::shared_ptr<const SmpPhysics::ParseResult>> parsedRigs;
	std::unordered_map<std::string, std::string> declinedFiles;
	std::unordered_map<std::string, SmpPhysics::ClaimTrim> claimTrims;  ///< By normalized file path: how many of its bones its claims kept. Guarded by stateMutex.
	SmpPhysics::ClaimTally claimTally;
	std::vector<SmpPhysics::CollisionOverride> collisionOverrides;          ///< Debugging switches applied to each claim as it is built, guarded by stateMutex.
	SmpPhysics::RepairLog repairLog;                                        ///< What was wrong in each physics file and what was done about it.
	bool claimsHandedBack{};                                                ///< Set once the claims were handed back to hdtSMP64 after Enabled went off, so a disable from any path releases them. Frame-sync thread only.
	std::map<std::pair<RE::FormID, std::string>, ClaimRow> disabledClaims;  ///< Actor and lower-case physics file pairs the user switched off, with the row the table keeps showing, guarded by stateMutex.
	SmpPhysics::ClaimOverrides keepRules;                                   ///< Saved rules that keep a physics file with hdtSMP64, guarded by stateMutex.
	std::set<RE::FormID> overrideBlockedActors;                             ///< Actors whose meshes a saved rule declined, so removing the rule can offer them again; guarded by stateMutex.
	/** @brief Asks hdtSMP64 to offer these actors' meshes again; a no-op while it is not linked. */
	void ReloadActors(const std::set<RE::FormID>& a_actorFormIDs);
	/** @brief A triangle shape from a rig that is not a hair rig, such as the body collision mesh, which hair rigs on the same actor collide with. */
	struct BodyShape
	{
		std::string file;
		SmpPhysics::RigShape shape;
	};
	std::unordered_map<RE::FormID, std::vector<BodyShape>> bodyShapes;
	std::unordered_map<RE::FormID, std::vector<std::string>> clothMeshNames;  ///< Per actor, the mesh names its cloth claims simulated, sorted; guarded by stateMutex.
	std::vector<RE::FormID> bodyShapeActors;                                  ///< actors whose hair must be offered again because a body shape appeared
	double totalStepMicroseconds{};
	std::uint32_t registrationsThisFrame{};  ///< Claims registered with the GPU solver so far this frame, capped at kRegistrationsPerFrame.

	struct BoneRecording
	{
		bool active{};
		RE::FormID actorFormID{};
		std::string prefix;
		int framesRemaining{};
		double elapsedSeconds{};
		bool world{};
		bool joints{};  ///< Also record each simulated joint's angles of the claim whose file contains jointFilter.
		std::string jointFilter;
		std::vector<std::pair<std::size_t, SmpPhysics::JointFrames>> jointFrames;  ///< Per recorded joint, its joint index in the claim's rig and its frames.
		std::uint64_t jointToken{};
		json jointInfo;
		std::vector<std::string> extraNodes;
		json names = json::array();
		json frames = json::array();
	};
	mutable std::mutex recordingMutex;
	BoneRecording recording;

	MeshOwner meshOwner{ *this };
	FrameListener frameListener{ *this };
	FrameSyncListener frameSyncListener{ *this };
	SmpPhysics::CrowdWigs crowdWigs;  ///< What this feature adds to the test crowd.
	SmpPhysics::HairTarget hairTarget;
};
