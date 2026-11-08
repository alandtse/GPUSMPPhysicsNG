#pragma once

#include "Collision.h"
#include "SkinnedSurface.h"
#include "SmpRig.h"

#include <vector>

namespace SmpPhysics
{
	/** @brief How far out from the body axis, in Skyrim units, the retarget looks for the armor face a sphere would exit through. */
	inline constexpr float kRetargetReach = 10.0f;
	/** @brief How far in front of the face a retargeted sphere's centre is placed, so the runtime contact holds it there. */
	inline constexpr float kRetargetClearance = 0.2f;
	/** @brief Root-to-tip sweeps the retarget makes; a joint near the tip only sees what the sweep before left inside. */
	inline constexpr int kRetargetPasses = 3;
	/** @brief How far, in radians, a swing axis may be turned past its authored limit: the limits shape the hair on the bare body, not on what is worn over it. */
	inline constexpr float kRetargetMaxSwing = 0.6f;
	/** @brief How much a chain's settled segment must hang down, 1 straight down and 0 level, for the retarget to keep it. */
	inline constexpr float kRetargetMinimumHang = 0.5f;
	/** @brief How much less than its authored segment a retargeted one may hang before the whole chain keeps the authored rest. */
	inline constexpr float kRetargetMaxHangLoss = 0.3f;
	/** @brief The share of the spheres inside the armor the retarget must free, or the authored rest is kept: hair half in and half standing out looks worse than hair inside. */
	inline constexpr float kRetargetMinimumFreed = 0.67f;

	/** @brief The share of its springs' stiffness a cape keeps while the drape lets gravity settle it, so the body, not the authored rest, decides how it hangs. */
	inline constexpr float kDrapeSpringScale = 0.05f;

	/** @brief The gravity and spring scale the solver will run the rig with, so the retarget can predict where it settles. */
	struct RetargetLoad
	{
		float gravity{};  ///< Skyrim units per second squared, already scaled by the gravity setting
		float stiffnessScale{ 1.0f };
	};

	/** @brief What RetargetRestToSurfaces produced: the rest pose to simulate from, and the joint angles it spent to get there. */
	struct RestRetarget
	{
		std::vector<Transform> restPose;  ///< one per rig bone, kinematic bones as given
		std::vector<Transform> sagPose;   ///< where restPose is predicted to settle under gravity, for comparing with the solver
		std::vector<Vec3> angleOffsets;   ///< one per rig bone: the retargeted rest's joint angles in the authored joint frame, zero for an untouched or kinematic bone
		int behindSpheres{};              ///< spheres that settle inside a worn shell when the authored rest sags under gravity
		int residualSpheres{};            ///< spheres still inside one once every joint's budget was spent
		int retargetedJoints{};
		int revertedChains{};  ///< chains that kept the authored rest because the retarget left a segment hanging far less than authored
		bool keptAuthored{};   ///< the retarget could not free enough of the hair, so the authored rest was kept
		float maxAngle{};      ///< largest angle any joint turned, in radians
		float maxBoneMove{};   ///< farthest any bone's rest moved from where the rig authored it, in Skyrim units
		float sagMaxMove{};    ///< farthest gravity is predicted to carry any bone from its rest, in Skyrim units
	};

	/**
	 * @brief Where the solver's springs settle a rest pose under gravity, joint by joint, within the joint limits and
	 *        resting on the body.
	 * @details Mirrors the solver's static balance: each joint's spring holds its equilibrium against the gravity torque on
	 *          its own bone, so the angle it settles at is the equilibrium plus that torque over the spring's stiffness,
	 *          clamped to the joint's limits. A joint without a spring falls to the limit gravity pulls it toward. A bone
	 *          whose spheres would sink into a surface that reaches deep behind its faces, or into the skin, deeper than
	 *          they rest already, measured out from the body axis through a_axisPoint, sags only as far as that surface
	 *          lets it, as the runtime contact would hold it.
	 * @param a_restPose The pose the joint frames are resolved from, one transform per rig bone.
	 * @param a_spheres The hair's collision spheres per rig bone, in each bone's own frame.
	 * @param a_driverWorld The driver transforms the surfaces are posed with.
	 */
	[[nodiscard]] std::vector<Transform> PredictGravitySag(const SmpRig& a_rig, const std::vector<Transform>& a_restPose, const std::vector<std::vector<BoneSphere>>& a_spheres,
		const std::vector<SkinnedSurface>& a_surfaces, const std::vector<Transform>& a_driverWorld, const Vec3& a_axisPoint, const RetargetLoad& a_load);

	/**
	 * @brief Turns the joints of a cape or cloak, authored to hang over armor, in toward the body it hangs in front of.
	 * @details Gravity pulls the cape down with its springs nearly slack (kDrapeSpringScale), so it comes to rest against the body
	 *          and worn skin, within the joint limits; the rest is then moved so that, with the springs at full stiffness,
	 *          gravity settles it at that pose. The one-sided armor shells are not used.
	 * @return The same result as RetargetRestToSurfaces; behindSpheres and residualSpheres stay zero.
	 */
	[[nodiscard]] RestRetarget DrapeRestOntoSupports(const SmpRig& a_rig, const std::vector<Transform>& a_restPose,
		const std::vector<std::vector<BoneSphere>>& a_spheres, const std::vector<SkinnedSurface>& a_surfaces,
		const std::vector<Transform>& a_driverWorld, const Vec3& a_axisPoint, const RetargetLoad& a_load);

	/**
	 * @brief Turns the joints of hair authored inside a worn surface until it settles just outside that surface.
	 * @details Hair is modelled on a bare body, so where it settles runs through whatever the character wears over that
	 *          body. A joint spring's equilibrium is the rest pose, so no contact can hold the hair out at run time
	 *          without fighting the spring every frame. This instead moves the rest: the pose the hair sags to under
	 *          gravity (PredictGravitySag) is turned, roots first and by up to kRetargetMaxSwing past the rig's own limits,
	 *          until the spheres below each joint are no longer inside a shell as TriangleGrid::RadialDepthInside measures
	 *          it, out from the body axis through a_axisPoint; the joints below the plate are then turned back toward the
	 *          way they hung, as far as the hair stays clear, so it drapes over the plate rather than standing out from
	 *          it; and the rest pose is moved by the same joint angles so the sagged hair lands there. Only one-sided surfaces that do not reach deep behind their faces take part, and the
	 *          skin is never retargeted against, so a bare body leaves everything untouched. Pure geometry: nothing is
	 *          simulated.
	 * @param a_restPose The authored rest pose, one transform per rig bone.
	 * @param a_spheres The hair's collision spheres per rig bone, in each bone's own frame.
	 * @param a_driverWorld The driver transforms the surfaces are posed with at claim time.
	 * @param a_axisPoint A point on the body's axis, such as the hair anchor, that the depth is measured out from.
	 */
	[[nodiscard]] RestRetarget RetargetRestToSurfaces(const SmpRig& a_rig, const std::vector<Transform>& a_restPose,
		const std::vector<std::vector<BoneSphere>>& a_spheres, const std::vector<SkinnedSurface>& a_surfaces,
		const std::vector<Transform>& a_driverWorld, const Vec3& a_axisPoint, const RetargetLoad& a_load);
}
