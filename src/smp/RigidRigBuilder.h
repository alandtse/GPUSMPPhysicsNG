#pragma once

#include "smp/RigidReference.h"
#include "smp/SmpRig.h"
#include "smp/SolverCommon.h"

#include <vector>

namespace SmpPhysics
{
	/**
	 * @brief The stop CFM hdtSMP64 gives every angular constraint axis when its file does not set it.
	 * @details SMP writes the angular motors' stop CFM directly while its linear axes keep Bullet's zero, which is
	 *          what every recorded trace header carries.
	 */
	inline constexpr float kRigidAngularStopCfm = 0.001f;

	/**
	 * @brief The rigid world hdtSMP64 would build for a rig: body i is rig bone i, one constraint per joint.
	 * @param a_restWorld Each bone's world transform at rest; dynamic bodies start there with no velocity.
	 * @param a_scale The actor's node scale. SMP rebuilds each dynamic body's mass as m * s^3 and its inertia as
	 *                I * s^5, and scales each constraint's frame origins and linear limits by s and its angular
	 *                spring stiffness by s^5.
	 * @param a_params Supplies the gravity every body is weighted by its own gravity factor for.
	 * @details Spring limits are the file's raw values: Bullet caps them inside the constraint, which the
	 *          reference and the compute solver both do.
	 */
	[[nodiscard]] RigidReference::World BuildRigidWorld(const SmpRig& a_rig, const std::vector<Transform>& a_restWorld,
		float a_scale, const SolverParams& a_params);

	/**
	 * @brief Each bone's rigid-body frame given its node's world transform: a dynamic bone's node composed with its
	 *        centerOfMassTransform, which is how hdtSMP64 places the body; a kinematic bone keeps its node's frame.
	 * @param a_scale The actor's node scale; hdtSMP64 scales the transform's origin by it.
	 */
	[[nodiscard]] std::vector<Transform> BodyFrames(const SmpRig& a_rig, const std::vector<Transform>& a_nodeWorld, float a_scale);

	/** @brief Each bone's node frame relative to its body frame (the inverse of its centerOfMassTransform); empty when every bone is identity. */
	[[nodiscard]] std::vector<Transform> NodeFromBody(const SmpRig& a_rig, float a_scale);
}
