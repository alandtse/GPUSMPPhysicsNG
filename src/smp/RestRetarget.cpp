#include "RestRetarget.h"

#include "TriangleGrid.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>

namespace SmpPhysics
{
	namespace
	{
		constexpr float kMinimumLever = 0.5f;
		constexpr float kMinimumTurn = 1.0e-4f;
		constexpr float kUnlimitedTurn = 0.5f;       ///< what a joint with its limits switched off may turn per sweep
		constexpr float kTightAxisFraction = 0.25f;  ///< an axis allowed less than this share of the widest limit is a twist axis, not a swing axis
		constexpr int kSagIterations = 6;            ///< the gravity torque depends on the settled orientation, so the balance is found by iteration
		constexpr int kSupportBisections = 8;        ///< halvings of a bone's sag to find where it comes to rest on a support
		constexpr int kHangBisections = 8;           ///< halvings of a joint's turn back toward its settled direction to find where its hair stays clear
		constexpr float kSupportSlack = 1.0e-3f;

		struct Deficit
		{
			float amount{};  ///< how far the sphere must move out from the body axis to leave the shell
			Vec3 normal{};
			Vec3 centre{};
		};

		struct RigTopology
		{
			std::vector<int> jointOfBone;
			std::vector<int> order;  ///< dynamic bones, parents first
			std::vector<JointFrames> frames;
		};

		RigTopology ResolveTopology(const SmpRig& a_rig, const std::vector<Transform>& a_restPose)
		{
			RigTopology topology;
			const std::size_t boneCount = a_rig.bones.size();
			topology.jointOfBone.assign(boneCount, -1);
			for (std::size_t index = 0; index < a_rig.joints.size(); ++index) {
				const int child = a_rig.joints[index].child;
				if (child >= 0 && static_cast<std::size_t>(child) < boneCount)
					topology.jointOfBone[static_cast<std::size_t>(child)] = static_cast<int>(index);
			}
			topology.order = BuildParentFirstOrder(a_rig, topology.jointOfBone);
			topology.frames.resize(boneCount);
			for (const int bone : topology.order) {
				const RigJoint& joint = a_rig.joints[static_cast<std::size_t>(topology.jointOfBone[static_cast<std::size_t>(bone)])];
				topology.frames[static_cast<std::size_t>(bone)] = ResolveJointFrames(joint, a_restPose[static_cast<std::size_t>(bone)], a_restPose[static_cast<std::size_t>(joint.parent)]);
			}
			return topology;
		}

		Quat JointRotation(const Transform& a_parentPose, const JointFrames& a_frames) noexcept
		{
			return a_parentPose.rotation * a_frames.frameB.rotation;
		}

		Vec3 Pivot(const Transform& a_parentPose, const JointFrames& a_frames) noexcept
		{
			return a_parentPose.translation + a_parentPose.rotation.Rotate(a_frames.frameB.translation);
		}

		/** @brief The bone pose a joint at a_angles produces, as the solver's PoseFor composes it. */
		Transform PoseFromAngles(const Quat& a_jointRotation, const JointFrames& a_frames, const Vec3& a_pivot, const Vec3& a_angles) noexcept
		{
			Transform pose;
			pose.rotation = (a_jointRotation * FromEulerXYZ(a_angles) * Conjugate(a_frames.frameA.rotation)).Normalized();
			pose.translation = a_pivot - pose.rotation.Rotate(a_frames.frameA.translation);
			return pose;
		}

		Vec3 JointAngles(const Transform& a_parentPose, const Transform& a_pose, const JointFrames& a_frames) noexcept
		{
			return ToEulerXYZ(Conjugate(JointRotation(a_parentPose, a_frames)) * a_pose.rotation * a_frames.frameA.rotation);
		}

		/** @brief The angles a bone settles at under gravity when its parent is at a_parentPose: the solver's static balance per axis. */
		Vec3 SettleUnderGravity(const RigBone& a_bone, const RigJoint& a_joint, const JointFrames& a_frames, const Transform& a_parentPose, const RetargetLoad& a_load) noexcept
		{
			const Quat jointRotation = JointRotation(a_parentPose, a_frames);
			const Vec3 pivot = Pivot(a_parentPose, a_frames);
			const float lower[3] = { a_joint.angularLower.x, a_joint.angularLower.y, a_joint.angularLower.z };
			const float upper[3] = { a_joint.angularUpper.x, a_joint.angularUpper.y, a_joint.angularUpper.z };
			const float equilibrium[3] = { a_joint.angularEquilibrium.x, a_joint.angularEquilibrium.y, a_joint.angularEquilibrium.z };
			const float stiffness[3] = { a_joint.angularStiffness.x, a_joint.angularStiffness.y, a_joint.angularStiffness.z };
			const Vec3 weight{ 0.0f, 0.0f, -a_load.gravity * a_bone.mass * a_bone.gravityFactor };
			Vec3 settled = a_joint.angularEquilibrium;
			for (int iteration = 0; iteration < kSagIterations; ++iteration) {
				const Transform pose = PoseFromAngles(jointRotation, a_frames, pivot, settled);
				const Vec3 torque = Conjugate(jointRotation).Rotate(Cross(pose.translation - pivot, weight));
				const float torqueAxis[3] = { torque.x, torque.y, torque.z };
				const float current[3] = { settled.x, settled.y, settled.z };
				// The torque falls as the bone swings toward the vertical, so the step toward the balance is damped by
				// that slope, the lever times the weight, or the iteration overshoots and converges slowly.
				const float slope = Length(pose.translation - pivot) * std::fabs(weight.z);
				float angle[3];
				for (int axis = 0; axis < 3; ++axis) {
					const bool limited = lower[axis] <= upper[axis];
					const float spring = a_joint.angularSpringsEnabled ? stiffness[axis] * a_load.stiffnessScale : 0.0f;
					if (spring > 0.0f)
						angle[axis] = current[axis] + (equilibrium[axis] + torqueAxis[axis] / spring - current[axis]) * spring / (spring + slope);
					else if (limited && torqueAxis[axis] != 0.0f)
						angle[axis] = torqueAxis[axis] > 0.0f ? upper[axis] : lower[axis];
					else
						angle[axis] = equilibrium[axis];
					if (limited)
						angle[axis] = std::clamp(angle[axis], lower[axis], upper[axis]);
				}
				settled = { angle[0], angle[1], angle[2] };
			}
			return settled;
		}

		/** @brief The surfaces the hair rests on while it sags: the body under the hair, and any worn mesh that stands in for it. */
		std::vector<TriangleGrid> BuildSupports(const std::vector<SkinnedSurface>& a_surfaces, const std::vector<Transform>& a_driverWorld)
		{
			std::vector<TriangleGrid> supports;
			for (const SkinnedSurface& surface : a_surfaces) {
				if ((surface.deepPush || surface.skin) && !surface.indices.empty())
					supports.emplace_back().Build(PoseSurface(surface, a_driverWorld), surface.indices, kSurfaceCellSize);
			}
			return supports;
		}

		/** @brief How deep inside a support the deepest of a bone's spheres sits, out from the body axis; zero when all are outside. */
		float Immersion(const Transform& a_pose, const std::vector<BoneSphere>& a_spheres, const std::vector<TriangleGrid>& a_supports, const Vec3& a_axisPoint) noexcept
		{
			float deepest = 0.0f;
			for (const BoneSphere& sphere : a_spheres) {
				const Vec3 centre = a_pose.TransformPoint(sphere.centerLocal);
				for (const TriangleGrid& support : a_supports) {
					const float depth = support.RadialDepthInside(centre, kRetargetReach, 0.0f, a_axisPoint);
					if (std::isfinite(depth))
						deepest = std::max(deepest, -depth);
				}
			}
			return deepest;
		}

		struct Sag
		{
			std::vector<Transform> pose;
			float maxMove{};
		};

		/** @brief Sags each joint in turn, holding a bone's sag back where its spheres would sink into a support deeper than they rest already. */
		Sag SagUnderGravity(const SmpRig& a_rig, const std::vector<Transform>& a_restPose, const RigTopology& a_topology, const RetargetLoad& a_load,
			const std::vector<std::vector<BoneSphere>>& a_spheres, const std::vector<TriangleGrid>& a_supports, const Vec3& a_axisPoint)
		{
			Sag sag;
			sag.pose = a_restPose;
			for (const int bone : a_topology.order) {
				const std::size_t index = static_cast<std::size_t>(bone);
				const RigJoint& joint = a_rig.joints[static_cast<std::size_t>(a_topology.jointOfBone[index])];
				const Transform& parentPose = sag.pose[static_cast<std::size_t>(joint.parent)];
				const JointFrames& frames = a_topology.frames[index];
				const Quat jointRotation = JointRotation(parentPose, frames);
				const Vec3 pivot = Pivot(parentPose, frames);
				const Vec3 settled = SettleUnderGravity(a_rig.bones[index], joint, frames, parentPose, a_load);
				const auto poseAt = [&](float a_fraction) { return PoseFromAngles(jointRotation, frames, pivot, settled * a_fraction); };
				float fraction = 1.0f;
				if (!a_supports.empty() && index < a_spheres.size() && !a_spheres[index].empty()) {
					const float resting = Immersion(poseAt(0.0f), a_spheres[index], a_supports, a_axisPoint) + kSupportSlack;
					if (Immersion(poseAt(1.0f), a_spheres[index], a_supports, a_axisPoint) > resting) {
						float clear = 0.0f;
						float sunk = 1.0f;
						for (int step = 0; step < kSupportBisections; ++step) {
							const float middle = 0.5f * (clear + sunk);
							(Immersion(poseAt(middle), a_spheres[index], a_supports, a_axisPoint) > resting ? sunk : clear) = middle;
						}
						fraction = clear;
					}
				}
				sag.pose[index] = poseAt(fraction);
				sag.maxMove = std::max(sag.maxMove, Length(sag.pose[index].translation - a_restPose[index].translation));
			}
			return sag;
		}

		int CountInsideShell(const std::vector<int>& a_bones, const std::vector<Transform>& a_pose, const std::vector<std::vector<BoneSphere>>& a_spheres,
			const std::vector<TriangleGrid>& a_grids, const std::vector<float>& a_allowances, const Vec3& a_axisPoint)
		{
			int behind = 0;
			for (const int bone : a_bones) {
				const std::size_t index = static_cast<std::size_t>(bone);
				if (index >= a_spheres.size())
					continue;
				for (const BoneSphere& sphere : a_spheres[index]) {
					const Vec3 centre = a_pose[index].TransformPoint(sphere.centerLocal);
					for (std::size_t grid = 0; grid < a_grids.size(); ++grid) {
						if (std::isfinite(a_grids[grid].RadialDepthInside(centre, kRetargetReach, a_allowances[grid], a_axisPoint))) {
							++behind;
							break;
						}
					}
				}
			}
			return behind;
		}

		/** @brief The farthest a sphere of a_bone's subtree still has to move to leave a shell, and which way. */
		Deficit WorstDeficit(const std::vector<int>& a_subtree, const std::vector<Transform>& a_rest,
			const std::vector<std::vector<BoneSphere>>& a_spheres, const std::vector<TriangleGrid>& a_grids,
			const std::vector<float>& a_allowances, const Vec3& a_axisPoint)
		{
			Deficit worst;
			for (const int bone : a_subtree) {
				const std::size_t index = static_cast<std::size_t>(bone);
				if (index >= a_spheres.size())
					continue;
				for (const BoneSphere& sphere : a_spheres[index]) {
					const Vec3 centre = a_rest[index].TransformPoint(sphere.centerLocal);
					for (std::size_t grid = 0; grid < a_grids.size(); ++grid) {
						Vec3 normal{};
						const float distance = a_grids[grid].RadialDepthInside(centre, kRetargetReach, a_allowances[grid], a_axisPoint, &normal);
						if (!std::isfinite(distance))
							continue;
						const float amount = kRetargetClearance - distance;
						if (amount > worst.amount)
							worst = { amount, normal, centre };
					}
				}
			}
			return worst;
		}

		/** @brief Drops the part of a joint-frame turn axis that lies along the joint's tight axes, so the turn uses the swing axes only. */
		Vec3 SwingOnly(Vec3 a_axis, const RigJoint& a_joint) noexcept
		{
			float range[3]{};
			const float lower[3] = { a_joint.angularLower.x, a_joint.angularLower.y, a_joint.angularLower.z };
			const float upper[3] = { a_joint.angularUpper.x, a_joint.angularUpper.y, a_joint.angularUpper.z };
			float widest = 0.0f;
			for (int axis = 0; axis < 3; ++axis) {
				range[axis] = lower[axis] <= upper[axis] ? upper[axis] - lower[axis] : 2.0f * kUnlimitedTurn;
				widest = std::max(widest, range[axis]);
			}
			float* component[3] = { &a_axis.x, &a_axis.y, &a_axis.z };
			for (int axis = 0; axis < 3; ++axis) {
				if (range[axis] < kTightAxisFraction * widest)
					*component[axis] = 0.0f;
			}
			return a_axis;
		}

		/** @brief How much of a turn that moved a joint's angles from a_before to a_after fits inside its limits widened by kRetargetMaxSwing, 1 for all of it. */
		float LimitScale(const Vec3& a_before, const Vec3& a_after, const RigJoint& a_joint) noexcept
		{
			float scale = 1.0f;
			const float before[3] = { a_before.x, a_before.y, a_before.z };
			const float after[3] = { a_after.x, a_after.y, a_after.z };
			const float lower[3] = { a_joint.angularLower.x, a_joint.angularLower.y, a_joint.angularLower.z };
			const float upper[3] = { a_joint.angularUpper.x, a_joint.angularUpper.y, a_joint.angularUpper.z };
			for (int axis = 0; axis < 3; ++axis) {
				const bool limited = lower[axis] <= upper[axis];
				const float low = limited ? std::min(lower[axis], -kRetargetMaxSwing) : -kUnlimitedTurn;
				const float high = limited ? std::max(upper[axis], kRetargetMaxSwing) : kUnlimitedTurn;
				const float step = after[axis] - before[axis];
				if (std::fabs(step) < 1.0e-7f)
					continue;
				const float bound = step > 0.0f ? high : low;
				if ((step > 0.0f && after[axis] > high) || (step < 0.0f && after[axis] < low))
					scale = std::min(scale, std::clamp((bound - before[axis]) / step, 0.0f, 1.0f));
			}
			return scale;
		}

		/** @brief Turns a bone and everything below it rigidly about a_pivot. */
		void TurnSubtree(std::vector<Transform>& a_pose, const std::vector<int>& a_subtree, const Vec3& a_pivot, const Quat& a_spin)
		{
			for (const int below : a_subtree) {
				Transform& moved = a_pose[static_cast<std::size_t>(below)];
				moved.rotation = (a_spin * moved.rotation).Normalized();
				moved.translation = a_pivot + a_spin.Rotate(moved.translation - a_pivot);
			}
		}

		/** @brief Poses the dynamic bones from their kinematic roots with a_angles at every joint, in the authored joint frames. */
		void PoseFromOffsets(const SmpRig& a_rig, const RigTopology& a_topology, const std::vector<Vec3>& a_angles, std::vector<Transform>& a_pose)
		{
			for (const int bone : a_topology.order) {
				const std::size_t index = static_cast<std::size_t>(bone);
				const RigJoint& joint = a_rig.joints[static_cast<std::size_t>(a_topology.jointOfBone[index])];
				const Transform& parentPose = a_pose[static_cast<std::size_t>(joint.parent)];
				const JointFrames& frames = a_topology.frames[index];
				a_pose[index] = PoseFromAngles(JointRotation(parentPose, frames), frames, Pivot(parentPose, frames), a_angles[index]);
			}
		}

		Vec3 AnglesAt(const SmpRig& a_rig, const RigTopology& a_topology, const std::vector<Transform>& a_pose, int a_bone)
		{
			const std::size_t index = static_cast<std::size_t>(a_bone);
			const RigJoint& joint = a_rig.joints[static_cast<std::size_t>(a_topology.jointOfBone[index])];
			return JointAngles(a_pose[static_cast<std::size_t>(joint.parent)], a_pose[index], a_topology.frames[index]);
		}

		/**
		 * @brief Moves the rest so that, once gravity sags it again, the turned axes land at a_target; an axis not turned keeps its authored rest.
		 * @param a_offsets The rest's joint angles in the authored joint frame, refined in place.
		 * @return The pose the rest is predicted to settle at.
		 */
		std::vector<Transform> ChaseAngleTargets(const SmpRig& a_rig, const RigTopology& a_topology, const std::vector<Vec3>& a_target,
			const std::vector<std::array<bool, 3>>& a_turnedAxis, const std::vector<std::vector<BoneSphere>>& a_spheres,
			const std::vector<TriangleGrid>& a_supports, const Vec3& a_axisPoint, const RetargetLoad& a_load,
			std::vector<Vec3>& a_offsets, std::vector<Transform>& a_restPose)
		{
			std::vector<Transform> resettled = a_restPose;
			for (int iteration = 0; iteration <= kSagIterations; ++iteration) {
				PoseFromOffsets(a_rig, a_topology, a_offsets, a_restPose);
				resettled = SagUnderGravity(a_rig, a_restPose, ResolveTopology(a_rig, a_restPose), a_load, a_spheres, a_supports, a_axisPoint).pose;
				if (iteration == kSagIterations)
					break;
				for (const int bone : a_topology.order) {
					const std::size_t index = static_cast<std::size_t>(bone);
					const RigJoint& joint = a_rig.joints[static_cast<std::size_t>(a_topology.jointOfBone[index])];
					const Vec3 landed = AnglesAt(a_rig, a_topology, resettled, bone);
					const float miss[3] = { a_target[index].x - landed.x, a_target[index].y - landed.y, a_target[index].z - landed.z };
					const float lower[3] = { joint.angularLower.x, joint.angularLower.y, joint.angularLower.z };
					const float upper[3] = { joint.angularUpper.x, joint.angularUpper.y, joint.angularUpper.z };
					float* offset[3] = { &a_offsets[index].x, &a_offsets[index].y, &a_offsets[index].z };
					for (int axis = 0; axis < 3; ++axis) {
						if (!a_turnedAxis[index][static_cast<std::size_t>(axis)])
							continue;
						// A landing the limits or a support will not allow is not chased past the swing the sweeps were given.
						const float low = (lower[axis] <= upper[axis] ? lower[axis] : 0.0f) - kRetargetMaxSwing;
						const float high = (lower[axis] <= upper[axis] ? upper[axis] : 0.0f) + kRetargetMaxSwing;
						*offset[axis] = std::clamp(*offset[axis] + miss[axis], low, high);
					}
				}
			}
			return resettled;
		}

		/** @brief For each bone, itself and every dynamic bone below it. */
		std::vector<std::vector<int>> BuildSubtrees(const SmpRig& a_rig, const RigTopology& a_topology)
		{
			const std::vector<int>& order = a_topology.order;
			std::vector<std::vector<int>> subtree(a_rig.bones.size());
			for (const int bone : order)
				subtree[static_cast<std::size_t>(bone)].push_back(bone);
			for (auto bone = order.rbegin(); bone != order.rend(); ++bone) {
				const int parent = a_rig.joints[static_cast<std::size_t>(a_topology.jointOfBone[static_cast<std::size_t>(*bone)])].parent;
				if (parent >= 0 && !a_rig.bones[static_cast<std::size_t>(parent)].kinematic) {
					std::vector<int>& above = subtree[static_cast<std::size_t>(parent)];
					const std::vector<int>& below = subtree[static_cast<std::size_t>(*bone)];
					above.insert(above.end(), below.begin(), below.end());
				}
			}
			return subtree;
		}
	}

	std::vector<Transform> PredictGravitySag(const SmpRig& a_rig, const std::vector<Transform>& a_restPose, const std::vector<std::vector<BoneSphere>>& a_spheres,
		const std::vector<SkinnedSurface>& a_surfaces, const std::vector<Transform>& a_driverWorld, const Vec3& a_axisPoint, const RetargetLoad& a_load)
	{
		if (a_rig.bones.size() != a_restPose.size())
			return a_restPose;
		return SagUnderGravity(a_rig, a_restPose, ResolveTopology(a_rig, a_restPose), a_load, a_spheres, BuildSupports(a_surfaces, a_driverWorld), a_axisPoint).pose;
	}

	RestRetarget DrapeRestOntoSupports(const SmpRig& a_rig, const std::vector<Transform>& a_restPose, const std::vector<std::vector<BoneSphere>>& a_spheres,
		const std::vector<SkinnedSurface>& a_surfaces, const std::vector<Transform>& a_driverWorld, const Vec3& a_axisPoint, const RetargetLoad& a_load)
	{
		RestRetarget result;
		result.restPose = a_restPose;
		result.angleOffsets.assign(a_restPose.size(), Vec3{});
		const std::size_t boneCount = a_rig.bones.size();
		if (boneCount != a_restPose.size() || a_driverWorld.empty())
			return result;

		const RigTopology topology = ResolveTopology(a_rig, a_restPose);
		const std::vector<TriangleGrid> supports = BuildSupports(a_surfaces, a_driverWorld);
		if (supports.empty())
			return result;
		const Sag sag = SagUnderGravity(a_rig, a_restPose, topology, a_load, a_spheres, supports, a_axisPoint);
		result.sagMaxMove = sag.maxMove;
		result.sagPose = sag.pose;

		std::vector<Vec3> sagAngles(boneCount);
		for (const int bone : topology.order)
			sagAngles[static_cast<std::size_t>(bone)] = AnglesAt(a_rig, topology, sag.pose, bone);
		const RetargetLoad limp{ a_load.gravity, a_load.stiffnessScale * kDrapeSpringScale };
		const std::vector<Transform> settled = SagUnderGravity(a_rig, a_restPose, topology, limp, a_spheres, supports, a_axisPoint).pose;

		std::vector<Vec3> target(boneCount);
		std::vector<std::array<bool, 3>> turnedAxis(boneCount, std::array<bool, 3>{});
		for (const int bone : topology.order) {
			const std::size_t index = static_cast<std::size_t>(bone);
			target[index] = AnglesAt(a_rig, topology, settled, bone);
			const float moved[3] = { target[index].x - sagAngles[index].x, target[index].y - sagAngles[index].y, target[index].z - sagAngles[index].z };
			for (int axis = 0; axis < 3; ++axis)
				turnedAxis[index][static_cast<std::size_t>(axis)] = std::fabs(moved[axis]) > kMinimumTurn;
			result.angleOffsets[index] = { turnedAxis[index][0] ? moved[0] : 0.0f, turnedAxis[index][1] ? moved[1] : 0.0f, turnedAxis[index][2] ? moved[2] : 0.0f };
		}
		std::vector<Transform> resettled = ChaseAngleTargets(a_rig, topology, target, turnedAxis, a_spheres, supports, a_axisPoint, a_load, result.angleOffsets, result.restPose);

		for (const int bone : topology.order) {
			const std::size_t index = static_cast<std::size_t>(bone);
			const Vec3& offset = result.angleOffsets[index];
			const float magnitude = std::max({ std::fabs(offset.x), std::fabs(offset.y), std::fabs(offset.z) });
			if (magnitude > kMinimumTurn)
				++result.retargetedJoints;
			result.maxAngle = std::max(result.maxAngle, magnitude);
			result.maxBoneMove = std::max(result.maxBoneMove, Length(result.restPose[index].translation - a_restPose[index].translation));
		}
		if (result.retargetedJoints == 0)
			result.restPose = a_restPose;
		else
			result.sagPose = std::move(resettled);
		return result;
	}

	RestRetarget RetargetRestToSurfaces(const SmpRig& a_rig, const std::vector<Transform>& a_restPose,
		const std::vector<std::vector<BoneSphere>>& a_spheres, const std::vector<SkinnedSurface>& a_surfaces,
		const std::vector<Transform>& a_driverWorld, const Vec3& a_axisPoint, const RetargetLoad& a_load)
	{
		RestRetarget result;
		result.restPose = a_restPose;
		result.angleOffsets.assign(a_restPose.size(), Vec3{});
		const std::size_t boneCount = a_rig.bones.size();
		if (boneCount != a_restPose.size() || a_driverWorld.empty())
			return result;

		const RigTopology topology = ResolveTopology(a_rig, a_restPose);
		const std::vector<TriangleGrid> supports = BuildSupports(a_surfaces, a_driverWorld);
		const Sag sag = SagUnderGravity(a_rig, a_restPose, topology, a_load, a_spheres, supports, a_axisPoint);
		result.sagMaxMove = sag.maxMove;
		result.sagPose = sag.pose;

		std::vector<TriangleGrid> grids;
		std::vector<float> allowances;
		for (const SkinnedSurface& surface : a_surfaces) {
			if (surface.deepPush || surface.skin || !(surface.contactPenetration > 0.0f) || surface.indices.empty())
				continue;
			grids.emplace_back().Build(PoseSurface(surface, a_driverWorld), surface.indices, kSurfaceCellSize);
			allowances.push_back(surface.contactPenetration);
		}
		if (grids.empty())
			return result;

		const std::vector<int>& order = topology.order;
		const std::vector<std::vector<int>> subtree = BuildSubtrees(a_rig, topology);

		std::vector<Transform> settled = sag.pose;
		result.behindSpheres = CountInsideShell(order, settled, a_spheres, grids, allowances, a_axisPoint);
		if (result.behindSpheres == 0)
			return result;

		const auto anglesIn = [&](const std::vector<Transform>& a_pose, int a_bone) { return AnglesAt(a_rig, topology, a_pose, a_bone); };
		std::vector<Vec3> sagAngles(boneCount);
		for (const int bone : order)
			sagAngles[static_cast<std::size_t>(bone)] = anglesIn(settled, bone);

		for (int pass = 0; pass < kRetargetPasses; ++pass) {
			for (const int bone : order) {
				const std::size_t index = static_cast<std::size_t>(bone);
				const RigJoint& joint = a_rig.joints[static_cast<std::size_t>(topology.jointOfBone[index])];
				const Deficit deficit = WorstDeficit(subtree[index], settled, a_spheres, grids, allowances, a_axisPoint);
				if (!(deficit.amount > 0.0f))
					continue;
				const Transform& parentPose = settled[static_cast<std::size_t>(joint.parent)];
				const Quat jointRotation = JointRotation(parentPose, topology.frames[index]);
				const Vec3 pivot = Pivot(parentPose, topology.frames[index]);
				const Vec3 lever = deficit.centre - pivot;
				Vec3 axis = SwingOnly(Conjugate(jointRotation).Rotate(Cross(lever, deficit.normal)), joint);
				if (!(Length(axis) > 1.0e-6f))
					continue;
				axis = jointRotation.Rotate(Normalized(axis));
				const float reach = std::max(Length(Cross(axis, lever)), kMinimumLever);
				float turn = deficit.amount / reach;
				Quat spin = FromAxisAngle(axis, turn);
				const Transform before = settled[index];
				const Vec3 anglesBefore = anglesIn(settled, bone);
				for (int attempt = 0; attempt < 3; ++attempt) {
					settled[index].rotation = (spin * before.rotation).Normalized();
					const float scale = LimitScale(anglesBefore, anglesIn(settled, bone), joint);
					if (scale >= 1.0f)
						break;
					turn *= scale;
					spin = FromAxisAngle(axis, turn);
				}
				settled[index] = before;
				if (!(turn > kMinimumTurn))
					continue;
				TurnSubtree(settled, subtree[index], pivot, spin);
			}
		}

		const std::vector<float> noAllowance(allowances.size(), 0.0f);
		for (const int bone : order) {
			const std::size_t index = static_cast<std::size_t>(bone);
			const RigJoint& joint = a_rig.joints[static_cast<std::size_t>(topology.jointOfBone[index])];
			const Transform& parentPose = settled[static_cast<std::size_t>(joint.parent)];
			const Quat jointRotation = JointRotation(parentPose, topology.frames[index]);
			const Vec3 pivot = Pivot(parentPose, topology.frames[index]);
			const Vec3 back = SwingOnly(Conjugate(jointRotation).Rotate(ToRotationVector((sag.pose[index].rotation * Conjugate(settled[index].rotation)).Normalized())), joint);
			const float full = Length(back);
			if (!(full > kMinimumTurn))
				continue;
			const Vec3 axis = jointRotation.Rotate(back / full);
			const auto clearAt = [&](float a_fraction) {
				std::vector<Transform> trial = settled;
				TurnSubtree(trial, subtree[index], pivot, FromAxisAngle(axis, full * a_fraction));
				return CountInsideShell(subtree[index], trial, a_spheres, grids, noAllowance, a_axisPoint) == 0;
			};
			float fraction = 1.0f;
			if (!clearAt(1.0f)) {
				float clear = 0.0f;
				float blocked = 1.0f;
				for (int step = 0; step < kHangBisections; ++step) {
					const float middle = 0.5f * (clear + blocked);
					(clearAt(middle) ? clear : blocked) = middle;
				}
				fraction = clear;
			}
			if (fraction * full > kMinimumTurn)
				TurnSubtree(settled, subtree[index], pivot, FromAxisAngle(axis, full * fraction));
		}

		std::vector<Vec3> target(boneCount);
		std::vector<std::array<bool, 3>> turnedAxis(boneCount, std::array<bool, 3>{});
		for (const int bone : order) {
			const std::size_t index = static_cast<std::size_t>(bone);
			target[index] = anglesIn(settled, bone);
			const float moved[3] = { target[index].x - sagAngles[index].x, target[index].y - sagAngles[index].y, target[index].z - sagAngles[index].z };
			for (int axis = 0; axis < 3; ++axis)
				turnedAxis[index][static_cast<std::size_t>(axis)] = std::fabs(moved[axis]) > kMinimumTurn;
			result.angleOffsets[index] = { turnedAxis[index][0] ? moved[0] : 0.0f, turnedAxis[index][1] ? moved[1] : 0.0f, turnedAxis[index][2] ? moved[2] : 0.0f };
		}
		std::vector<Transform> resettled = ChaseAngleTargets(a_rig, topology, target, turnedAxis, a_spheres, supports, a_axisPoint, a_load, result.angleOffsets, result.restPose);
		// A strand left pointing up or standing out from the body, where the author had it hanging, looks worse than one inside the
		// armor, so a chain with any such segment keeps its authored rest.
		const auto hang = [&](const std::vector<Transform>& a_pose, int a_bone) {
			const std::size_t index = static_cast<std::size_t>(a_bone);
			const int parent = a_rig.joints[static_cast<std::size_t>(topology.jointOfBone[index])].parent;
			const Vec3 segment = a_pose[index].translation - a_pose[static_cast<std::size_t>(parent)].translation;
			const float length = Length(segment);
			return length > 1.0e-3f ? -segment.z / length : 1.0f;
		};
		std::vector<int> rootOf(boneCount, -1);
		for (const int bone : order) {
			int root = bone;
			while (true) {
				const int parent = a_rig.joints[static_cast<std::size_t>(topology.jointOfBone[static_cast<std::size_t>(root)])].parent;
				if (parent < 0 || a_rig.bones[static_cast<std::size_t>(parent)].kinematic)
					break;
				root = parent;
			}
			rootOf[static_cast<std::size_t>(bone)] = root;
		}
		std::vector<bool> revertRoot(boneCount, false);
		for (const int bone : order) {
			const float now = hang(resettled, bone);
			if (now < kRetargetMinimumHang && now < hang(sag.pose, bone) - kRetargetMaxHangLoss)
				revertRoot[static_cast<std::size_t>(rootOf[static_cast<std::size_t>(bone)])] = true;
		}
		for (std::size_t root = 0; root < boneCount; ++root)
			result.revertedChains += revertRoot[root] ? 1 : 0;
		if (result.revertedChains > 0) {
			for (const int bone : order) {
				if (revertRoot[static_cast<std::size_t>(rootOf[static_cast<std::size_t>(bone)])])
					result.angleOffsets[static_cast<std::size_t>(bone)] = Vec3{};
			}
			PoseFromOffsets(a_rig, topology, result.angleOffsets, result.restPose);
			resettled = SagUnderGravity(a_rig, result.restPose, ResolveTopology(a_rig, result.restPose), a_load, a_spheres, supports, a_axisPoint).pose;
		}

		for (const int bone : order) {
			const std::size_t index = static_cast<std::size_t>(bone);
			const Vec3& offset = result.angleOffsets[index];
			const float magnitude = std::max({ std::fabs(offset.x), std::fabs(offset.y), std::fabs(offset.z) });
			if (magnitude > kMinimumTurn)
				++result.retargetedJoints;
			result.maxAngle = std::max(result.maxAngle, magnitude);
			result.maxBoneMove = std::max(result.maxBoneMove, Length(result.restPose[index].translation - a_restPose[index].translation));
		}
		result.residualSpheres = CountInsideShell(order, resettled, a_spheres, grids, allowances, a_axisPoint);
		const int freed = result.behindSpheres - result.residualSpheres;
		if (static_cast<float>(freed) < kRetargetMinimumFreed * static_cast<float>(result.behindSpheres)) {
			result.keptAuthored = true;
			result.restPose = a_restPose;
			result.angleOffsets.assign(a_restPose.size(), Vec3{});
			result.residualSpheres = result.behindSpheres;
			result.retargetedJoints = 0;
			result.maxAngle = 0.0f;
			result.maxBoneMove = 0.0f;
			return result;
		}
		if (result.retargetedJoints == 0)
			result.restPose = a_restPose;
		else
			result.sagPose = std::move(resettled);
		return result;
	}
}
