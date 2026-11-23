#include "smp/RigidRigBuilder.h"

#include <algorithm>
#include <cmath>
#include <utility>

namespace SmpPhysics
{
	namespace
	{
		RigidReference::Vec3 ToReference(const Vec3& a_value) noexcept
		{
			return { a_value.x, a_value.y, a_value.z };
		}

		RigidReference::Transform ToReference(const Transform& a_value) noexcept
		{
			return RigidReference::MakeTransform(ToReference(a_value.translation),
				{ a_value.rotation.x, a_value.rotation.y, a_value.rotation.z, a_value.rotation.w });
		}

		float Reciprocal(float a_value) noexcept
		{
			return a_value > 0.0f ? 1.0f / a_value : 0.0f;
		}

		RigidReference::Body BuildBody(const RigBone& a_bone, const Transform& a_rest, float a_scale, const SolverParams& a_params)
		{
			RigidReference::Body body;
			body.kinematic = a_bone.kinematic;
			body.worldTransform = ToReference(a_rest);
			if (a_bone.kinematic)
				return body;

			const float scale2 = a_scale * a_scale;
			const float scale3 = scale2 * a_scale;
			const float scale5 = scale3 * scale2;
			const float mass = a_bone.mass * scale3;
			body.inverseMass = Reciprocal(mass);
			body.inverseInertiaLocal = { Reciprocal(a_bone.inertia.x * scale5), Reciprocal(a_bone.inertia.y * scale5),
				Reciprocal(a_bone.inertia.z * scale5) };
			body.linearDamping = std::clamp(a_bone.linearDamping * a_params.dampingScale, 0.0f, 1.0f);
			body.angularDamping = std::clamp(a_bone.angularDamping * a_params.dampingScale, 0.0f, 1.0f);
			body.friction = a_bone.friction;
			body.gravity = { 0.0f, 0.0f, -a_params.gravity * a_params.gravityScale * a_bone.gravityFactor * mass };
			return body;
		}

		RigidReference::RotationalMotor BuildAngularAxis(const RigJoint& a_joint, int a_axis, float a_scale, const SolverParams& a_params)
		{
			const auto component = [a_axis](const Vec3& a_value) {
				return a_axis == 0 ? a_value.x : (a_axis == 1 ? a_value.y : a_value.z);
			};
			RigidReference::RotationalMotor motor;
			motor.lower = component(a_joint.angularLower);
			motor.upper = component(a_joint.angularUpper);
			motor.bounce = component(a_joint.angularBounce);
			motor.stopErp = a_joint.stopErp;
			motor.stopCfm = a_joint.stopCfm.value_or(kRigidAngularStopCfm);
			motor.motorErp = a_joint.motorErp;
			motor.motorCfm = a_joint.motorCfm;
			motor.motorEnabled = a_joint.angularMotors;
			motor.servoEnabled = a_joint.angularServoMotors;
			motor.servoTarget = component(a_joint.angularEquilibrium);
			motor.targetVelocity = component(a_joint.angularTargetVelocity);
			motor.maxMotorForce = component(a_joint.angularMaxMotorForce);
			motor.springEnabled = a_joint.angularSpringsEnabled;
			motor.springStiffness = component(a_joint.angularStiffness) * a_scale * a_scale * a_scale * a_scale * a_scale * a_params.stiffnessScale;
			motor.springStiffnessLimited = a_joint.angularStiffnessLimited;
			motor.springDamping = component(a_joint.angularDamping) * a_params.dampingScale;
			motor.springDampingLimited = a_joint.springDampingLimited;
			motor.equilibrium = component(a_joint.angularEquilibrium);
			motor.nonHookeanStiffness = component(a_joint.angularNonHookeanStiffness);
			motor.nonHookeanDamping = component(a_joint.angularNonHookeanDamping);
			return motor;
		}

		RigidReference::Generic6DofSpring2 BuildConstraint(const RigJoint& a_joint, const std::vector<Transform>& a_restWorld, float a_scale,
			const SolverParams& a_params)
		{
			RigJoint scaledJoint = a_joint;
			if (scaledJoint.frameKind != FrameKind::FrameInLerp)
				scaledJoint.frame.translation = scaledJoint.frame.translation * a_scale;
			const JointFrames frames = ResolveJointFrames(scaledJoint, a_restWorld[static_cast<std::size_t>(a_joint.child)],
				a_restWorld[static_cast<std::size_t>(a_joint.parent)]);
			RigidReference::Generic6DofSpring2 constraint;
			constraint.bodyA = a_joint.child;
			constraint.bodyB = a_joint.parent;
			constraint.frameA = ToReference(frames.frameA);
			constraint.frameB = ToReference(frames.frameB);
			constraint.linear.lower = ToReference(a_joint.linearLower * a_scale);
			constraint.linear.upper = ToReference(a_joint.linearUpper * a_scale);
			constraint.linear.stopErp = { a_joint.stopErp, a_joint.stopErp, a_joint.stopErp };
			const float linearStopCfm = a_joint.stopCfm.value_or(0.0f);
			constraint.linear.stopCfm = { linearStopCfm, linearStopCfm, linearStopCfm };
			constraint.linear.motorErp = { a_joint.motorErp, a_joint.motorErp, a_joint.motorErp };
			constraint.linear.motorCfm = { a_joint.motorCfm, a_joint.motorCfm, a_joint.motorCfm };
			constraint.linear.motorEnabled = { a_joint.linearMotors, a_joint.linearMotors, a_joint.linearMotors };
			constraint.linear.servoEnabled = { a_joint.linearServoMotors, a_joint.linearServoMotors, a_joint.linearServoMotors };
			constraint.linear.servoTarget = ToReference(a_joint.linearEquilibrium * a_scale);
			constraint.linear.targetVelocity = ToReference(a_joint.linearTargetVelocity);
			constraint.linear.maxMotorForce = ToReference(a_joint.linearMaxMotorForce);
			constraint.linear.springEnabled = { a_joint.linearSpringsEnabled, a_joint.linearSpringsEnabled, a_joint.linearSpringsEnabled };
			constraint.linear.springStiffness = ToReference(a_joint.linearStiffness * (a_scale * a_scale * a_scale) * a_params.stiffnessScale);
			constraint.linear.springStiffnessLimited = { a_joint.linearStiffnessLimited, a_joint.linearStiffnessLimited, a_joint.linearStiffnessLimited };
			constraint.linear.springDamping = ToReference(a_joint.linearDamping * a_params.dampingScale);
			constraint.linear.springDampingLimited = { a_joint.springDampingLimited, a_joint.springDampingLimited, a_joint.springDampingLimited };
			constraint.linear.equilibrium = ToReference(a_joint.linearEquilibrium * a_scale);
			constraint.linear.bounce = ToReference(a_joint.linearBounce);
			constraint.linear.nonHookeanStiffness = ToReference(a_joint.linearNonHookeanStiffness);
			constraint.linear.nonHookeanDamping = ToReference(a_joint.linearNonHookeanDamping);
			for (int axis = 0; axis < 3; ++axis)
				constraint.angular[static_cast<std::size_t>(axis)] = BuildAngularAxis(a_joint, axis, a_scale, a_params);
			if (a_joint.useLinearReferenceFrameA != a_joint.reversed) {
				std::swap(constraint.bodyA, constraint.bodyB);
				std::swap(constraint.frameA, constraint.frameB);
			}
			return constraint;
		}
	}

	RigidReference::World BuildRigidWorld(const SmpRig& a_rig, const std::vector<Transform>& a_restWorld, float a_scale,
		const SolverParams& a_params)
	{
		RigidReference::World world;
		world.bodies.reserve(a_rig.bones.size());
		for (std::size_t index = 0; index < a_rig.bones.size(); ++index)
			world.bodies.push_back(BuildBody(a_rig.bones[index], a_restWorld[index], a_scale, a_params));
		world.accumulatedForce.assign(world.bodies.size(), {});

		world.constraints.reserve(a_rig.joints.size() + a_rig.additionalJoints.size());
		for (const RigJoint& joint : a_rig.joints)
			world.constraints.push_back(BuildConstraint(joint, a_restWorld, a_scale, a_params));
		for (const RigJoint& joint : a_rig.additionalJoints)
			world.constraints.push_back(BuildConstraint(joint, a_restWorld, a_scale, a_params));
		return world;
	}

	namespace
	{
		Transform ScaledComTransform(const RigBone& a_bone, float a_scale) noexcept
		{
			Transform scaled = a_bone.comTransform;
			scaled.translation = scaled.translation * a_scale;
			return scaled;
		}
	}

	std::vector<Transform> BodyFrames(const SmpRig& a_rig, const std::vector<Transform>& a_nodeWorld, float a_scale)
	{
		std::vector<Transform> frames = a_nodeWorld;
		for (std::size_t index = 0; index < a_rig.bones.size() && index < frames.size(); ++index) {
			if (!a_rig.bones[index].kinematic)
				frames[index] = a_nodeWorld[index] * ScaledComTransform(a_rig.bones[index], a_scale);
		}
		return frames;
	}

	std::vector<Transform> NodeFromBody(const SmpRig& a_rig, float a_scale)
	{
		const auto isIdentity = [](const Transform& a_transform) {
			return std::abs(a_transform.rotation.x) + std::abs(a_transform.rotation.y) + std::abs(a_transform.rotation.z) <= 1e-6f &&
			       Length(a_transform.translation) <= 1e-6f;
		};
		if (std::all_of(a_rig.bones.begin(), a_rig.bones.end(), [&](const RigBone& a_bone) { return isIdentity(a_bone.comTransform); }))
			return {};
		std::vector<Transform> inverses;
		inverses.reserve(a_rig.bones.size());
		for (const RigBone& bone : a_rig.bones)
			inverses.push_back(ScaledComTransform(bone, a_scale).Inverse());
		return inverses;
	}

	Vec3 NodePointToBody(const std::vector<Transform>& a_nodeFromBody, int a_body, const Vec3& a_point)
	{
		if (a_body < 0 || static_cast<std::size_t>(a_body) >= a_nodeFromBody.size())
			return a_point;
		return a_nodeFromBody[static_cast<std::size_t>(a_body)].TransformPoint(a_point);
	}
}
