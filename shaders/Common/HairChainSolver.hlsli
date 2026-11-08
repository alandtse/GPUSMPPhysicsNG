#ifndef __HAIR_CHAIN_SOLVER_DEPENDENCY_HLSL__
#define __HAIR_CHAIN_SOLVER_DEPENDENCY_HLSL__

// Pure functions for hair chain physics integration, constraints, and contacts.
// Every value-returning function has a single exit, or FXC reports the return as uninitialized (X4000).

namespace HairChainSolver
{
	/** @brief Length below which a vector is treated as having no usable direction. */
	static const float kDirectionEpsilon = 1e-8f;
	/** @brief Rotation magnitude below which FromRotationVector uses its series expansion. */
	static const float kSmallRotationAngle = 1e-4f;
	/** @brief Cosine above which Slerp falls back to a normalized lerp. */
	static const float kSlerpLinearThreshold = 0.9995f;
	/** @brief Inertia floor, below which the torque to velocity division would explode. */
	static const float kMinimumInertia = 1e-6f;
	/** @brief Smallest half-extent an ellipsoid keeps, below which its scaled-space projection would divide by zero. Mirrors kMinimumHalfExtent in Collision.h. */
	static const float kMinimumHalfExtent = 1e-3f;
	/** @brief Colliders a sphere's rest-exemption mask can name. Mirrors kMaximumExemptColliders in Collision.h. */
	static const uint kMaximumExemptColliders = 32u;
	/** @brief Depth a posed collision surface tolerates before the contact clamp holds a bone back. Mirrors kDefaultPenetration in Collision.h. */
	static const float kSurfacePenetration = 0.1f;
	/** @brief Slack a bone's bound sphere carries when it decides whether the whole bone can be skipped. Mirrors kBoundCullPad in SolverCommon.h. */
	static const float kBoundCullPad = 0.5f;
	/** @brief Fastest, in radians per second, a joint beyond its limit returns inside it. Mirrors kLimitReturnRate in SolverCommon.h. */
	static const float kLimitReturnRate = 8.0f;
	/** @brief Collider kind values, mirroring Collider::Type in Collision.h. */
	static const int kColliderEllipsoid = 0;
	static const int kColliderCapsule = 1;
	static const int kColliderPlane = 2;

	struct Quat
	{
		float w;
		float x;
		float y;
		float z;
	};

	struct Transform
	{
		Quat rotation;
		float3 translation;
	};

	/** @brief The joint frames expressed in each body's local frame, as SMP's calcFrame derives them. */
	struct JointFrame
	{
		Quat rotationA;
		float3 originA;
		Quat rotationB;
		float3 originB;
	};

	struct BoneParameters
	{
		float mass;
		float inertiaMean;
		float angularDamping;
		float gravityFactor;
		float windFactor;
		float3 windAxis;  // the strand's direction in the bone's frame
		float windArea;   // segment length times width
	};

	/** @brief Per-axis angular springs of one joint, plus the per-axis limits they act inside. */
	struct JointSprings
	{
		float3 stiffness;
		float3 damping;
		float3 equilibrium;
		float3 lower;
		float3 upper;
		bool enabled;
	};

	/** @brief Per-bone state the solver carries between frames. */
	struct BoneState
	{
		Quat rotation;
		float3 angularVelocity;
		float3 previousPivot;
		float3 previousPivotVelocity;
		Quat previousParentRotation;
		bool hasHistory;
		bool hasVelocityHistory;
		bool limitsFreed;  // a hand held the strand below this joint last step
	};

	/**
	 * @brief The scales and clamps a sanitized SolverParams imposes on every bone.
	 * @details Contact spring parameters come from SolverCommon.h's kContactStiffness and kContactDamping.
	 */
	struct SolverScales
	{
		float gravity;
		float gravityScale;
		float stiffnessScale;
		float dampingScale;
		float maxAngularSpeed;
		float contactStiffness;
		float contactDamping;
		float contactSkin;
		float tieStiffness;  // spring stiffness of the pull that keeps a tied bone inside its joint's linear limits, 0 switches ties off
		bool windChainLoad;  // whether a joint also carries the wind load of every bone below it
		float3 wind;         // hdtSMP64's world wind vector, zero when wind is off
		float windTime;      // the gust clock
		float windFactor;    // the actor's wind exposure
	};

	/** @brief One analytic collision proxy, expressed in the frame of the driver it follows. */
	struct Collider
	{
		int kind;
		Quat orientation;
		float3 center;
		float3 halfExtents;
		float3 segmentA;
		float3 segmentB;
		float radius;
		float3 normal;
		float penetration;
	};

	/** @brief The deepest contact one query found. Depth is how far the queried sphere reaches past the surface. */
	struct Contact
	{
		float depth;
		float3 normal;
		int collider;
	};

	/** @brief The contact springs one bone's spheres accumulated, folded into the implicit update. */
	struct ContactTerms
	{
		float3 torque;
		float3 stiffness;
		float3 damping;
		float3 subtreeGravityTorque;  // gravity moment of the bones below the joint about its pivot
		float3 subtreeMassMoment;     // sum of those bones' mass times their centre's offset from the pivot
		float subtreeInertia;         // their inertia about the pivot
	};

	/** @brief What one bone's advance produced, so a contact clamp can adjust the pose before the caller stores it. */
	struct BoneAdvance
	{
		Quat jointRotation;
		float3 pivot;
		Transform pose;
		float3 angles;
		float3 updatedAngles;
		float3 updatedVelocity;
		float3 parentAngularVelocity;
		bool beyondLimits;  // a joint is still beyond a limit it was freed from, so the next step eases it back
	};

	Quat QuatIdentity()
	{
		Quat result;
		result.w = 1.0f;
		result.x = 0.0f;
		result.y = 0.0f;
		result.z = 0.0f;
		return result;
	}

	Quat QuatNegate(Quat a_value)
	{
		Quat result;
		result.w = -a_value.w;
		result.x = -a_value.x;
		result.y = -a_value.y;
		result.z = -a_value.z;
		return result;
	}

	Quat QuatAdd(Quat a_left, Quat a_right)
	{
		Quat result;
		result.w = a_left.w + a_right.w;
		result.x = a_left.x + a_right.x;
		result.y = a_left.y + a_right.y;
		result.z = a_left.z + a_right.z;
		return result;
	}

	Quat QuatScale(Quat a_value, float a_scale)
	{
		Quat result;
		result.w = a_value.w * a_scale;
		result.x = a_value.x * a_scale;
		result.y = a_value.y * a_scale;
		result.z = a_value.z * a_scale;
		return result;
	}

	Quat QuatMul(Quat a_left, Quat a_right)
	{
		Quat result;
		result.w = a_left.w * a_right.w - a_left.x * a_right.x - a_left.y * a_right.y - a_left.z * a_right.z;
		result.x = a_left.w * a_right.x + a_left.x * a_right.w + a_left.y * a_right.z - a_left.z * a_right.y;
		result.y = a_left.w * a_right.y - a_left.x * a_right.z + a_left.y * a_right.w + a_left.z * a_right.x;
		result.z = a_left.w * a_right.z + a_left.x * a_right.y - a_left.y * a_right.x + a_left.z * a_right.w;
		return result;
	}

	Quat QuatConjugate(Quat a_value)
	{
		Quat result;
		result.w = a_value.w;
		result.x = -a_value.x;
		result.y = -a_value.y;
		result.z = -a_value.z;
		return result;
	}

	/** @brief Unit quaternion, or the identity when the input has no usable magnitude. */
	Quat QuatNormalized(Quat a_value)
	{
		Quat result;
		float length = sqrt(a_value.w * a_value.w + a_value.x * a_value.x + a_value.y * a_value.y + a_value.z * a_value.z);
		if (length > kDirectionEpsilon) {
			result.w = a_value.w / length;
			result.x = a_value.x / length;
			result.y = a_value.y / length;
			result.z = a_value.z / length;
		} else {
			result = QuatIdentity();
		}
		return result;
	}

	/** @brief Rotates a vector by a unit quaternion. */
	float3 QuatRotate(Quat a_rotation, float3 a_value)
	{
		float3 axis = float3(a_rotation.x, a_rotation.y, a_rotation.z);
		float3 twiceCross = cross(axis, a_value) * 2.0f;
		return a_value + twiceCross * a_rotation.w + cross(axis, twiceCross);
	}

	/** @brief Unit vector, or zero when the input is too short to have a direction. */
	float3 Direction(float3 a_value)
	{
		float length = sqrt(dot(a_value, a_value));
		return length > kDirectionEpsilon ? a_value / length : float3(0.0f, 0.0f, 0.0f);
	}

	/** @brief One of the three coordinate axes, for code that indexes axes rather than naming them. */
	float3 AxisUnit(uint a_axis)
	{
		float3 result = float3(0.0f, 0.0f, 1.0f);
		if (a_axis == 0u)
			result = float3(1.0f, 0.0f, 0.0f);
		else if (a_axis == 1u)
			result = float3(0.0f, 1.0f, 0.0f);
		return result;
	}

	/** @brief A unit vector perpendicular to a_axis, so a contact at a capsule's axis still has a usable normal. */
	float3 AnyPerpendicular(float3 a_axis)
	{
		float3 result = float3(0.0f, 0.0f, 1.0f);
		if (length(a_axis) > kDirectionEpsilon) {
			float3 reference = abs(a_axis.z) > 0.9f ? float3(1.0f, 0.0f, 0.0f) : float3(0.0f, 0.0f, 1.0f);
			result = Direction(cross(a_axis, reference));
		}
		return result;
	}

	/** @return The distance from a_point to the segment, and the closest point on it. */
	float DistanceToSegment(float3 a_point, float3 a_start, float3 a_end, out float3 a_closest)
	{
		float3 direction = a_end - a_start;
		float lengthSquared = dot(direction, direction);
		float amount = 0.0f;
		if (lengthSquared > kDirectionEpsilon)
			amount = clamp(dot(a_point - a_start, direction) / lengthSquared, 0.0f, 1.0f);
		a_closest = a_start + direction * amount;
		return length(a_point - a_closest);
	}

	/** @brief Signed distance from a_point to an ellipsoid, with the outward unit normal. */
	float EllipsoidDistance(Collider a_collider, Transform a_world, float3 a_point, out float3 a_outward)
	{
		Quat rotation = QuatNormalized(QuatMul(a_world.rotation, a_collider.orientation));
		float3 centre = a_world.translation + QuatRotate(a_world.rotation, a_collider.center);
		float3 local = QuatRotate(QuatConjugate(rotation), a_point - centre);
		float3 halfExtents = max(a_collider.halfExtents, float3(kMinimumHalfExtent, kMinimumHalfExtent, kMinimumHalfExtent));
		float radiusScale = length(local / halfExtents);
		float distance = 0.0f;
		if (radiusScale > kDirectionEpsilon) {
			a_outward = QuatRotate(rotation, Direction(local));
			distance = length(local) * (radiusScale - 1.0f) / radiusScale;
		} else {
			uint smallest = 0u;
			float smallestExtent = halfExtents.x;
			if (halfExtents.y < smallestExtent) {
				smallest = 1u;
				smallestExtent = halfExtents.y;
			}
			if (halfExtents.z < smallestExtent) {
				smallest = 2u;
				smallestExtent = halfExtents.z;
			}
			a_outward = QuatRotate(rotation, AxisUnit(smallest));
			distance = -smallestExtent;
		}
		return distance;
	}

	float CapsuleDistance(Collider a_collider, Transform a_world, float3 a_point, out float3 a_outward)
	{
		float3 start = a_world.translation + QuatRotate(a_world.rotation, a_collider.segmentA);
		float3 end = a_world.translation + QuatRotate(a_world.rotation, a_collider.segmentB);
		float3 closest;
		float distance = DistanceToSegment(a_point, start, end, closest);
		a_outward = Direction(a_point - closest);
		if (length(a_outward) <= kDirectionEpsilon)
			a_outward = AnyPerpendicular(Direction(end - start));
		return distance - a_collider.radius;
	}

	float PlaneDistance(Collider a_collider, Transform a_world, float3 a_point, out float3 a_outward)
	{
		a_outward = Direction(QuatRotate(a_world.rotation, a_collider.normal));
		if (length(a_outward) <= kDirectionEpsilon)
			a_outward = float3(0.0f, 0.0f, 1.0f);
		return dot(a_point - (a_world.translation + QuatRotate(a_world.rotation, a_collider.center)), a_outward);
	}

	/** @brief Signed distance from a world point to a collider of any kind, with the outward unit normal. */
	float ColliderDistance(Collider a_collider, Transform a_world, float3 a_point, out float3 a_outward)
	{
		float distance;
		if (a_collider.kind == kColliderCapsule)
			distance = CapsuleDistance(a_collider, a_world, a_point, a_outward);
		else if (a_collider.kind == kColliderPlane)
			distance = PlaneDistance(a_collider, a_world, a_point, a_outward);
		else
			distance = EllipsoidDistance(a_collider, a_world, a_point, a_outward);
		return distance;
	}

	/** @brief No contact, which is what a query that touches nothing reports. */
	Contact NoContact()
	{
		Contact result;
		result.depth = 0.0f;
		result.normal = float3(0.0f, 0.0f, 0.0f);
		result.collider = -1;
		return result;
	}

	/** @brief Folds one collider's contact with a sphere into the deepest contact a query has found so far. */
	void MergeContact(inout Contact a_best, Collider a_collider, Transform a_world, float3 a_point, float a_radius,
		int a_index)
	{
		float3 outward = float3(0.0f, 0.0f, 0.0f);
		float depth = a_radius - ColliderDistance(a_collider, a_world, a_point, outward);
		if (depth > a_best.depth) {
			a_best.depth = depth;
			a_best.normal = outward;
			a_best.collider = a_index;
		}
	}

	/** @brief No accumulated contact, which is what a bone with no collider in reach folds into its update. */
	ContactTerms NoContactTerms()
	{
		ContactTerms result;
		result.torque = float3(0.0f, 0.0f, 0.0f);
		result.stiffness = float3(0.0f, 0.0f, 0.0f);
		result.damping = float3(0.0f, 0.0f, 0.0f);
		result.subtreeGravityTorque = float3(0.0f, 0.0f, 0.0f);
		result.subtreeMassMoment = float3(0.0f, 0.0f, 0.0f);
		result.subtreeInertia = 0.0f;
		return result;
	}

	/**
	 * @brief Folds one sphere's contact into the per-axis terms the implicit update takes:
	 *        a contact is a unilateral spring along the surface normal.
	 */
	// Mirrored by kMaxGrabPull and kMinGrabDistance in SolverCommon.h.
	static const float kMaxGrabPull = 25.0f;
	static const float kMinGrabDistance = 1.0e-3f;

	/** @brief How hard and which way a grab pin pulls a bone toward its target, the port of ComputeGrabPull; depth 0 for no pull. */
	Contact GrabPull(float3 a_position, float3 a_target)
	{
		Contact result;
		result.depth = 0.0f;
		result.normal = float3(0.0f, 0.0f, 0.0f);
		result.collider = -1;
		float3 toTarget = a_target - a_position;
		float distance = length(toTarget);
		if (!(distance > kMinGrabDistance))
			return result;
		result.depth = min(distance, kMaxGrabPull);
		result.normal = toTarget / distance;
		return result;
	}

	void AddContactTorque(inout ContactTerms a_terms, SolverScales a_scales, Quat a_jointRotation, float3 a_offset,
		Contact a_contact)
	{
		[unroll] for (uint axis = 0u; axis < 3u; ++axis)
		{
			float jacobian = dot(cross(QuatRotate(a_jointRotation, AxisUnit(axis)), a_offset), a_contact.normal);
			a_terms.torque[axis] += jacobian * a_scales.contactStiffness * a_contact.depth;
			a_terms.stiffness[axis] += a_scales.contactStiffness * jacobian * jacobian;
			a_terms.damping[axis] += a_scales.contactDamping * jacobian * jacobian;
		}
	}

	/** @brief The world pose a bone's joint frame gives it, which both the advance and its contact query need. */
	void JointPose(Transform a_parentPose, JointFrame a_frame, out Quat a_jointRotation, out float3 a_pivot)
	{
		a_jointRotation = QuatMul(a_parentPose.rotation, a_frame.rotationB);
		a_pivot = a_parentPose.translation + QuatRotate(a_parentPose.rotation, a_frame.originB);
	}

	Transform TransformInverse(Transform a_value)
	{
		Quat inverseRotation = QuatConjugate(QuatNormalized(a_value.rotation));
		Transform result;
		result.rotation = inverseRotation;
		result.translation = -QuatRotate(inverseRotation, a_value.translation);
		return result;
	}

	Transform TransformMul(Transform a_left, Transform a_right)
	{
		Transform result;
		result.rotation = QuatMul(a_left.rotation, a_right.rotation);
		result.translation = a_left.translation + QuatRotate(a_left.rotation, a_right.translation);
		return result;
	}

	bool IsFiniteVector(float3 a_value)
	{
		return isfinite(a_value.x) && isfinite(a_value.y) && isfinite(a_value.z);
	}

	bool IsFiniteQuat(Quat a_value)
	{
		return isfinite(a_value.w) && isfinite(a_value.x) && isfinite(a_value.y) && isfinite(a_value.z);
	}

	bool IsFiniteTransform(Transform a_value)
	{
		return IsFiniteQuat(a_value.rotation) && IsFiniteVector(a_value.translation);
	}

	/** @brief Exponential map: axis scaled by angle. Stable for angles down to zero. */
	Quat FromRotationVector(float3 a_rotation)
	{
		float angle = length(a_rotation);
		Quat result;
		if (angle < kSmallRotationAngle) {
			Quat series;
			series.w = 1.0f;
			series.x = a_rotation.x * 0.5f;
			series.y = a_rotation.y * 0.5f;
			series.z = a_rotation.z * 0.5f;
			result = QuatNormalized(series);
		} else {
			float half = angle * 0.5f;
			float sineScale = sin(half) / angle;
			result.w = cos(half);
			result.x = a_rotation.x * sineScale;
			result.y = a_rotation.y * sineScale;
			result.z = a_rotation.z * sineScale;
		}
		return result;
	}

	/** @brief Rotation vector of a quaternion, taking the shorter of the two equivalent rotations. */
	float3 ToRotationVector(Quat a_rotation)
	{
		Quat value = QuatNormalized(a_rotation);
		if (value.w < 0.0f)
			value = QuatNegate(value);
		float axisLength = sqrt(value.x * value.x + value.y * value.y + value.z * value.z);
		float3 result = float3(0.0f, 0.0f, 0.0f);
		if (axisLength > kDirectionEpsilon) {
			float angle = 2.0f * atan2(axisLength, value.w);
			float scale = angle / axisLength;
			result = float3(value.x * scale, value.y * scale, value.z * scale);
		}
		return result;
	}

	Quat Slerp(Quat a_from, Quat a_to, float a_amount)
	{
		Quat start = QuatNormalized(a_from);
		Quat end = QuatNormalized(a_to);
		float cosine = start.w * end.w + start.x * end.x + start.y * end.y + start.z * end.z;
		if (cosine < 0.0f) {
			end = QuatNegate(end);
			cosine = -cosine;
		}
		Quat result;
		if (cosine > kSlerpLinearThreshold) {
			result = QuatNormalized(QuatAdd(QuatScale(start, 1.0f - a_amount), QuatScale(end, a_amount)));
		} else {
			float angle = acos(clamp(cosine, -1.0f, 1.0f));
			float sine = sin(angle);
			result = QuatNormalized(QuatAdd(QuatScale(start, sin((1.0f - a_amount) * angle) / sine),
				QuatScale(end, sin(a_amount * angle) / sine)));
		}
		return result;
	}

	/** @brief Euler angles of R = Rx(x) * Ry(y) * Rz(z), unique only while |y| < pi/2. */
	float3 ToEulerXYZ(Quat a_rotation)
	{
		Quat value = QuatNormalized(a_rotation);
		float r00 = 1.0f - 2.0f * (value.y * value.y + value.z * value.z);
		float r01 = 2.0f * (value.x * value.y - value.w * value.z);
		float r02 = 2.0f * (value.x * value.z + value.w * value.y);
		float r12 = 2.0f * (value.y * value.z - value.w * value.x);
		float r22 = 1.0f - 2.0f * (value.x * value.x + value.y * value.y);
		return float3(atan2(-r12, r22), asin(clamp(r02, -1.0f, 1.0f)), atan2(-r01, r00));
	}

	/** @brief Quaternion of R = Rx(x) * Ry(y) * Rz(z). */
	Quat FromEulerXYZ(float3 a_angles)
	{
		Quat rotationX;
		rotationX.w = cos(a_angles.x * 0.5f);
		rotationX.x = sin(a_angles.x * 0.5f);
		rotationX.y = 0.0f;
		rotationX.z = 0.0f;
		Quat rotationY;
		rotationY.w = cos(a_angles.y * 0.5f);
		rotationY.x = 0.0f;
		rotationY.y = sin(a_angles.y * 0.5f);
		rotationY.z = 0.0f;
		Quat rotationZ;
		rotationZ.w = cos(a_angles.z * 0.5f);
		rotationZ.x = 0.0f;
		rotationZ.y = 0.0f;
		rotationZ.z = sin(a_angles.z * 0.5f);
		return QuatNormalized(QuatMul(QuatMul(rotationX, rotationY), rotationZ));
	}

	/** @brief The world transform a bone's joint angles give it, about a pivot fixed in its parent. */
	Transform PoseFor(Quat a_jointRotation, JointFrame a_frame, float3 a_pivot, float3 a_angles)
	{
		Transform result;
		result.rotation = QuatNormalized(QuatMul(QuatMul(a_jointRotation, FromEulerXYZ(a_angles)), QuatConjugate(a_frame.rotationA)));
		result.translation = a_pivot - QuatRotate(result.rotation, a_frame.originA);
		return result;
	}

	/**
	 * @brief Stores a bone's integrated pose and velocity, the tail the advance and the contact clamp share.
	 * @param a_worldTranslation The bone's new world translation, valid whether or not the step stayed finite.
	 */
	void StoreBonePose(inout BoneState a_state, Quat a_jointRotation, Transform a_pose, float3 a_updatedVelocity,
		float3 a_parentAngularVelocity, SolverScales a_scales, out float3 a_worldTranslation)
	{
		a_state.rotation = a_pose.rotation;
		float3 worldVelocity = a_parentAngularVelocity + QuatRotate(a_jointRotation, a_updatedVelocity);
		float speed = length(worldVelocity);
		if (speed > a_scales.maxAngularSpeed)
			worldVelocity = worldVelocity * (a_scales.maxAngularSpeed / speed);
		a_state.angularVelocity = worldVelocity;
		a_worldTranslation = a_pose.translation;
	}

	/**
	 * @brief Reverts the axes a sphere that reached past a collider's allowance pushed further in.
	 *        Stores the clamped pose and velocity.
	 * @param a_advance What the bone's advance produced, including the angles a clamp may revert.
	 * @param a_offset The clamped sphere's offset from the pivot, taken before the update.
	 * @param a_normal The contact normal at the clamped sphere's updated centre.
	 */
	void ClampBoneContact(inout BoneState a_state, JointFrame a_frame, BoneAdvance a_advance, float3 a_offset,
		float3 a_normal, SolverScales a_scales, out float3 a_worldTranslation)
	{
		float3 updatedAngles = a_advance.updatedAngles;
		float3 updatedVelocity = a_advance.updatedVelocity;
		[unroll] for (uint axis = 0u; axis < 3u; ++axis)
		{
			float jacobian = dot(cross(QuatRotate(a_advance.jointRotation, AxisUnit(axis)), a_offset), a_normal);
			if (jacobian * (updatedAngles[axis] - a_advance.angles[axis]) < 0.0f) {
				updatedAngles[axis] = a_advance.angles[axis];
				updatedVelocity[axis] = 0.0f;
			}
		}
		Transform pose = PoseFor(a_advance.jointRotation, a_frame, a_advance.pivot, updatedAngles);
		StoreBonePose(a_state, a_advance.jointRotation, pose, updatedVelocity, a_advance.parentAngularVelocity, a_scales,
			a_worldTranslation);
	}

	/** @brief The rest relationship of a bone to its parent, as ChainSolver::Reset derives it. */
	Transform ResetTransform(Transform a_parentPose, JointFrame a_frame)
	{
		Quat rotation = QuatNormalized(QuatMul(QuatMul(a_parentPose.rotation, a_frame.rotationB), QuatConjugate(a_frame.rotationA)));
		Transform result;
		result.rotation = rotation;
		result.translation = a_parentPose.translation + QuatRotate(a_parentPose.rotation, a_frame.originB) -
		                     QuatRotate(rotation, a_frame.originA);
		return result;
	}

	/** @brief Clears the history a reset bone must forget, leaving only its rest rotation. */
	BoneState ResetBoneState(Transform a_restPose)
	{
		BoneState result;
		result.rotation = a_restPose.rotation;
		result.angularVelocity = float3(0.0f, 0.0f, 0.0f);
		result.previousPivot = float3(0.0f, 0.0f, 0.0f);
		result.previousPivotVelocity = float3(0.0f, 0.0f, 0.0f);
		result.previousParentRotation = QuatIdentity();
		result.hasHistory = false;
		result.hasVelocityHistory = false;
		result.limitsFreed = false;
		return result;
	}

	/** @brief The force of wind drag on a bone's hair segment. Mirrors WindForce in SolverCommon.h. */
	float3 WindForce(SolverScales a_scales, BoneParameters a_bone, float a_boneFactor, float3 a_axis, float3 a_origin, float3 a_velocity)
	{
		static const float kBulletPerSkyrimUnit = 0.01425f;
		static const float kMaxGustTime = 4096.0f;
		static const float kEpsilon = 1.1920929e-7f;
		static const float kWindDrag = 1.0e-6f;
		static const float kMaxWindAcceleration = 400.0f;
		static const float kGustSpeedPerAcceleration = 0.05f;
		static const float kMinGustSpeed = 180.0f;
		static const float kMaxGustSpeed = 1200.0f;
		static const float kCrosswindTimeScale = 0.004f;
		static const float kVerticalPhaseScale = 0.006f;
		static const float kLongGustRate = 0.55f;
		static const float kLongGustVertical = 0.35f;
		static const float kMidGustRate = 1.35f;
		static const float kMidGustFeedback = 0.5f;
		static const float kFlutterRate = 4.15f;
		static const float kFlutterVertical = 2.2f;
		static const float kPulseRate = 0.85f;
		static const float kPulseVertical = 0.6f;
		static const float kSidewaysMid = 0.13f;
		static const float kSidewaysFlutter = 0.06f;
		static const float kSidewaysBurst = 0.04f;
		static const float kLiftRate = 1.9f;
		static const float kLiftVertical = 1.4f;
		static const float kLiftGust = 0.018f;
		static const float kLiftFlutter = 0.01f;

		float magnitude = length(a_scales.wind);
		if (!(magnitude > kEpsilon) || !(a_boneFactor > kEpsilon) || !(a_bone.windArea > kEpsilon))
			return float3(0.0f, 0.0f, 0.0f);
		float3 relativeWind = a_scales.wind - a_velocity;
		float3 across = relativeWind - a_axis * dot(a_axis, relativeWind);
		float speed = length(across);
		if (!(speed > kEpsilon))
			return float3(0.0f, 0.0f, 0.0f);
		float3 force = across * (kWindDrag * a_bone.windArea * a_boneFactor * speed);
		float forceLength = length(force);
		float maximumForce = a_bone.mass * kMaxWindAcceleration;
		if (forceLength > maximumForce) {
			force *= maximumForce / forceLength;
			forceLength = maximumForce;
		}

		float time = fmod(a_scales.windTime, kMaxGustTime);
		float3 direction = a_scales.wind / magnitude;
		float3 up = float3(0.0f, 0.0f, 1.0f);
		float3 side = cross(direction, up);
		side = dot(side, side) < kEpsilon ? float3(1.0f, 0.0f, 0.0f) : normalize(side);

		float gustSpeed = clamp(magnitude * kGustSpeedPerAcceleration, kMinGustSpeed, kMaxGustSpeed);
		float3 origin = a_origin * kBulletPerSkyrimUnit;
		float advected = time - dot(origin, direction) / gustSpeed - dot(origin, side) * kCrosswindTimeScale;
		float vertical = origin.z * kVerticalPhaseScale;

		float longGust = sin(advected * kLongGustRate + vertical * kLongGustVertical);
		float midGust = sin(advected * kMidGustRate + vertical + longGust * kMidGustFeedback);
		float flutter = sin(advected * kFlutterRate + vertical * kFlutterVertical + midGust);
		float pulse = 0.5f + 0.5f * sin(advected * kPulseRate + vertical * kPulseVertical);
		float burst = pulse * pulse;

		float sideways = (midGust * kSidewaysMid + flutter * kSidewaysFlutter + burst * kSidewaysBurst) * forceLength;
		float lift = (sin(advected * kLiftRate + vertical * kLiftVertical) * kLiftGust + flutter * kLiftFlutter) * forceLength;
		return force + side * sideways + up * lift;
	}

	/** @brief Where a joint that integrated to a_angle, past a_limit, ends up. */
	float ReturnToLimit(float a_previous, float a_angle, float a_limit, float a_step)
	{
		bool wasInside = (a_angle > a_limit) ? a_previous <= a_limit : a_previous >= a_limit;
		if (wasInside)
			return a_limit;
		float stepLimit = kLimitReturnRate * a_step;
		float moved = a_previous + clamp(a_limit - a_previous, -stepLimit, stepLimit);
		return abs(a_angle - a_limit) < abs(moved - a_limit) ? a_angle : moved;
	}

	/**
	 * @brief Advances one bone by a_step seconds about its pivot.
	 * @param a_contact The contact springs the caller's collider query accumulated for this bone; all zero
	 *                  leaves the update exactly as it is for a bone with no collider in reach.
	 * @param a_windForce The wind force on the bone and, when chain loading, below it, from the substep's start.
	 * @param a_windMoment The moment of those forces about the world origin.
	 * @param a_advance What the advance produced, which a caller that finds a penetrating sphere hands to
	 *                  ClampBoneContact before it stores the pose.
	 * @param a_worldTranslation The bone's new world translation, valid whether or not the step stayed finite.
	 * @return false when the bone's new state stopped being finite, so the caller resets the tree.
	 */
	bool AdvanceBone(inout BoneState a_state, BoneParameters a_bone, JointFrame a_frame, JointSprings a_springs,
		Transform a_parentPose, SolverScales a_scales, float a_step, ContactTerms a_contact, float3 a_windForce, float3 a_windMoment, bool a_easeBack, out BoneAdvance a_advance,
		out float3 a_worldTranslation)
	{
		Quat jointRotation;
		float3 pivot;
		JointPose(a_parentPose, a_frame, jointRotation, pivot);

		float3 pivotVelocity = float3(0.0f, 0.0f, 0.0f);
		float3 pivotAcceleration = float3(0.0f, 0.0f, 0.0f);
		if (a_state.hasHistory) {
			pivotVelocity = (pivot - a_state.previousPivot) / a_step;
			if (a_state.hasVelocityHistory)
				pivotAcceleration = (pivotVelocity - a_state.previousPivotVelocity) / a_step;
			a_state.hasVelocityHistory = true;
		}
		a_state.previousPivot = pivot;
		a_state.previousPivotVelocity = pivotVelocity;

		float3 parentAngularVelocity = float3(0.0f, 0.0f, 0.0f);
		if (a_state.hasHistory)
			parentAngularVelocity = ToRotationVector(QuatMul(a_parentPose.rotation, QuatConjugate(a_state.previousParentRotation))) / a_step;
		a_state.previousParentRotation = a_parentPose.rotation;
		a_state.hasHistory = true;

		float3 angles = ToEulerXYZ(QuatMul(QuatMul(QuatConjugate(jointRotation), a_state.rotation), a_frame.rotationA));
		float3 relativeAngularVelocity = QuatRotate(QuatConjugate(jointRotation), a_state.angularVelocity - parentAngularVelocity);

		float3 centreOffset = -QuatRotate(a_state.rotation, a_frame.originA);
		float inertia = a_bone.inertiaMean + a_bone.mass * dot(centreOffset, centreOffset) + a_contact.subtreeInertia;
		if (!(inertia > kMinimumInertia))
			inertia = kMinimumInertia;

		float3 gravityVector = float3(0.0f, 0.0f, -a_scales.gravity);
		float3 externalTorque =
			cross(centreOffset, gravityVector * (a_bone.mass * a_bone.gravityFactor * a_scales.gravityScale)) +
			cross(centreOffset, -pivotAcceleration * a_bone.mass) + a_contact.subtreeGravityTorque - cross(a_contact.subtreeMassMoment, pivotAcceleration);
		externalTorque += a_windMoment - cross(pivot, a_windForce);
		float3 jointTorque = QuatRotate(QuatConjugate(jointRotation), externalTorque);

		float dampingBase = 1.0f - a_bone.angularDamping * a_scales.dampingScale;
		if (dampingBase < 0.0f)
			dampingBase = 0.0f;
		float bodyDamping = pow(abs(dampingBase), a_step);

		float3 updatedAngles = float3(0.0f, 0.0f, 0.0f);
		float3 updatedVelocity = float3(0.0f, 0.0f, 0.0f);
		bool beyondLimits = false;
		[unroll] for (uint axis = 0u; axis < 3u; ++axis)
		{
			float stiffness = a_springs.enabled ? a_springs.stiffness[axis] * a_scales.stiffnessScale : 0.0f;
			float damping = a_springs.enabled ? a_springs.damping[axis] * a_scales.dampingScale : 0.0f;
			float velocity = (relativeAngularVelocity[axis] +
								 (a_step / inertia) * (jointTorque[axis] + a_contact.torque[axis] +
														  stiffness * (a_springs.equilibrium[axis] - angles[axis]))) /
			                 (1.0f + a_step * (damping + a_contact.damping[axis]) / inertia +
								 a_step * a_step * (stiffness + a_contact.stiffness[axis]) / inertia);
			velocity *= bodyDamping;
			float angle = angles[axis] + a_step * velocity;
			if (a_springs.lower[axis] <= a_springs.upper[axis]) {
				if (angle < a_springs.lower[axis]) {
					angle = a_easeBack ? ReturnToLimit(angles[axis], angle, a_springs.lower[axis], a_step) : a_springs.lower[axis];
					beyondLimits = beyondLimits || angle < a_springs.lower[axis];
					if (velocity < 0.0f)
						velocity = 0.0f;
				} else if (angle > a_springs.upper[axis]) {
					angle = a_easeBack ? ReturnToLimit(angles[axis], angle, a_springs.upper[axis], a_step) : a_springs.upper[axis];
					beyondLimits = beyondLimits || angle > a_springs.upper[axis];
					if (velocity > 0.0f)
						velocity = 0.0f;
				}
				if (a_springs.lower[axis] == a_springs.upper[axis])
					velocity = 0.0f;
			}
			updatedAngles[axis] = angle;
			updatedVelocity[axis] = velocity;
		}

		Transform pose = PoseFor(jointRotation, a_frame, pivot, updatedAngles);
		StoreBonePose(a_state, jointRotation, pose, updatedVelocity, parentAngularVelocity, a_scales, a_worldTranslation);
		a_advance.jointRotation = jointRotation;
		a_advance.pivot = pivot;
		a_advance.pose = pose;
		a_advance.angles = angles;
		a_advance.updatedAngles = updatedAngles;
		a_advance.updatedVelocity = updatedVelocity;
		a_advance.parentAngularVelocity = parentAngularVelocity;
		a_advance.beyondLimits = beyondLimits;

		Transform world;
		world.rotation = a_state.rotation;
		world.translation = a_worldTranslation;
		return IsFiniteTransform(world) && IsFiniteVector(a_state.angularVelocity);
	}
}

#endif
