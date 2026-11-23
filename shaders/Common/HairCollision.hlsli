#ifndef __HAIR_COLLISION_DEPENDENCY_HLSL__
#define __HAIR_COLLISION_DEPENDENCY_HLSL__

// Pure functions for the collision contact queries the contact passes make.
// Every value-returning function has a single exit, or FXC reports the return as uninitialized (X4000).

namespace HairCollision
{
	/** @brief Length below which a vector is treated as having no usable direction. */
	static const float kDirectionEpsilon = 1e-8f;
	/** @brief Smallest half-extent an ellipsoid keeps, below which its scaled-space projection would divide by zero. Mirrors kMinimumHalfExtent in Collision.h. */
	static const float kMinimumHalfExtent = 1e-3f;
	/** @brief Colliders a sphere's rest-exemption mask can name. Mirrors kMaximumExemptColliders in Collision.h. */
	static const uint kMaximumExemptColliders = 32u;
	/** @brief Surfaces a bone's surface mask can name. Mirrors kMaskedSurfaceLimit in Math.h. */
	static const uint kMaskedSurfaceLimit = 32u;
	/** @brief |z| of an axis above which the x axis, not z, is the reference for building a perpendicular. */
	static const float kNearVerticalAxis = 0.9f;
	/** @brief Slack a bone's bound sphere carries when it decides whether the whole bone can be skipped. */
	static const float kBoundCullPad = 0.5f;
	/** @brief Collider kind values, mirroring Collider::Type in Collision.h; an ellipsoid is 0. */
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

	/** @brief A quaternion stored as (w, x, y, z) in a float4. */
	Quat LoadQuat(float4 a_value)
	{
		Quat result;
		result.w = a_value.x;
		result.x = a_value.y;
		result.y = a_value.z;
		result.z = a_value.w;
		return result;
	}

	Transform LoadTransform(float4 a_rotation, float4 a_translation)
	{
		Transform result;
		result.rotation = LoadQuat(a_rotation);
		result.translation = a_translation.xyz;
		return result;
	}

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
		uint driver;  // the global driver slot that carries the contacted surface or collider, plus one; 0 when unknown
		uint driverOther;  // a second driver the contacted face is weighted to, plus one; 0 when it has none
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
			float3 reference = abs(a_axis.z) > kNearVerticalAxis ? float3(1.0f, 0.0f, 0.0f) : float3(0.0f, 0.0f, 1.0f);
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
		result.driver = 0u;
		result.driverOther = 0u;
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
			a_best.driver = 0u;
			a_best.driverOther = 0u;
		}
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
}

#endif
