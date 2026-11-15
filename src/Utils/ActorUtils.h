

#pragma once
#include "RE/Skyrim.h"
#include <functional>
#include <optional>
#include <span>
#include <string>
#include <vector>

namespace RE
{
	class bhkNiCollisionObject;
	class hkpShape;
	class NiPoint3;
}

namespace Util
{
	/** @brief A Havok or Gamebryo point as the float3 the shaders and constant buffers use. */
	inline float3 ToFloat3(const RE::NiPoint3& a_point)
	{
		return { a_point.x, a_point.y, a_point.z };
	}

	/** @brief World-space capsule used to approximate an actor collision shape. */
	struct ShapeCollisionCapsule
	{
		RE::NiPoint3 pointA;
		RE::NiPoint3 pointB;
		float radius;
	};

	/**
	 * @brief Extracts a world-space capsule from a collision object.
	 *
	 * Native capsules retain their endpoints and radius. Other supported shapes
	 * are returned as degenerate capsules with coincident endpoints.
	 * @param collisionObj Collision object whose Havok shape is queried.
	 * @param capsule Resulting world-space collision capsule.
	 * @return True when a supported shape was extracted.
	 */
	bool GetShapeCollisionCapsule(RE::bhkNiCollisionObject* collisionObj, ShapeCollisionCapsule& capsule);

	/**
	 * @brief Reduces a rigid body's shape to world-space capsules, so each part of a weapon is covered by its own capsule.
	 * @details A capsule shape is used as it is. Any other shape, such as a box or convex hull, becomes a capsule along the longest
	 *          side of its bounds in the body's own frame, with the mean of the other two half sides as its radius, so a haft or a
	 *          blade stays long and thin. A shape list gives one capsule per child, such as one for an axe's haft and one for its
	 *          head, or a single capsule from the bounding box it keeps when it has more children than a_capsules holds.
	 * @param a_body Rigid body that is in a physics world.
	 * @param a_capsules Receives the capsules.
	 * @return How many entries of a_capsules were filled.
	 */
	size_t GetRigidBodyCapsules(RE::bhkRigidBody* a_body, std::span<ShapeCollisionCapsule> a_capsules);

	/** @brief The first collision along a segment other than the actor's own: how far along it, and the reference that owns it, or 0 for terrain and other unowned geometry. */
	struct OccluderHit
	{
		float distance{};
		RE::FormID reference{};
	};

	/**
	 * @brief The first collision along the segment from a_from to a_to other than a_actor's own.
	 * @return The hit, or nothing when the segment is clear.
	 */
	[[nodiscard]] std::optional<OccluderHit> FirstOccluder(RE::Actor& a_actor, const RE::NiPoint3& a_from, const RE::NiPoint3& a_to);

	/**
	 * @brief Visits actors currently resolvable from the player and high-process actor list.
	 * @param a_callback Callback invoked once for each actor.
	 */
	void ForEachLoadedActor(const std::function<void(RE::Actor*)>& a_callback);

	/**
	 * @brief Visits every geometry below a scenegraph root.
	 * @param a_root Scenegraph root to traverse.
	 * @param a_callback Callback invoked for each geometry.
	 */
	void ForEachGeometry(RE::NiAVObject* a_root, const std::function<void(RE::BSGeometry*)>& a_callback);

	/**
	 * @brief Visits geometry in both first- and third-person actor roots.
	 * @param a_actor Actor whose loaded geometry should be traversed.
	 * @param a_callback Callback invoked for each geometry.
	 */
	void ForEachActorGeometry(RE::Actor* a_actor, const std::function<void(RE::BSGeometry*)>& a_callback);

	/**
     * @brief Extracts the shape bounds from a hkpShape.
     * @param shape Pointer to the shape.
     * @param radius Output: radius of the shape.
     * @return True if bounds were successfully extracted, false otherwise.
     */
	bool ExtractShapeBound(const RE::hkpShape* shape, float& radius);
}
