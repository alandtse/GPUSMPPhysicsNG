#pragma once

#include "Utils/BodyCapsules.h"

#include <vector>

namespace Util::HavokObjects
{
	/** @brief Most object capsules one query returns. */
	inline constexpr size_t MaxObjectCapsules = 16;

	/**
	 * @brief Loose objects the havok world is moving or can move, as capsules in world space.
	 * @details Each is the longest axis of the reference's bound box. Actors, fixed collision (statics) and objects without a
	 *          rigid body are left out. The candidate list is refreshed a few times a second, the capsules every call.
	 * @param a_centre Middle of the region the caller cares about.
	 * @param a_radius Radius of that region; an object whose capsule is farther than a_radius plus a_reach is skipped.
	 * @param a_reach Extra distance past a_radius that still counts.
	 * @return At most MaxObjectCapsules capsules, nearest first. `owner` holds the object's form id.
	 */
	[[nodiscard]] std::vector<BodyCapsules::Capsule> Near(const RE::NiPoint3& a_centre, float a_radius, float a_reach);
}
