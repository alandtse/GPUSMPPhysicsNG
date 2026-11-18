#pragma once

#include <cstddef>

namespace SmpPhysics
{
	/**
	 * @brief Whether a rig whose collision could not be built may still be claimed, without collision.
	 * @param a_noGeometry The rig's meshes gave no collision geometry: a collision add-on whose meshes are not on the actor, or a hair whose
	 *                     mesh matches the rig but weights no vertex to a dynamic bone above the file's threshold.
	 * @details hdtSMP64 simulates such a rig without collision, and its dynamic bones still move on their own. A rig with no dynamic bone
	 *          has nothing to simulate.
	 */
	[[nodiscard]] constexpr bool MayClaimWithoutCollision(bool a_noGeometry, int a_dynamicBones) noexcept
	{
		return a_noGeometry && a_dynamicBones > 0;
	}

	/**
	 * @brief Whether a hair mesh that built no collision spheres is a valid rig with nothing to collide, not a mesh that does not belong to it.
	 * @details A mesh sharing no skin bone with the rig is the wrong mesh for the file; one that shares bones but has no vertex weighted
	 *          to a dynamic bone above the threshold is a hair the file simply gives no collision to.
	 */
	[[nodiscard]] constexpr bool HasNoCollisionGeometry(std::size_t a_mappedSkinBones, int a_sphereCount) noexcept
	{
		return a_sphereCount == 0 && a_mappedSkinBones > 0;
	}
}
