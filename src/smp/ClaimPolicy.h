#pragma once

namespace SmpPhysics
{
	/**
	 * @brief Whether a rig whose collision meshes are not on the actor may still be claimed, without collision.
	 * @details A rig of only triangle shapes is a body collision add-on: the meshes it names exist only when the
	 *          actor wears the matching collision body, and without them hdtSMP64 has no collision for it either.
	 *          Its dynamic bones still move on their own. A rig with no dynamic bone has nothing to simulate.
	 */
	[[nodiscard]] constexpr bool MayClaimWithoutCollision(bool a_meshesAbsent, int a_dynamicBones) noexcept
	{
		return a_meshesAbsent && a_dynamicBones > 0;
	}
}
