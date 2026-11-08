#pragma once

#include "Collision.h"
#include "SkinnedSurface.h"

#include <vector>

namespace SmpPhysics
{
	/** @brief How far, in Skyrim units, from a hair point toward the wind the wearer's own body and armour are searched for shelter. */
	inline constexpr float kSelfShelterReach = 150.0f;

	/**
	 * @brief The share of a_samples the wind reaches, given the wearer's own head, body and worn meshes.
	 * @details Each sample casts a segment of a_reach toward the wind's source. It is sheltered when the segment
	 *          crosses one of the claim's analytic colliders or one of its posed collision surfaces, which are
	 *          the head, body and armour the hair collides with. A sample that already lies inside a collider is
	 *          not sheltered by it, and ground planes never shelter. The world's own shelter is a separate test.
	 * @param a_driverWorld This frame's world transform of each collision driver.
	 * @param a_towardSource Unit vector pointing from the hair toward where the wind comes from.
	 * @return 0 when every sample is sheltered, 1 when none is or a_samples is empty.
	 */
	[[nodiscard]] float SelfWindExposure(const std::vector<Collider>& a_colliders, const std::vector<SkinnedSurface>& a_surfaces,
		const std::vector<Transform>& a_driverWorld, const std::vector<Vec3>& a_samples, const Vec3& a_towardSource, float a_reach);
}
