#pragma once

#include "Features/Wind/WindShelter.h"

#include <cstdint>

namespace WindShelter
{
	/** @brief What EstimateShelter found upwind of a point. */
	struct Estimate
	{
		float exposure{ 1.0f };                                   ///< 1 for open ground down to 0 for full shelter.
		std::array<float, 3> hitDistance{ -1.0f, -1.0f, -1.0f };  ///< Per kRays, how far it travelled, or negative when clear.
		float blockerDistance{ -1.0f };                           ///< Distance to the first-ray hit, else the nearest other hit, or negative.
		std::uint32_t blockerReference{};                         ///< The reference that owns that hit, or 0 for terrain and other unowned geometry.
	};

	/**
	 * @brief Casts kRays upwind of a point and estimates how much of the wind the world leaves it.
	 * @details Runs Havok ray queries, so call it on the game thread, and spread callers across a slow tick.
	 *          a_ignored's own collision is skipped; any other character, the player included, shelters.
	 * @param a_origin Where the wind is felt, in game units.
	 * @param a_windVelocity The world wind vector; only its horizontal heading is used. Exposure is 1 when it has none.
	 */
	[[nodiscard]] Estimate EstimateShelter(RE::Actor& a_ignored, const RE::NiPoint3& a_origin, const RE::NiPoint3& a_windVelocity);
}
