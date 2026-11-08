#pragma once

#include "pch.h"

#include <span>

/**
 * @brief The wind hair is pushed by: Community Shaders' published wind when it is installed, else the weather's wind
 *        read the way hdtSMP64's weather manager reads it, one world-wide vector from the sky.
 */
struct Wind
{
	/** @brief The wind at a point; the weather wind is uniform, so only windExcludingHavokImpulses is filled. */
	struct PublishedWindSample
	{
		float3 windExcludingHavokImpulses{};
	};

	/** @brief Fetches Community Shaders' API once every plugin has loaded; without it the weather wind is used. */
	void ConnectCommunityShaders();

	/** @return false when there is no sky or the player is in an interior or a worldspace without weather. */
	[[nodiscard]] bool SamplePublishedWind(std::span<const float3> a_worldPositions, std::span<PublishedWindSample> a_samples) const noexcept;
};

namespace globals::features
{
	inline Wind wind;
}
