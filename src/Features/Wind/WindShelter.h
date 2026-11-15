#pragma once

#include <array>

/** @brief How much of the weather wind reaches a point, judged from the geometry upwind of it. Pure maths; the game queries are in WindShelterQuery.h. */
namespace WindShelter
{
	/** @brief One upwind shelter ray: its heading and elevation relative to the wind, and how much of the shelter it speaks for. */
	struct Ray
	{
		float azimuthDegrees;    ///< Heading offset from the direction the wind comes from.
		float elevationDegrees;  ///< Degrees above the horizontal.
		float weight;            ///< Share of the shelter a block on this ray accounts for; the weights sum to one.
	};

	/**
	 * @brief The three rays the world's shelter is estimated from.
	 * @details A level-ish ray in the wind's heading catches the wall or hill the wind is running into, and two
	 *          rays 20 degrees either side and higher catch the obstacle's width and height.
	 */
	inline constexpr std::array<Ray, 3> kRays{ Ray{ 0.0f, 6.0f, 0.55f }, Ray{ -20.0f, 14.0f, 0.225f }, Ray{ 20.0f, 14.0f, 0.225f } };
	/** @brief A hit this close on the first ray is a wall against the point, which shelters fully whatever its height. */
	inline constexpr float kNearWall = 50.0f;
	/** @brief How far, in Skyrim units, each shelter ray reaches. */
	inline constexpr float kReach = 3000.0f;

	/**
	 * @brief The share of the wind the world leaves, from which of kRays were blocked.
	 * @param a_hitDistance Per ray, how far it travelled before it hit, or a negative value for a clear ray.
	 * @return 1 for open ground, down to 0 when every ray is blocked or a wall stands against the point.
	 */
	[[nodiscard]] float Exposure(const std::array<float, 3>& a_hitDistance);
}
