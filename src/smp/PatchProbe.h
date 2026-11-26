#pragma once

#include "smp/Math.h"

#include <cstdint>
#include <span>

namespace SmpPhysics
{
	/** @brief How far a set of skin points sits outside a cloth surface, measured at one pose. */
	struct PatchSample
	{
		std::uint32_t measured{};   ///< Skin points with a cloth face within the reach.
		std::uint32_t outside{};    ///< Of those, how many lie in front of the face by more than the threshold.
		float maxOutside{};         ///< The greatest distance in front of a face, zero when none is.
		std::uint32_t worst{};      ///< Index into the skin points of the one at maxOutside.
		std::uint32_t worstFace{};  ///< Index of the cloth face that point was measured against.
	};

	/**
	 * @brief Counts the skin points that lie in front of the cloth, the way a dress is meant to hide them.
	 * @param a_skin Skin points in world space.
	 * @param a_cloth Cloth vertices in world space.
	 * @param a_indices Three cloth vertex indices per face.
	 * @param a_orientation +1 when a face's winding points away from the body, -1 when it points toward it.
	 * @param a_reach Faces farther than this from a point are ignored, so a far layer is not read as the same cloth.
	 * @param a_threshold Distance in front of the face above which a point counts as outside.
	 */
	[[nodiscard]] PatchSample MeasureSkinOutsideCloth(std::span<const Vec3> a_skin, std::span<const Vec3> a_cloth,
		std::span<const std::uint32_t> a_indices, float a_orientation, float a_reach, float a_threshold) noexcept;
}
