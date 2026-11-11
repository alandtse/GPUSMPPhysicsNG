// SPDX-License-Identifier: GPL-3.0-or-later WITH LicenseRef-Modding-Exception

#pragma once

#include "smp/Collision.h"

#include <cstdint>
#include <string>
#include <vector>

namespace SmpPhysics
{
	/** @brief The target that switches off a piece's collision with the body proxies rather than with a surface. */
	inline constexpr const char* kOverrideColliders = "colliders";

	/** @brief One debugging switch: a piece of a rig stops colliding with a named surface, or with every proxy. */
	struct CollisionOverride
	{
		std::string piece;   ///< The rig shape whose bones' spheres are affected.
		std::string target;  ///< A surface's name, or kOverrideColliders.
	};

	/**
	 * @brief Applies debugging switches to a claim's packed collision after it is built.
	 * @param a_boneShapeName Per rig bone, the shape its spheres came from.
	 * @param a_surfaceNames Per surface, its name; only the first 32 can be masked.
	 * @param a_spheres Per rig bone; a bone of a switched piece gets every collider exempt for kOverrideColliders.
	 * @param a_boneSurfaceMask Per rig bone; created with every surface allowed when a surface switch needs it and it is empty.
	 * @return How many bones a switch changed.
	 */
	std::size_t ApplyCollisionOverrides(const std::vector<CollisionOverride>& a_overrides, const std::vector<std::string>& a_boneShapeName,
		const std::vector<std::string>& a_surfaceNames, std::vector<std::vector<BoneSphere>>& a_spheres, std::vector<std::uint32_t>& a_boneSurfaceMask);
}
