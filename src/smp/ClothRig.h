// SPDX-License-Identifier: GPL-3.0-or-later WITH LicenseRef-Modding-Exception

#pragma once

#include <cstddef>

namespace SmpPhysics
{
	/**
	 * @brief Whether a rig is a skirt-style cloth rig: it declares per-vertex shapes, but none is skinned to a bone it simulates (they are
	 *        body collision), and a per-triangle shape is. Hair rigs skin their per-vertex shape to the simulated bones.
	 */
	[[nodiscard]] constexpr bool IsClothRig(std::size_t a_vertexShapes, std::size_t a_vertexShapesOnSimulatedBones, std::size_t a_triangleShapesOnSimulatedBones) noexcept
	{
		return a_vertexShapes > 0 && a_vertexShapesOnSimulatedBones == 0 && a_triangleShapesOnSimulatedBones > 0;
	}

	/**
	 * @brief Whether collision spheres are built shape by shape rather than from one merged set of vertex-shape vertices: a rig with no
	 *        per-vertex shape, a cloth rig, or one whose per-triangle shape (a dress) is skinned to simulated bones and so collides on its own.
	 */
	[[nodiscard]] constexpr bool BuildsSpheresPerShape(std::size_t a_vertexShapes, bool a_clothRig, std::size_t a_triangleShapesOnSimulatedBones) noexcept
	{
		return a_vertexShapes == 0 || a_clothRig || a_triangleShapesOnSimulatedBones > 0;
	}

	/** @brief The penetration a rig's own per-triangle shape must allow before it holds a sphere deep behind it, in Skyrim units. */
	inline constexpr float kDeepPushPenetration = 0.5f;

	/**
	 * @brief Whether a rig surface also holds a sphere that rests deep behind it (kDeepPushReach): in a rig of several sphere shapes, for
	 *        every surface but the rig's own per-triangle shapes that allow less than kDeepPushPenetration.
	 * @details hdtSMP64 collides a per-vertex shape as spheres, which push from any side, and a per-triangle shape as a one-sided window
	 *          as deep as its penetration. A file that gives a thin one rests spheres behind it (a soft body over its rigid body), and
	 *          pushing them out fights the rig's joints for good; one a unit deep is the body under a skirt, which has to push a bone
	 *          back out.
	 */
	[[nodiscard]] constexpr bool HoldsDeepSpheres(std::size_t a_sphereShapes, bool a_ownTriangleShape, float a_penetration) noexcept
	{
		return a_sphereShapes > 1 && (!a_ownTriangleShape || a_penetration >= kDeepPushPenetration);
	}
}
