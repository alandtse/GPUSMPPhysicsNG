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
}
