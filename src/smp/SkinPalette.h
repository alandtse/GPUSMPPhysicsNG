#pragma once

#include "Math.h"

#include <array>
#include <cstdint>

namespace SmpPhysics
{
	/** @brief A palette slot the solver does not simulate, which keeps its engine matrix. */
	inline constexpr std::uint32_t kStaticBone = 0xFFFFFFFFu;

	/** @brief Three rows of a 3x4 matrix, row major, translation in the fourth column: the layout of one bone in the engine's skinning buffers. */
	using SkinRows = std::array<float, 12>;

	/**
	 * @brief Packs a skin-to-bone transform as SkinRows with its scale folded into the rotation.
	 * @param a_rotation The transform's 3x3 rotation, row major.
	 */
	[[nodiscard]] inline SkinRows PackSkinToBone(const float (&a_rotation)[3][3], float a_scale, float a_translationX, float a_translationY, float a_translationZ) noexcept
	{
		return { a_rotation[0][0] * a_scale, a_rotation[0][1] * a_scale, a_rotation[0][2] * a_scale, a_translationX,
			a_rotation[1][0] * a_scale, a_rotation[1][1] * a_scale, a_rotation[1][2] * a_scale, a_translationY,
			a_rotation[2][0] * a_scale, a_rotation[2][1] * a_scale, a_rotation[2][2] * a_scale, a_translationZ };
	}

	/**
	 * @brief The matrix the engine uploads for a bone: its world transform composed with its skin-to-bone transform.
	 * @param a_world The bone's world pose as the solver tracks it, which carries no scale.
	 * @param a_worldScale The scale of the bone's node, which the solver leaves untouched.
	 * @param a_skinToBone The bone's skin-to-bone transform from PackSkinToBone.
	 */
	[[nodiscard]] inline SkinRows ComposeSkinRows(const Transform& a_world, float a_worldScale, const SkinRows& a_skinToBone) noexcept
	{
		float rotation[3][3];
		ToRotationMatrix(a_world.rotation, rotation);
		const float translation[3] = { a_world.translation.x, a_world.translation.y, a_world.translation.z };
		SkinRows result{};
		for (int row = 0; row < 3; ++row) {
			for (int column = 0; column < 4; ++column) {
				float sum = 0.0f;
				for (int inner = 0; inner < 3; ++inner)
					sum += rotation[row][inner] * a_skinToBone[static_cast<std::size_t>(inner * 4 + column)];
				result[static_cast<std::size_t>(row * 4 + column)] = sum * a_worldScale;
			}
			result[static_cast<std::size_t>(row * 4 + 3)] += translation[row];
		}
		return result;
	}
}
