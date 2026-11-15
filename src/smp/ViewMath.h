#pragma once

#include <algorithm>
#include <cmath>

namespace SmpPhysics
{
	/** @brief Frustum sides a view-projection matrix gives: left, right, bottom, top. */
	inline constexpr int kFrustumSides = 4;
	/** @brief The matrix column that holds a clip-space w, which every side plane adds or subtracts its axis column from. */
	inline constexpr int kClipWColumn = 3;
	inline constexpr float kPercent = 100.0f;

	/**
	 * @brief Whether a sphere touches the volume a row-vector view-projection matrix sees.
	 * @param a_viewProj The 4x4 matrix, rows as the engine stores them.
	 * @param a_x,a_y,a_z The sphere's centre relative to the eye.
	 * @details The near and far planes are not tested, so a sphere behind the eye but inside the side planes counts as in view.
	 */
	[[nodiscard]] inline bool SphereInFrustum(const float (&a_viewProj)[4][4], float a_x, float a_y, float a_z, float a_radius) noexcept
	{
		for (int side = 0; side < kFrustumSides; ++side) {
			const int axis = side / 2;
			const float sign = side % 2 == 0 ? 1.0f : -1.0f;
			const float px = a_viewProj[0][kClipWColumn] + sign * a_viewProj[0][axis];
			const float py = a_viewProj[1][kClipWColumn] + sign * a_viewProj[1][axis];
			const float pz = a_viewProj[2][kClipWColumn] + sign * a_viewProj[2][axis];
			const float pw = a_viewProj[3][kClipWColumn] + sign * a_viewProj[3][axis];
			const float length = std::sqrt(px * px + py * py + pz * pz);
			if (length > 0.0f && (px * a_x + py * a_y + pz * a_z + pw) / length < -a_radius)
				return false;
		}
		return true;
	}

	/** @brief The percent of the screen height a sphere covers, or 0 when it is behind the eye; a_x,a_y,a_z is its centre relative to the eye. */
	[[nodiscard]] inline float SphereScreenHeightPercent(const float (&a_viewProj)[4][4], float a_x, float a_y, float a_z, float a_radius) noexcept
	{
		const float depth = a_viewProj[0][kClipWColumn] * a_x + a_viewProj[1][kClipWColumn] * a_y + a_viewProj[2][kClipWColumn] * a_z + a_viewProj[3][kClipWColumn];
		if (depth <= 0.0f)
			return 0.0f;
		const float scale = std::sqrt(a_viewProj[0][1] * a_viewProj[0][1] + a_viewProj[1][1] * a_viewProj[1][1] + a_viewProj[2][1] * a_viewProj[2][1]);
		return kPercent * a_radius * scale / depth;
	}
}
