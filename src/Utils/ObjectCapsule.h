#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <limits>

namespace Util::ObjectCapsule
{
	inline constexpr float kMinRadius = 1.0f;

	/** @brief Values of RE::hkpMotion::MotionType this module reasons about, restated so it needs no CommonLib. */
	enum class Motion : int
	{
		kInvalid = 0,
		kDynamic = 1,
		kSphereInertia = 2,
		kBoxInertia = 3,
		kKeyframed = 4,
		kFixed = 5,
		kThinBoxInertia = 6,
		kCharacter = 7,
	};

	/** @brief A loose object's bound box as a capsule along its longest axis. */
	struct Shape
	{
		int axis = 2;  ///< The axis the capsule runs along: 0 for x, 1 for y, 2 for z.
		float radius = 0.0f;
		float halfLength = 0.0f;
	};

	/**
	 * @brief Whether a havok motion type is a loose object a cloth mesh should collide with.
	 * @details Only free rigid bodies qualify. A door or a lever is kKeyframed: its motion is fixed per frame, but the engine
	 *          does not flag it kFixed, so keyframes have to be left out by name rather than treated as statics.
	 */
	[[nodiscard]] constexpr bool IsLoose(Motion a_motion)
	{
		return a_motion == Motion::kDynamic || a_motion == Motion::kSphereInertia || a_motion == Motion::kBoxInertia ||
		       a_motion == Motion::kThinBoxInertia;
	}

	/**
	 * @brief Fits a capsule to a box's half extents, running along its longest axis.
	 * @details The radius is the thinnest of the other two axes, so a panel stays a panel. Taking the widest instead turns a
	 *          262 by 32 by 333 door into a 131-radius blob that reaches through an actor standing beside it.
	 * @param a_half Half extents, all positive and finite.
	 * @param a_boundRadius The object's bounding sphere radius, 0 when it has none.
	 * @return The capsule's axis, radius and half length.
	 */
	[[nodiscard]] inline Shape Fit(const std::array<float, 3>& a_half, float a_boundRadius)
	{
		const int longest = a_half[0] >= a_half[1] && a_half[0] >= a_half[2] ? 0 : (a_half[1] >= a_half[2] ? 1 : 2);
		float radius = std::numeric_limits<float>::max();
		for (int axis = 0; axis < 3; ++axis) {
			if (axis != longest)
				radius = std::min(radius, a_half[static_cast<std::size_t>(axis)]);
		}
		radius = std::max(radius, kMinRadius);
		if (a_boundRadius > 0.0f)
			radius = std::min(radius, a_boundRadius);
		const float halfLength = std::max(a_half[static_cast<std::size_t>(longest)] - radius, 0.0f);
		return Shape{ longest, radius, halfLength };
	}
}
