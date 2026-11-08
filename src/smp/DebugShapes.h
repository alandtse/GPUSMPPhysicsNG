#pragma once

#include "Math.h"

#include <cstdint>
#include <vector>

namespace SmpPhysics
{
	/** @brief What a debug line shows; each has its own colour. */
	enum class DebugColor : std::uint32_t
	{
		Held,         ///< A strand a hand holds (yellow).
		InReach,      ///< A strand a closing grip would take (green).
		Influenced,   ///< Hair a hand's capsules push.
		Pull,         ///< From a held point to where the hand wants it.
		Hand,         ///< A capsule of another body's hand or held item.
		Collider,     ///< A proxy of the wearer's own head, body or armor that hair collides with.
		Joint,        ///< A bone of the simulated chain, drawn from its parent, with a cross at its origin.
		Stretched,    ///< A joint whose length differs from its rest length by more than the tolerance.
		Skeleton,     ///< A node of an actor's skeleton, drawn from its parent.
		Fingers,      ///< A node of the player's hand skeleton, fingers included.
		SphereGap,    ///< A hair collision sphere with clear space between it and the nearest collision surface.
		SphereTouch,  ///< A hair collision sphere resting on a collision surface.
		Count
	};

	/** @brief One line of the debug overlay, in world space. */
	struct DebugSegment
	{
		Vec3 start{};
		Vec3 end{};
		DebugColor color{ DebugColor::Held };
	};

	/** @brief Appends a ring of a_radius around a_centre in the plane perpendicular to a_axis. */
	void AppendRing(std::vector<DebugSegment>& a_out, const Vec3& a_centre, const Vec3& a_axis, float a_radius, DebugColor a_color);

	/** @brief The distance from a_point to the triangle a_a, a_b, a_c, and the nearest point of it in a_closest. */
	[[nodiscard]] float DistanceToTriangle(const Vec3& a_point, const Vec3& a_a, const Vec3& a_b, const Vec3& a_c, Vec3& a_closest) noexcept;

	/** @brief An actor and how far it is from the camera, for choosing which actors the overlays draw. */
	struct ActorDistance
	{
		std::uint32_t actor{};
		float distance{};
	};

	/** @brief The a_max nearest distinct actors of a_candidates, nearest first; an actor listed more than once counts once, at its nearest. */
	[[nodiscard]] std::vector<std::uint32_t> NearestActors(std::vector<ActorDistance> a_candidates, int a_max);

	/** @brief Appends a three-axis cross of half-length a_size at a_centre. */
	void AppendCross(std::vector<DebugSegment>& a_out, const Vec3& a_centre, float a_size, DebugColor a_color);

	/** @brief Appends the outline of a capsule: a ring at each end and midway, four lines along it, or three rings for a sphere. */
	void AppendCapsuleOutline(std::vector<DebugSegment>& a_out, const Vec3& a_endA, const Vec3& a_endB, float a_radius, DebugColor a_color);

	/** @brief Appends the three principal ellipses of an ellipsoid whose axes are a_orientation rotated, with a_halfExtents along them. */
	void AppendEllipsoidOutline(std::vector<DebugSegment>& a_out, const Vec3& a_centre, const Quat& a_orientation, const Vec3& a_halfExtents, DebugColor a_color);
}
