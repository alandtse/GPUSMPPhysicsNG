#include "DebugShapes.h"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace SmpPhysics
{
	namespace
	{
		constexpr int kRingSegments = 12;
		constexpr float kSphereLengthEpsilon = 1e-3f;

		/** @brief A closed polyline of kRingSegments lines around a_centre in the plane of a_u and a_v, which carry the radii. */
		void AppendRing(std::vector<DebugSegment>& a_out, const Vec3& a_centre, const Vec3& a_u, const Vec3& a_v, DebugColor a_color)
		{
			Vec3 previous = a_centre + a_u;
			for (int step = 1; step <= kRingSegments; ++step) {
				const float angle = 2.0f * std::numbers::pi_v<float> * static_cast<float>(step) / static_cast<float>(kRingSegments);
				const Vec3 next = a_centre + a_u * std::cos(angle) + a_v * std::sin(angle);
				a_out.push_back({ previous, next, a_color });
				previous = next;
			}
		}
	}

	void AppendRing(std::vector<DebugSegment>& a_out, const Vec3& a_centre, const Vec3& a_axis, float a_radius, DebugColor a_color)
	{
		if (!(a_radius > 0.0f) || !std::isfinite(a_radius) || Length(a_axis) < kSphereLengthEpsilon)
			return;
		const Vec3 direction = Normalized(a_axis);
		const Vec3 reference = std::abs(direction.z) > 0.9f ? Vec3{ 1.0f, 0.0f, 0.0f } : Vec3{ 0.0f, 0.0f, 1.0f };
		const Vec3 u = Normalized(Cross(direction, reference)) * a_radius;
		const Vec3 v = Normalized(Cross(direction, u)) * a_radius;
		AppendRing(a_out, a_centre, u, v, a_color);
	}

	float DistanceToTriangle(const Vec3& a_point, const Vec3& a_a, const Vec3& a_b, const Vec3& a_c, Vec3& a_closest) noexcept
	{
		// Ericson, Real-Time Collision Detection, 5.1.5.
		const Vec3 ab = a_b - a_a;
		const Vec3 ac = a_c - a_a;
		const Vec3 ap = a_point - a_a;
		const float d1 = Dot(ab, ap);
		const float d2 = Dot(ac, ap);
		if (d1 <= 0.0f && d2 <= 0.0f) {
			a_closest = a_a;
			return Length(a_point - a_closest);
		}
		const Vec3 bp = a_point - a_b;
		const float d3 = Dot(ab, bp);
		const float d4 = Dot(ac, bp);
		if (d3 >= 0.0f && d4 <= d3) {
			a_closest = a_b;
			return Length(a_point - a_closest);
		}
		const float vc = d1 * d4 - d3 * d2;
		if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f) {
			a_closest = a_a + ab * (d1 / (d1 - d3));
			return Length(a_point - a_closest);
		}
		const Vec3 cp = a_point - a_c;
		const float d5 = Dot(ab, cp);
		const float d6 = Dot(ac, cp);
		if (d6 >= 0.0f && d5 <= d6) {
			a_closest = a_c;
			return Length(a_point - a_closest);
		}
		const float vb = d5 * d2 - d1 * d6;
		if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f) {
			a_closest = a_a + ac * (d2 / (d2 - d6));
			return Length(a_point - a_closest);
		}
		const float va = d3 * d6 - d5 * d4;
		if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f) {
			a_closest = a_b + (a_c - a_b) * ((d4 - d3) / ((d4 - d3) + (d5 - d6)));
			return Length(a_point - a_closest);
		}
		const float denominator = 1.0f / (va + vb + vc);
		a_closest = a_a + ab * (vb * denominator) + ac * (vc * denominator);
		return Length(a_point - a_closest);
	}

	std::vector<std::uint32_t> NearestActors(std::vector<ActorDistance> a_candidates, int a_max)
	{
		std::stable_sort(a_candidates.begin(), a_candidates.end(), [](const ActorDistance& a_left, const ActorDistance& a_right) { return a_left.distance < a_right.distance; });
		std::vector<std::uint32_t> chosen;
		for (const ActorDistance& candidate : a_candidates) {
			if (static_cast<int>(chosen.size()) >= a_max)
				break;
			if (std::find(chosen.begin(), chosen.end(), candidate.actor) == chosen.end())
				chosen.push_back(candidate.actor);
		}
		return chosen;
	}

	void AppendCross(std::vector<DebugSegment>& a_out, const Vec3& a_centre, float a_size, DebugColor a_color)
	{
		for (const Vec3& axis : { Vec3{ a_size, 0.0f, 0.0f }, Vec3{ 0.0f, a_size, 0.0f }, Vec3{ 0.0f, 0.0f, a_size } })
			a_out.push_back({ a_centre - axis, a_centre + axis, a_color });
	}

	void AppendCapsuleOutline(std::vector<DebugSegment>& a_out, const Vec3& a_endA, const Vec3& a_endB, float a_radius, DebugColor a_color)
	{
		const Vec3 axis = a_endB - a_endA;
		const float length = Length(axis);
		if (!(a_radius > 0.0f) || !std::isfinite(length) || !std::isfinite(a_radius))
			return;
		if (length < kSphereLengthEpsilon) {
			const Vec3 x{ a_radius, 0.0f, 0.0f };
			const Vec3 y{ 0.0f, a_radius, 0.0f };
			const Vec3 z{ 0.0f, 0.0f, a_radius };
			AppendRing(a_out, a_endA, x, y, a_color);
			AppendRing(a_out, a_endA, y, z, a_color);
			AppendRing(a_out, a_endA, z, x, a_color);
			return;
		}
		const Vec3 direction = axis / length;
		const Vec3 reference = std::abs(direction.z) > 0.9f ? Vec3{ 1.0f, 0.0f, 0.0f } : Vec3{ 0.0f, 0.0f, 1.0f };
		const Vec3 u = Normalized(Cross(direction, reference)) * a_radius;
		const Vec3 v = Normalized(Cross(direction, u)) * a_radius;
		AppendRing(a_out, a_endA, u, v, a_color);
		AppendRing(a_out, a_endB, u, v, a_color);
		AppendRing(a_out, (a_endA + a_endB) * 0.5f, u, v, a_color);
		for (const Vec3& side : { u, v, u * -1.0f, v * -1.0f })
			a_out.push_back({ a_endA + side, a_endB + side, a_color });
	}

	void AppendEllipsoidOutline(std::vector<DebugSegment>& a_out, const Vec3& a_centre, const Quat& a_orientation, const Vec3& a_halfExtents, DebugColor a_color)
	{
		if (!std::isfinite(a_halfExtents.x + a_halfExtents.y + a_halfExtents.z))
			return;
		const Vec3 x = a_orientation.Rotate({ a_halfExtents.x, 0.0f, 0.0f });
		const Vec3 y = a_orientation.Rotate({ 0.0f, a_halfExtents.y, 0.0f });
		const Vec3 z = a_orientation.Rotate({ 0.0f, 0.0f, a_halfExtents.z });
		AppendRing(a_out, a_centre, x, y, a_color);
		AppendRing(a_out, a_centre, y, z, a_color);
		AppendRing(a_out, a_centre, z, x, a_color);
	}
}
