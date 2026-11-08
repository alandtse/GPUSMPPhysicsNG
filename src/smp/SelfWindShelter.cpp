#include "SelfWindShelter.h"

#include <algorithm>
#include <cmath>

namespace SmpPhysics
{
	namespace
	{
		constexpr float kEpsilon = 1e-8f;

		/** @brief Squared distance between the closest points of two segments. */
		float SegmentSegmentDistanceSquared(const Vec3& a_p1, const Vec3& a_q1, const Vec3& a_p2, const Vec3& a_q2) noexcept
		{
			const Vec3 d1 = a_q1 - a_p1;
			const Vec3 d2 = a_q2 - a_p2;
			const Vec3 r = a_p1 - a_p2;
			const float a = Dot(d1, d1);
			const float e = Dot(d2, d2);
			const float f = Dot(d2, r);
			float s = 0.0f;
			float t = 0.0f;
			if (a <= kEpsilon && e <= kEpsilon) {
				return Dot(r, r);
			}
			if (a <= kEpsilon) {
				t = std::clamp(f / e, 0.0f, 1.0f);
			} else {
				const float c = Dot(d1, r);
				if (e <= kEpsilon) {
					s = std::clamp(-c / a, 0.0f, 1.0f);
				} else {
					const float b = Dot(d1, d2);
					const float denominator = a * e - b * b;
					s = denominator > kEpsilon ? std::clamp((b * f - c * e) / denominator, 0.0f, 1.0f) : 0.0f;
					t = (b * s + f) / e;
					if (t < 0.0f) {
						t = 0.0f;
						s = std::clamp(-c / a, 0.0f, 1.0f);
					} else if (t > 1.0f) {
						t = 1.0f;
						s = std::clamp((b - c) / a, 0.0f, 1.0f);
					}
				}
			}
			const Vec3 offset = (a_p1 + d1 * s) - (a_p2 + d2 * t);
			return Dot(offset, offset);
		}

		bool TriangleBlocks(const Vec3& a_origin, const Vec3& a_direction, float a_reach, const Vec3& a_a, const Vec3& a_b, const Vec3& a_c) noexcept
		{
			const Vec3 edge1 = a_b - a_a;
			const Vec3 edge2 = a_c - a_a;
			const Vec3 p = Cross(a_direction, edge2);
			const float determinant = Dot(edge1, p);
			if (std::abs(determinant) < kEpsilon)
				return false;
			const float inverse = 1.0f / determinant;
			const Vec3 offset = a_origin - a_a;
			const float u = Dot(offset, p) * inverse;
			if (u < 0.0f || u > 1.0f)
				return false;
			const Vec3 q = Cross(offset, edge1);
			const float v = Dot(a_direction, q) * inverse;
			if (v < 0.0f || u + v > 1.0f)
				return false;
			const float t = Dot(edge2, q) * inverse;
			return t > 0.0f && t < a_reach;
		}

		/** @brief Whether the segment from a_origin along a_direction for a_reach crosses a collider, which a start inside it does not. */
		bool ColliderBlocks(const Collider& a_collider, const Transform& a_driver, const Vec3& a_origin, const Vec3& a_direction, float a_reach) noexcept
		{
			const Vec3 end = a_origin + a_direction * a_reach;
			if (a_collider.type == Collider::Type::Capsule) {
				const Vec3 a = a_driver.TransformPoint(a_collider.segmentA);
				const Vec3 b = a_driver.TransformPoint(a_collider.segmentB);
				const float radiusSquared = a_collider.radius * a_collider.radius;
				if (SegmentSegmentDistanceSquared(a_origin, a_origin, a, b) < radiusSquared)
					return false;
				return SegmentSegmentDistanceSquared(a_origin, end, a, b) < radiusSquared;
			}
			if (a_collider.type == Collider::Type::Ellipsoid) {
				const Transform world = a_driver * Transform{ a_collider.orientation, a_collider.center };
				const Transform inverse = world.Inverse();
				const Vec3 extents{ std::max(a_collider.halfExtents.x, kMinimumHalfExtent), std::max(a_collider.halfExtents.y, kMinimumHalfExtent),
					std::max(a_collider.halfExtents.z, kMinimumHalfExtent) };
				const auto scaled = [&](const Vec3& a_point) {
					const Vec3 local = inverse.TransformPoint(a_point);
					return Vec3{ local.x / extents.x, local.y / extents.y, local.z / extents.z };
				};
				const Vec3 start = scaled(a_origin);
				if (Dot(start, start) < 1.0f)
					return false;
				const Vec3 stop = scaled(end);
				const Vec3 zero{};
				return SegmentSegmentDistanceSquared(start, stop, zero, zero) < 1.0f;
			}
			return false;
		}
	}

	float SelfWindExposure(const std::vector<Collider>& a_colliders, const std::vector<SkinnedSurface>& a_surfaces,
		const std::vector<Transform>& a_driverWorld, const std::vector<Vec3>& a_samples, const Vec3& a_towardSource, float a_reach)
	{
		if (a_samples.empty() || !(a_reach > 0.0f))
			return 1.0f;

		std::vector<std::vector<Vec3>> posed;
		posed.reserve(a_surfaces.size());
		for (const SkinnedSurface& surface : a_surfaces)
			posed.push_back(PoseSurface(surface, a_driverWorld));

		std::size_t exposed = 0;
		for (const Vec3& sample : a_samples) {
			bool blocked = false;
			for (const Collider& collider : a_colliders) {
				if (collider.driverIndex < 0 || static_cast<std::size_t>(collider.driverIndex) >= a_driverWorld.size())
					continue;
				if (ColliderBlocks(collider, a_driverWorld[static_cast<std::size_t>(collider.driverIndex)], sample, a_towardSource, a_reach)) {
					blocked = true;
					break;
				}
			}
			for (std::size_t surface = 0; surface < a_surfaces.size() && !blocked; ++surface) {
				const std::vector<Vec3>& positions = posed[surface];
				const std::vector<std::uint32_t>& indices = a_surfaces[surface].indices;
				for (std::size_t index = 0; index + 2 < indices.size(); index += 3) {
					if (indices[index] >= positions.size() || indices[index + 1] >= positions.size() || indices[index + 2] >= positions.size())
						continue;
					if (TriangleBlocks(sample, a_towardSource, a_reach, positions[indices[index]], positions[indices[index + 1]], positions[indices[index + 2]])) {
						blocked = true;
						break;
					}
				}
			}
			if (!blocked)
				++exposed;
		}
		return static_cast<float>(exposed) / static_cast<float>(a_samples.size());
	}
}
