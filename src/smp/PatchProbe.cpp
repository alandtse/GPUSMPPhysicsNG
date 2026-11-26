#include "smp/PatchProbe.h"

#include <cfloat>
#include <cmath>
#include <unordered_map>
#include <vector>

namespace SmpPhysics
{
	namespace
	{
		Vec3 Sub(const Vec3& a, const Vec3& b) noexcept { return { a.x - b.x, a.y - b.y, a.z - b.z }; }
		Vec3 Add(const Vec3& a, const Vec3& b) noexcept { return { a.x + b.x, a.y + b.y, a.z + b.z }; }
		Vec3 Scale(const Vec3& a, float s) noexcept { return { a.x * s, a.y * s, a.z * s }; }

		Vec3 ClosestOnFace(const Vec3& p, const Vec3& a, const Vec3& b, const Vec3& c) noexcept
		{
			const Vec3 ab = Sub(b, a);
			const Vec3 ac = Sub(c, a);
			const Vec3 ap = Sub(p, a);
			const float d1 = Dot(ab, ap);
			const float d2 = Dot(ac, ap);
			if (d1 <= 0.0f && d2 <= 0.0f)
				return a;
			const Vec3 bp = Sub(p, b);
			const float d3 = Dot(ab, bp);
			const float d4 = Dot(ac, bp);
			if (d3 >= 0.0f && d4 <= d3)
				return b;
			const float vc = d1 * d4 - d3 * d2;
			if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f)
				return Add(a, Scale(ab, d1 / (d1 - d3)));
			const Vec3 cp = Sub(p, c);
			const float d5 = Dot(ab, cp);
			const float d6 = Dot(ac, cp);
			if (d6 >= 0.0f && d5 <= d6)
				return c;
			const float vb = d5 * d2 - d1 * d6;
			if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f)
				return Add(a, Scale(ac, d2 / (d2 - d6)));
			const float va = d3 * d6 - d5 * d4;
			if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f)
				return Add(b, Scale(Sub(c, b), (d4 - d3) / ((d4 - d3) + (d5 - d6))));
			const float denominator = 1.0f / (va + vb + vc);
			return Add(a, Add(Scale(ab, vb * denominator), Scale(ac, vc * denominator)));
		}
	}

	PatchSample MeasureSkinOutsideCloth(std::span<const Vec3> a_skin, std::span<const Vec3> a_cloth,
		std::span<const std::uint32_t> a_indices, float a_orientation, float a_reach, float a_threshold) noexcept
	{
		PatchSample sample;
		const std::size_t faceCount = a_indices.size() / 3;
		const float reachSquared = a_reach * a_reach;
		const float cell = std::max(a_reach, 1.0f);
		const auto cellOf = [cell](float a_value) { return static_cast<std::int64_t>(std::floor(a_value / cell)); };
		const auto keyOf = [](std::int64_t a_x, std::int64_t a_y, std::int64_t a_z) {
			return static_cast<std::uint64_t>((a_x & 0x1FFFFF) | ((a_y & 0x1FFFFF) << 21) | ((a_z & 0x1FFFFF) << 42));
		};
		std::unordered_map<std::uint64_t, std::vector<std::uint32_t>> grid;
		grid.reserve(faceCount * 2);
		for (std::size_t face = 0; face < faceCount; ++face) {
			const std::uint32_t i0 = a_indices[face * 3];
			const std::uint32_t i1 = a_indices[face * 3 + 1];
			const std::uint32_t i2 = a_indices[face * 3 + 2];
			if (i0 >= a_cloth.size() || i1 >= a_cloth.size() || i2 >= a_cloth.size())
				continue;
			const Vec3& a = a_cloth[i0];
			const Vec3& b = a_cloth[i1];
			const Vec3& c = a_cloth[i2];
			const std::int64_t x0 = cellOf(std::min({ a.x, b.x, c.x }) - a_reach);
			const std::int64_t x1 = cellOf(std::max({ a.x, b.x, c.x }) + a_reach);
			const std::int64_t y0 = cellOf(std::min({ a.y, b.y, c.y }) - a_reach);
			const std::int64_t y1 = cellOf(std::max({ a.y, b.y, c.y }) + a_reach);
			const std::int64_t z0 = cellOf(std::min({ a.z, b.z, c.z }) - a_reach);
			const std::int64_t z1 = cellOf(std::max({ a.z, b.z, c.z }) + a_reach);
			if ((x1 - x0 + 1) * (y1 - y0 + 1) * (z1 - z0 + 1) > 512)
				continue;
			for (std::int64_t x = x0; x <= x1; ++x)
				for (std::int64_t y = y0; y <= y1; ++y)
					for (std::int64_t z = z0; z <= z1; ++z)
						grid[keyOf(x, y, z)].push_back(static_cast<std::uint32_t>(face));
		}
		for (std::size_t point = 0; point < a_skin.size(); ++point) {
			const auto found = grid.find(keyOf(cellOf(a_skin[point].x), cellOf(a_skin[point].y), cellOf(a_skin[point].z)));
			if (found == grid.end())
				continue;
			float nearest = FLT_MAX;
			float signedDistance = 0.0f;
			std::uint32_t nearestFace = 0;
			for (const std::uint32_t face : found->second) {
				const std::uint32_t i0 = a_indices[face * 3];
				const std::uint32_t i1 = a_indices[face * 3 + 1];
				const std::uint32_t i2 = a_indices[face * 3 + 2];
				const Vec3 closest = ClosestOnFace(a_skin[point], a_cloth[i0], a_cloth[i1], a_cloth[i2]);
				const Vec3 offset = Sub(a_skin[point], closest);
				const float squared = Dot(offset, offset);
				if (squared >= nearest || squared > reachSquared)
					continue;
				const Vec3 normal = Normalized(Cross(Sub(a_cloth[i1], a_cloth[i0]), Sub(a_cloth[i2], a_cloth[i0])));
				nearest = squared;
				nearestFace = face;
				signedDistance = a_orientation * Dot(offset, normal);
			}
			if (nearest == FLT_MAX)
				continue;
			++sample.measured;
			if (signedDistance > a_threshold) {
				++sample.outside;
				if (signedDistance > sample.maxOutside) {
					sample.maxOutside = signedDistance;
					sample.worst = static_cast<std::uint32_t>(point);
					sample.worstFace = nearestFace;
				}
			}
		}
		return sample;
	}
}
