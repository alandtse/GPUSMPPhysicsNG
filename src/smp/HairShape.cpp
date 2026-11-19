// SPDX-License-Identifier: GPL-3.0-or-later WITH LicenseRef-Modding-Exception

#include "smp/HairShape.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace SmpPhysics
{
	namespace
	{
		constexpr int kCellBits = 21;
		constexpr std::int64_t kCellOffset = 1 << (kCellBits - 1);
		constexpr float kBoxPadding = kHairVoxel * 0.5f;
		constexpr double kReachPercentile = 0.95;

		Float3 Subtract(const Float3& a_left, const Float3& a_right)
		{
			return { a_left[0] - a_right[0], a_left[1] - a_right[1], a_left[2] - a_right[2] };
		}

		bool IsFinite(const Float3& a_point)
		{
			return std::isfinite(a_point[0]) && std::isfinite(a_point[1]) && std::isfinite(a_point[2]);
		}

		float TriangleArea(const HairTriangle& a_triangle)
		{
			const Float3 u = Subtract(a_triangle.b, a_triangle.a);
			const Float3 v = Subtract(a_triangle.c, a_triangle.a);
			const float x = u[1] * v[2] - u[2] * v[1];
			const float y = u[2] * v[0] - u[0] * v[2];
			const float z = u[0] * v[1] - u[1] * v[0];
			return 0.5f * std::sqrt(x * x + y * y + z * z);
		}

		/** @brief The a_index-th term of the van der Corput sequence in a_base: evenly spread values in [0, 1). */
		float RadicalInverse(std::uint32_t a_index, std::uint32_t a_base)
		{
			float value = 0.0f;
			float scale = 1.0f / static_cast<float>(a_base);
			for (; a_index > 0; a_index /= a_base, scale /= static_cast<float>(a_base))
				value += static_cast<float>(a_index % a_base) * scale;
			return value;
		}

		std::uint64_t CellKey(const Float3& a_point)
		{
			std::uint64_t key = 0;
			for (const float coordinate : a_point) {
				const std::int64_t cell = static_cast<std::int64_t>(std::floor(coordinate / kHairVoxel)) + kCellOffset;
				key = (key << kCellBits) | (static_cast<std::uint64_t>(std::clamp<std::int64_t>(cell, 0, (1 << kCellBits) - 1)));
			}
			return key;
		}

		float DistanceSquared(const Float3& a_left, const Float3& a_right)
		{
			const Float3 d = Subtract(a_left, a_right);
			return d[0] * d[0] + d[1] * d[1] + d[2] * d[2];
		}

		struct DirectedDistances
		{
			float mean{};
			float percentile{};  ///< kReachPercentile of the nearest distances.
			float farShare{};    ///< Share of samples farther than kFarSample.
		};

		/** @brief How far each of a_from's samples is from the closest of a_to's: the mean, a high percentile, and the share beyond kFarSample. */
		DirectedDistances DirectedDistance(const std::vector<Float3>& a_from, const std::vector<Float3>& a_to)
		{
			std::vector<float> distances;
			distances.reserve(a_from.size());
			double sum = 0.0;
			std::size_t far = 0;
			for (const Float3& point : a_from) {
				float nearest = std::numeric_limits<float>::max();
				for (const Float3& other : a_to)
					nearest = std::min(nearest, DistanceSquared(point, other));
				const float distance = std::sqrt(nearest);
				distances.push_back(distance);
				sum += distance;
				far += distance > kFarSample ? 1 : 0;
			}
			const double count = static_cast<double>(a_from.size());
			const auto rank = distances.begin() + static_cast<std::ptrdiff_t>(static_cast<double>(distances.size() - 1) * kReachPercentile);
			std::ranges::nth_element(distances, rank);
			return { static_cast<float>(sum / count), *rank, static_cast<float>(static_cast<double>(far) / count) };
		}

		float BoxVolume(const Float3& a_min, const Float3& a_max)
		{
			return std::max(0.0f, a_max[0] - a_min[0]) * std::max(0.0f, a_max[1] - a_min[1]) * std::max(0.0f, a_max[2] - a_min[2]);
		}

		float BoxOverlap(const HairDescriptor& a_left, const HairDescriptor& a_right)
		{
			Float3 leftMin{};
			Float3 leftMax{};
			Float3 rightMin{};
			Float3 rightMax{};
			Float3 low{};
			Float3 high{};
			for (std::size_t axis = 0; axis < 3; ++axis) {
				leftMin[axis] = a_left.boundsMin[axis] - kBoxPadding;
				leftMax[axis] = a_left.boundsMax[axis] + kBoxPadding;
				rightMin[axis] = a_right.boundsMin[axis] - kBoxPadding;
				rightMax[axis] = a_right.boundsMax[axis] + kBoxPadding;
				low[axis] = std::max(leftMin[axis], rightMin[axis]);
				high[axis] = std::min(leftMax[axis], rightMax[axis]);
			}
			const float shared = BoxVolume(low, high);
			const float total = BoxVolume(leftMin, leftMax) + BoxVolume(rightMin, rightMax) - shared;
			return total > 0.0f ? shared / total : 0.0f;
		}

		float Jaccard(const std::vector<std::uint64_t>& a_left, const std::vector<std::uint64_t>& a_right)
		{
			std::size_t shared = 0;
			std::size_t left = 0;
			std::size_t right = 0;
			while (left < a_left.size() && right < a_right.size()) {
				if (a_left[left] == a_right[right]) {
					++shared;
					++left;
					++right;
				} else if (a_left[left] < a_right[right]) {
					++left;
				} else {
					++right;
				}
			}
			const std::size_t total = a_left.size() + a_right.size() - shared;
			return total ? static_cast<float>(shared) / static_cast<float>(total) : 0.0f;
		}
	}

	HairDescriptor DescribeHair(std::span<const HairTriangle> a_triangles)
	{
		HairDescriptor descriptor;
		std::vector<const HairTriangle*> usable;
		std::vector<double> running;
		double total = 0.0;
		for (const HairTriangle& triangle : a_triangles) {
			if (!IsFinite(triangle.a) || !IsFinite(triangle.b) || !IsFinite(triangle.c))
				continue;
			const float area = TriangleArea(triangle);
			if (!(area > 0.0f))
				continue;
			total += area;
			usable.push_back(&triangle);
			running.push_back(total);
		}
		if (usable.empty())
			return descriptor;

		descriptor.boundsMin = { std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max() };
		descriptor.boundsMax = { std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest() };
		for (const HairTriangle* triangle : usable) {
			for (const Float3* point : { &triangle->a, &triangle->b, &triangle->c }) {
				for (std::size_t axis = 0; axis < 3; ++axis) {
					descriptor.boundsMin[axis] = std::min(descriptor.boundsMin[axis], (*point)[axis]);
					descriptor.boundsMax[axis] = std::max(descriptor.boundsMax[axis], (*point)[axis]);
				}
			}
		}
		descriptor.area = static_cast<float>(total);

		descriptor.samples.reserve(kHairSamples);
		for (std::size_t index = 0; index < kHairSamples; ++index) {
			const double target = (static_cast<double>(index) + 0.5) / static_cast<double>(kHairSamples) * total;
			const std::size_t pick = std::min<std::size_t>(static_cast<std::size_t>(std::ranges::lower_bound(running, target) - running.begin()), usable.size() - 1);
			const HairTriangle& triangle = *usable[pick];
			const float root = std::sqrt(RadicalInverse(static_cast<std::uint32_t>(index) + 1, 2));
			const float along = RadicalInverse(static_cast<std::uint32_t>(index) + 1, 3);
			Float3 point{};
			for (std::size_t axis = 0; axis < 3; ++axis)
				point[axis] = (1.0f - root) * triangle.a[axis] + root * (1.0f - along) * triangle.b[axis] + root * along * triangle.c[axis];
			descriptor.samples.push_back(point);
			descriptor.cells.push_back(CellKey(point));
		}
		std::ranges::sort(descriptor.cells);
		descriptor.cells.erase(std::ranges::unique(descriptor.cells).begin(), descriptor.cells.end());
		return descriptor;
	}

	HairMatch CompareHair(const HairDescriptor& a_left, const HairDescriptor& a_right)
	{
		HairMatch match;
		if (a_left.samples.empty() || a_right.samples.empty()) {
			match.chamfer = std::numeric_limits<float>::max();
			return match;
		}
		const DirectedDistances leftToRight = DirectedDistance(a_left.samples, a_right.samples);
		const DirectedDistances rightToLeft = DirectedDistance(a_right.samples, a_left.samples);
		match.chamfer = 0.5f * (leftToRight.mean + rightToLeft.mean);
		match.reach = std::max(leftToRight.percentile, rightToLeft.percentile);
		match.coverage = std::max(leftToRight.farShare, rightToLeft.farShare);
		match.overlap = Jaccard(a_left.cells, a_right.cells);
		match.boxOverlap = BoxOverlap(a_left, a_right);
		const float larger = std::max(a_left.area, a_right.area);
		match.areaRatio = larger > 0.0f ? std::min(a_left.area, a_right.area) / larger : 0.0f;
		return match;
	}
}
