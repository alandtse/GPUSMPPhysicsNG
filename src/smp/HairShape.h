// SPDX-License-Identifier: GPL-3.0-or-later WITH LicenseRef-Modding-Exception

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

namespace SmpPhysics
{
	using Float3 = std::array<float, 3>;

	struct HairTriangle
	{
		Float3 a;
		Float3 b;
		Float3 c;
	};

	/** @brief What a hair mesh's shape is compared by: its surface sampled evenly by area, the box around it and the cells it fills. */
	struct HairDescriptor
	{
		Float3 boundsMin{};
		Float3 boundsMax{};
		float area{};                      ///< Total triangle area in squared game units.
		std::vector<Float3> samples;       ///< Points spread over the surface in proportion to triangle area.
		std::vector<std::uint64_t> cells;  ///< Sorted, unique keys of the voxels the samples fall in.
	};

	/** @brief How alike two hair meshes are, both read in one shared frame. Lower distances and higher overlaps are closer. */
	struct HairMatch
	{
		float chamfer{};     ///< Mean of the two directions' mean nearest-sample distances, in game units.
		float reach{};       ///< Of the two directions, the larger 95th-percentile nearest-sample distance: how far the worst-matching parts are, in game units.
		float coverage{};    ///< Of the two directions, the larger share of samples farther than kFarSample from the other shape.
		float overlap{};     ///< Voxel Jaccard index, 0 to 1.
		float boxOverlap{};  ///< Intersection over union of the bounding boxes, 0 to 1.
		float areaRatio{};   ///< Smaller area over larger, 0 to 1.
	};

	inline constexpr std::size_t kHairSamples = 1024;
	inline constexpr float kHairVoxel = 6.0f;
	inline constexpr float kFarSample = 15.0f;

	/** @brief Samples a_triangles by area; the points are the same every time for the same input. An empty or zero-area input gives an empty descriptor. */
	[[nodiscard]] HairDescriptor DescribeHair(std::span<const HairTriangle> a_triangles);

	/** @brief Compares two descriptors; an empty one matches nothing, with a chamfer of the largest float. */
	[[nodiscard]] HairMatch CompareHair(const HairDescriptor& a_left, const HairDescriptor& a_right);
}
