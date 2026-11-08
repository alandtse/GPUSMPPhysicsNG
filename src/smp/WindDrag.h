#pragma once

#include "CardSkin.h"
#include "SmpRig.h"

#include <vector>

namespace SmpPhysics
{
	/** @brief Width, in Skyrim units, assumed for a bone no mesh measures. */
	inline constexpr float kDefaultWindWidth = 3.0f;
	/** @brief Bounds on a measured area as a width, so a stray triangle or a sliver cannot make a bone immune to wind or a sail. */
	inline constexpr float kMinWindWidth = 1.0f;
	inline constexpr float kMaxWindWidth = 40.0f;
	/** @brief Shortest segment a bone's wind area is taken over, so two coincident bones still catch some wind. */
	inline constexpr float kMinWindLength = 1.0f;

	/** @brief What a bone shows the wind: the direction its hair runs, and the area it presents when the wind is square on. */
	struct BoneWindGeometry
	{
		Vec3 axis{ 0.0f, 0.0f, -1.0f };  ///< Unit vector along the bone's strand in the bone's own frame.
		float area{};                    ///< Area the bone presents square on, zero for a kinematic bone.
	};

	/**
	 * @brief Each dynamic bone's wind-facing segment, from the rest pose.
	 * @details The segment runs from the bone to its first child; a tip, which has none, takes the segment that
	 *          arrives from its parent. Its area is the measured mesh area where a_areas holds one, kept within
	 *          kMinWindWidth to kMaxWindWidth times the segment length, and the length times kDefaultWindWidth otherwise.
	 * @param a_areas One per rig bone, or empty: the mesh surface each bone drives, zero where none is measured.
	 * @return One entry per rig bone.
	 */
	[[nodiscard]] std::vector<BoneWindGeometry> ComputeBoneWindGeometry(const SmpRig& a_rig, const std::vector<Transform>& a_restWorld,
		const std::vector<float>& a_areas);

	/**
	 * @brief The surface area of the meshes each rig bone drives.
	 * @details Each triangle's area is split between its three vertices, and a vertex gives its share to the rig
	 *          bones it is weighted to, in proportion to the weights. A mesh without triangles contributes nothing.
	 * @param a_skins Meshes whose triangles are filled in, with their bone maps.
	 * @return One per rig bone; zero where no triangle reaches it.
	 */
	[[nodiscard]] std::vector<float> EstimateBoneWindAreas(const SmpRig& a_rig, const std::vector<CardSkin>& a_skins);
}
