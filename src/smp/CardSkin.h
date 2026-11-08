#pragma once

#include "Collision.h"
#include "Math.h"

#include <cstdint>
#include <vector>

namespace SmpPhysics
{
	/** @brief A skin bone's bind transform, as NiSkinData stores it: bone space = scale * rotation * skin space + translation. */
	struct BindTransform
	{
		float rotation[3][3]{ { 1.0f, 0.0f, 0.0f }, { 0.0f, 1.0f, 0.0f }, { 0.0f, 0.0f, 1.0f } };
		float scale{ 1.0f };
		Vec3 translation{};
	};

	/** @brief One hair card mesh as the wind estimate reads it. */
	struct CardSkin
	{
		std::vector<int> rigBone;         ///< Per skin bone, the rig bone it is, or -1 for a bone the rig does not simulate.
		std::vector<BindTransform> bind;  ///< Per skin bone.
		std::vector<SkinnedVertex> vertices;
		std::vector<std::uint16_t> triangles;  ///< Three vertex indices each; empty until ReadCardTriangles fills it.
	};
}
