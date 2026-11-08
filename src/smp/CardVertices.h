#pragma once

#include "CardSkin.h"
#include "Collision.h"

#include <string>
#include <vector>

namespace RE
{
	class NiAVObject;
}

namespace SmpPhysics
{
	/**
	 * @brief Reads a skinned hair card mesh's vertices in skin space, with the bones and weights each one is skinned to.
	 * @param a_reason Set to why the mesh could not be read.
	 * @return false for a mesh that is not skinned or whose vertex data is no longer in memory.
	 */
	[[nodiscard]] bool ReadCardVertices(RE::NiAVObject* a_geometry, std::vector<SkinnedVertex>& a_vertices, std::string& a_reason);

	/**
	 * @brief Reads a skinned mesh's triangle list.
	 * @return false for a mesh with no skin partition or triangle list, or one stored as strips.
	 */
	[[nodiscard]] bool ReadCardTriangles(RE::NiAVObject* a_geometry, std::vector<std::uint16_t>& a_triangles, std::string& a_reason);
}
