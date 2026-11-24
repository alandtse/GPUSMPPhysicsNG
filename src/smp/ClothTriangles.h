// SPDX-License-Identifier: GPL-3.0-or-later WITH LicenseRef-Modding-Exception

#pragma once

#include "Collision.h"
#include "Math.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

/**
 * @brief A cloth's own triangles as contact geometry: hdtSMP64 collides a per-triangle shape with a per-vertex
 *        shape by testing each vertex of the second, as a sphere, against each triangle of the first.
 */
namespace SmpPhysics
{
	/**
	 * @brief Below this a triangle's doubled area, a penetration or a depth counts as none, hdtSMP64's FLT_EPSILON.
	 * @note Mirrored by kTriangleContactEpsilon in ContactCS.hlsl.
	 */
	inline constexpr float kTriangleContactEpsilon = 1.1920929e-7f;
	/**
	 * @brief Hits one cloth triangle keeps per substep: its deepest, and its deepest on another driver.
	 * @note Mirrored by kClothTriangleHits in ContactCS.hlsl.
	 */
	inline constexpr std::uint32_t kClothTriangleHits = 2;

	/** @brief One cloth mesh vertex as a blend over the rig bones it is skinned to. */
	struct ClothVertex
	{
		int bone{ -1 };                                               ///< the rig bone that carries most of the vertex
		Vec3 local;                                                   ///< the vertex in that bone's frame
		float weight{ 1.0f };                                         ///< that bone's skin weight
		std::array<SphereInfluence, Util::kSkinInfluences> others{};  ///< the vertex's other rig bones
		float marginMultiplier{ 1.0f };                               ///< the weight blend of its bones' margin multipliers
	};

	/** @brief One cloth triangle and the window about its plane a body vertex is held in. */
	struct ClothTriangle
	{
		std::array<std::uint32_t, 3> corners{};     ///< indices into ClothTriangleSet::vertices
		float margin{};                             ///< the shape's margin times the mean of the corners' margin multipliers
		float penetration{};                        ///< the shape's penetration times that mean; its sign picks the side, as in TestVertexAgainstTriangle
		std::uint32_t surfaceMask{ kAllMaskBits };  ///< the first 32 of the claim's surfaces it may touch, one bit each
	};

	/** @brief One triangle a simulated bone takes contacts from. */
	struct ClothBoneTriangle
	{
		std::uint32_t triangle{};  ///< index into ClothTriangleSet::triangles
		float flexible{};          ///< the mean of the corners' summed weights on simulated bones, which scales the depth
		std::uint32_t corners{};   ///< corner weights the bone holds above its threshold; hdtSMP64 adds the contact once for each
	};

	/** @brief The triangles one shape added to a set. */
	struct ClothPiece
	{
		std::string name;  ///< the rig shape
		std::uint32_t firstTriangle{};
		std::uint32_t triangleCount{};
	};

	/** @brief A claim's cloth triangles, from every per-triangle shape skinned to its simulated bones. */
	struct ClothTriangleSet
	{
		std::vector<ClothVertex> vertices;
		std::vector<ClothTriangle> triangles;
		std::vector<std::vector<ClothBoneTriangle>> boneTriangles;  ///< one list per rig bone, empty for a bone that owns none
		std::vector<ClothPiece> pieces;
	};

	/** @brief One per-triangle shape's mesh and how it maps onto the rig. */
	struct ClothMesh
	{
		std::string name;
		const std::vector<SkinnedVertex>* vertices{};   ///< mesh space; bone indices are the mesh's skin bones
		const std::vector<std::uint16_t>* triangles{};  ///< three vertex indices each
		const std::vector<int>* rigBoneOfSkinBone{};    ///< -1 for a skin bone the rig does not have
		const std::vector<Transform>* skinToBone{};     ///< one per skin bone, mesh space to bone space
		const std::vector<bool>* dynamicBone{};         ///< one per rig bone
		const std::vector<float>* marginMultiplier{};   ///< one per rig bone; a missing entry is 1
		const std::vector<float>* weightThreshold{};    ///< one per rig bone; a missing entry is 0
		float margin{};
		float penetration{};
	};

	/**
	 * @brief Appends a shape's triangles that a simulated bone owns, with the vertices they use and each owner's list.
	 * @details A bone owns a triangle when it is simulated and holds a corner weight above its threshold, which is
	 *          whom hdtSMP64's merge credits a triangle's contact to. A triangle with a corner no rig bone poses is left out.
	 * @return How many triangles were appended.
	 */
	std::size_t AppendClothTriangles(ClothTriangleSet& a_set, const ClothMesh& a_mesh);

	/** @brief Takes the surfaces a_mask leaves out away from every triangle of one piece. */
	void MaskClothPiece(ClothTriangleSet& a_set, const ClothPiece& a_piece, std::uint32_t a_mask) noexcept;

	/** @brief A cloth vertex posed by its bones' world transforms, the CPU twin of ClothVertices in ContactCS.hlsl. */
	[[nodiscard]] Vec3 PoseClothVertex(const ClothVertex& a_vertex, const std::vector<Transform>& a_boneWorld) noexcept;

	/** @brief What a body vertex does to a cloth triangle. Depth is zero when they do not touch. */
	struct TriangleContact
	{
		float depth{};
		Vec3 normal{};     ///< unit, the way the cloth is pushed: along the triangle's normal, away from the vertex
		Vec3 point{};      ///< the vertex projected onto the triangle's plane
		Vec3 bodyPoint{};  ///< the point of the vertex's sphere nearest the plane
	};

	/**
	 * @brief hdtSMP64's sphere-against-triangle test, the CPU twin of ClothTriangleContact in ContactCS.hlsl.
	 * @details The triangle's normal is turned around when a_penetration is negative. The vertex touches when its
	 *          signed distance along that normal lies in [-|a_penetration|, a_radius + a_margin) and its projection is
	 *          inside the triangle; with no penetration the triangle is two-sided and the distance is taken unsigned.
	 * @param a_radius The body vertex's sphere radius.
	 * @param a_margin The triangle's margin.
	 */
	[[nodiscard]] TriangleContact TestVertexAgainstTriangle(const Vec3& a_vertex, float a_radius, const Vec3& a_a, const Vec3& a_b,
		const Vec3& a_c, float a_margin, float a_penetration) noexcept;
}
