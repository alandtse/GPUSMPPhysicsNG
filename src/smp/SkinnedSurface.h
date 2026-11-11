#pragma once

#include "Collision.h"
#include "TriangleGrid.h"

#include <cstdint>
#include <string>
#include <vector>

namespace SmpPhysics
{
	/**
	 * @brief One skinned collision mesh, as an SMP per-triangle-shape names it: triangles plus the skinning that
	 *        poses them from the skeleton every frame.
	 */
	struct SkinnedSurface
	{
		std::string name;                          ///< the shape's mesh name in the rig file
		std::vector<SkinnedVertex> vertices;       ///< bind-space positions; bone indices are local to the mesh's skin bones
		std::vector<std::uint32_t> indices;        ///< the kept triangles, three vertex indices each
		std::vector<Transform> skinToBone;         ///< one per skin bone, as NiSkinData stores it
		std::vector<int> boneDriver;               ///< one per skin bone: its index in the claim's driver list, or -1
		float margin{};                            ///< the shape's collision margin, which thickens the surface
		float contactPenetration{};                ///< the shape's penetration allowance in hdtSMP64's contact rule, see TriangleGrid::Query
		float penetration{ kDefaultPenetration };  ///< depth the solver's safety clamp tolerates before it holds a bone back
		bool deepPush{ true };                     ///< whether a one-sided face also holds a sphere resting deep behind it, see kDeepPushReach
		bool clothOnly{};                          ///< admitted only for the claim's own pieces of cloth to collide with each other; the rest-pose retarget and the wind shelter leave it out
		bool skin{};                               ///< the mesh is the character's skin, which the hair was authored against, rather than something worn over it
	};

	/**
	 * @brief The triangles hdtSMP64 would still collide with once a shape's weight-thresholds are applied.
	 * @details SMP clips a collider unless one of its bone weights is above both zero and that bone's threshold, so
	 *          a triangle is kept when any of its vertices has such a slot. A bone the rig lists no threshold for has
	 *          threshold zero.
	 * @param a_threshold One value per skin bone.
	 */
	[[nodiscard]] std::vector<std::uint32_t> KeepWeightedTriangles(const std::vector<SkinnedVertex>& a_vertices,
		const std::vector<std::uint32_t>& a_indices, const std::vector<float>& a_threshold);

	/**
	 * @brief Poses a surface's vertices in world space from the drivers' current world transforms.
	 * @details A skin bone with no driver is skipped by the blend; a vertex none of whose bones is driven keeps its bind position.
	 * @return One position per vertex.
	 */
	[[nodiscard]] std::vector<Vec3> PoseSurface(const SkinnedSurface& a_surface, const std::vector<Transform>& a_driverWorld);

	/**
	 * @brief The triangles of a mesh whose three vertices all lie within a_radius of a_centre.
	 * @param a_positions The mesh's vertices, in the space the test is wanted in.
	 * @param a_indices Three vertex indices per triangle; a triangle naming a vertex past a_positions is dropped.
	 * @return The kept triangles, as a new index list.
	 */
	[[nodiscard]] std::vector<std::uint32_t> KeepTrianglesNear(const std::vector<Vec3>& a_positions,
		const std::vector<std::uint32_t>& a_indices, const Vec3& a_centre, float a_radius);

	/**
	 * @brief Drops the vertices no kept triangle uses, and remaps the indices onto the ones that stay.
	 * @details The solver skins every vertex of a surface every frame, so a mesh cropped to one region must not
	 *          carry the rest of itself. skinToBone and boneDriver are per skin bone and are left alone.
	 * @param a_surface The surface to compact in place; a triangle naming a dropped-index vertex is left out.
	 */
	void CompactSurface(SkinnedSurface& a_surface);

	/** @brief Grid cell edge, in Skyrim units, used for posed collision surfaces; a little over a hair sphere's diameter. */
	inline constexpr float kSurfaceCellSize = 3.0f;
	/** @brief How far from the hair's own bones a worn mesh is read, in Skyrim units; the rest of a cuirass is out of reach. */
	inline constexpr float kArmorSurfaceRadius = 80.0f;
	/** @brief Collision margin a worn mesh's surface is given, thickening it as a rig file's own margin does. */
	inline constexpr float kArmorSurfaceMargin = 0.3f;
	/** @brief Contact allowance of a worn mesh's surface: the one-sided allowance SMP's body shapes are authored with. */
	inline constexpr float kArmorSurfacePenetration = 0.4f;
	/** @brief Collision margin of a simulated cloth mesh's surface, which is two-sided so hair is held to whichever side it lies on. */
	inline constexpr float kClothSurfaceMargin = 0.5f;
	/** @brief Most worn meshes one claim adds a surface from; further qualifying meshes are left out, the largest kept. */
	inline constexpr std::size_t kMaxArmorSurfaces = 4;
}
