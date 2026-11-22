#pragma once

#include "Collision.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace SmpPhysics
{
	/** @brief Cells a grid may have along all axes together; a larger mesh gets bigger cells instead. */
	inline constexpr std::size_t kMaximumGridCells = 1u << 20;
	/**
	 * @brief How far, in Skyrim units, behind a one-sided face a sphere's centre still makes contact.
	 * @details A sphere authored inside the body, or tunnelled in during a fast move, is deeper behind a face
	 *          than the shape's allowance; without this reach it would rest inside the mesh forever.
	 * @note Mirrored by kDeepPushReach in shaders/Common/HairCollision.hlsli.
	 */
	inline constexpr float kDeepPushReach = 8.0f;

	/**
	 * @brief A triangle soup bucketed into a uniform grid, for sphere contact queries against a skinned surface.
	 * @details The caller rebuilds it every frame from the surface's skinned world-space triangles. Triangles wind
	 *          counter-clockwise seen from outside, so a face normal points out of the body.
	 */
	class TriangleGrid
	{
	public:
		struct Triangle
		{
			Vec3 a;
			Vec3 b;
			Vec3 c;
			Vec3 normal;
			float twiceArea;
		};

		/**
		 * @param a_vertices World-space vertex positions.
		 * @param a_indices Three vertex indices per triangle; an out-of-range or degenerate triangle is skipped.
		 * @param a_cellSize Grid cell edge, in Skyrim units; raised when the mesh would need more than kMaximumGridCells.
		 */
		void Build(const std::vector<Vec3>& a_vertices, const std::vector<std::uint32_t>& a_indices, float a_cellSize);

		/**
		 * @brief The deepest contact a sphere has with the surface, by hdtSMP64's per-triangle rule.
		 * @param a_reach The sphere's radius plus the surface margin.
		 * @param a_penetration The shape's penetration allowance. Positive makes a face one-sided: a sphere whose
		 *        centre is more than this far behind it, plus kDeepPushReach, is ignored. Negative flips the
		 *        faces. Zero makes a face two-sided.
		 * @details A face only touches a sphere whose centre projects inside the triangle and lies less than a_reach
		 *          from its plane, or, for a one-sided face, no more than kDeepPushReach behind it, so edges and
		 *          corners make no contact. Depth is a_reach minus the signed distance, and the normal is the face
		 *          normal. The collider index is left for the caller to set.
		 * @return A zero depth when the sphere touches nothing.
		 */
		[[nodiscard]] Contact Query(const Vec3& a_point, float a_reach, float a_penetration, float a_deepReach = kDeepPushReach) const noexcept;

		/**
		 * @brief Signed distance from a_point to the nearest face it projects onto within a_range, positive on the outer side.
		 * @param a_normal Receives that face's normal when given and a face is found.
		 * @return Infinity when no face is in range.
		 */
		[[nodiscard]] float NearestFaceDistance(const Vec3& a_point, float a_range, Vec3* a_normal = nullptr) const noexcept;

		/**
		 * @brief How deep inside a worn shell a_point sits, measured straight out from the vertical axis through a_axisPoint:
		 *        the distance along that horizontal ray to the face it exits the shell through, within a_reach.
		 * @details The nearest hit decides: an outward face is an exit, so the point was inside; a face turned toward the
		 *          point is the far plate across an air gap, so it was not. An exit within a_allowance is left to the contact.
		 *          Ledges, linings and the far side of the body never lie on the ray, which is what makes this robust.
		 * @param a_normal Receives the outward direction when given and the point is inside.
		 * @return Minus the exit distance, or infinity when the point is not inside a shell.
		 */
		[[nodiscard]] float RadialDepthInside(const Vec3& a_point, float a_reach, float a_allowance, const Vec3& a_axisPoint, Vec3* a_normal = nullptr) const noexcept;

		[[nodiscard]] std::size_t TriangleCount() const noexcept { return m_triangles.size(); }

	private:
		[[nodiscard]] bool CellRange(const Vec3& a_low, const Vec3& a_high, int (&a_first)[3], int (&a_last)[3]) const noexcept;
		[[nodiscard]] std::size_t CellIndex(int a_x, int a_y, int a_z) const noexcept;

		struct Box
		{
			Vec3 low;
			Vec3 high;
		};

		std::vector<Triangle> m_triangles;
		/** @name Build's scratch, kept so a grid that is rebuilt every frame does not reallocate. @{ */
		std::vector<Box> m_boxes;
		std::vector<std::array<int, 6>> m_ranges;  ///< per triangle: first then last cell on each axis; last below first when it misses the grid
		std::vector<std::uint32_t> m_fill;
		/** @} */
		std::vector<std::uint32_t> m_cellStart;  ///< one entry per cell plus one, indexing m_cellItems
		std::vector<std::uint32_t> m_cellItems;
		Vec3 m_origin{};
		float m_cellSize{ 1.0f };
		int m_dimensions[3]{};
	};
}
