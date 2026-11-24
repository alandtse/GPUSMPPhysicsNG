// SPDX-License-Identifier: GPL-3.0-or-later WITH LicenseRef-Modding-Exception

#include "ClothTriangles.h"

#include <algorithm>
#include <cmath>

namespace SmpPhysics
{
	namespace
	{
		constexpr std::uint32_t kNoClothVertex = 0xFFFFFFFFu;
		constexpr std::size_t kTriangleCorners = 3;

		struct MeshView
		{
			const ClothMesh& mesh;

			[[nodiscard]] int RigBoneOf(int a_skinBone) const noexcept
			{
				if (a_skinBone < 0 || static_cast<std::size_t>(a_skinBone) >= mesh.rigBoneOfSkinBone->size() ||
					static_cast<std::size_t>(a_skinBone) >= mesh.skinToBone->size())
					return -1;
				const int rigBone = (*mesh.rigBoneOfSkinBone)[static_cast<std::size_t>(a_skinBone)];
				return rigBone >= 0 && static_cast<std::size_t>(rigBone) < mesh.dynamicBone->size() ? rigBone : -1;
			}

			[[nodiscard]] float MarginMultiplierOf(int a_rigBone) const noexcept
			{
				return a_rigBone >= 0 && static_cast<std::size_t>(a_rigBone) < mesh.marginMultiplier->size() ? (*mesh.marginMultiplier)[static_cast<std::size_t>(a_rigBone)] : 1.0f;
			}

			[[nodiscard]] float ThresholdOf(int a_rigBone) const noexcept
			{
				return static_cast<std::size_t>(a_rigBone) < mesh.weightThreshold->size() ? (*mesh.weightThreshold)[static_cast<std::size_t>(a_rigBone)] : 0.0f;
			}

			[[nodiscard]] float FlexibleOf(const SkinnedVertex& a_vertex) const noexcept
			{
				float flexible = 0.0f;
				for (std::size_t slot = 0; slot < Util::kSkinInfluences; ++slot) {
					const int rigBone = RigBoneOf(a_vertex.bone[slot]);
					if (rigBone >= 0 && a_vertex.weight[slot] > 0.0f && (*mesh.dynamicBone)[static_cast<std::size_t>(rigBone)])
						flexible += a_vertex.weight[slot];
				}
				return flexible;
			}

			/** @brief False when no rig bone carries the vertex, so nothing can pose it. */
			[[nodiscard]] bool MakeVertex(const SkinnedVertex& a_vertex, ClothVertex& a_out) const noexcept
			{
				std::size_t ownerSlot = Util::kSkinInfluences;
				float multiplierSum = 0.0f;
				float weightSum = 0.0f;
				for (std::size_t slot = 0; slot < Util::kSkinInfluences; ++slot) {
					if (!(a_vertex.weight[slot] > 0.0f))
						continue;
					const int rigBone = RigBoneOf(a_vertex.bone[slot]);
					multiplierSum += a_vertex.weight[slot] * MarginMultiplierOf(rigBone);
					weightSum += a_vertex.weight[slot];
					if (rigBone >= 0 && (ownerSlot == Util::kSkinInfluences || a_vertex.weight[slot] > a_vertex.weight[ownerSlot]))
						ownerSlot = slot;
				}
				if (ownerSlot == Util::kSkinInfluences)
					return false;
				const auto localIn = [&](std::size_t a_slot) {
					return (*mesh.skinToBone)[static_cast<std::size_t>(a_vertex.bone[a_slot])].TransformPoint(a_vertex.position);
				};
				a_out = {};
				a_out.bone = RigBoneOf(a_vertex.bone[ownerSlot]);
				a_out.local = localIn(ownerSlot);
				a_out.weight = a_vertex.weight[ownerSlot];
				a_out.marginMultiplier = multiplierSum / weightSum;
				std::size_t used = 0;
				for (std::size_t slot = 0; slot < Util::kSkinInfluences; ++slot) {
					const int rigBone = RigBoneOf(a_vertex.bone[slot]);
					if (slot == ownerSlot || rigBone < 0 || !(a_vertex.weight[slot] > 0.0f))
						continue;
					a_out.others[used++] = { rigBone, localIn(slot), a_vertex.weight[slot] };
				}
				return true;
			}
		};

		struct Owner
		{
			int bone;
			std::uint32_t corners;
		};
	}

	std::size_t AppendClothTriangles(ClothTriangleSet& a_set, const ClothMesh& a_mesh)
	{
		if (!a_mesh.vertices || !a_mesh.triangles || !a_mesh.rigBoneOfSkinBone || !a_mesh.skinToBone || !a_mesh.dynamicBone ||
			!a_mesh.marginMultiplier || !a_mesh.weightThreshold)
			return 0;
		const MeshView view{ a_mesh };
		const std::vector<SkinnedVertex>& vertices = *a_mesh.vertices;
		const std::vector<std::uint16_t>& triangles = *a_mesh.triangles;
		if (a_set.boneTriangles.size() < a_mesh.dynamicBone->size())
			a_set.boneTriangles.resize(a_mesh.dynamicBone->size());

		const std::size_t firstTriangle = a_set.triangles.size();
		std::vector<std::uint32_t> slotOfVertex(vertices.size(), kNoClothVertex);
		std::vector<Owner> owners;
		for (std::size_t triangle = 0; triangle + 2 < triangles.size(); triangle += kTriangleCorners) {
			const std::uint16_t* corners = triangles.data() + triangle;
			if (corners[0] >= vertices.size() || corners[1] >= vertices.size() || corners[2] >= vertices.size())
				continue;

			owners.clear();
			for (std::size_t corner = 0; corner < kTriangleCorners; ++corner) {
				const SkinnedVertex& weighted = vertices[corners[corner]];
				for (std::size_t slot = 0; slot < Util::kSkinInfluences; ++slot) {
					const int rigBone = view.RigBoneOf(weighted.bone[slot]);
					if (rigBone < 0 || !(*a_mesh.dynamicBone)[static_cast<std::size_t>(rigBone)] || weighted.weight[slot] <= view.ThresholdOf(rigBone))
						continue;
					const auto found = std::ranges::find_if(owners, [rigBone](const Owner& a_owner) { return a_owner.bone == rigBone; });
					if (found == owners.end())
						owners.push_back({ rigBone, 1u });
					else
						++found->corners;
				}
			}
			const float flexible = (view.FlexibleOf(vertices[corners[0]]) + view.FlexibleOf(vertices[corners[1]]) + view.FlexibleOf(vertices[corners[2]])) / 3.0f;
			if (owners.empty() || flexible < kTriangleContactEpsilon)
				continue;

			std::array<ClothVertex, kTriangleCorners> posed;
			bool posable = true;
			for (std::size_t corner = 0; corner < kTriangleCorners && posable; ++corner)
				posable = slotOfVertex[corners[corner]] != kNoClothVertex || view.MakeVertex(vertices[corners[corner]], posed[corner]);
			if (!posable)
				continue;

			ClothTriangle packed;
			float multiplier = 0.0f;
			for (std::size_t corner = 0; corner < kTriangleCorners; ++corner) {
				std::uint32_t& slot = slotOfVertex[corners[corner]];
				if (slot == kNoClothVertex) {
					slot = static_cast<std::uint32_t>(a_set.vertices.size());
					a_set.vertices.push_back(posed[corner]);
				}
				packed.corners[corner] = slot;
				multiplier += a_set.vertices[slot].marginMultiplier;
			}
			multiplier /= 3.0f;
			packed.margin = a_mesh.margin * multiplier;
			packed.penetration = a_mesh.penetration * multiplier;
			const auto index = static_cast<std::uint32_t>(a_set.triangles.size());
			a_set.triangles.push_back(packed);
			for (const Owner& owner : owners)
				a_set.boneTriangles[static_cast<std::size_t>(owner.bone)].push_back({ index, flexible, owner.corners });
		}

		const std::size_t added = a_set.triangles.size() - firstTriangle;
		if (added > 0)
			a_set.pieces.push_back({ a_mesh.name, static_cast<std::uint32_t>(firstTriangle), static_cast<std::uint32_t>(added) });
		return added;
	}

	void MaskClothPiece(ClothTriangleSet& a_set, const ClothPiece& a_piece, std::uint32_t a_mask) noexcept
	{
		for (std::size_t index = a_piece.firstTriangle; index < std::size_t{ a_piece.firstTriangle } + a_piece.triangleCount && index < a_set.triangles.size(); ++index)
			a_set.triangles[index].surfaceMask &= a_mask;
	}

	Vec3 PoseClothVertex(const ClothVertex& a_vertex, const std::vector<Transform>& a_boneWorld) noexcept
	{
		if (a_vertex.bone < 0 || static_cast<std::size_t>(a_vertex.bone) >= a_boneWorld.size())
			return a_vertex.local;
		const Vec3 owned = a_boneWorld[static_cast<std::size_t>(a_vertex.bone)].TransformPoint(a_vertex.local);
		float total = a_vertex.weight;
		Vec3 blended = owned * total;
		for (const SphereInfluence& other : a_vertex.others) {
			if (other.bone < 0 || static_cast<std::size_t>(other.bone) >= a_boneWorld.size())
				continue;
			blended = blended + a_boneWorld[static_cast<std::size_t>(other.bone)].TransformPoint(other.local) * other.weight;
			total += other.weight;
		}
		return total > 0.0f && IsFiniteVector(blended) ? blended / total : owned;
	}

	TriangleContact TestVertexAgainstTriangle(const Vec3& a_vertex, float a_radius, const Vec3& a_a, const Vec3& a_b, const Vec3& a_c,
		float a_margin, float a_penetration) noexcept
	{
		TriangleContact result;
		const Vec3 raw = Cross(a_b - a_a, a_c - a_a);
		const float twiceArea = Length(raw);
		if (!(twiceArea >= kTriangleContactEpsilon))
			return result;
		Vec3 normal = raw / twiceArea;
		float penetration = std::fabs(a_penetration) < kTriangleContactEpsilon ? 0.0f : a_penetration;
		if (penetration < 0.0f) {
			normal = -normal;
			penetration = -penetration;
		}

		float distance = Dot(a_vertex - a_a, normal);
		const Vec3 projection = a_vertex - normal * distance;
		const float reach = a_radius + a_margin;
		if (penetration >= kTriangleContactEpsilon) {
			if (!(distance < reach && distance >= -penetration))
				return result;
		} else {
			if (distance < 0.0f) {
				distance = -distance;
				normal = -normal;
			}
			if (!(distance < reach))
				return result;
		}

		const Vec3 toA = projection - a_a;
		const Vec3 toB = projection - a_b;
		const Vec3 toC = projection - a_c;
		const float areaA = Length(Cross(toB, toC));
		const float areaB = Length(Cross(toC, toA));
		const float areaC = Length(Cross(toA, toB));
		if (areaA + areaB > twiceArea || areaB + areaC > twiceArea || areaC + areaA > twiceArea)
			return result;

		const float depth = reach - distance;
		if (!(depth > kTriangleContactEpsilon))
			return result;
		result.depth = depth;
		result.normal = -normal;
		result.point = projection;
		result.bodyPoint = a_vertex - normal * a_radius;
		return result;
	}
}
