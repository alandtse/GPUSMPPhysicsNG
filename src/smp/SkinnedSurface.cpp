#include "SkinnedSurface.h"

#include <array>
#include <cfloat>
#include <cstddef>
#include <utility>

namespace SmpPhysics
{
	namespace
	{
		/** @brief Slot value CompactSurface writes for a vertex no kept triangle has claimed yet. */
		constexpr std::uint32_t kUnclaimedVertex = 0xFFFFFFFFu;

		bool TriangleIsWeighted(const SkinnedVertex& a_first, const SkinnedVertex& a_second, const SkinnedVertex& a_third,
			const std::vector<float>& a_threshold) noexcept
		{
			std::array<int, 12> bones{};
			std::array<float, 12> sums{};
			int count = 0;
			for (const SkinnedVertex* vertex : { &a_first, &a_second, &a_third }) {
				for (std::size_t slot = 0; slot < Util::kSkinInfluences; ++slot) {
					const int bone = vertex->bone[slot];
					if (bone < 0 || !(vertex->weight[slot] >= FLT_EPSILON))
						continue;
					int found = 0;
					while (found < count && bones[static_cast<std::size_t>(found)] != bone)
						++found;
					if (found == count) {
						bones[static_cast<std::size_t>(count)] = bone;
						++count;
					}
					sums[static_cast<std::size_t>(found)] += vertex->weight[slot];
				}
			}
			for (int index = 0; index < count; ++index) {
				const auto bone = static_cast<std::size_t>(bones[static_cast<std::size_t>(index)]);
				const float threshold = bone < a_threshold.size() ? a_threshold[bone] : 0.0f;
				if (sums[static_cast<std::size_t>(index)] > FLT_EPSILON && sums[static_cast<std::size_t>(index)] > threshold)
					return true;
			}
			return false;
		}

		bool WithinRadius(const Vec3& a_point, const Vec3& a_centre, float a_radiusSquared) noexcept
		{
			const Vec3 offset = a_point - a_centre;
			return Dot(offset, offset) <= a_radiusSquared;
		}
	}

	std::vector<std::uint32_t> KeepWeightedTriangles(const std::vector<SkinnedVertex>& a_vertices,
		const std::vector<std::uint32_t>& a_indices, const std::vector<float>& a_threshold)
	{
		std::vector<std::uint32_t> kept;
		kept.reserve(a_indices.size());
		for (std::size_t index = 0; index + 2 < a_indices.size(); index += 3) {
			const std::uint32_t first = a_indices[index];
			const std::uint32_t second = a_indices[index + 1];
			const std::uint32_t third = a_indices[index + 2];
			if (first >= a_vertices.size() || second >= a_vertices.size() || third >= a_vertices.size())
				continue;
			if (TriangleIsWeighted(a_vertices[first], a_vertices[second], a_vertices[third], a_threshold))
				kept.insert(kept.end(), { first, second, third });
		}
		return kept;
	}

	std::vector<Vec3> PoseSurface(const SkinnedSurface& a_surface, const std::vector<Transform>& a_driverWorld)
	{
		const std::size_t boneCount = a_surface.skinToBone.size();
		std::vector<Transform> skin(boneCount);
		std::vector<char> driven(boneCount, 0);
		for (std::size_t bone = 0; bone < boneCount; ++bone) {
			const int driver = bone < a_surface.boneDriver.size() ? a_surface.boneDriver[bone] : -1;
			if (driver >= 0 && static_cast<std::size_t>(driver) < a_driverWorld.size()) {
				skin[bone] = a_driverWorld[static_cast<std::size_t>(driver)] * a_surface.skinToBone[bone];
				driven[bone] = 1;
			}
		}

		std::vector<Vec3> positions;
		positions.reserve(a_surface.vertices.size());
		for (const SkinnedVertex& vertex : a_surface.vertices) {
			Vec3 sum{};
			float total = 0.0f;
			for (std::size_t slot = 0; slot < Util::kSkinInfluences; ++slot) {
				const int bone = vertex.bone[slot];
				if (bone < 0 || static_cast<std::size_t>(bone) >= boneCount || driven[static_cast<std::size_t>(bone)] == 0)
					continue;
				const float weight = vertex.weight[slot];
				if (!(weight > 0.0f))
					continue;
				const float scale = static_cast<std::size_t>(bone) < a_surface.skinScale.size() ? a_surface.skinScale[static_cast<std::size_t>(bone)] : 1.0f;
				sum = sum + ScaledSkinPoint(skin[static_cast<std::size_t>(bone)], vertex.position, scale) * weight;
				total += weight;
			}
			positions.push_back(total > 0.0f ? sum / total : vertex.position);
		}
		return positions;
	}

	std::vector<std::uint32_t> KeepTrianglesNear(const std::vector<Vec3>& a_positions,
		const std::vector<std::uint32_t>& a_indices, const Vec3& a_centre, float a_radius)
	{
		const float radiusSquared = a_radius * a_radius;
		std::vector<std::uint32_t> kept;
		kept.reserve(a_indices.size());
		for (std::size_t index = 0; index + 2 < a_indices.size(); index += 3) {
			const std::uint32_t first = a_indices[index];
			const std::uint32_t second = a_indices[index + 1];
			const std::uint32_t third = a_indices[index + 2];
			if (first >= a_positions.size() || second >= a_positions.size() || third >= a_positions.size())
				continue;
			if (!WithinRadius(a_positions[first], a_centre, radiusSquared) ||
				!WithinRadius(a_positions[second], a_centre, radiusSquared) ||
				!WithinRadius(a_positions[third], a_centre, radiusSquared))
				continue;
			kept.insert(kept.end(), { first, second, third });
		}
		return kept;
	}

	void CompactSurface(SkinnedSurface& a_surface)
	{
		const std::size_t vertexCount = a_surface.vertices.size();
		std::vector<std::uint32_t> remap(vertexCount, kUnclaimedVertex);
		for (const std::uint32_t vertex : a_surface.indices) {
			if (vertex < vertexCount)
				remap[vertex] = 0;
		}

		std::vector<SkinnedVertex> vertices;
		vertices.reserve(vertexCount);
		for (std::size_t vertex = 0; vertex < vertexCount; ++vertex) {
			if (remap[vertex] == kUnclaimedVertex)
				continue;
			remap[vertex] = static_cast<std::uint32_t>(vertices.size());
			vertices.push_back(a_surface.vertices[vertex]);
		}

		std::vector<std::uint32_t> indices;
		indices.reserve(a_surface.indices.size());
		for (std::size_t index = 0; index + 2 < a_surface.indices.size(); index += 3) {
			const std::uint32_t first = a_surface.indices[index];
			const std::uint32_t second = a_surface.indices[index + 1];
			const std::uint32_t third = a_surface.indices[index + 2];
			if (first >= vertexCount || second >= vertexCount || third >= vertexCount)
				continue;
			indices.insert(indices.end(), { remap[first], remap[second], remap[third] });
		}

		a_surface.vertices = std::move(vertices);
		a_surface.indices = std::move(indices);
	}
}
