#include "WindDrag.h"

#include <algorithm>
#include <cmath>

namespace SmpPhysics
{
	std::vector<BoneWindGeometry> ComputeBoneWindGeometry(const SmpRig& a_rig, const std::vector<Transform>& a_restWorld,
		const std::vector<float>& a_areas)
	{
		const std::size_t boneCount = a_rig.bones.size();
		std::vector<BoneWindGeometry> geometry(boneCount);
		if (a_restWorld.size() != boneCount)
			return geometry;

		std::vector<int> firstChild(boneCount, -1);
		std::vector<int> parentOf(boneCount, -1);
		for (const RigJoint& joint : a_rig.joints) {
			const auto child = static_cast<std::size_t>(joint.child);
			const auto parent = static_cast<std::size_t>(joint.parent);
			if (child >= boneCount || parent >= boneCount)
				continue;
			parentOf[child] = joint.parent;
			if (firstChild[parent] < 0)
				firstChild[parent] = joint.child;
		}

		for (std::size_t bone = 0; bone < boneCount; ++bone) {
			if (a_rig.bones[bone].kinematic)
				continue;
			Vec3 segment{ 0.0f, 0.0f, -1.0f };
			if (firstChild[bone] >= 0)
				segment = a_restWorld[static_cast<std::size_t>(firstChild[bone])].translation - a_restWorld[bone].translation;
			else if (parentOf[bone] >= 0)
				segment = a_restWorld[bone].translation - a_restWorld[static_cast<std::size_t>(parentOf[bone])].translation;
			const float segmentLength = Length(segment);
			const float length = std::max(segmentLength, kMinWindLength);
			const Vec3 worldAxis = segmentLength > 1e-6f ? segment / segmentLength : Vec3{ 0.0f, 0.0f, -1.0f };
			const float measured = bone < a_areas.size() ? a_areas[bone] : 0.0f;
			geometry[bone].axis = Conjugate(a_restWorld[bone].rotation).Rotate(worldAxis);
			geometry[bone].area = measured > 0.0f ? std::clamp(measured, length * kMinWindWidth, length * kMaxWindWidth) : length * kDefaultWindWidth;
		}
		return geometry;
	}

	std::vector<float> EstimateBoneWindAreas(const SmpRig& a_rig, const std::vector<CardSkin>& a_skins)
	{
		const std::size_t boneCount = a_rig.bones.size();
		std::vector<float> areas(boneCount, 0.0f);
		for (const CardSkin& skin : a_skins) {
			for (std::size_t corner = 0; corner + 2 < skin.triangles.size(); corner += 3) {
				const std::size_t index[3] = { skin.triangles[corner], skin.triangles[corner + 1], skin.triangles[corner + 2] };
				if (index[0] >= skin.vertices.size() || index[1] >= skin.vertices.size() || index[2] >= skin.vertices.size())
					continue;
				const Vec3 first = skin.vertices[index[0]].position;
				const float area = 0.5f * Length(Cross(skin.vertices[index[1]].position - first, skin.vertices[index[2]].position - first));
				if (!(area > 0.0f))
					continue;
				for (const std::size_t vertexIndex : index) {
					const SkinnedVertex& vertex = skin.vertices[vertexIndex];
					float total = 0.0f;
					for (int slot = 0; slot < 4; ++slot) {
						if (vertex.weight[slot] > 0.0f)
							total += vertex.weight[slot];
					}
					if (!(total > 0.0f))
						continue;
					for (int slot = 0; slot < 4; ++slot) {
						const int skinBone = vertex.bone[slot];
						if (!(vertex.weight[slot] > 0.0f) || skinBone < 0 || static_cast<std::size_t>(skinBone) >= skin.rigBone.size())
							continue;
						const int rigBone = skin.rigBone[static_cast<std::size_t>(skinBone)];
						if (rigBone >= 0 && static_cast<std::size_t>(rigBone) < boneCount)
							areas[static_cast<std::size_t>(rigBone)] += area / 3.0f * vertex.weight[slot] / total;
					}
				}
			}
		}
		return areas;
	}
}
