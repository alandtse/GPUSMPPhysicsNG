#include "ClothGraph.h"

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace SmpPhysics
{
	namespace
	{
		bool IsBoneIndex(int a_index, std::size_t a_boneCount) noexcept
		{
			return a_index >= 0 && static_cast<std::size_t>(a_index) < a_boneCount;
		}

		float HalfWindow(float a_lower, float a_upper) noexcept
		{
			if (a_lower > a_upper)
				return kUnlimitedLateralLength;
			return (a_upper - a_lower) * 0.5f;
		}

		bool IsDuplicate(const std::vector<ClothConstraint>& a_constraints, std::uint32_t a_first, std::uint32_t a_second,
			ClothConstraintKind a_kind) noexcept
		{
			const std::uint32_t low = std::min(a_first, a_second);
			const std::uint32_t high = std::max(a_first, a_second);
			for (const ClothConstraint& constraint : a_constraints) {
				if (constraint.kind != a_kind)
					continue;
				if (std::min(constraint.first, constraint.second) == low && std::max(constraint.first, constraint.second) == high)
					return true;
			}
			return false;
		}

		void AddConstraint(const SmpRig& a_rig, ClothGraph& a_graph, std::uint32_t a_first, std::uint32_t a_second,
			float a_restLength, float a_minLength, float a_maxLength, ClothConstraintKind a_kind)
		{
			const std::size_t boneCount = a_rig.bones.size();
			if (a_first >= boneCount || a_second >= boneCount)
				return;
			if (a_rig.bones[a_first].kinematic && a_rig.bones[a_second].kinematic)
				return;
			if (!std::isfinite(a_restLength) || !std::isfinite(a_minLength) || !std::isfinite(a_maxLength))
				return;
			if (IsDuplicate(a_graph.constraints, a_first, a_second, a_kind))
				return;
			a_graph.constraints.push_back({ a_first, a_second, a_restLength, a_minLength, a_maxLength, a_kind, 0u });
		}

		std::uint32_t AssignBatches(std::vector<ClothConstraint>& a_constraints, std::size_t a_particleCount)
		{
			std::vector<std::vector<bool>> usedParticles;
			for (ClothConstraint& constraint : a_constraints) {
				std::uint32_t batch = 0;
				for (; batch < usedParticles.size(); ++batch) {
					const std::vector<bool>& used = usedParticles[batch];
					if (!used[constraint.first] && !used[constraint.second])
						break;
				}
				if (batch == usedParticles.size())
					usedParticles.emplace_back(a_particleCount, false);
				usedParticles[batch][constraint.first] = true;
				usedParticles[batch][constraint.second] = true;
				constraint.batch = batch;
			}
			std::stable_sort(a_constraints.begin(), a_constraints.end(),
				[](const ClothConstraint& a_left, const ClothConstraint& a_right) { return a_left.batch < a_right.batch; });
			return static_cast<std::uint32_t>(usedParticles.size());
		}
	}

	bool IsLatticeRig(const SmpRig& a_rig) noexcept
	{
		const std::size_t boneCount = a_rig.bones.size();
		for (const RigJoint& joint : a_rig.additionalJoints) {
			if (!IsBoneIndex(joint.child, boneCount) || !IsBoneIndex(joint.parent, boneCount))
				continue;
			if (!a_rig.bones[static_cast<std::size_t>(joint.child)].kinematic &&
				!a_rig.bones[static_cast<std::size_t>(joint.parent)].kinematic)
				return true;
		}
		return false;
	}

	ClothGraph BuildClothGraph(const SmpRig& a_rig, const std::vector<Transform>& a_restWorld)
	{
		ClothGraph graph;
		if (a_rig.bones.size() != a_restWorld.size())
			return graph;

		const std::size_t boneCount = a_rig.bones.size();
		graph.particles.resize(boneCount);
		for (std::size_t index = 0; index < boneCount; ++index) {
			const RigBone& bone = a_rig.bones[index];
			ClothParticle& particle = graph.particles[index];
			particle.bone = static_cast<std::uint32_t>(index);
			particle.inverseMass = bone.kinematic || !(bone.mass > 0.0f) ? 0.0f : 1.0f / bone.mass;
		}

		for (const RigJoint& joint : a_rig.joints) {
			if (!IsBoneIndex(joint.child, boneCount) || !IsBoneIndex(joint.parent, boneCount))
				continue;
			const Vec3 childPosition = a_restWorld[static_cast<std::size_t>(joint.child)].translation;
			const Vec3 parentPosition = a_restWorld[static_cast<std::size_t>(joint.parent)].translation;
			const float restLength = Length(childPosition - parentPosition);
			AddConstraint(a_rig, graph, static_cast<std::uint32_t>(joint.child), static_cast<std::uint32_t>(joint.parent),
				restLength, restLength, restLength, ClothConstraintKind::ChainLength);
		}

		std::vector<int> parentOfBone(boneCount, -1);
		for (const RigJoint& joint : a_rig.joints) {
			if (IsBoneIndex(joint.child, boneCount))
				parentOfBone[static_cast<std::size_t>(joint.child)] = joint.parent;
		}
		for (const RigJoint& joint : a_rig.joints) {
			if (!IsBoneIndex(joint.child, boneCount) || !IsBoneIndex(joint.parent, boneCount))
				continue;
			const int grandparent = parentOfBone[static_cast<std::size_t>(joint.parent)];
			if (!IsBoneIndex(grandparent, boneCount))
				continue;
			const Vec3 childPosition = a_restWorld[static_cast<std::size_t>(joint.child)].translation;
			const Vec3 grandparentPosition = a_restWorld[static_cast<std::size_t>(grandparent)].translation;
			const float restLength = Length(childPosition - grandparentPosition);
			AddConstraint(a_rig, graph, static_cast<std::uint32_t>(joint.child), static_cast<std::uint32_t>(grandparent),
				restLength, kBendMinimumFraction * restLength, restLength, ClothConstraintKind::Bend);
		}

		for (const RigJoint& joint : a_rig.additionalJoints) {
			if (!IsBoneIndex(joint.child, boneCount) || !IsBoneIndex(joint.parent, boneCount))
				continue;
			if (a_rig.bones[static_cast<std::size_t>(joint.child)].kinematic ||
				a_rig.bones[static_cast<std::size_t>(joint.parent)].kinematic)
				continue;
			const Transform& childRest = a_restWorld[static_cast<std::size_t>(joint.child)];
			const Transform& parentRest = a_restWorld[static_cast<std::size_t>(joint.parent)];
			const Quat inverseParentRotation = Conjugate(parentRest.rotation.Normalized());
			const Vec3 restOffset = inverseParentRotation.Rotate(childRest.translation - parentRest.translation);
			const float restLength = Length(restOffset);
			const Vec3 halfWindow{ HalfWindow(joint.linearLower.x, joint.linearUpper.x),
				HalfWindow(joint.linearLower.y, joint.linearUpper.y),
				HalfWindow(joint.linearLower.z, joint.linearUpper.z) };
			const float slack = Length(halfWindow);
			AddConstraint(a_rig, graph, static_cast<std::uint32_t>(joint.child), static_cast<std::uint32_t>(joint.parent),
				restLength, std::max(0.0f, restLength - slack), restLength + slack, ClothConstraintKind::Lateral);
		}

		graph.batchCount = AssignBatches(graph.constraints, graph.particles.size());
		return graph;
	}
}
