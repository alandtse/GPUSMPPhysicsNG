#include "GpuRigPacking.h"

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstddef>
#include <limits>
#include <utility>

namespace SmpPhysics::GpuRig
{
	namespace
	{
		/** @brief Mirrors TriangleGrid's own rejection threshold, so both solvers pack the same triangle set. */
		constexpr float kMinimumTriangleArea = 1e-8f;

		Float4 PackQuat(const Quat& a_value) noexcept
		{
			return { a_value.w, a_value.x, a_value.y, a_value.z };
		}

		Float4 PackVec3(const Vec3& a_value) noexcept
		{
			return { a_value.x, a_value.y, a_value.z, 0.0f };
		}

		/** @brief Writes a triangle's corners, unit normal and twice its area; false and untouched when it is degenerate. */
		bool FillSurfaceTriangle(SurfaceTriangle& a_record, const Vec3& a_a, const Vec3& a_b, const Vec3& a_c, float a_sourceBits) noexcept
		{
			const Vec3 cross = Cross(a_b - a_a, a_c - a_a);
			const float twiceArea = Length(cross);
			if (!(twiceArea > kMinimumTriangleArea))
				return false;
			a_record.a = { a_a.x, a_a.y, a_a.z, a_sourceBits };
			a_record.b = { a_b.x, a_b.y, a_b.z, 0.0f };
			a_record.c = { a_c.x, a_c.y, a_c.z, 0.0f };
			a_record.normalArea = { cross.x / twiceArea, cross.y / twiceArea, cross.z / twiceArea, twiceArea };
			return true;
		}

		/** @brief The root of a dynamic bone's tree: the last dynamic ancestor before the chain reaches a kinematic bone. */
		int FindTreeRoot(const SmpRig& a_rig, const std::vector<int>& a_jointOfBone, int a_bone) noexcept
		{
			int current = a_bone;
			while (true) {
				const int parent = a_rig.joints[static_cast<std::size_t>(a_jointOfBone[static_cast<std::size_t>(current)])].parent;
				if (a_rig.bones[static_cast<std::size_t>(parent)].kinematic)
					return current;
				current = parent;
			}
		}
	}

	RigData BuildRigData(const SmpRig& a_rig, const std::vector<Transform>& a_restWorld, const std::vector<float>& a_windAreas)
	{
		RigData data;
		if (a_rig.bones.size() != a_restWorld.size())
			return data;

		const std::size_t boneCount = a_rig.bones.size();
		std::vector<int> jointOfBone(boneCount, -1);
		for (std::size_t index = 0; index < a_rig.joints.size(); ++index)
			jointOfBone[static_cast<std::size_t>(a_rig.joints[index].child)] = static_cast<int>(index);

		data.bones.resize(boneCount);
		const std::vector<BoneWindGeometry> windGeometry = ComputeBoneWindGeometry(a_rig, a_restWorld, a_windAreas);
		for (std::size_t index = 0; index < boneCount; ++index) {
			const RigBone& bone = a_rig.bones[index];
			BoneStatic& packed = data.bones[index];
			packed.body = { bone.mass, (bone.inertia.x + bone.inertia.y + bone.inertia.z) / 3.0f,
				bone.angularDamping, bone.gravityFactor };
			packed.wind = { windGeometry[index].axis.x, windGeometry[index].axis.y, windGeometry[index].axis.z, windGeometry[index].area };
			if (bone.kinematic)
				continue;

			const RigJoint& joint = a_rig.joints[static_cast<std::size_t>(jointOfBone[index])];
			const Transform& childRest = a_restWorld[static_cast<std::size_t>(joint.child)];
			const Transform& parentRest = a_restWorld[static_cast<std::size_t>(joint.parent)];
			const JointFrames resolved = ResolveJointFrames(joint, childRest, parentRest);
			packed.rotationA = PackQuat(resolved.frameA.rotation);
			packed.originA = PackVec3(resolved.frameA.translation);
			packed.rotationB = PackQuat(resolved.frameB.rotation);
			packed.originB = PackVec3(resolved.frameB.translation);
			packed.stiffness = { joint.angularStiffness.x, joint.angularStiffness.y, joint.angularStiffness.z,
				joint.angularSpringsEnabled ? 1.0f : 0.0f };
			packed.damping = PackVec3(joint.angularDamping);
			packed.equilibrium = PackVec3(joint.angularEquilibrium);
			packed.lower = PackVec3(joint.angularLower);
			packed.upper = PackVec3(joint.angularUpper);
			packed.topology = { static_cast<std::uint32_t>(joint.parent),
				a_rig.bones[static_cast<std::size_t>(joint.parent)].kinematic ? 1u : 0u,
				std::bit_cast<std::uint32_t>(bone.windFactor), 0u };
		}

		const std::vector<int> order = BuildParentFirstOrder(a_rig, jointOfBone);
		std::vector<std::vector<std::uint32_t>> members;
		std::vector<int> treeOfRoot(boneCount, -1);
		for (const int bone : order) {
			const int root = FindTreeRoot(a_rig, jointOfBone, bone);
			if (treeOfRoot[static_cast<std::size_t>(root)] < 0) {
				treeOfRoot[static_cast<std::size_t>(root)] = static_cast<int>(members.size());
				members.emplace_back();
			}
			members[static_cast<std::size_t>(treeOfRoot[static_cast<std::size_t>(root)])].push_back(static_cast<std::uint32_t>(bone));
		}

		data.trees.reserve(members.size());
		data.order.reserve(order.size());
		for (std::size_t tree = 0; tree < members.size(); ++tree) {
			const std::uint32_t root = members[tree].front();
			const int jointIndex = jointOfBone[root];
			TreeStatic packedTree;
			packedTree.range = { static_cast<std::uint32_t>(data.order.size()), static_cast<std::uint32_t>(members[tree].size()),
				static_cast<std::uint32_t>(a_rig.joints[static_cast<std::size_t>(jointIndex)].parent), 0u };
			data.trees.push_back(packedTree);
			data.order.insert(data.order.end(), members[tree].begin(), members[tree].end());
		}
		return data;
	}

	CollisionData PackCollision(const std::vector<std::vector<BoneSphere>>& a_spheres,
		const std::vector<Collider>& a_colliders, std::uint32_t a_driverCount, const std::vector<std::uint32_t>& a_boneSurfaceMask)
	{
		CollisionData data;
		data.driverCount = a_driverCount;
		data.spheres.resize(a_spheres.size());
		for (std::size_t bone = 0; bone < a_spheres.size(); ++bone) {
			std::vector<BoneSphere> spheres = a_spheres[bone];
			if (spheres.size() > static_cast<std::size_t>(kMaxContactSpheresPerBone))
				spheres.resize(kMaxContactSpheresPerBone);
			const SphereBound bound = BoundSpheres(spheres);
			BoneSpheres& packed = data.spheres[bone];
			const bool masked = bone < a_boneSurfaceMask.size();
			packed.range = { static_cast<std::uint32_t>(data.sphereList.size()), static_cast<std::uint32_t>(spheres.size()), masked ? a_boneSurfaceMask[bone] : 0u, masked ? 1u : 0u };
			packed.bound = { bound.centre.x, bound.centre.y, bound.centre.z, bound.radius };
			for (const BoneSphere& sphere : spheres) {
				// The rest-exemption masks are one bit per claim-local collider, so laying the claim out does not move them.
				data.sphereList.push_back({ { sphere.centerLocal.x, sphere.centerLocal.y, sphere.centerLocal.z, sphere.radius },
					{ sphere.exemptMask, 0u, 0u, 0u } });
			}
		}

		data.colliders.reserve(a_colliders.size());
		for (const Collider& collider : a_colliders)
			data.colliders.push_back(PackCollider(collider));
		return data;
	}

	ColliderPacked PackCollider(const Collider& a_collider)
	{
		ColliderPacked packed;
		packed.orientation = PackQuat(a_collider.orientation);
		packed.center = PackVec3(a_collider.center);
		packed.halfExtents = PackVec3(a_collider.halfExtents);
		packed.halfExtents.w = a_collider.radius;
		packed.segmentA = PackVec3(a_collider.segmentA);
		packed.segmentB = PackVec3(a_collider.segmentB);
		packed.normal = PackVec3(a_collider.normal);
		packed.normal.w = a_collider.penetration;
		packed.kind = { static_cast<std::uint32_t>(a_collider.type),
			a_collider.driverIndex < 0 ? kInvalidDriverSlot : static_cast<std::uint32_t>(a_collider.driverIndex), 0u, 0u };
		return packed;
	}

	void AppendSurface(SurfaceFrame& a_frame, const SkinnedSurface& a_surface, const std::vector<Transform>& a_driverWorld,
		TriangleBvh& a_bvh)
	{
		const std::vector<Vec3> posed = PoseSurface(a_surface, a_driverWorld);
		if (!a_bvh.Ready() || a_bvh.TriangleCount() != a_surface.indices.size() / 3)
			a_bvh.Build(posed, a_surface.indices);
		else
			a_bvh.Refit(posed, a_surface.indices);
		if (!a_bvh.Ready())
			return;

		const std::vector<std::uint32_t>& leafOrder = a_bvh.LeafOrder();
		std::vector<SurfaceTriangle> triangles(leafOrder.size());
		bool anyUsable = false;
		for (std::size_t slot = 0; slot < leafOrder.size(); ++slot) {
			const std::uint32_t source = leafOrder[slot];
			const float sourceBits = std::bit_cast<float>(source);
			SurfaceTriangle& record = triangles[slot];
			record.a.w = sourceBits;
			const std::uint32_t i0 = a_surface.indices[static_cast<std::size_t>(source) * 3];
			const std::uint32_t i1 = a_surface.indices[static_cast<std::size_t>(source) * 3 + 1];
			const std::uint32_t i2 = a_surface.indices[static_cast<std::size_t>(source) * 3 + 2];
			if (i0 >= posed.size() || i1 >= posed.size() || i2 >= posed.size())
				continue;
			const Vec3& a = posed[i0];
			const Vec3& b = posed[i1];
			const Vec3& c = posed[i2];
			if (!IsFiniteVector(a) || !IsFiniteVector(b) || !IsFiniteVector(c))
				continue;
			if (FillSurfaceTriangle(record, a, b, c, sourceBits))
				anyUsable = true;
		}
		if (!anyUsable)
			return;

		SurfaceMeta meta;
		meta.dimensions = { 0u, 0u, 0u, static_cast<std::uint32_t>(leafOrder.size()) };
		meta.base = { static_cast<std::uint32_t>(a_frame.triangles.size()), static_cast<std::uint32_t>(a_frame.nodes.size()),
			static_cast<std::uint32_t>(a_bvh.Nodes().size()), 0u };
		meta.contact = { a_surface.margin, a_surface.contactPenetration, a_surface.deepPush ? kDeepPushReach : 0.0f, 0.0f };
		a_frame.metas.push_back(meta);

		a_frame.triangles.insert(a_frame.triangles.end(), triangles.begin(), triangles.end());
		a_frame.nodes.insert(a_frame.nodes.end(), a_bvh.Nodes().begin(), a_bvh.Nodes().end());
	}

	void AppendSurfaces(SurfaceFrame& a_frame, const std::vector<SkinnedSurface>& a_surfaces,
		const std::vector<Transform>& a_driverWorld, std::uint32_t& a_firstMeta, std::uint32_t& a_metaCount)
	{
		std::vector<TriangleBvh> bvhs(a_surfaces.size());
		a_firstMeta = static_cast<std::uint32_t>(a_frame.metas.size());
		for (std::size_t index = 0; index < a_surfaces.size(); ++index)
			AppendSurface(a_frame, a_surfaces[index], a_driverWorld, bvhs[index]);
		a_metaCount = static_cast<std::uint32_t>(a_frame.metas.size()) - a_firstMeta;
	}

	void MergeSurfaceFrame(SurfaceFrame& a_frame, const SurfaceFrame& a_part, std::uint32_t& a_firstMeta)
	{
		a_firstMeta = static_cast<std::uint32_t>(a_frame.metas.size());
		const std::uint32_t triangleBase = static_cast<std::uint32_t>(a_frame.triangles.size());
		const std::uint32_t nodeBase = static_cast<std::uint32_t>(a_frame.nodes.size());
		for (SurfaceMeta meta : a_part.metas) {
			meta.base.x += triangleBase;
			meta.base.y += nodeBase;
			a_frame.metas.push_back(meta);
		}
		a_frame.triangles.insert(a_frame.triangles.end(), a_part.triangles.begin(), a_part.triangles.end());
		a_frame.nodes.insert(a_frame.nodes.end(), a_part.nodes.begin(), a_part.nodes.end());
	}

	void SetCollision(RigData& a_rig, CollisionData a_collision) noexcept
	{
		const std::uint32_t colliderCount = static_cast<std::uint32_t>(a_collision.colliders.size());
		for (TreeStatic& tree : a_rig.trees)
			tree.contact = { 0u, colliderCount, 0u, a_collision.driverCount };
		a_rig.collision = std::move(a_collision);
	}

	std::uint32_t SlotSpan(const RigData& a_rig) noexcept
	{
		return static_cast<std::uint32_t>(a_rig.bones.size() + a_rig.collision.colliders.size()) +
		       a_rig.collision.driverCount;
	}

	std::vector<std::uint32_t> BoneTreeSlots(const RigData& a_rig, std::uint32_t a_boneOffset, std::uint32_t a_boneCount)
	{
		std::vector<std::uint32_t> slots(a_boneCount, kNoTree);
		for (std::size_t tree = 0; tree < a_rig.trees.size(); ++tree) {
			const TreeStatic& packed = a_rig.trees[tree];
			for (std::uint32_t index = 0; index < packed.range.y; ++index) {
				const std::uint32_t bone = a_rig.order[packed.range.x - a_boneOffset + index];
				if (bone >= a_boneOffset && bone - a_boneOffset < a_boneCount)
					slots[bone - a_boneOffset] = a_boneOffset + static_cast<std::uint32_t>(tree);
			}
		}
		return slots;
	}

	ClaimStatics PackClaimStatics(const RigData& a_rig, std::uint32_t a_boneOffset, std::uint32_t a_boneCount, std::uint32_t a_sphereBase)
	{
		ClaimStatics statics;
		statics.bones = a_rig.bones;
		statics.trees = a_rig.trees;
		statics.order = a_rig.order;
		statics.colliders = a_rig.collision.colliders;
		statics.boneTrees = BoneTreeSlots(a_rig, a_boneOffset, a_boneCount);
		statics.spheres = a_rig.collision.spheres;
		for (BoneSpheres& packed : statics.spheres)
			packed.range.x += a_sphereBase;
		statics.sphereList = a_rig.collision.sphereList;
		for (std::size_t bone = 0; bone < a_rig.collision.spheres.size(); ++bone) {
			const BoneSpheres& packed = a_rig.collision.spheres[bone];
			for (std::uint32_t member = 0; member < packed.range.y; ++member) {
				const std::size_t record = packed.range.x + member;
				if (record < statics.sphereList.size())
					statics.sphereList[record].exempt.y = a_boneOffset + static_cast<std::uint32_t>(bone);
			}
		}
		return statics;
	}

	void ShiftToSlotBase(RigData& a_rig, std::uint32_t a_base) noexcept
	{
		const std::uint32_t colliderBase = a_base + static_cast<std::uint32_t>(a_rig.bones.size());
		const std::uint32_t driverBase = colliderBase + static_cast<std::uint32_t>(a_rig.collision.colliders.size());
		for (BoneStatic& bone : a_rig.bones)
			bone.topology.x += a_base;
		for (ColliderPacked& collider : a_rig.collision.colliders) {
			if (collider.kind.y != kInvalidDriverSlot)
				collider.kind.y += driverBase;
		}
		for (TreeStatic& tree : a_rig.trees) {
			tree.range.x += a_base;
			tree.range.z += a_base;
			tree.contact.x += colliderBase;
			tree.contact.z += driverBase;
		}
		for (std::uint32_t& bone : a_rig.order)
			bone += a_base;
	}

	std::vector<int> BuildTreeKinematicParents(const SmpRig& a_rig)
	{
		const std::size_t boneCount = a_rig.bones.size();
		std::vector<int> parents(boneCount);
		for (std::size_t index = 0; index < boneCount; ++index)
			parents[index] = static_cast<int>(index);
		if (boneCount == 0)
			return parents;

		std::vector<int> jointOfBone(boneCount, -1);
		for (std::size_t index = 0; index < a_rig.joints.size(); ++index)
			jointOfBone[static_cast<std::size_t>(a_rig.joints[index].child)] = static_cast<int>(index);

		for (std::size_t index = 0; index < boneCount; ++index) {
			if (a_rig.bones[index].kinematic)
				continue;
			parents[index] = a_rig.joints[static_cast<std::size_t>(jointOfBone[FindTreeRoot(a_rig, jointOfBone, static_cast<int>(index))])].parent;
		}
		return parents;
	}

	std::vector<float> BuildBoneReach(const SmpRig& a_rig, const std::vector<Transform>& a_restPose, float a_slack, float a_margin,
		float a_linearScale)
	{
		const std::size_t boneCount = a_rig.bones.size();
		std::vector<float> reach(boneCount, 0.0f);
		if (a_restPose.size() != boneCount)
			return reach;
		std::vector<int> parentOf(boneCount, -1);
		std::vector<float> travelOf(boneCount, 0.0f);
		for (const RigJoint& joint : a_rig.joints) {
			parentOf[static_cast<std::size_t>(joint.child)] = joint.parent;
			const float lower[3]{ joint.linearLower.x, joint.linearLower.y, joint.linearLower.z };
			const float upper[3]{ joint.linearUpper.x, joint.linearUpper.y, joint.linearUpper.z };
			float travelSquared = 0.0f;
			for (int axis = 0; axis < 3; ++axis) {
				if (lower[axis] > upper[axis])
					travelSquared = std::numeric_limits<float>::infinity();
				else
					travelSquared += (upper[axis] - lower[axis]) * (upper[axis] - lower[axis]);
			}
			travelOf[static_cast<std::size_t>(joint.child)] = std::sqrt(travelSquared) * a_linearScale;
		}

		for (std::size_t index = 0; index < boneCount; ++index) {
			if (a_rig.bones[index].kinematic)
				continue;
			float length = 0.0f;
			std::size_t bone = index;
			for (std::size_t step = 0; step < boneCount && !a_rig.bones[bone].kinematic; ++step) {
				const int parent = parentOf[bone];
				if (parent < 0)
					break;
				length += Length(a_restPose[bone].translation - a_restPose[static_cast<std::size_t>(parent)].translation) + travelOf[bone];
				bone = static_cast<std::size_t>(parent);
			}
			reach[index] = length * a_slack + a_margin;
		}
		return reach;
	}

	bool PoseExceedsReach(const SmpRig& a_rig, const std::vector<Transform>& a_bones, const std::vector<float>& a_reach)
	{
		if (a_bones.size() != a_rig.bones.size() || a_reach.size() != a_bones.size())
			return false;
		for (std::size_t index = 0; index < a_bones.size(); ++index) {
			if (a_rig.bones[index].kinematic)
				continue;
			if (!IsFiniteTransform(a_bones[index]) || Length(a_bones[index].translation) > a_reach[index])
				return true;
		}
		return false;
	}

	bool AddGrab(TreeInput& a_input, std::uint32_t a_slot, const Vec3& a_local, const Vec3& a_target, float a_stiffness) noexcept
	{
		std::uint32_t* slots[kMaxGrabPinsPerTree] = { &a_input.grabSlots.x, &a_input.grabSlots.y, &a_input.grabSlots.z, &a_input.grabSlots.w };
		for (std::size_t pin = 0; pin < kMaxGrabPinsPerTree; ++pin) {
			if (*slots[pin] != 0u)
				continue;
			*slots[pin] = a_slot + 1u;
			a_input.grabLocals[pin] = { a_local.x, a_local.y, a_local.z, 0.0f };
			a_input.grabTargets[pin] = { a_target.x, a_target.y, a_target.z, a_stiffness };
			return true;
		}
		return false;
	}

	TreeInput MakeTreeInput(const SolverParams& a_params, const SubStepPlan& a_plan,
		const Transform& a_previousKinematic, const Transform& a_currentKinematic,
		bool a_hasPreviousInput, bool a_resetRequested)
	{
		TreeInput input;
		input.previousRotation = PackQuat(a_previousKinematic.rotation);
		input.previousTranslation = PackVec3(a_previousKinematic.translation);
		input.currentRotation = PackQuat(a_currentKinematic.rotation);
		input.currentTranslation = PackVec3(a_currentKinematic.translation);
		input.solver = { a_plan.step, a_params.gravity, a_params.gravityScale, a_params.maxAngularSpeed };
		input.scales = { a_params.stiffnessScale, a_params.dampingScale, a_params.windFactor, 0.0f };
		input.contact = { kContactStiffness, kContactDamping, kContactSkin, 0.0f };
		input.wind = { a_params.wind.x, a_params.wind.y, a_params.wind.z, a_params.windTime };
		input.control = { static_cast<std::uint32_t>(a_plan.count),
			a_hasPreviousInput ? 1u : 0u, a_resetRequested ? 1u : 0u, 0u };
		return input;
	}

	namespace
	{
		Vec3 UnpackPrepPoint(const Float4& a_value) noexcept
		{
			return { a_value.x, a_value.y, a_value.z };
		}

		bool PrepTriangleIsUsable(const SurfacePrepTriangle& a_triangle, const std::vector<Vec3>& a_posed) noexcept
		{
			if (a_triangle.corners.x >= a_posed.size() || a_triangle.corners.y >= a_posed.size() || a_triangle.corners.z >= a_posed.size())
				return false;
			return IsFiniteVector(a_posed[a_triangle.corners.x]) && IsFiniteVector(a_posed[a_triangle.corners.y]) &&
			       IsFiniteVector(a_posed[a_triangle.corners.z]);
		}

		void AddPrepTriangleBox(const SurfacePrepTriangle& a_triangle, const std::vector<Vec3>& a_posed, Vec3& a_low, Vec3& a_high) noexcept
		{
			if (!PrepTriangleIsUsable(a_triangle, a_posed))
				return;
			const Vec3& a = a_posed[a_triangle.corners.x];
			const Vec3& b = a_posed[a_triangle.corners.y];
			const Vec3& c = a_posed[a_triangle.corners.z];
			a_low = { std::min(a_low.x, std::min(a.x, std::min(b.x, c.x))),
				std::min(a_low.y, std::min(a.y, std::min(b.y, c.y))),
				std::min(a_low.z, std::min(a.z, std::min(b.z, c.z))) };
			a_high = { std::max(a_high.x, std::max(a.x, std::max(b.x, c.x))),
				std::max(a_high.y, std::max(a.y, std::max(b.y, c.y))),
				std::max(a_high.z, std::max(a.z, std::max(b.z, c.z))) };
		}

		void AddPrepNodeBox(const BvhNode& a_node, Vec3& a_low, Vec3& a_high) noexcept
		{
			a_low = { std::min(a_low.x, a_node.low[0]), std::min(a_low.y, a_node.low[1]), std::min(a_low.z, a_node.low[2]) };
			a_high = { std::max(a_high.x, a_node.high[0]), std::max(a_high.y, a_node.high[1]), std::max(a_high.z, a_node.high[2]) };
		}
	}

	void SurfacePrepData::Clear() noexcept
	{
		vertices.clear();
		bones.clear();
		triangles.clear();
		nodes.clear();
		leaves.clear();
		refit.clear();
		metas.clear();
		refitLevelStart.clear();
		refitLevels = 0;
	}

	bool AppendSurfacePrep(SurfacePrepData& a_data, const SkinnedSurface& a_surface, const TriangleBvh& a_bvh,
		std::uint32_t a_driverOffset, std::uint32_t a_driverCount)
	{
		if (!a_bvh.Ready() || a_bvh.TriangleCount() != a_surface.indices.size() / 3)
			return false;

		const std::vector<BvhNode>& sourceNodes = a_bvh.Nodes();
		std::vector<std::uint32_t> depths(sourceNodes.size(), 0);
		std::uint32_t deepestInternal = 0;
		for (std::size_t index = 0; index < sourceNodes.size(); ++index) {
			const BvhNode& node = sourceNodes[index];
			if (node.leaf != 0)
				continue;
			deepestInternal = std::max(deepestInternal, depths[index]);
			depths[index + 1] = depths[index] + 1;
			depths[sourceNodes[index + 1].skip] = depths[index] + 1;
		}
		if (deepestInternal >= kMaxRefitLevels)
			return false;

		const std::uint32_t vertexBase = static_cast<std::uint32_t>(a_data.vertices.size());
		const std::uint32_t boneBase = static_cast<std::uint32_t>(a_data.bones.size());
		const std::uint32_t triangleBase = static_cast<std::uint32_t>(a_data.triangles.size());
		const std::uint32_t nodeBase = static_cast<std::uint32_t>(a_data.nodes.size());
		const std::uint32_t vertexCount = static_cast<std::uint32_t>(a_surface.vertices.size());
		const std::uint32_t skinBoneCount = static_cast<std::uint32_t>(a_surface.skinToBone.size());

		const auto globalBone = [&](int a_bone) noexcept -> std::uint32_t {
			return a_bone >= 0 && static_cast<std::uint32_t>(a_bone) < skinBoneCount ?
			           boneBase + static_cast<std::uint32_t>(a_bone) :
			           kInvalidDriverSlot;
		};

		a_data.vertices.reserve(a_data.vertices.size() + a_surface.vertices.size());
		for (const SkinnedVertex& vertex : a_surface.vertices) {
			SurfacePrepVertex packed;
			packed.position = { vertex.position.x, vertex.position.y, vertex.position.z, 0.0f };
			packed.bones = { globalBone(vertex.bone[0]), globalBone(vertex.bone[1]), globalBone(vertex.bone[2]),
				globalBone(vertex.bone[3]) };
			packed.weights = { vertex.weight[0], vertex.weight[1], vertex.weight[2], vertex.weight[3] };
			a_data.vertices.push_back(packed);
		}

		a_data.bones.reserve(a_data.bones.size() + a_surface.skinToBone.size());
		for (std::size_t bone = 0; bone < a_surface.skinToBone.size(); ++bone) {
			const Transform& skin = a_surface.skinToBone[bone];
			const int driver = bone < a_surface.boneDriver.size() ? a_surface.boneDriver[bone] : -1;
			const std::uint32_t driverSlot = driver >= 0 && static_cast<std::uint32_t>(driver) < a_driverCount ?
			                                     a_driverOffset + static_cast<std::uint32_t>(driver) :
			                                     kInvalidDriverSlot;
			SurfacePrepBone packed;
			packed.rotation = { skin.rotation.w, skin.rotation.x, skin.rotation.y, skin.rotation.z };
			packed.translation = { skin.translation.x, skin.translation.y, skin.translation.z, 0.0f };
			packed.driver = { driverSlot, 0u, 0u, 0u };
			a_data.bones.push_back(packed);
		}

		const std::vector<std::uint32_t>& leafOrder = a_bvh.LeafOrder();
		a_data.triangles.reserve(a_data.triangles.size() + leafOrder.size());
		for (const std::uint32_t source : leafOrder) {
			const std::uint32_t i0 = a_surface.indices[static_cast<std::size_t>(source) * 3];
			const std::uint32_t i1 = a_surface.indices[static_cast<std::size_t>(source) * 3 + 1];
			const std::uint32_t i2 = a_surface.indices[static_cast<std::size_t>(source) * 3 + 2];
			SurfacePrepTriangle packed;
			packed.corners = i0 < vertexCount && i1 < vertexCount && i2 < vertexCount ?
			                     Uint4{ vertexBase + i0, vertexBase + i1, vertexBase + i2, source } :
			                     Uint4{ kInvalidDriverSlot, kInvalidDriverSlot, kInvalidDriverSlot, source };
			a_data.triangles.push_back(packed);
		}

		a_data.nodes.reserve(a_data.nodes.size() + sourceNodes.size());
		for (std::size_t index = 0; index < sourceNodes.size(); ++index) {
			const BvhNode& node = sourceNodes[index];
			SurfacePrepNode packed;
			if (node.leaf != 0)
				packed.topology = { node.skip, node.leaf, kInvalidDriverSlot, kInvalidDriverSlot };
			else
				packed.topology = { node.skip, node.leaf, nodeBase + static_cast<std::uint32_t>(index) + 1u,
					nodeBase + sourceNodes[index + 1].skip };
			a_data.nodes.push_back(packed);
		}

		a_data.leaves.reserve(a_data.leaves.size() + sourceNodes.size());
		for (std::size_t index = 0; index < sourceNodes.size(); ++index) {
			const BvhNode& node = sourceNodes[index];
			if (node.leaf == 0)
				continue;
			a_data.leaves.push_back({ { nodeBase + static_cast<std::uint32_t>(index),
				triangleBase + (node.leaf >> kBvhLeafSlotShift), node.leaf & kBvhLeafCountMask, 0u } });
		}

		a_data.refit.reserve(a_data.refit.size() + sourceNodes.size());
		for (std::size_t index = 0; index < sourceNodes.size(); ++index) {
			const BvhNode& node = sourceNodes[index];
			if (node.leaf != 0)
				continue;
			a_data.refit.push_back({ { nodeBase + static_cast<std::uint32_t>(index),
				nodeBase + static_cast<std::uint32_t>(index) + 1u, nodeBase + sourceNodes[index + 1].skip, depths[index] } });
		}

		SurfaceMeta meta;
		meta.dimensions = { 0u, 0u, 0u, static_cast<std::uint32_t>(leafOrder.size()) };
		meta.base = { triangleBase, nodeBase, static_cast<std::uint32_t>(sourceNodes.size()), 0u };
		meta.contact = { a_surface.margin, a_surface.contactPenetration, a_surface.deepPush ? kDeepPushReach : 0.0f, 0.0f };
		a_data.metas.push_back(meta);
		return true;
	}

	void FinalizeSurfacePrep(SurfacePrepData& a_data)
	{
		std::stable_sort(a_data.refit.begin(), a_data.refit.end(),
			[](const SurfacePrepRefit& a_left, const SurfacePrepRefit& a_right) {
				return a_left.link.w > a_right.link.w;
			});

		a_data.refitLevelStart.clear();
		a_data.refitLevels = 0;
		for (std::size_t index = 0; index < a_data.refit.size(); ++index) {
			if (index == 0 || a_data.refit[index].link.w != a_data.refit[index - 1].link.w) {
				a_data.refitLevelStart.push_back(static_cast<std::uint32_t>(index));
				++a_data.refitLevels;
			}
		}
		a_data.refitLevelStart.push_back(static_cast<std::uint32_t>(a_data.refit.size()));
	}

	void ReferenceSurfacePrep(const SurfacePrepData& a_data, const std::vector<Transform>& a_driverWorldBySlot,
		std::vector<SurfaceTriangle>& a_triangles, std::vector<BvhNode>& a_nodes)
	{
		std::vector<Transform> skin(a_data.bones.size());
		std::vector<char> driven(a_data.bones.size(), 0);
		for (std::size_t bone = 0; bone < a_data.bones.size(); ++bone) {
			const SurfacePrepBone& packed = a_data.bones[bone];
			if (packed.driver.x == kInvalidDriverSlot || static_cast<std::size_t>(packed.driver.x) >= a_driverWorldBySlot.size())
				continue;
			const Quat rotation{ packed.rotation.x, packed.rotation.y, packed.rotation.z, packed.rotation.w };
			const Transform skinToBone{ rotation, UnpackPrepPoint(packed.translation) };
			skin[bone] = a_driverWorldBySlot[packed.driver.x] * skinToBone;
			driven[bone] = 1;
		}

		std::vector<Vec3> posed(a_data.vertices.size());
		for (std::size_t index = 0; index < a_data.vertices.size(); ++index) {
			const SurfacePrepVertex& vertex = a_data.vertices[index];
			const Vec3 bind = UnpackPrepPoint(vertex.position);
			const std::uint32_t bones[4] = { vertex.bones.x, vertex.bones.y, vertex.bones.z, vertex.bones.w };
			const float weights[4] = { vertex.weights.x, vertex.weights.y, vertex.weights.z, vertex.weights.w };
			Vec3 sum{};
			float total = 0.0f;
			for (std::size_t slot = 0; slot < Util::kSkinInfluences; ++slot) {
				const std::uint32_t bone = bones[slot];
				if (bone == kInvalidDriverSlot || static_cast<std::size_t>(bone) >= a_data.bones.size() || driven[bone] == 0)
					continue;
				const float weight = weights[slot];
				if (!(weight > 0.0f))
					continue;
				sum = sum + skin[bone].TransformPoint(bind) * weight;
				total += weight;
			}
			posed[index] = total > 0.0f ? sum / total : bind;
		}

		a_triangles.assign(a_data.triangles.size(), SurfaceTriangle{});
		for (std::size_t slot = 0; slot < a_data.triangles.size(); ++slot) {
			const SurfacePrepTriangle& prep = a_data.triangles[slot];
			SurfaceTriangle& record = a_triangles[slot];
			const float sourceBits = std::bit_cast<float>(prep.corners.w);
			record.a.w = sourceBits;
			if (!PrepTriangleIsUsable(prep, posed))
				continue;
			const Vec3& a = posed[prep.corners.x];
			const Vec3& b = posed[prep.corners.y];
			const Vec3& c = posed[prep.corners.z];
			FillSurfaceTriangle(record, a, b, c, sourceBits);
		}

		a_nodes.assign(a_data.nodes.size(), BvhNode{});
		for (std::size_t index = 0; index < a_data.nodes.size(); ++index) {
			BvhNode& node = a_nodes[index];
			node.skip = a_data.nodes[index].topology.x;
			node.leaf = a_data.nodes[index].topology.y;
			for (int axis = 0; axis < 3; ++axis) {
				node.low[axis] = kBvhEmptyLow;
				node.high[axis] = kBvhEmptyHigh;
			}
		}

		for (const SurfacePrepLeaf& leaf : a_data.leaves) {
			Vec3 low{ kBvhEmptyLow, kBvhEmptyLow, kBvhEmptyLow };
			Vec3 high{ kBvhEmptyHigh, kBvhEmptyHigh, kBvhEmptyHigh };
			for (std::uint32_t slot = leaf.range.y; slot < leaf.range.y + leaf.range.z; ++slot)
				AddPrepTriangleBox(a_data.triangles[slot], posed, low, high);
			BvhNode& node = a_nodes[leaf.range.x];
			node.low[0] = low.x;
			node.low[1] = low.y;
			node.low[2] = low.z;
			node.high[0] = high.x;
			node.high[1] = high.y;
			node.high[2] = high.z;
		}

		for (const SurfacePrepRefit& refit : a_data.refit) {
			Vec3 low{ kBvhEmptyLow, kBvhEmptyLow, kBvhEmptyLow };
			Vec3 high{ kBvhEmptyHigh, kBvhEmptyHigh, kBvhEmptyHigh };
			AddPrepNodeBox(a_nodes[refit.link.y], low, high);
			AddPrepNodeBox(a_nodes[refit.link.z], low, high);
			BvhNode& node = a_nodes[refit.link.x];
			node.low[0] = low.x;
			node.low[1] = low.y;
			node.low[2] = low.z;
			node.high[0] = high.x;
			node.high[1] = high.y;
			node.high[2] = high.z;
		}
	}

	SurfacePrepComparison CompareSurfacePrep(const std::vector<SurfaceTriangle>& a_expectedTriangles,
		const std::vector<BvhNode>& a_expectedNodes, const SurfaceTriangle* a_actualTriangles, const BvhNode* a_actualNodes,
		std::uint32_t a_actualTriangleCount, std::uint32_t a_actualNodeCount) noexcept
	{
		SurfacePrepComparison result;
		const std::size_t expectedTriangles = a_expectedTriangles.size();
		const std::size_t expectedNodes = a_expectedNodes.size();
		const std::size_t actualTriangles = a_actualTriangles ? a_actualTriangleCount : 0u;
		const std::size_t actualNodes = a_actualNodes ? a_actualNodeCount : 0u;
		const auto countDifference = [](std::size_t a_expected, std::size_t a_actual) noexcept {
			return static_cast<std::uint32_t>(a_expected > a_actual ? a_expected - a_actual : a_actual - a_expected);
		};
		result.triangles = static_cast<std::uint32_t>(std::min(expectedTriangles, actualTriangles));
		result.nodes = static_cast<std::uint32_t>(std::min(expectedNodes, actualNodes));
		result.triangleMismatches = countDifference(expectedTriangles, actualTriangles);
		result.topologyMismatches = countDifference(expectedNodes, actualNodes);

		const auto differs = [](float a_expected, float a_actual) noexcept {
			return !(std::fabs(a_expected - a_actual) <= kSurfacePrepTolerance + kSurfacePrepRelativeTolerance * std::max(std::fabs(a_expected), std::fabs(a_actual)));
		};
		for (std::uint32_t slot = 0; slot < result.triangles; ++slot) {
			const SurfaceTriangle& expected = a_expectedTriangles[slot];
			const SurfaceTriangle& actual = a_actualTriangles[slot];
			const float expectedCorners[9] = { expected.a.x, expected.a.y, expected.a.z, expected.b.x, expected.b.y, expected.b.z,
				expected.c.x, expected.c.y, expected.c.z };
			const float actualCorners[9] = { actual.a.x, actual.a.y, actual.a.z, actual.b.x, actual.b.y, actual.b.z,
				actual.c.x, actual.c.y, actual.c.z };
			bool mismatch = std::bit_cast<std::uint32_t>(expected.a.w) != std::bit_cast<std::uint32_t>(actual.a.w);
			for (int component = 0; component < 9; ++component) {
				result.maxTriangleError = std::max(result.maxTriangleError, std::fabs(expectedCorners[component] - actualCorners[component]));
				mismatch = differs(expectedCorners[component], actualCorners[component]) || mismatch;
			}
			if (expected.normalArea.w > kSurfacePrepConditionedArea || actual.normalArea.w > kSurfacePrepConditionedArea) {
				float magnitude = 0.0f;
				for (const float corner : expectedCorners)
					magnitude = std::max(magnitude, std::fabs(corner));
				const float shape = std::max(expected.normalArea.w, actual.normalArea.w);
				const float normalTolerance = kSurfacePrepNormalTolerance + kSurfacePrepRoundingPerMagnitude * magnitude / std::sqrt(shape);
				const float expectedNormal[3] = { expected.normalArea.x, expected.normalArea.y, expected.normalArea.z };
				const float actualNormal[3] = { actual.normalArea.x, actual.normalArea.y, actual.normalArea.z };
				for (int component = 0; component < 3; ++component) {
					result.maxNormalError = std::max(result.maxNormalError, std::fabs(expectedNormal[component] - actualNormal[component]));
					mismatch = !(std::fabs(expectedNormal[component] - actualNormal[component]) <= normalTolerance) || mismatch;
				}
				mismatch = !(std::fabs(expected.normalArea.w - actual.normalArea.w) <= normalTolerance * shape) || mismatch;
			}
			if (mismatch)
				++result.triangleMismatches;
		}

		const auto compareComponent = [&result, &differs](float a_expected, float a_actual) {
			if (std::fabs(a_expected) > kSurfacePrepEmptySentinel && std::fabs(a_actual) > kSurfacePrepEmptySentinel)
				return false;
			result.maxNodeError = std::max(result.maxNodeError, std::fabs(a_expected - a_actual));
			return differs(a_expected, a_actual);
		};
		for (std::uint32_t index = 0; index < result.nodes; ++index) {
			const BvhNode& expected = a_expectedNodes[index];
			const BvhNode& actual = a_actualNodes[index];
			bool mismatch = false;
			for (int axis = 0; axis < 3; ++axis) {
				mismatch = compareComponent(expected.low[axis], actual.low[axis]) || mismatch;
				mismatch = compareComponent(expected.high[axis], actual.high[axis]) || mismatch;
			}
			if (mismatch)
				++result.nodeMismatches;
			if (expected.skip != actual.skip || expected.leaf != actual.leaf)
				++result.topologyMismatches;
		}
		return result;
	}
}
