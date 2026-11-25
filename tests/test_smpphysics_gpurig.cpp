// Unit tests for the rig-to-GPU-record packing
// (src/smp/GpuRigPacking.cpp). Pure maths; no game/D3D dependency.

#include "smp/GpuRigPacking.h"
#include "smp/TriangleGrid.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <unordered_map>
#include <vector>

using namespace SmpPhysics;
using Catch::Approx;

namespace
{
	/**
	 * @brief A three level rig with a branch and two trees: Head hangs Hair01 (which branches into
	 *        Hair02 and Hair03) and Hair04 (which hangs Hair05).
	 */
	SmpRig MakeBranchingRig()
	{
		SmpRig rig;
		rig.bones.resize(6);
		rig.bones[0].name = "Head";
		rig.bones[0].kinematic = true;
		rig.bones[0].mass = 0.0f;
		for (std::size_t index = 1; index < rig.bones.size(); ++index) {
			RigBone& bone = rig.bones[index];
			bone.name = "Hair0" + std::to_string(index);
			bone.mass = 0.01f * static_cast<float>(index);
			bone.inertia = { 1.0f, 2.0f, 3.0f };
			bone.angularDamping = 0.1f * static_cast<float>(index);
			bone.gravityFactor = 1.0f;
		}

		const int parents[5] = { 0, 1, 1, 0, 4 };
		for (int index = 0; index < 5; ++index) {
			RigJoint& joint = rig.joints.emplace_back();
			joint.child = index + 1;
			joint.parent = parents[index];
			joint.frameKind = FrameKind::FrameInB;
			joint.frame.rotation = FromAxisAngle(Vec3{ 1.0f, 2.0f, 3.0f }, 0.2f * static_cast<float>(index));
			joint.frame.translation = { 0.5f * static_cast<float>(index), -1.0f, 2.0f };
			joint.linearLower = {};
			joint.linearUpper = {};
			joint.angularLower = { -1.5f, -1.5f, -1.5f };
			joint.angularUpper = { 1.5f, 1.5f, 1.5f };
			joint.angularStiffness = { 3.0f, 4.0f, 5.0f };
			joint.angularDamping = { 0.5f, 0.6f, 0.7f };
			joint.angularEquilibrium = { 0.1f, 0.2f, 0.3f };
			joint.angularSpringsEnabled = true;
		}
		return rig;
	}

	std::vector<Transform> MakeRestWorld(const SmpRig& a_rig)
	{
		std::vector<Transform> rest(a_rig.bones.size());
		for (std::size_t index = 0; index < rest.size(); ++index) {
			rest[index].rotation = FromAxisAngle(Vec3{ 0.0f, 0.0f, 1.0f }, 0.1f * static_cast<float>(index));
			rest[index].translation = { 4.0f * static_cast<float>(index), 10.0f - static_cast<float>(index), 100.0f };
		}
		return rest;
	}

	Quat UnpackQuat(const GpuRig::Float4& a_value)
	{
		return { a_value.x, a_value.y, a_value.z, a_value.w };
	}

	Vec3 UnpackVec3(const GpuRig::Float4& a_value)
	{
		return { a_value.x, a_value.y, a_value.z };
	}

	void RequireVecNear(const Vec3& a_actual, const Vec3& a_expected, float a_margin)
	{
		REQUIRE(a_actual.x == Approx(a_expected.x).margin(a_margin));
		REQUIRE(a_actual.y == Approx(a_expected.y).margin(a_margin));
		REQUIRE(a_actual.z == Approx(a_expected.z).margin(a_margin));
	}

	void RequireQuatNear(const Quat& a_actual, const Quat& a_expected, float a_margin)
	{
		REQUIRE(a_actual.w == Approx(a_expected.w).margin(a_margin));
		REQUIRE(a_actual.x == Approx(a_expected.x).margin(a_margin));
		REQUIRE(a_actual.y == Approx(a_expected.y).margin(a_margin));
		REQUIRE(a_actual.z == Approx(a_expected.z).margin(a_margin));
	}
}

TEST_CASE("The packed joint frames satisfy the constraint frame coincidence", "[SmpPhysics]")
{
	const SmpRig rig = MakeBranchingRig();
	const std::vector<Transform> rest = MakeRestWorld(rig);

	const GpuRig::RigData data = GpuRig::BuildRigData(rig, rest);
	REQUIRE(data.bones.size() == rig.bones.size());
	REQUIRE(data.bones.size() == 6);

	for (const RigJoint& joint : rig.joints) {
		const GpuRig::BoneStatic& packed = data.bones[static_cast<std::size_t>(joint.child)];
		const Transform frameA{ UnpackQuat(packed.rotationA), UnpackVec3(packed.originA) };
		const Transform frameB{ UnpackQuat(packed.rotationB), UnpackVec3(packed.originB) };

		// The two frames describe one pose: childRest * frameA and parentRest * frameB must agree.
		const Transform throughChild = rest[static_cast<std::size_t>(joint.child)] * frameA;
		const Transform throughParent = rest[static_cast<std::size_t>(joint.parent)] * frameB;
		RequireQuatNear(throughChild.rotation, throughParent.rotation, 1e-5f);
		RequireVecNear(throughChild.translation, throughParent.translation, 1e-4f);

		REQUIRE(packed.topology.x == static_cast<std::uint32_t>(joint.parent));
		REQUIRE(packed.topology.y == (rig.bones[static_cast<std::size_t>(joint.parent)].kinematic ? 1u : 0u));
		REQUIRE(packed.body.x == Approx(rig.bones[static_cast<std::size_t>(joint.child)].mass));
		REQUIRE(packed.body.y == Approx(2.0f));
		REQUIRE(packed.body.z == Approx(rig.bones[static_cast<std::size_t>(joint.child)].angularDamping));
		REQUIRE(packed.body.w == Approx(1.0f));
		RequireVecNear({ packed.stiffness.x, packed.stiffness.y, packed.stiffness.z }, joint.angularStiffness, 1e-6f);
		REQUIRE(packed.stiffness.w == Approx(1.0f));
		RequireVecNear({ packed.damping.x, packed.damping.y, packed.damping.z }, joint.angularDamping, 1e-6f);
		RequireVecNear({ packed.equilibrium.x, packed.equilibrium.y, packed.equilibrium.z }, joint.angularEquilibrium, 1e-6f);
		RequireVecNear({ packed.lower.x, packed.lower.y, packed.lower.z }, joint.angularLower, 1e-6f);
		RequireVecNear({ packed.upper.x, packed.upper.y, packed.upper.z }, joint.angularUpper, 1e-6f);
	}
}

TEST_CASE("The packed tree table covers every dynamic bone once, parent first", "[SmpPhysics]")
{
	const SmpRig rig = MakeBranchingRig();
	const std::vector<Transform> rest = MakeRestWorld(rig);
	const GpuRig::RigData data = GpuRig::BuildRigData(rig, rest);

	REQUIRE(data.order.size() == 5);
	REQUIRE(data.trees.size() == 2);

	std::vector<int> seen(6, 0);
	for (const std::uint32_t bone : data.order) {
		REQUIRE(bone >= 1);
		REQUIRE(bone < 6);
		++seen[bone];
	}
	for (std::size_t bone = 1; bone < seen.size(); ++bone)
		REQUIRE(seen[bone] == 1);
	REQUIRE(seen[0] == 0);

	std::uint32_t nextOffset = 0;
	for (const GpuRig::TreeStatic& tree : data.trees) {
		REQUIRE(tree.range.x == nextOffset);
		REQUIRE(tree.range.y > 0);
		REQUIRE(tree.range.z == 0);  // both trees hang off the kinematic Head
		nextOffset += tree.range.y;
	}
	REQUIRE(nextOffset == data.order.size());

	// A parent must be visited before its child, whichever tree either belongs to.
	std::vector<int> visitOrder(6, -1);
	for (std::uint32_t slot = 0; slot < data.order.size(); ++slot)
		visitOrder[data.order[slot]] = static_cast<int>(slot);
	for (const RigJoint& joint : rig.joints) {
		REQUIRE(visitOrder[static_cast<std::size_t>(joint.parent)] < visitOrder[static_cast<std::size_t>(joint.child)]);
	}

	// The branch means Hair02 and Hair03 share the first tree with their parent.
	REQUIRE(data.order[0] == 1);
	REQUIRE(data.order[1] == 2);
	REQUIRE(data.order[2] == 3);
	REQUIRE(data.order[3] == 4);
	REQUIRE(data.order[4] == 5);
}

TEST_CASE("Every dynamic bone is mapped to its tree's kinematic parent", "[SmpPhysics]")
{
	const SmpRig rig = MakeBranchingRig();
	const std::vector<int> parents = GpuRig::BuildTreeKinematicParents(rig);

	REQUIRE(parents.size() == 6);
	REQUIRE(parents[0] == 0);
	for (std::size_t bone = 1; bone < parents.size(); ++bone)
		REQUIRE(parents[bone] == 0);
}

TEST_CASE("A bone's reach is the rest length of its chain to the kinematic root, with slack", "[SmpPhysics]")
{
	const SmpRig rig = MakeBranchingRig();
	std::vector<Transform> rest(rig.bones.size());
	rest[1].translation = { 0.0f, 0.0f, -10.0f };
	rest[2].translation = { 0.0f, 0.0f, -20.0f };
	rest[3].translation = { 5.0f, 0.0f, -20.0f };
	rest[4].translation = { -5.0f, 0.0f, -8.0f };
	rest[5].translation = { -5.0f, 0.0f, -18.0f };

	const std::vector<float> reach = GpuRig::BuildBoneReach(rig, rest, 2.0f, 5.0f);

	REQUIRE(reach.size() == 6);
	REQUIRE(reach[0] == 0.0f);
	REQUIRE(reach[1] == Approx(10.0f * 2.0f + 5.0f));
	REQUIRE(reach[2] == Approx(20.0f * 2.0f + 5.0f));
	REQUIRE(reach[3] == Approx((10.0f + std::sqrt(125.0f)) * 2.0f + 5.0f));
	REQUIRE(reach[4] == Approx(std::sqrt(89.0f) * 2.0f + 5.0f));
	REQUIRE(reach[5] == Approx((std::sqrt(89.0f) + 10.0f) * 2.0f + 5.0f));
}

TEST_CASE("A limited linear axis widens its bone's reach and a free one removes the limit", "[SmpPhysics]")
{
	SmpRig rig = MakeBranchingRig();
	std::vector<Transform> rest(rig.bones.size());
	rest[1].translation = { 0.0f, 0.0f, -10.0f };
	rest[2].translation = { 0.0f, 0.0f, -20.0f };
	rest[3].translation = { 5.0f, 0.0f, -20.0f };
	rest[4].translation = { -5.0f, 0.0f, -8.0f };
	rest[5].translation = { -5.0f, 0.0f, -18.0f };

	rig.joints[1].linearLower = { -1.0f, -1.0f, -1.0f };
	rig.joints[1].linearUpper = { 1.0f, 1.0f, 1.0f };
	const std::vector<float> limited = GpuRig::BuildBoneReach(rig, rest, 2.0f, 5.0f, 1.5f);
	const float window = std::sqrt(12.0f) * 1.5f;
	REQUIRE(limited[1] == Approx(10.0f * 2.0f + 5.0f));
	REQUIRE(limited[2] == Approx((20.0f + window) * 2.0f + 5.0f));
	REQUIRE(limited[3] == Approx((10.0f + std::sqrt(125.0f)) * 2.0f + 5.0f));

	rig.joints[1].linearLower = { 1.0f, 0.0f, 0.0f };
	rig.joints[1].linearUpper = { -1.0f, 0.0f, 0.0f };
	const std::vector<float> free = GpuRig::BuildBoneReach(rig, rest, 2.0f, 5.0f);
	REQUIRE(std::isinf(free[2]));
	REQUIRE(free[1] == Approx(25.0f));
}

TEST_CASE("A solver pose is rejected only when a dynamic bone is not finite", "[SmpPhysics]")
{
	const SmpRig rig = MakeBranchingRig();
	std::vector<Transform> rest(rig.bones.size());
	rest[1].translation = { 0.0f, 0.0f, -10.0f };
	rest[2].translation = { 0.0f, 0.0f, -20.0f };
	rest[3].translation = { 5.0f, 0.0f, -20.0f };
	rest[4].translation = { -5.0f, 0.0f, -8.0f };
	rest[5].translation = { -5.0f, 0.0f, -18.0f };

	std::vector<Transform> pose = rest;
	REQUIRE(GpuRig::PoseIsFinite(rig, pose));

	pose[2].translation = { 0.0f, 0.0f, -500.0f };
	REQUIRE(GpuRig::PoseIsFinite(rig, pose));

	pose = rest;
	pose[3].translation.x = std::numeric_limits<float>::quiet_NaN();
	REQUIRE_FALSE(GpuRig::PoseIsFinite(rig, pose));

	pose = rest;
	pose[0].translation = { 1.0e6f, 0.0f, 0.0f };
	REQUIRE(GpuRig::PoseIsFinite(rig, pose));

	REQUIRE(GpuRig::PoseIsFinite(rig, std::vector<Transform>(2)));
}

TEST_CASE("The per-frame tree input carries the surface range, the rigid flag and the contact skin", "[SmpPhysics]")
{
	const GpuRig::TreeInput input = GpuRig::MakeTreeInput(7u, 3u, true, 24u);
	REQUIRE(input.surface.x == 7u);
	REQUIRE(input.surface.y == 3u);
	REQUIRE(input.surface.z == 1u);
	REQUIRE(input.surface.w == 24u);
	REQUIRE(input.contact.z == Approx(kContactSkin));

	REQUIRE(GpuRig::MakeTreeInput(0u, 0u, false, 1u).surface.z == 0u);
}

TEST_CASE("Shifting a packed rig to a slot base moves every stored bone index", "[SmpPhysics]")
{
	const SmpRig rig = MakeBranchingRig();
	const std::vector<Transform> rest = MakeRestWorld(rig);
	const GpuRig::RigData local = GpuRig::BuildRigData(rig, rest);
	GpuRig::RigData shifted = local;
	constexpr std::uint32_t kBase = 40;
	GpuRig::ShiftToSlotBase(shifted, kBase);

	REQUIRE(shifted.order.size() == local.order.size());
	for (std::size_t slot = 0; slot < local.order.size(); ++slot)
		REQUIRE(shifted.order[slot] == local.order[slot] + kBase);
	for (std::size_t bone = 0; bone < local.bones.size(); ++bone)
		REQUIRE(shifted.bones[bone].topology.x == local.bones[bone].topology.x + kBase);
	for (std::size_t tree = 0; tree < local.trees.size(); ++tree) {
		REQUIRE(shifted.trees[tree].range.x == local.trees[tree].range.x + kBase);
		REQUIRE(shifted.trees[tree].range.y == local.trees[tree].range.y);
		REQUIRE(shifted.trees[tree].range.z == local.trees[tree].range.z + kBase);
	}
}

namespace
{
	/** @brief A rig's collision as the feature builds it: spheres on two bones and a proxy per collider kind. */
	struct PackedCollision
	{
		std::vector<std::vector<BoneSphere>> spheres;
		std::vector<Collider> colliders;
	};

	PackedCollision MakeCollision(const SmpRig& a_rig)
	{
		PackedCollision collision;
		collision.spheres.resize(a_rig.bones.size());
		collision.spheres[1].push_back({ Vec3{ 1.0f, 2.0f, 3.0f }, 0.5f });
		collision.spheres[1].push_back({ Vec3{ -1.0f, 0.0f, 0.0f }, 0.75f });
		collision.spheres[4].push_back({ Vec3{ 0.0f, 0.0f, 9.0f }, 1.25f });
		collision.spheres[4].push_back({ Vec3{ 1.0f, 0.0f, 0.0f }, 0.1f });
		collision.spheres[4].push_back({ Vec3{ 2.0f, 0.0f, 0.0f }, 0.2f });
		collision.spheres[4].push_back({ Vec3{ 3.0f, 0.0f, 0.0f }, 0.3f });

		collision.colliders.push_back(MakeDefaultHeadCollider(0));
		collision.colliders.push_back(MakeDefaultCapsule(Vec3{}, Vec3{ 0.0f, 0.0f, -10.0f }, 3.5f, 1));
		collision.colliders.push_back(MakeGroundPlane(Vec3{ 0.0f, 0.0f, 90.0f }, Vec3{ 0.0f, 0.0f, 1.0f }, -1));
		return collision;
	}
}

TEST_CASE("A claim's collision packs its spheres and colliders for the shader", "[SmpPhysics]")
{
	const SmpRig rig = MakeBranchingRig();
	const std::vector<Transform> rest = MakeRestWorld(rig);
	const PackedCollision collision = MakeCollision(rig);

	GpuRig::RigData data = GpuRig::BuildRigData(rig, rest);
	REQUIRE(data.collision.colliders.empty());
	for (const GpuRig::TreeStatic& tree : data.trees)
		REQUIRE(tree.contact.y == 0u);  // a rig with no proxies leaves the contact path off

	GpuRig::SetCollision(data, GpuRig::PackCollision(collision.spheres, collision.colliders, 2));
	REQUIRE(data.collision.spheres.size() == rig.bones.size());
	REQUIRE(data.collision.colliders.size() == 3);
	REQUIRE(data.collision.driverCount == 2u);
	for (const GpuRig::TreeStatic& tree : data.trees) {
		REQUIRE(tree.contact.x == 0u);
		REQUIRE(tree.contact.y == 3u);
		REQUIRE(tree.contact.z == 0u);
		REQUIRE(tree.contact.w == 2u);
	}

	// A sphere keeps its bone's rest-frame centre and its radius, and a bone's spheres are a slice of one list.
	const auto sphereAt = [&](std::size_t a_bone, std::uint32_t a_index) -> const GpuRig::SphereRecord& {
		return data.collision.sphereList[data.collision.spheres[a_bone].range.x + a_index];
	};
	REQUIRE(data.collision.spheres[0].range.y == 0u);
	REQUIRE(data.collision.spheres[1].range.y == 2u);
	REQUIRE(sphereAt(1, 0).centreRadius.x == Approx(1.0f));
	REQUIRE(sphereAt(1, 0).centreRadius.y == Approx(2.0f));
	REQUIRE(sphereAt(1, 0).centreRadius.z == Approx(3.0f));
	REQUIRE(sphereAt(1, 0).centreRadius.w == Approx(0.5f));
	REQUIRE(sphereAt(1, 1).centreRadius.x == Approx(-1.0f));
	REQUIRE(sphereAt(1, 1).centreRadius.w == Approx(0.75f));
	REQUIRE(data.collision.spheres[4].range.y == 4u);
	REQUIRE(sphereAt(4, 0).centreRadius.z == Approx(9.0f));
	REQUIRE(sphereAt(4, 2).centreRadius.x == Approx(2.0f));
	REQUIRE(data.collision.sphereList.size() == 6u);
	// The bound encloses the bone's spheres: the mean centre, reaching the farthest sphere's far side.
	REQUIRE(data.collision.spheres[1].bound.x == Approx(0.0f));
	REQUIRE(data.collision.spheres[1].bound.y == Approx(1.0f));
	REQUIRE(data.collision.spheres[1].bound.z == Approx(1.5f));
	REQUIRE(data.collision.spheres[1].bound.w == Approx(std::sqrt(1.0f + 1.0f + 2.25f) + 0.75f));

	// Each collider keeps its kind, its parameters and the driver it follows.
	REQUIRE(data.collision.colliders[0].kind.x == static_cast<std::uint32_t>(GpuRig::ColliderKind::Ellipsoid));
	REQUIRE(data.collision.colliders[0].kind.y == 0u);
	REQUIRE(data.collision.colliders[0].halfExtents.x == Approx(kDefaultHeadRadius));
	REQUIRE(data.collision.colliders[0].normal.w == Approx(kDefaultPenetration));
	REQUIRE(data.collision.colliders[1].kind.x == static_cast<std::uint32_t>(GpuRig::ColliderKind::Capsule));
	REQUIRE(data.collision.colliders[1].kind.y == 1u);
	REQUIRE(data.collision.colliders[1].halfExtents.w == Approx(3.5f));
	REQUIRE(data.collision.colliders[1].segmentB.z == Approx(-10.0f));
	REQUIRE(data.collision.colliders[2].kind.x == static_cast<std::uint32_t>(GpuRig::ColliderKind::Plane));
	REQUIRE(data.collision.colliders[2].normal.z == Approx(1.0f));
	REQUIRE(data.collision.colliders[2].center.z == Approx(90.0f));
	// A proxy that follows no node names no driver, so the shader skips it as QueryColliders does.
	REQUIRE(data.collision.colliders[2].kind.y == GpuRig::kInvalidDriverSlot);

	REQUIRE(GpuRig::SlotSpan(data) == 6u + 3u + 2u);

	constexpr std::uint32_t kBase = 40u;
	constexpr std::uint32_t kColliderBase = kBase + 6u;
	constexpr std::uint32_t kDriverBase = kColliderBase + 3u;
	GpuRig::ShiftToSlotBase(data, kBase);
	REQUIRE(data.collision.colliders[0].kind.y == kDriverBase);
	REQUIRE(data.collision.colliders[1].kind.y == 1u + kDriverBase);
	REQUIRE(data.collision.colliders[2].kind.y == GpuRig::kInvalidDriverSlot);
	for (const GpuRig::TreeStatic& tree : data.trees) {
		REQUIRE(tree.contact.x == kColliderBase);
		REQUIRE(tree.contact.y == 3u);
		REQUIRE(tree.contact.z == kDriverBase);
		REQUIRE(tree.contact.w == 2u);
	}
	// The sphere list is indexed by bone, so it moves with the bone base rather than an index of its own.
	REQUIRE(data.collision.spheres.size() == rig.bones.size());
	REQUIRE(data.collision.sphereList[data.collision.spheres[4].range.x].centreRadius.z == Approx(9.0f));
}

TEST_CASE("A sphere's rest-exemption mask rides in the record's spare lanes", "[SmpPhysics]")
{
	const SmpRig rig = MakeBranchingRig();
	PackedCollision collision = MakeCollision(rig);
	collision.spheres[1][0].exemptMask = 0x1u;
	collision.spheres[1][1].exemptMask = 0x5u;
	collision.spheres[4][0].exemptMask = 0x1u;
	collision.spheres[4][1].exemptMask = 0x2u;
	collision.spheres[4][2].exemptMask = 0x80000000u;
	collision.spheres[4][3].exemptMask = 0x4u;

	GpuRig::RigData data = GpuRig::BuildRigData(rig, MakeRestWorld(rig));
	GpuRig::SetCollision(data, GpuRig::PackCollision(collision.spheres, collision.colliders, 2));
	const auto maskOf = [&](std::size_t a_bone, std::uint32_t a_index) {
		return data.collision.sphereList[data.collision.spheres[a_bone].range.x + a_index].exempt.x;
	};
	REQUIRE(maskOf(1, 0) == 0x1u);
	REQUIRE(maskOf(1, 1) == 0x5u);
	REQUIRE(maskOf(4, 0) == 0x1u);
	REQUIRE(maskOf(4, 1) == 0x2u);
	REQUIRE(maskOf(4, 2) == 0x80000000u);
	REQUIRE(maskOf(4, 3) == 0x4u);

	// A mask names a collider by its claim-local index, so laying the claim out must leave it where it was.
	GpuRig::ShiftToSlotBase(data, 40u);
	REQUIRE(maskOf(1, 1) == 0x5u);
	REQUIRE(maskOf(4, 2) == 0x80000000u);
}

TEST_CASE("A second claim's slots start past the first claim's", "[SmpPhysics]")
{
	const SmpRig rig = MakeBranchingRig();
	const std::vector<Transform> rest = MakeRestWorld(rig);
	const PackedCollision collision = MakeCollision(rig);

	GpuRig::RigData first = GpuRig::BuildRigData(rig, rest);
	GpuRig::SetCollision(first, GpuRig::PackCollision(collision.spheres, collision.colliders, 2));
	const std::uint32_t span = GpuRig::SlotSpan(first);
	GpuRig::ShiftToSlotBase(first, 0u);

	GpuRig::RigData second = GpuRig::BuildRigData(rig, rest);
	GpuRig::SetCollision(second, GpuRig::PackCollision(collision.spheres, collision.colliders, 2));
	GpuRig::ShiftToSlotBase(second, span);

	REQUIRE(second.bones[0].topology.x == span);
	REQUIRE(second.trees[0].contact.x == span + 6u);
	REQUIRE(second.trees[0].contact.z == span + 6u + 3u);
	REQUIRE(second.collision.colliders[0].kind.y == span + 6u + 3u);
	// Laying the second claim out leaves the first one's own ranges where they were.
	REQUIRE(first.trees[0].contact.x == 6u);
	REQUIRE(first.collision.colliders[0].kind.y == 9u);
}

TEST_CASE("A frameInLerp joint packs the same frames the solver resolves", "[SmpPhysics]")
{
	SmpRig rig = MakeBranchingRig();
	rig.joints[0].frameKind = FrameKind::FrameInLerp;
	rig.joints[0].translationLerp = 0.3f;
	rig.joints[0].rotationLerp = 0.6f;
	const std::vector<Transform> rest = MakeRestWorld(rig);
	const GpuRig::RigData data = GpuRig::BuildRigData(rig, rest);

	const RigJoint& joint = rig.joints[0];
	const JointFrames expected = ResolveJointFrames(joint, rest[static_cast<std::size_t>(joint.child)], rest[static_cast<std::size_t>(joint.parent)]);
	const GpuRig::BoneStatic& packed = data.bones[static_cast<std::size_t>(joint.child)];
	REQUIRE(packed.rotationA.x == Approx(expected.frameA.rotation.w).margin(1e-6f));
	REQUIRE(packed.rotationB.x == Approx(expected.frameB.rotation.w).margin(1e-6f));
	REQUIRE(packed.originA.x == Approx(expected.frameA.translation.x).margin(1e-5f));
	REQUIRE(packed.originB.y == Approx(expected.frameB.translation.y).margin(1e-5f));

	rig.joints[0].frameKind = FrameKind::FrameInB;
	const GpuRig::RigData plain = GpuRig::BuildRigData(rig, rest);
	const bool differs = std::abs(plain.bones[static_cast<std::size_t>(joint.child)].originA.x - packed.originA.x) > 1e-6f ||
	                     std::abs(plain.bones[static_cast<std::size_t>(joint.child)].rotationA.x - packed.rotationA.x) > 1e-6f;
	REQUIRE(differs);
}

namespace
{
	void RequireSameNode(const BvhNode& a_actual, const BvhNode& a_expected)
	{
		REQUIRE(a_actual.skip == a_expected.skip);
		REQUIRE(a_actual.leaf == a_expected.leaf);
		for (int axis = 0; axis < 3; ++axis) {
			REQUIRE(a_actual.low[axis] == a_expected.low[axis]);
			REQUIRE(a_actual.high[axis] == a_expected.high[axis]);
		}
	}

	/** @brief A wavy square mesh subdivided a_side times per axis, skinned wholly to driver 0. */
	SkinnedSurface MakeGridSurface(int a_side)
	{
		SkinnedSurface surface;
		surface.skinToBone = { Transform{} };
		surface.boneDriver = { 0 };
		for (int row = 0; row <= a_side; ++row) {
			for (int column = 0; column <= a_side; ++column) {
				SkinnedVertex vertex;
				vertex.position = { 2.0f * static_cast<float>(column), 2.0f * static_cast<float>(row),
					1.5f * std::sin(0.9f * static_cast<float>(row)) * std::cos(0.7f * static_cast<float>(column)) };
				vertex.bone[0] = 0;
				vertex.weight[0] = 1.0f;
				surface.vertices.push_back(vertex);
			}
		}
		for (int row = 0; row < a_side; ++row) {
			for (int column = 0; column < a_side; ++column) {
				const std::uint32_t corner = static_cast<std::uint32_t>(row * (a_side + 1) + column);
				const std::uint32_t stride = static_cast<std::uint32_t>(a_side) + 1u;
				surface.indices.insert(surface.indices.end(),
					{ corner, corner + 1u, corner + stride, corner + 1u, corner + stride + 1u, corner + stride });
			}
		}
		return surface;
	}
}

TEST_CASE("Surface triangles are packed in leaf order and carry their source index", "[smpphysics][gpurig]")
{
	const SkinnedSurface mesh = MakeGridSurface(4);
	const std::vector<Transform> drivers = { Transform{} };
	const std::vector<Vec3> posed = PoseSurface(mesh, drivers);
	TriangleBvh expected;
	REQUIRE(expected.Build(posed, mesh.indices));
	REQUIRE(expected.TriangleCount() == mesh.indices.size() / 3);

	GpuRig::SurfaceFrame frame;
	TriangleBvh bvh;
	GpuRig::AppendSurface(frame, mesh, drivers, bvh);

	REQUIRE(frame.metas.size() == 1);
	REQUIRE(frame.metas[0].dimensions.w == expected.TriangleCount());
	REQUIRE(frame.triangles.size() == expected.LeafOrder().size());
	REQUIRE(frame.nodes.size() == expected.Nodes().size());
	for (std::size_t slot = 0; slot < expected.LeafOrder().size(); ++slot) {
		const std::uint32_t source = expected.LeafOrder()[slot];
		const GpuRig::SurfaceTriangle& record = frame.triangles[slot];
		REQUIRE(std::bit_cast<std::uint32_t>(record.a.w) == source);

		const Vec3 a = posed[mesh.indices[static_cast<std::size_t>(source) * 3]];
		const Vec3 b = posed[mesh.indices[static_cast<std::size_t>(source) * 3 + 1]];
		const Vec3 c = posed[mesh.indices[static_cast<std::size_t>(source) * 3 + 2]];
		const Vec3 cross = Cross(b - a, c - a);
		const float twiceArea = Length(cross);
		REQUIRE(record.a.x == Approx(a.x));
		REQUIRE(record.a.y == Approx(a.y));
		REQUIRE(record.a.z == Approx(a.z));
		REQUIRE(record.b.x == Approx(b.x));
		REQUIRE(record.b.y == Approx(b.y));
		REQUIRE(record.b.z == Approx(b.z));
		REQUIRE(record.c.x == Approx(c.x));
		REQUIRE(record.c.y == Approx(c.y));
		REQUIRE(record.c.z == Approx(c.z));
		REQUIRE(record.normalArea.x == Approx(cross.x / twiceArea));
		REQUIRE(record.normalArea.y == Approx(cross.y / twiceArea));
		REQUIRE(record.normalArea.z == Approx(cross.z / twiceArea));
		REQUIRE(record.normalArea.w == Approx(twiceArea));
	}
}

TEST_CASE("A second surface's nodes start past the first surface's and the merge leaves them untouched", "[smpphysics][gpurig]")
{
	SkinnedSurface quad;
	quad.margin = 0.5f;
	quad.contactPenetration = 0.4f;
	quad.skinToBone = { Transform{} };
	quad.boneDriver = { 0 };
	for (const Vec3& corner : { Vec3{ -5.0f, -5.0f, 0.0f }, Vec3{ 5.0f, -5.0f, 0.0f }, Vec3{ 5.0f, 5.0f, 0.0f }, Vec3{ -5.0f, 5.0f, 0.0f } }) {
		SkinnedVertex vertex;
		vertex.position = corner;
		vertex.bone[0] = 0;
		vertex.weight[0] = 1.0f;
		quad.vertices.push_back(vertex);
	}
	quad.indices = { 0, 1, 2, 0, 2, 3 };
	const SkinnedSurface other = MakeGridSurface(2);

	const std::vector<Transform> drivers = { Transform{} };
	TriangleBvh quadBvh;
	TriangleBvh otherBvh;
	GpuRig::SurfaceFrame frame;
	GpuRig::AppendSurface(frame, quad, drivers, quadBvh);
	GpuRig::AppendSurface(frame, other, drivers, otherBvh);

	REQUIRE(frame.metas.size() == 2);
	REQUIRE(frame.metas[0].base.y == 0u);
	REQUIRE(frame.metas[0].base.z == quadBvh.Nodes().size());
	REQUIRE(frame.metas[1].base.y == quadBvh.Nodes().size());
	REQUIRE(frame.metas[1].base.z == otherBvh.Nodes().size());
	REQUIRE(frame.nodes.size() == quadBvh.Nodes().size() + otherBvh.Nodes().size());
	for (std::size_t index = 0; index < quadBvh.Nodes().size(); ++index)
		RequireSameNode(frame.nodes[index], quadBvh.Nodes()[index]);
	for (std::size_t index = 0; index < otherBvh.Nodes().size(); ++index)
		RequireSameNode(frame.nodes[quadBvh.Nodes().size() + index], otherBvh.Nodes()[index]);

	GpuRig::SurfaceFrame quadPart;
	GpuRig::SurfaceFrame otherPart;
	TriangleBvh quadPartBvh;
	TriangleBvh otherPartBvh;
	GpuRig::AppendSurface(quadPart, quad, drivers, quadPartBvh);
	GpuRig::AppendSurface(otherPart, other, drivers, otherPartBvh);

	GpuRig::SurfaceFrame merged;
	std::uint32_t firstMeta = 99u;
	std::uint32_t secondMeta = 99u;
	GpuRig::MergeSurfaceFrame(merged, quadPart, firstMeta);
	GpuRig::MergeSurfaceFrame(merged, otherPart, secondMeta);
	REQUIRE(firstMeta == 0u);
	REQUIRE(secondMeta == 1u);
	REQUIRE(merged.metas[1].base.x == quadPart.triangles.size());
	REQUIRE(merged.metas[1].base.y == quadPart.nodes.size());
	REQUIRE(merged.metas[1].base.z == otherPart.nodes.size());
	REQUIRE(merged.nodes.size() == frame.nodes.size());
	for (std::size_t index = 0; index < frame.nodes.size(); ++index)
		RequireSameNode(merged.nodes[index], frame.nodes[index]);
	REQUIRE(merged.triangles.size() == frame.triangles.size());
	for (std::size_t slot = 0; slot < frame.triangles.size(); ++slot)
		REQUIRE(std::bit_cast<std::uint32_t>(merged.triangles[slot].a.w) == std::bit_cast<std::uint32_t>(frame.triangles[slot].a.w));
}

TEST_CASE("A degenerate triangle keeps its slot and packs as inactive", "[smpphysics][gpurig]")
{
	SkinnedSurface mesh;
	mesh.skinToBone = { Transform{} };
	mesh.boneDriver = { 0 };
	for (const Vec3& corner : { Vec3{ -1.0f, 0.0f, 0.0f }, Vec3{ 1.0f, 0.0f, 0.0f }, Vec3{ 1.0f, 0.0f, 1.0f }, Vec3{ -1.0f, 0.0f, 1.0f } }) {
		SkinnedVertex vertex;
		vertex.position = corner;
		vertex.bone[0] = 0;
		vertex.weight[0] = 1.0f;
		mesh.vertices.push_back(vertex);
	}
	mesh.indices = { 0, 1, 2, 0, 2, 3, 1, 1, 1 };

	const std::vector<Transform> drivers = { Transform{} };
	TriangleBvh bvh;
	GpuRig::SurfaceFrame frame;
	GpuRig::AppendSurface(frame, mesh, drivers, bvh);

	REQUIRE(frame.metas.size() == 1);
	REQUIRE(frame.metas[0].dimensions.w == 3u);
	REQUIRE(frame.triangles.size() == 3u);
	std::uint32_t inactive = 0;
	for (const GpuRig::SurfaceTriangle& record : frame.triangles) {
		if (std::bit_cast<std::uint32_t>(record.a.w) != 2u) {
			REQUIRE(record.normalArea.w > 0.0f);
			continue;
		}
		++inactive;
		REQUIRE(record.normalArea.w == 0.0f);
		for (const GpuRig::Float4* corner : { &record.a, &record.b, &record.c }) {
			REQUIRE(corner->x == 0.0f);
			REQUIRE(corner->y == 0.0f);
			REQUIRE(corner->z == 0.0f);
		}
		REQUIRE(record.normalArea.x == 0.0f);
		REQUIRE(record.normalArea.y == 0.0f);
		REQUIRE(record.normalArea.z == 0.0f);
	}
	REQUIRE(inactive == 1u);

	TriangleGrid grid;
	grid.Build(PoseSurface(mesh, drivers), mesh.indices, kSurfaceCellSize);
	REQUIRE(grid.TriangleCount() == 2u);
}

TEST_CASE("Refitting a surface's hierarchy after a move packs the same triangles as a fresh build", "[smpphysics][gpurig]")
{
	const SkinnedSurface mesh = MakeGridSurface(4);
	const std::vector<Transform> before = { Transform{} };
	TriangleBvh reused;
	GpuRig::SurfaceFrame firstFrame;
	GpuRig::AppendSurface(firstFrame, mesh, before, reused);
	REQUIRE(reused.Ready());

	std::vector<Transform> after = { Transform{} };
	after[0].translation = { 3.0f, -4.0f, 2.5f };
	GpuRig::SurfaceFrame refitted;
	GpuRig::AppendSurface(refitted, mesh, after, reused);

	TriangleBvh fresh;
	GpuRig::SurfaceFrame rebuilt;
	GpuRig::AppendSurface(rebuilt, mesh, after, fresh);

	REQUIRE(refitted.triangles.size() == rebuilt.triangles.size());
	REQUIRE(refitted.triangles[0].a.x == Approx(firstFrame.triangles[0].a.x + 3.0f));
	const auto requireNear = [](const GpuRig::Float4& a_actual, const GpuRig::Float4& a_expected) {
		REQUIRE(a_actual.x == Approx(a_expected.x).margin(1e-4f));
		REQUIRE(a_actual.y == Approx(a_expected.y).margin(1e-4f));
		REQUIRE(a_actual.z == Approx(a_expected.z).margin(1e-4f));
		REQUIRE(a_actual.w == Approx(a_expected.w).margin(1e-4f));
	};
	std::unordered_map<std::uint32_t, const GpuRig::SurfaceTriangle*> bySource;
	for (const GpuRig::SurfaceTriangle& triangle : rebuilt.triangles)
		bySource[std::bit_cast<std::uint32_t>(triangle.a.w)] = &triangle;
	for (const GpuRig::SurfaceTriangle& actual : refitted.triangles) {
		const auto found = bySource.find(std::bit_cast<std::uint32_t>(actual.a.w));
		REQUIRE(found != bySource.end());
		const GpuRig::SurfaceTriangle& expected = *found->second;
		requireNear(actual.a, expected.a);
		requireNear(actual.b, expected.b);
		requireNear(actual.c, expected.c);
		requireNear(actual.normalArea, expected.normalArea);
	}
}

TEST_CASE("Packed bones carry the wind-facing segment and the card width", "[SmpPhysics]")
{
	const SmpRig rig = MakeBranchingRig();
	const std::vector<Transform> rest = MakeRestWorld(rig);
	std::vector<float> widths(rig.bones.size(), 0.0f);
	widths[1] = 9.0f;

	const std::vector<BoneWindGeometry> expected = ComputeBoneWindGeometry(rig, rest, widths);
	const GpuRig::RigData data = GpuRig::BuildRigData(rig, rest, widths);
	REQUIRE(data.bones.size() == rig.bones.size());
	for (std::size_t bone = 0; bone < rig.bones.size(); ++bone) {
		REQUIRE(data.bones[bone].wind.x == Approx(expected[bone].axis.x).margin(1e-6f));
		REQUIRE(data.bones[bone].wind.y == Approx(expected[bone].axis.y).margin(1e-6f));
		REQUIRE(data.bones[bone].wind.z == Approx(expected[bone].axis.z).margin(1e-6f));
		REQUIRE(data.bones[bone].wind.w == Approx(expected[bone].area).margin(1e-6f));
	}
}

namespace
{
	/** @brief A 12x12 vertex grid skinned to two bones by a column gradient, with its first vertex unweighted. */
	SkinnedSurface MakeTwoBoneGridSurface()
	{
		SkinnedSurface surface;
		surface.name = "two-bone grid";
		surface.margin = 0.85f;
		surface.contactPenetration = 0.35f;
		surface.skinToBone = {
			Transform{ FromAxisAngle(Vec3{ 0.0f, 0.0f, 1.0f }, 0.3f), Vec3{ 0.5f, -0.25f, 1.0f } },
			Transform{ FromAxisAngle(Vec3{ 1.0f, 0.0f, 0.0f }, -0.4f), Vec3{ -0.75f, 0.5f, 0.25f } }
		};
		surface.boneDriver = { 0, 1 };

		constexpr int kSide = 12;
		for (int row = 0; row < kSide; ++row) {
			for (int column = 0; column < kSide; ++column) {
				SkinnedVertex vertex;
				vertex.position = { 2.0f * static_cast<float>(column), 2.0f * static_cast<float>(row),
					1.5f * std::sin(0.9f * static_cast<float>(row)) * std::cos(0.7f * static_cast<float>(column)) };
				const float gradient = static_cast<float>(column) / static_cast<float>(kSide - 1);
				vertex.bone[0] = 0;
				vertex.weight[0] = 1.0f - gradient;
				vertex.bone[1] = 1;
				vertex.weight[1] = gradient;
				surface.vertices.push_back(vertex);
			}
		}

		SkinnedVertex& unweighted = surface.vertices[0];
		unweighted.bone[0] = -1;
		unweighted.bone[1] = -1;
		unweighted.weight[0] = 0.0f;
		unweighted.weight[1] = 0.0f;

		const std::uint32_t stride = static_cast<std::uint32_t>(kSide);
		for (int row = 0; row < kSide - 1; ++row) {
			for (int column = 0; column < kSide - 1; ++column) {
				const std::uint32_t corner = static_cast<std::uint32_t>(row * kSide + column);
				surface.indices.insert(surface.indices.end(),
					{ corner, corner + 1u, corner + stride, corner + 1u, corner + stride + 1u, corner + stride });
			}
		}
		return surface;
	}

	std::vector<Transform> FirstDriverPose()
	{
		return { Transform{ FromAxisAngle(Vec3{ 0.0f, 0.0f, 1.0f }, 0.25f), Vec3{ 1.0f, 2.0f, 3.0f } },
			Transform{ FromAxisAngle(Vec3{ 1.0f, 1.0f, 0.0f }, -0.55f), Vec3{ -2.0f, 0.5f, 4.0f } } };
	}

	std::vector<Transform> SecondDriverPose()
	{
		return { Transform{ FromAxisAngle(Vec3{ 0.0f, 0.0f, 1.0f }, 0.9f), Vec3{ 5.0f, -1.0f, 0.5f } },
			Transform{ FromAxisAngle(Vec3{ 0.0f, 1.0f, 1.0f }, 0.35f), Vec3{ 1.5f, 6.0f, -2.0f } } };
	}

	void RequireSameSurfaceTriangle(const GpuRig::SurfaceTriangle& a_actual, const GpuRig::SurfaceTriangle& a_expected)
	{
		REQUIRE(std::bit_cast<std::uint32_t>(a_actual.a.w) == std::bit_cast<std::uint32_t>(a_expected.a.w));
		const float actual[4][3] = {
			{ a_actual.a.x, a_actual.a.y, a_actual.a.z },
			{ a_actual.b.x, a_actual.b.y, a_actual.b.z },
			{ a_actual.c.x, a_actual.c.y, a_actual.c.z },
			{ a_actual.normalArea.x, a_actual.normalArea.y, a_actual.normalArea.z }
		};
		const float expected[4][3] = {
			{ a_expected.a.x, a_expected.a.y, a_expected.a.z },
			{ a_expected.b.x, a_expected.b.y, a_expected.b.z },
			{ a_expected.c.x, a_expected.c.y, a_expected.c.z },
			{ a_expected.normalArea.x, a_expected.normalArea.y, a_expected.normalArea.z }
		};
		for (int corner = 0; corner < 4; ++corner) {
			for (int axis = 0; axis < 3; ++axis)
				REQUIRE(actual[corner][axis] == Approx(expected[corner][axis]).margin(1e-5f));
		}
		REQUIRE(a_actual.normalArea.w == Approx(a_expected.normalArea.w).margin(1e-5f));
	}
}

TEST_CASE("The reference surface pass matches posing, refitting and packing on the CPU", "[smpphysics][gpurig]")
{
	const SkinnedSurface surface = MakeTwoBoneGridSurface();
	const std::vector<Transform> before = FirstDriverPose();
	const std::vector<Transform> after = SecondDriverPose();

	TriangleBvh bvh;
	REQUIRE(bvh.Build(PoseSurface(surface, before), surface.indices));

	GpuRig::SurfacePrepData data;
	REQUIRE(GpuRig::AppendSurfacePrep(data, surface, bvh, 0u, static_cast<std::uint32_t>(before.size())));
	GpuRig::FinalizeSurfacePrep(data);

	REQUIRE(data.metas.size() == 1);
	REQUIRE(data.metas[0].dimensions.w == surface.indices.size() / 3);
	REQUIRE(data.metas[0].base.x == 0u);
	REQUIRE(data.metas[0].base.y == 0u);
	REQUIRE(data.metas[0].base.z == bvh.Nodes().size());
	REQUIRE(data.metas[0].contact.x == Approx(surface.margin));
	REQUIRE(data.metas[0].contact.y == Approx(surface.contactPenetration));
	REQUIRE(data.metas[0].contact.z == Approx(kDeepPushReach));
	REQUIRE(data.vertices.size() == surface.vertices.size());
	REQUIRE(data.bones.size() == surface.skinToBone.size());
	REQUIRE(data.triangles.size() == surface.indices.size() / 3);
	REQUIRE(data.nodes.size() == bvh.Nodes().size());
	REQUIRE(data.leaves.size() + data.refit.size() == data.nodes.size());
	REQUIRE(data.refitLevels > 1u);

	TriangleBvh cpuBvh = bvh;
	GpuRig::SurfaceFrame frame;
	GpuRig::AppendSurface(frame, surface, after, cpuBvh);
	REQUIRE(frame.triangles.size() == data.triangles.size());

	std::vector<GpuRig::SurfaceTriangle> triangles;
	std::vector<BvhNode> nodes;
	GpuRig::ReferenceSurfacePrep(data, after, triangles, nodes);
	REQUIRE(triangles.size() == frame.triangles.size());
	REQUIRE(nodes.size() == frame.nodes.size());
	for (std::size_t slot = 0; slot < triangles.size(); ++slot)
		RequireSameSurfaceTriangle(triangles[slot], frame.triangles[slot]);
	for (std::size_t index = 0; index < nodes.size(); ++index)
		RequireSameNode(nodes[index], frame.nodes[index]);
}

TEST_CASE("Two surfaces share one prep buffer through their global offsets", "[smpphysics][gpurig]")
{
	const SkinnedSurface first = MakeGridSurface(4);
	SkinnedSurface second = MakeTwoBoneGridSurface();
	second.boneThreshold = { 0.25f };
	const std::vector<Transform> before = FirstDriverPose();
	const std::vector<Transform> after = SecondDriverPose();
	constexpr std::uint32_t kSecondDriverOffset = 3;

	TriangleBvh firstBvh;
	TriangleBvh secondBvh;
	REQUIRE(firstBvh.Build(PoseSurface(first, { before[0] }), first.indices));
	REQUIRE(secondBvh.Build(PoseSurface(second, before), second.indices));

	GpuRig::SurfacePrepData data;
	REQUIRE(GpuRig::AppendSurfacePrep(data, first, firstBvh, 0u, 1u));
	const std::uint32_t firstTriangles = static_cast<std::uint32_t>(first.indices.size() / 3);
	const std::uint32_t firstNodes = static_cast<std::uint32_t>(firstBvh.Nodes().size());
	REQUIRE(GpuRig::AppendSurfacePrep(data, second, secondBvh, kSecondDriverOffset, static_cast<std::uint32_t>(before.size())));
	GpuRig::FinalizeSurfacePrep(data);

	REQUIRE(data.metas.size() == 2);
	REQUIRE(data.metas[0].base.x == 0u);
	REQUIRE(data.metas[0].base.y == 0u);
	REQUIRE(data.metas[1].base.x == firstTriangles);
	REQUIRE(data.metas[1].base.y == firstNodes);
	REQUIRE(data.metas[1].base.z == secondBvh.Nodes().size());
	REQUIRE(data.metas[1].dimensions.w == second.indices.size() / 3);
	REQUIRE(data.vertices.size() == first.vertices.size() + second.vertices.size());
	REQUIRE(data.bones.size() == first.skinToBone.size() + second.skinToBone.size());

	const std::size_t boneBase = first.skinToBone.size();
	const std::uint32_t vertexBase = static_cast<std::uint32_t>(first.vertices.size());
	for (std::size_t bone = 0; bone < second.skinToBone.size(); ++bone) {
		REQUIRE(data.bones[boneBase + bone].driver.x ==
				kSecondDriverOffset + static_cast<std::uint32_t>(second.boneDriver[bone]));
		// A bone the surface lists no threshold for packs zero.
		REQUIRE(std::bit_cast<float>(data.bones[boneBase + bone].driver.y) == (bone == 0 ? 0.25f : 0.0f));
	}
	const auto expectedBone = [&](int a_bone) -> std::uint32_t {
		return a_bone >= 0 ? static_cast<std::uint32_t>(boneBase + static_cast<std::size_t>(a_bone)) :
		                     GpuRig::kInvalidDriverSlot;
	};
	for (std::size_t vertex = 0; vertex < second.vertices.size(); ++vertex) {
		const GpuRig::SurfacePrepVertex& packed = data.vertices[vertexBase + vertex];
		const std::uint32_t bones[4] = { packed.bones.x, packed.bones.y, packed.bones.z, packed.bones.w };
		for (int slot = 0; slot < 4; ++slot)
			REQUIRE(bones[slot] == expectedBone(second.vertices[vertex].bone[slot]));
	}
	for (std::uint32_t slot = 0; slot < data.metas[1].dimensions.w; ++slot) {
		const GpuRig::SurfacePrepTriangle& triangle = data.triangles[data.metas[1].base.x + slot];
		for (const std::uint32_t corner : { triangle.corners.x, triangle.corners.y, triangle.corners.z }) {
			if (corner == GpuRig::kInvalidDriverSlot)
				continue;
			REQUIRE(corner >= vertexBase);
			REQUIRE(corner < vertexBase + second.vertices.size());
		}
	}
	for (const GpuRig::SurfacePrepLeaf& leaf : data.leaves) {
		if (leaf.range.x < firstNodes)
			continue;
		REQUIRE(leaf.range.y >= firstTriangles);
		REQUIRE(leaf.range.y + leaf.range.z <= firstTriangles + second.indices.size() / 3);
	}
	for (std::uint32_t node = 0; node < data.metas[1].base.z; ++node) {
		const GpuRig::SurfacePrepNode& packed = data.nodes[data.metas[1].base.y + node];
		REQUIRE(packed.topology.x > node);
		REQUIRE(packed.topology.x <= data.metas[1].base.z);
	}

	std::vector<Transform> drivers(kSecondDriverOffset + after.size());
	drivers[kSecondDriverOffset] = after[0];
	drivers[kSecondDriverOffset + 1] = after[1];

	TriangleBvh cpuBvh = secondBvh;
	GpuRig::SurfaceFrame frame;
	GpuRig::AppendSurface(frame, second, after, cpuBvh);

	std::vector<GpuRig::SurfaceTriangle> triangles;
	std::vector<BvhNode> nodes;
	GpuRig::ReferenceSurfacePrep(data, drivers, triangles, nodes);
	REQUIRE(triangles.size() == data.triangles.size());
	REQUIRE(nodes.size() == data.nodes.size());
	for (std::size_t slot = 0; slot < frame.triangles.size(); ++slot)
		RequireSameSurfaceTriangle(triangles[data.metas[1].base.x + slot], frame.triangles[slot]);
	for (std::size_t index = 0; index < frame.nodes.size(); ++index)
		RequireSameNode(nodes[data.metas[1].base.y + index], frame.nodes[index]);
}

TEST_CASE("A surface whose hierarchy does not cover its triangles is not appended", "[smpphysics][gpurig]")
{
	const SkinnedSurface surface = MakeTwoBoneGridSurface();
	const std::vector<Transform> pose = FirstDriverPose();
	TriangleBvh bvh;
	REQUIRE(bvh.Build(PoseSurface(surface, pose), surface.indices));

	GpuRig::SurfacePrepData data;
	REQUIRE(GpuRig::AppendSurfacePrep(data, surface, bvh, 0u, static_cast<std::uint32_t>(pose.size())));
	GpuRig::FinalizeSurfacePrep(data);

	const std::size_t vertices = data.vertices.size();
	const std::size_t bones = data.bones.size();
	const std::size_t triangles = data.triangles.size();
	const std::size_t nodes = data.nodes.size();
	const std::size_t leaves = data.leaves.size();
	const std::size_t refit = data.refit.size();
	const std::size_t metas = data.metas.size();
	const std::vector<std::uint32_t> levelStart = data.refitLevelStart;
	const std::uint32_t levels = data.refitLevels;
	const auto requireUnchanged = [&]() {
		REQUIRE(data.vertices.size() == vertices);
		REQUIRE(data.bones.size() == bones);
		REQUIRE(data.triangles.size() == triangles);
		REQUIRE(data.nodes.size() == nodes);
		REQUIRE(data.leaves.size() == leaves);
		REQUIRE(data.refit.size() == refit);
		REQUIRE(data.metas.size() == metas);
		REQUIRE(data.refitLevelStart == levelStart);
		REQUIRE(data.refitLevels == levels);
	};

	TriangleBvh notBuilt;
	REQUIRE_FALSE(GpuRig::AppendSurfacePrep(data, surface, notBuilt, 0u, static_cast<std::uint32_t>(pose.size())));
	requireUnchanged();

	SkinnedSurface cropped = surface;
	cropped.indices.resize(cropped.indices.size() - 3);
	REQUIRE_FALSE(GpuRig::AppendSurfacePrep(data, cropped, bvh, 0u, static_cast<std::uint32_t>(pose.size())));
	requireUnchanged();
}

TEST_CASE("Refit dispatches run deepest level first", "[smpphysics][gpurig]")
{
	const SkinnedSurface first = MakeGridSurface(6);
	const SkinnedSurface second = MakeTwoBoneGridSurface();
	const std::vector<Transform> pose = FirstDriverPose();
	TriangleBvh firstBvh;
	TriangleBvh secondBvh;
	REQUIRE(firstBvh.Build(PoseSurface(first, { pose[0] }), first.indices));
	REQUIRE(secondBvh.Build(PoseSurface(second, pose), second.indices));

	GpuRig::SurfacePrepData data;
	REQUIRE(GpuRig::AppendSurfacePrep(data, first, firstBvh, 0u, 1u));
	REQUIRE(GpuRig::AppendSurfacePrep(data, second, secondBvh, 1u, static_cast<std::uint32_t>(pose.size())));
	GpuRig::FinalizeSurfacePrep(data);

	REQUIRE(data.refitLevels > 1u);
	REQUIRE(data.refitLevelStart.size() == static_cast<std::size_t>(data.refitLevels) + 1u);
	REQUIRE(data.refitLevelStart.front() == 0u);
	REQUIRE(data.refitLevelStart.back() == data.refit.size());
	for (std::uint32_t level = 0; level < data.refitLevels; ++level) {
		const std::uint32_t begin = data.refitLevelStart[level];
		const std::uint32_t end = data.refitLevelStart[level + 1];
		REQUIRE(begin < end);
		const std::uint32_t depth = data.refit[begin].link.w;
		for (std::uint32_t index = begin; index < end; ++index) {
			REQUIRE(data.refit[index].link.w == depth);
			REQUIRE(data.nodes[data.refit[index].link.x].topology.y == 0u);
		}
		if (level + 1 < data.refitLevels)
			REQUIRE(depth > data.refit[end].link.w);
	}
}

namespace
{
	GpuRig::SurfaceTriangle MakeCompareTriangle(float a_cornerOffset)
	{
		GpuRig::SurfaceTriangle triangle;
		triangle.a = { a_cornerOffset, 0.0f, 0.0f, std::bit_cast<float>(6u) };
		triangle.b = { 1.0f, 0.0f, 0.0f, 0.0f };
		triangle.c = { 0.0f, 1.0f, 0.0f, 0.0f };
		triangle.normalArea = { 0.0f, 0.0f, 1.0f, 1.0f };
		return triangle;
	}

	BvhNode MakeCompareNode(float a_lowY, std::uint32_t a_skip)
	{
		BvhNode node{};
		node.low[0] = -1.0f;
		node.low[1] = a_lowY;
		node.low[2] = -1.0f;
		node.high[0] = 1.0f;
		node.high[1] = 2.0f;
		node.high[2] = 1.0f;
		node.skip = a_skip;
		return node;
	}

	BvhNode MakeEmptyBoxNode()
	{
		BvhNode node{};
		for (int axis = 0; axis < 3; ++axis) {
			node.low[axis] = kBvhEmptyLow;
			node.high[axis] = kBvhEmptyHigh;
		}
		return node;
	}
}

TEST_CASE("Identical surface prep records compare with no error", "[smpphysics][gpurig]")
{
	const std::vector<GpuRig::SurfaceTriangle> triangles{ MakeCompareTriangle(0.0f), MakeCompareTriangle(2.0f) };
	const std::vector<BvhNode> nodes{ MakeCompareNode(0.0f, 4u), MakeCompareNode(-3.0f, 1u) };

	const GpuRig::SurfacePrepComparison comparison = GpuRig::CompareSurfacePrep(triangles, nodes,
		triangles.data(), nodes.data(), static_cast<std::uint32_t>(triangles.size()), static_cast<std::uint32_t>(nodes.size()));
	REQUIRE(comparison.triangles == 2u);
	REQUIRE(comparison.nodes == 2u);
	REQUIRE(comparison.maxTriangleError == 0.0f);
	REQUIRE(comparison.triangleMismatches == 0u);
	REQUIRE(comparison.maxNodeError == 0.0f);
	REQUIRE(comparison.nodeMismatches == 0u);
	REQUIRE(comparison.topologyMismatches == 0u);
}

TEST_CASE("A perturbed triangle component reports exactly one mismatch", "[smpphysics][gpurig]")
{
	const std::vector<GpuRig::SurfaceTriangle> triangles{ MakeCompareTriangle(0.0f), MakeCompareTriangle(2.0f) };
	const std::vector<BvhNode> nodes{ MakeCompareNode(0.0f, 4u), MakeCompareNode(-3.0f, 1u) };
	std::vector<GpuRig::SurfaceTriangle> actual = triangles;
	actual[1].c.z += 0.01f;

	const GpuRig::SurfacePrepComparison comparison = GpuRig::CompareSurfacePrep(triangles, nodes,
		actual.data(), nodes.data(), static_cast<std::uint32_t>(actual.size()), static_cast<std::uint32_t>(nodes.size()));
	REQUIRE(comparison.triangleMismatches == 1u);
	REQUIRE(comparison.maxTriangleError == Approx(0.01f));
	REQUIRE(comparison.maxNodeError == 0.0f);
}

TEST_CASE("A perturbed node bound reports exactly one node mismatch", "[smpphysics][gpurig]")
{
	const std::vector<GpuRig::SurfaceTriangle> triangles{ MakeCompareTriangle(0.0f) };
	const std::vector<BvhNode> nodes{ MakeCompareNode(0.0f, 4u), MakeCompareNode(-3.0f, 1u) };
	std::vector<BvhNode> actual = nodes;
	actual[0].low[1] += 0.02f;

	const GpuRig::SurfacePrepComparison comparison = GpuRig::CompareSurfacePrep(triangles, nodes,
		triangles.data(), actual.data(), static_cast<std::uint32_t>(triangles.size()), static_cast<std::uint32_t>(actual.size()));
	REQUIRE(comparison.nodeMismatches == 1u);
	REQUIRE(comparison.maxNodeError == Approx(0.02f));
	REQUIRE(comparison.triangleMismatches == 0u);
	REQUIRE(comparison.topologyMismatches == 0u);
}

TEST_CASE("A changed node skip reports exactly one topology mismatch", "[smpphysics][gpurig]")
{
	const std::vector<GpuRig::SurfaceTriangle> triangles{ MakeCompareTriangle(0.0f) };
	const std::vector<BvhNode> nodes{ MakeCompareNode(0.0f, 4u), MakeCompareNode(-3.0f, 1u) };
	std::vector<BvhNode> actual = nodes;
	actual[1].skip += 1u;

	const GpuRig::SurfacePrepComparison comparison = GpuRig::CompareSurfacePrep(triangles, nodes,
		triangles.data(), actual.data(), static_cast<std::uint32_t>(triangles.size()), static_cast<std::uint32_t>(actual.size()));
	REQUIRE(comparison.topologyMismatches == 1u);
	REQUIRE(comparison.nodeMismatches == 0u);
	REQUIRE(comparison.maxNodeError == 0.0f);
}

TEST_CASE("Empty-box sentinels on both sides are not compared", "[smpphysics][gpurig]")
{
	const std::vector<GpuRig::SurfaceTriangle> triangles;
	const std::vector<BvhNode> expected{ MakeEmptyBoxNode() };
	std::vector<BvhNode> actual = expected;
	for (int axis = 0; axis < 3; ++axis) {
		actual[0].low[axis] = 2.0f * kBvhEmptyLow;
		actual[0].high[axis] = 2.0f * kBvhEmptyHigh;
	}

	GpuRig::SurfacePrepComparison comparison = GpuRig::CompareSurfacePrep(triangles, expected,
		nullptr, actual.data(), 0u, static_cast<std::uint32_t>(actual.size()));
	REQUIRE(comparison.nodes == 1u);
	REQUIRE(comparison.maxNodeError == 0.0f);
	REQUIRE(comparison.nodeMismatches == 0u);
	REQUIRE(comparison.topologyMismatches == 0u);

	actual[0].low[0] = 0.5f;
	comparison = GpuRig::CompareSurfacePrep(triangles, expected, nullptr, actual.data(), 0u, static_cast<std::uint32_t>(actual.size()));
	REQUIRE(comparison.nodeMismatches == 1u);
}

TEST_CASE("Surface prep comparison tolerates rounding at large coordinates but not a real offset", "[smpphysics][gpurig]")
{
	const std::vector<BvhNode> nodes;
	std::vector<GpuRig::SurfaceTriangle> expected{ MakeCompareTriangle(30000.0f) };
	std::vector<GpuRig::SurfaceTriangle> actual = expected;
	actual[0].a.x += 0.004f;

	GpuRig::SurfacePrepComparison comparison = GpuRig::CompareSurfacePrep(expected, nodes, actual.data(), nodes.data(),
		static_cast<std::uint32_t>(actual.size()), 0u);
	REQUIRE(comparison.triangleMismatches == 0u);
	REQUIRE(comparison.maxTriangleError == Approx(0.004f).margin(1e-3f));

	actual[0].a.x += 1.0f;
	comparison = GpuRig::CompareSurfacePrep(expected, nodes, actual.data(), nodes.data(), static_cast<std::uint32_t>(actual.size()), 0u);
	REQUIRE(comparison.triangleMismatches == 1u);
}

TEST_CASE("Surface prep comparison checks the normal only of a well-conditioned triangle", "[smpphysics][gpurig]")
{
	const std::vector<BvhNode> nodes;
	std::vector<GpuRig::SurfaceTriangle> expected{ MakeCompareTriangle(0.0f) };
	std::vector<GpuRig::SurfaceTriangle> actual = expected;
	actual[0].normalArea.z = -1.0f;

	GpuRig::SurfacePrepComparison comparison = GpuRig::CompareSurfacePrep(expected, nodes, actual.data(), nodes.data(),
		static_cast<std::uint32_t>(actual.size()), 0u);
	REQUIRE(comparison.triangleMismatches == 0u);

	expected[0].normalArea.w = 4.0f;
	actual[0].normalArea.w = 4.0f;
	comparison = GpuRig::CompareSurfacePrep(expected, nodes, actual.data(), nodes.data(), static_cast<std::uint32_t>(actual.size()), 0u);
	REQUIRE(comparison.triangleMismatches == 1u);
	REQUIRE(comparison.maxNormalError == Approx(2.0f));
}

TEST_CASE("A claim's statics pack at its slots the same way appended or rebuilt", "[SmpPhysics]")
{
	const SmpRig rig = MakeBranchingRig();
	const std::vector<Transform> rest = MakeRestWorld(rig);
	const PackedCollision collision = MakeCollision(rig);
	GpuRig::RigData first = GpuRig::BuildRigData(rig, rest);
	GpuRig::SetCollision(first, GpuRig::PackCollision(collision.spheres, collision.colliders, 2));
	GpuRig::RigData second = first;
	constexpr std::uint32_t kSecondBase = 64;
	GpuRig::ShiftToSlotBase(second, kSecondBase);
	const auto boneCount = static_cast<std::uint32_t>(rig.bones.size());

	const GpuRig::ClaimStatics one = GpuRig::PackClaimStatics(first, 0, boneCount, 0);
	const auto oneSpheres = static_cast<std::uint32_t>(one.sphereList.size());
	const GpuRig::ClaimStatics two = GpuRig::PackClaimStatics(second, kSecondBase, boneCount, oneSpheres);
	REQUIRE(oneSpheres == first.collision.sphereList.size());
	REQUIRE(two.bones.size() == boneCount);
	REQUIRE(two.boneTrees.size() == boneCount);
	REQUIRE(two.order == second.order);
	REQUIRE(two.colliders.size() == second.collision.colliders.size());

	// Appended after the first claim, the second claim's ranges index past its records, and each record names its bone slot.
	std::vector<GpuRig::SphereRecord> shared = one.sphereList;
	shared.insert(shared.end(), two.sphereList.begin(), two.sphereList.end());
	for (std::size_t bone = 0; bone < boneCount; ++bone) {
		const GpuRig::BoneSpheres& local = second.collision.spheres[bone];
		REQUIRE(two.spheres[bone].range.y == local.range.y);
		REQUIRE(two.spheres[bone].range.x == local.range.x + oneSpheres);
		for (std::uint32_t member = 0; member < local.range.y; ++member) {
			const GpuRig::SphereRecord& record = shared[two.spheres[bone].range.x + member];
			REQUIRE(record.exempt.y == kSecondBase + bone);
			REQUIRE(record.centreRadius.w == Approx(second.collision.sphereList[local.range.x + member].centreRadius.w));
		}
	}

	// Each dynamic bone names the tree slot that carries it; a kinematic bone names none.
	for (std::size_t bone = 0; bone < boneCount; ++bone) {
		if (rig.bones[bone].kinematic) {
			REQUIRE(two.boneTrees[bone] == GpuRig::kNoTree);
			continue;
		}
		REQUIRE(two.boneTrees[bone] != GpuRig::kNoTree);
		const GpuRig::TreeStatic& tree = second.trees[two.boneTrees[bone] - kSecondBase];
		bool carried = false;
		for (std::uint32_t index = 0; index < tree.range.y; ++index)
			carried = carried || second.order[tree.range.x - kSecondBase + index] == kSecondBase + bone;
		REQUIRE(carried);
	}
}

TEST_CASE("A blended driver pose moves along the shorter arc and keeps its rotation unit length", "[smpphysics][gpurig]")
{
	GpuRig::DriverWorld from;
	from.rotation = { 0.0f, 0.0f, 0.0f, 1.0f };
	from.translation = { 0.0f, 10.0f, -4.0f, 0.0f };
	GpuRig::DriverWorld to;
	const float half = std::sqrt(0.5f);
	to.rotation = { 0.0f, 0.0f, half, half };
	to.translation = { 8.0f, 10.0f, 4.0f, 0.0f };

	const GpuRig::DriverWorld start = GpuRig::LerpDriverWorld(from, to, 0.0f);
	CHECK(start.rotation.w == Approx(1.0f));
	CHECK(start.translation.x == Approx(0.0f));

	const GpuRig::DriverWorld finish = GpuRig::LerpDriverWorld(from, to, 1.0f);
	CHECK(finish.rotation.z == Approx(half));
	CHECK(finish.translation.z == Approx(4.0f));

	const GpuRig::DriverWorld middle = GpuRig::LerpDriverWorld(from, to, 0.5f);
	CHECK(middle.translation.x == Approx(4.0f));
	CHECK(middle.translation.y == Approx(10.0f));
	const float length = std::sqrt(middle.rotation.x * middle.rotation.x + middle.rotation.y * middle.rotation.y +
								   middle.rotation.z * middle.rotation.z + middle.rotation.w * middle.rotation.w);
	CHECK(length == Approx(1.0f));
	CHECK(middle.rotation.z > 0.0f);

	GpuRig::DriverWorld flipped = to;
	flipped.rotation = { -to.rotation.x, -to.rotation.y, -to.rotation.z, -to.rotation.w };
	const GpuRig::DriverWorld viaShort = GpuRig::LerpDriverWorld(from, flipped, 0.5f);
	CHECK(viaShort.rotation.z == Approx(middle.rotation.z));
	CHECK(viaShort.rotation.w == Approx(middle.rotation.w));
}

TEST_CASE("A driver delta gives the velocity of the points the driver carried", "[smpphysics][gpurig]")
{
	GpuRig::DriverWorld from;
	from.rotation = { 1.0f, 0.0f, 0.0f, 0.0f };
	from.translation = { 10.0f, 0.0f, 0.0f, 0.0f };
	GpuRig::DriverWorld slid = from;
	slid.translation = { 10.0f, 3.0f, 0.0f, 0.0f };

	const GpuRig::DriverDelta slide = GpuRig::MakeDriverDelta(from, slid, 0.5f);
	const Vec3 slideVelocity = GpuRig::DriverPointVelocity(slide, { 12.0f, 1.0f, 4.0f });
	CHECK(slideVelocity.x == Approx(0.0f).margin(1e-4f));
	CHECK(slideVelocity.y == Approx(6.0f).margin(1e-4f));
	CHECK(slideVelocity.z == Approx(0.0f).margin(1e-4f));

	GpuRig::DriverWorld turned = from;
	const float half = std::sqrt(0.5f);
	turned.rotation = { half, 0.0f, 0.0f, half };
	const GpuRig::DriverDelta turn = GpuRig::MakeDriverDelta(from, turned, 1.0f);
	const Vec3 pivotVelocity = GpuRig::DriverPointVelocity(turn, { 10.0f, 0.0f, 0.0f });
	CHECK(pivotVelocity.x == Approx(0.0f).margin(1e-4f));
	CHECK(pivotVelocity.y == Approx(0.0f).margin(1e-4f));
	const Vec3 farVelocity = GpuRig::DriverPointVelocity(turn, { 12.0f, 0.0f, 0.0f });
	CHECK(farVelocity.x == Approx(-2.0f).margin(1e-4f));
	CHECK(farVelocity.y == Approx(2.0f).margin(1e-4f));

	CHECK(GpuRig::DriverPointVelocity(GpuRig::MakeDriverDelta(from, slid, 0.0f), { 1.0f, 1.0f, 1.0f }).y == 0.0f);
}

TEST_CASE("Blending and delta helpers touch only the live driver ranges", "[smpphysics][gpurig]")
{
	GpuRig::DriverWorld from;
	from.rotation = { 1.0f, 0.0f, 0.0f, 0.0f };
	from.translation = { 1.0f, 0.0f, 0.0f, 0.0f };
	GpuRig::DriverWorld to = from;
	to.translation = { 5.0f, 0.0f, 0.0f, 0.0f };

	const std::vector<GpuRig::DriverWorld> fromSlots(6, from);
	const std::vector<GpuRig::DriverWorld> toSlots(6, to);
	const std::vector<GpuRig::DriverRange> live{ { 1u, 2u }, { 4u, 1u } };

	GpuRig::DriverWorld sentinel;
	sentinel.translation = { -7.0f, 0.0f, 0.0f, 0.0f };
	std::vector<GpuRig::DriverWorld> blended(6, sentinel);
	GpuRig::BlendDriverRanges(fromSlots, toSlots, live, 0.5f, blended);
	for (const std::size_t slot : { 1u, 2u, 4u })
		CHECK(blended[slot].translation.x == Approx(3.0f).margin(1e-5f));
	for (const std::size_t slot : { 0u, 3u, 5u })
		CHECK(blended[slot].translation.x == -7.0f);

	std::vector<GpuRig::DriverDelta> deltas(6);
	GpuRig::FillDriverDeltas(fromSlots, toSlots, live, 0.5f, deltas);
	for (const std::size_t slot : { 1u, 2u, 4u }) {
		CHECK(deltas[slot].translation.x == Approx(4.0f).margin(1e-5f));
		CHECK(deltas[slot].translation.w == Approx(2.0f).margin(1e-5f));
	}
	for (const std::size_t slot : { 0u, 3u, 5u })
		CHECK(deltas[slot].translation.w == 0.0f);

	const std::vector<GpuRig::DriverRange> pastTheEnd{ { 4u, 10u } };
	GpuRig::BlendDriverRanges(fromSlots, toSlots, pastTheEnd, 0.5f, blended);
	GpuRig::FillDriverDeltas(fromSlots, toSlots, pastTheEnd, 0.5f, deltas);
	CHECK(blended.size() == 6);
	CHECK(deltas.size() == 6);
}

TEST_CASE("A driver delta does not depend on the substep fraction a pose is blended at", "[smpphysics][gpurig]")
{
	GpuRig::DriverWorld from;
	from.rotation = { 1.0f, 0.0f, 0.0f, 0.0f };
	GpuRig::DriverWorld to;
	const float half = std::sqrt(0.5f);
	to.rotation = { half, 0.0f, 0.0f, half };
	to.translation = { 3.0f, 2.0f, 1.0f, 0.0f };

	const std::vector<GpuRig::DriverWorld> fromSlots{ from };
	const std::vector<GpuRig::DriverWorld> toSlots{ to };
	const std::vector<GpuRig::DriverRange> live{ { 0u, 1u } };
	std::vector<GpuRig::DriverWorld> blended(1);
	std::vector<GpuRig::DriverDelta> first(1);
	std::vector<GpuRig::DriverDelta> second(1);
	GpuRig::BlendDriverRanges(fromSlots, toSlots, live, 0.0f, blended);
	GpuRig::FillDriverDeltas(fromSlots, toSlots, live, 1.0f / 60.0f, first);
	GpuRig::BlendDriverRanges(fromSlots, toSlots, live, 0.5f, blended);
	GpuRig::FillDriverDeltas(fromSlots, toSlots, live, 1.0f / 60.0f, second);
	CHECK(first[0].rotation.w == second[0].rotation.w);
	CHECK(first[0].translation.x == second[0].translation.x);
	CHECK(first[0].translation.w == second[0].translation.w);
}
