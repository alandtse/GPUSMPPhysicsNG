// Unit tests for KinematicAnchors (src/smp/SmpRig.cpp). Pure data; no game or D3D dependency.

#include "smp/SmpRig.h"

#include <catch2/catch_test_macros.hpp>

using namespace SmpPhysics;

namespace
{
	SmpRig MakeRig(const std::vector<bool>& a_kinematic, const std::vector<std::pair<int, int>>& a_childParent)
	{
		SmpRig rig;
		for (const bool kinematic : a_kinematic) {
			RigBone bone;
			bone.kinematic = kinematic;
			rig.bones.push_back(bone);
		}
		for (const auto& [child, parent] : a_childParent) {
			RigJoint joint;
			joint.child = child;
			joint.parent = parent;
			rig.joints.push_back(joint);
		}
		return rig;
	}
}

TEST_CASE("A dynamic bone anchors to the kinematic bone its joint chain ends at", "[smpphysics][kinematicanchor]")
{
	const SmpRig rig = MakeRig({ true, false, false, true, false }, { { 2, 1 }, { 1, 0 }, { 4, 3 } });

	const std::vector<int> anchors = KinematicAnchors(rig);

	REQUIRE(anchors.size() == 5);
	CHECK(anchors[0] == 0);
	CHECK(anchors[1] == 0);
	CHECK(anchors[2] == 0);
	CHECK(anchors[3] == 3);
	CHECK(anchors[4] == 3);
}

TEST_CASE("A dynamic bone with no kinematic ancestor has no anchor", "[smpphysics][kinematicanchor]")
{
	const SmpRig rig = MakeRig({ true, false, false }, { { 2, 1 } });

	const std::vector<int> anchors = KinematicAnchors(rig);

	CHECK(anchors[1] == -1);
	CHECK(anchors[2] == -1);
}

TEST_CASE("A joint loop between dynamic bones ends without an anchor", "[smpphysics][kinematicanchor]")
{
	const SmpRig rig = MakeRig({ true, false, false }, { { 1, 2 }, { 2, 1 } });

	const std::vector<int> anchors = KinematicAnchors(rig);

	CHECK(anchors[1] == -1);
	CHECK(anchors[2] == -1);
}
