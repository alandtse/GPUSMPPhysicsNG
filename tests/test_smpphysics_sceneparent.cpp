// Unit tests for the rig-bone ancestor walk (src/smp/SceneParent.h). Pure; no game dependency.

#include "smp/SceneParent.h"

#include <catch2/catch_test_macros.hpp>

using SmpPhysics::NearestBoneAncestor;

namespace
{
	struct Node
	{
		const Node* parent{};
		bool bone{};
	};

	const Node* Parent(const Node* a_node) { return a_node->parent; }
	bool IsBone(const Node* a_node) { return a_node->bone; }
}

TEST_CASE("The nearest rig bone above a node is its parent when that is a bone", "[smpphysics][sceneparent]")
{
	const Node root{ nullptr, true };
	const Node child{ &root, true };
	CHECK(NearestBoneAncestor(&child, Parent, IsBone) == &root);
}

TEST_CASE("Nodes that are not rig bones are walked through", "[smpphysics][sceneparent]")
{
	const Node arm{ nullptr, true };
	const Node added{ &arm, false };
	const Node sleeve{ &added, true };
	CHECK(NearestBoneAncestor(&sleeve, Parent, IsBone) == &arm);
}

TEST_CASE("Another rig bone in between is the nearest one", "[smpphysics][sceneparent]")
{
	const Node arm{ nullptr, true };
	const Node other{ &arm, true };
	const Node sleeve{ &other, true };
	CHECK(NearestBoneAncestor(&sleeve, Parent, IsBone) == &other);
}

TEST_CASE("No rig bone above the node gives null", "[smpphysics][sceneparent]")
{
	const Node top{ nullptr, false };
	const Node child{ &top, true };
	CHECK(NearestBoneAncestor(&child, Parent, IsBone) == nullptr);
	CHECK(NearestBoneAncestor<Node>(nullptr, Parent, IsBone) == nullptr);
}
