// Unit tests for resolving a rig bone to the node its mesh is skinned to (src/smp/BoundBones.h).
// Pure logic; no game/D3D dependency.

#include "smp/BoundBones.h"

#include <catch2/catch_test_macros.hpp>

using namespace SmpPhysics;

TEST_CASE("A rig bone resolves to the node its mesh is skinned to before the skeleton's", "[smpphysics][boundbones]")
{
	// A severed head: the skeleton keeps its head bone and the mesh is bound to the clone in the limb node.
	int skeletonHead = 0;
	int clonedHead = 0;
	int skeletonSpine = 0;
	BoundBones<int> bound;
	bound.Add("NPC Head [Head]", &clonedHead);

	CHECK(bound.Resolve("NPC Head [Head]", &skeletonHead) == &clonedHead);
	CHECK(bound.Resolve("NPC Spine2 [Spn2]", &skeletonSpine) == &skeletonSpine);
	CHECK(bound.Resolve("Absent", nullptr) == nullptr);
}

TEST_CASE("The first node bound under a name is kept, and empty entries are ignored", "[smpphysics][boundbones]")
{
	int first = 0;
	int second = 0;
	int skeleton = 0;
	BoundBones<int> bound;
	bound.Add("Bone", &first);
	bound.Add("Bone", &second);
	bound.Add("", &second);
	bound.Add("Null", nullptr);

	CHECK(bound.Resolve("Bone", &skeleton) == &first);
	CHECK(bound.Resolve("", &skeleton) == &skeleton);
	CHECK(bound.Resolve("Null", &skeleton) == &skeleton);
}

namespace
{
	struct Item
	{
		Item* parent{};
	};
	Item* ParentOf(Item* a_item) { return a_item->parent; }
}

TEST_CASE("Unbound rig bones are looked up in the subtree most bound bones hang in", "[smpphysics][boundbones]")
{
	// scene > skeleton root > { NPC > head > hair, limb > clonedHead > clonedHair }
	Item scene;
	Item skeletonRoot{ &scene };
	Item npc{ &skeletonRoot };
	Item head{ &npc };
	Item hair{ &head };
	Item limb{ &skeletonRoot };
	Item clonedHead{ &limb };
	Item clonedHair{ &clonedHead };
	Item stray;

	CHECK(SubtreeRoot(&hair, &npc, ParentOf) == &npc);
	CHECK(SubtreeRoot(&npc, &npc, ParentOf) == &npc);
	CHECK(SubtreeRoot(&clonedHair, &npc, ParentOf) == &limb);
	CHECK(SubtreeRoot(&stray, &npc, ParentOf) == nullptr);

	BoundBones<Item> ordinary;
	ordinary.Add("Head", &head);
	ordinary.Add("Hair", &hair);
	CHECK(ordinary.CommonRoot(&npc, ParentOf) == &npc);

	BoundBones<Item> severed;
	severed.Add("Head", &clonedHead);
	severed.Add("Hair", &clonedHair);
	severed.Add("Spine", &npc);
	CHECK(severed.CommonRoot(&npc, ParentOf) == &limb);

	CHECK(BoundBones<Item>{}.CommonRoot(&npc, ParentOf) == &npc);
}
