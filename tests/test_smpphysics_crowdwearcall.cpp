// Unit tests for the crowd commands the item pickers send (src/smp/CrowdWearCall.cpp).

#include "smp/CrowdWearCall.h"

#include <catch2/catch_test_macros.hpp>

using namespace SmpPhysics;

TEST_CASE("Putting an item on the crowd sends the command devbench uses, with the item's index", "[smpphysics][crowdwearcall]")
{
	const CrowdWearCall armor = CrowdWearCommand(TargetItem::Armor, false, 7);
	CHECK(armor.command == "setTestCrowdArmor");
	CHECK(armor.index == 7);
	CHECK_FALSE(armor.enabled);

	const CrowdWearCall cloak = CrowdWearCommand(TargetItem::Cloak, false, 0);
	CHECK(cloak.command == "setTestCrowdCloak");
	CHECK(cloak.index == 0);

	const CrowdWearCall wig = CrowdWearCommand(TargetItem::Wig, false, 12);
	CHECK(wig.command == "setTestCrowdWig");
	CHECK(wig.index == 12);
}

TEST_CASE("Taking an item off the crowd ignores the index and asks for what that kind needs", "[smpphysics][crowdwearcall]")
{
	const CrowdWearCall armor = CrowdWearCommand(TargetItem::Armor, true, 7);
	CHECK(armor.command == "stripTestCrowd");
	CHECK(armor.enabled == false);
	CHECK_FALSE(armor.index);

	const CrowdWearCall cloak = CrowdWearCommand(TargetItem::Cloak, true, 7);
	CHECK(cloak.command == "setTestCrowdCloak");
	CHECK(cloak.index == -1);

	const CrowdWearCall wig = CrowdWearCommand(TargetItem::Wig, true, 7);
	CHECK(wig.command == "takeOffTestCrowdWig");
	CHECK_FALSE(wig.index);
	CHECK_FALSE(wig.enabled);
}
