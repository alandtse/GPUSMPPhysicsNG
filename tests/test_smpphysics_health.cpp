// Unit tests for the per-file physics health used to colour the armor lists (src/smp/PhysicsHealth.cpp).

#include "smp/PhysicsHealth.h"

#include <catch2/catch_test_macros.hpp>

#include <array>

using namespace SmpPhysics;

TEST_CASE("A file that parsed cleanly is working and a repaired one is repaired", "[smpphysics][health]")
{
	CHECK(ClassifyFile(true, "", false, "") == Health::Working);
	CHECK(ClassifyFile(true, "", true, "") == Health::Repaired);
}

TEST_CASE("A file that cannot be read or was declined for its own fault is broken", "[smpphysics][health]")
{
	CHECK(ClassifyFile(false, "the physics file could not be opened", false, "") == Health::Broken);
	CHECK(ClassifyFile(true, "", true, "a bone is missing") == Health::Broken);
}

TEST_CASE("A decline that is not the file's fault does not colour it", "[smpphysics][health]")
{
	CHECK(ClassifyFile(true, "", false, "collision is switched off in the settings") == Health::Working);
}

TEST_CASE("Worse picks the more serious health and None gives way", "[smpphysics][health]")
{
	CHECK(Worse(Health::None, Health::Working) == Health::Working);
	CHECK(Worse(Health::Repaired, Health::Working) == Health::Repaired);
	CHECK(Worse(Health::Repaired, Health::Broken) == Health::Broken);
	CHECK(Worse(Health::None, Health::None) == Health::None);
}

TEST_CASE("Files a model names that do not exist do not break an item that has a working one", "[smpphysics][health]")
{
	const FileHealth missing{ Health::Broken, true };
	const FileHealth working{ Health::Working, false };
	const std::array both{ missing, working, missing };
	CHECK(ClassifyItem(both) == Health::Working);
}

TEST_CASE("An item whose every file is missing is broken", "[smpphysics][health]")
{
	const std::array onlyMissing{ FileHealth{ Health::Broken, true }, FileHealth{ Health::Broken, true } };
	CHECK(ClassifyItem(onlyMissing) == Health::Broken);
}

TEST_CASE("A file that exists and is broken still breaks the item", "[smpphysics][health]")
{
	const std::array mixed{ FileHealth{ Health::Broken, true }, FileHealth{ Health::Working, false }, FileHealth{ Health::Broken, false } };
	CHECK(ClassifyItem(mixed) == Health::Broken);
	CHECK(ClassifyItem({}) == Health::None);
}
