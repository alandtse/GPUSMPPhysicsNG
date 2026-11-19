// Unit tests for the hover text of a list item's physics files (src/smp/ItemStats.cpp).

#include <catch2/catch_test_macros.hpp>

#include "smp/ItemStats.h"

using namespace SmpPhysics;

TEST_CASE("An item with no physics file says so", "[smpphysics][itemstats]")
{
	CHECK(FormatItemStats({}) == "No physics file found.");
}

TEST_CASE("A file reports its full path, health and size", "[smpphysics][itemstats]")
{
	FileStats file;
	file.file = "Meshes\\armor\\Hair\\07.xml";
	file.health = Health::Working;
	file.read = true;
	file.bones = 633;
	file.dynamic = 171;
	file.joints = 526;

	CHECK(FormatItemStats({ &file, 1 }) == "Meshes\\armor\\Hair\\07.xml: works, 633 bones (171 dynamic), 526 joints");
}

TEST_CASE("A claim adds how many bones it kept, as a range across meshes", "[smpphysics][itemstats]")
{
	FileStats file;
	file.file = "a/b.xml";
	file.health = Health::Repaired;
	file.read = true;
	file.bones = 10;
	file.dynamic = 4;
	file.joints = 3;
	ClaimTrim trim;
	trim.Add(6);
	file.claim = trim;
	CHECK(FormatItemStats({ &file, 1 }) == "a/b.xml: repaired, 10 bones (4 dynamic), 3 joints; claimed keeping 6 bones");

	trim.Add(9);
	trim.Add(7);
	file.claim = trim;
	CHECK(FormatItemStats({ &file, 1 }) == "a/b.xml: repaired, 10 bones (4 dynamic), 3 joints; claimed keeping 6-9 bones");
}

TEST_CASE("A file that was not read gives no counts", "[smpphysics][itemstats]")
{
	FileStats file;
	file.file = "x.xml";
	file.health = Health::Broken;

	CHECK(FormatItemStats({ &file, 1 }) == "x.xml: broken, not read");
}

TEST_CASE("Only the first files are listed", "[smpphysics][itemstats]")
{
	std::vector<FileStats> files(6);
	for (FileStats& file : files)
		file.file = "f.xml";

	const std::string text = FormatItemStats(files, 2);
	CHECK(text.ends_with("\nand 4 more"));
	CHECK(std::count(text.begin(), text.end(), '\n') == 2);
}
