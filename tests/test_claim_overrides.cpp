// Unit tests for the saved rules that keep a physics file with hdtSMP64 and for the order the simulation budget fills in.

#include "Features/SmpPhysics/ClaimOverrides.h"
#include "Features/SmpPhysics/ClaimPriority.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <vector>

using namespace SmpPhysics;

TEST_CASE("A file path is normalised to lower case with forward slashes", "[overrides]")
{
	REQUIRE(ClaimOverrides::NormalizeFile("Meshes\\xSummer\\S4 Hair\\ColdHeart.xml") == "meshes/xsummer/s4 hair/coldheart.xml");
}

TEST_CASE("A rule without a base form blocks the file for every character", "[overrides]")
{
	ClaimOverrides overrides;
	REQUIRE(overrides.Add({ "", "Meshes\\Hair\\A.xml" }));
	REQUIRE(overrides.Blocks("Skyrim.esm|0x000007", "meshes/hair/a.xml"));
	REQUIRE(overrides.Blocks("", "MESHES\\HAIR\\A.XML"));
	REQUIRE_FALSE(overrides.Blocks("Skyrim.esm|0x000007", "meshes/hair/b.xml"));
}

TEST_CASE("A rule with a base form blocks the file only for that character type", "[overrides]")
{
	ClaimOverrides overrides;
	REQUIRE(overrides.Add({ "Skyrim.esm|0x013BB8", "meshes/hair/a.xml" }));
	REQUIRE(overrides.Blocks("skyrim.esm|0x013bb8", "meshes/hair/a.xml"));
	REQUIRE_FALSE(overrides.Blocks("Skyrim.esm|0x000007", "meshes/hair/a.xml"));
	REQUIRE_FALSE(overrides.Blocks("", "meshes/hair/a.xml"));
}

TEST_CASE("A rule is refused when it has no file or repeats one", "[overrides]")
{
	ClaimOverrides overrides;
	REQUIRE_FALSE(overrides.Add({ "Skyrim.esm|0x1", "" }));
	REQUIRE(overrides.Add({ "", "meshes/a.xml" }));
	REQUIRE_FALSE(overrides.Add({ "", "MESHES\\A.XML" }));
	REQUIRE(overrides.Rules().size() == 1);
}

TEST_CASE("Removing a rule stops it blocking", "[overrides]")
{
	ClaimOverrides overrides;
	overrides.Add({ "", "meshes/a.xml" });
	overrides.Add({ "", "meshes/b.xml" });
	REQUIRE(overrides.Remove(0));
	REQUIRE_FALSE(overrides.Blocks("", "meshes/a.xml"));
	REQUIRE(overrides.Blocks("", "meshes/b.xml"));
	REQUIRE_FALSE(overrides.Remove(5));
}

TEST_CASE("Rules survive a round trip through JSON", "[overrides]")
{
	ClaimOverrides saved;
	saved.Add({ "", "meshes/a.xml" });
	saved.Add({ "Skyrim.esm|0x13BB8", "meshes/b.xml" });

	ClaimOverrides loaded;
	loaded.FromJson(saved.ToJson());
	REQUIRE(loaded.Rules() == saved.Rules());
}

TEST_CASE("Malformed entries in the saved file are skipped", "[overrides]")
{
	const auto document = nlohmann::json::parse(R"({"keepWithHdtSmp64":[1,{"base":"x"},{"file":5},{"file":"meshes/ok.xml"},"text"]})");
	ClaimOverrides overrides;
	overrides.FromJson(document);
	REQUIRE(overrides.Rules().size() == 1);
	REQUIRE(overrides.Blocks("", "meshes/ok.xml"));

	overrides.FromJson(nlohmann::json::parse("[]"));
	REQUIRE(overrides.Rules().empty());
	overrides.FromJson(nlohmann::json::parse(R"({"keepWithHdtSmp64":"nope"})"));
	REQUIRE(overrides.Rules().empty());
}

TEST_CASE("The rule list stops growing at its limit", "[overrides]")
{
	ClaimOverrides overrides;
	for (std::size_t index = 0; index < ClaimOverrides::kMaxRules; ++index)
		REQUIRE(overrides.Add({ "", "meshes/" + std::to_string(index) + ".xml" }));
	REQUIRE_FALSE(overrides.Add({ "", "meshes/one-more.xml" }));
}

TEST_CASE("The budget fills with the player, then followers, then everyone else", "[priority]")
{
	std::vector<RankedClaim> claims{
		{ ClaimTier::Other, 100.0f },
		{ ClaimTier::Follower, 900.0f },
		{ ClaimTier::Other, 50.0f },
		{ ClaimTier::Player, 5000.0f },
		{ ClaimTier::Follower, 400.0f },
	};
	std::ranges::sort(claims, RanksBefore);
	REQUIRE(claims[0].tier == ClaimTier::Player);
	REQUIRE(claims[1].tier == ClaimTier::Follower);
	REQUIRE(claims[1].distance == 400.0f);
	REQUIRE(claims[2].distance == 900.0f);
	REQUIRE(claims[3].distance == 50.0f);
	REQUIRE(claims[4].distance == 100.0f);
}
