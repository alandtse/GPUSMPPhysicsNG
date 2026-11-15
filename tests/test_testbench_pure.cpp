// Unit tests for the engine-free helpers behind the test crowd: layout, filtering, base picking and hairstyle picking.

#include "Features/SmpPhysics/CrowdStyles.h"
#include "Utils/TestBench/CrowdBases.h"
#include "Utils/TestBench/CrowdFilter.h"
#include "Utils/TestBench/TestCrowdLayout.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <set>

using Catch::Approx;
using namespace TestBench;

TEST_CASE("The crowd stands in rows centred on the line of sight", "[testbench]")
{
	const auto first = TestCrowdSlotFor(0);
	CHECK(first.forward == Approx(kTestCrowdFirstRow));
	CHECK(first.right == Approx(-2.0f * kTestCrowdSpacing));

	const auto middle = TestCrowdSlotFor(2);
	CHECK(middle.right == Approx(0.0f));

	const auto nextRow = TestCrowdSlotFor(kTestCrowdColumns);
	CHECK(nextRow.forward == Approx(kTestCrowdFirstRow + kTestCrowdSpacing));
	CHECK(nextRow.right == Approx(first.right));
}

TEST_CASE("The default idle prefers a Cicero dance", "[testbench]")
{
	CHECK_FALSE(PickDefaultIdle({}).has_value());
	CHECK_FALSE(PickDefaultIdle({ "IdleWave", "IdleStop" }).has_value());
	CHECK(PickDefaultIdle({ "IdleWave", "CiceroIdleSad", "CiceroDance2" }) == 2u);
	CHECK(PickDefaultIdle({ "IdleWave", "CiceroIdleSad", "CiceroIdleMad" }) == 1u);
}

TEST_CASE("Idle end events and bases", "[testbench]")
{
	CHECK(IsIdleEndEvent("IdleStop"));
	CHECK_FALSE(IsIdleEndEvent("idlestop"));
	CHECK(BaseForSlot(0, {}, 7) == 7u);
	CHECK(BaseForSlot(4, { 10, 20, 30 }, 7) == 20u);
}

TEST_CASE("The crowd filter leaves out children and the elderly", "[testbench]")
{
	CHECK(IsElderlyRace("NordRaceElder"));
	CHECK(IsElderlyRace("OldPeopleRace"));
	CHECK(IsElderlyRace("ELDERRACE"));
	CHECK_FALSE(IsElderlyRace("NordRace"));

	CHECK(PassesFilter({ false, "NordRace" }, {}));
	CHECK_FALSE(PassesFilter({ true, "NordRace" }, {}));
	CHECK_FALSE(PassesFilter({ false, "ElderRace" }, {}));
	CHECK(PassesFilter({ true, "ElderRace" }, { false, false }));
	CHECK(PassesFilter({ true, "NordRace" }, { false, true }));
}

TEST_CASE("Cloak paths match a segment that starts with cloak or cape", "[testbench]")
{
	CHECK(IsCloakModelPath(R"(Armor\Cloaks\Imperial\cloak_m.nif)"));
	CHECK(IsCloakModelPath("meshes/CAPE/model.nif"));
	CHECK(IsCloakModelPath("meshes/armor/cape01.nif"));
	CHECK_FALSE(IsCloakModelPath("meshes/armor/stormcloaks/helm.nif"));
	CHECK_FALSE(IsCloakModelPath("meshes/landscape/rock.nif"));
	CHECK_FALSE(IsCloakModelPath(""));
}

TEST_CASE("PickFromIds takes bases in turn or as shuffled decks", "[testbench]")
{
	const std::vector<std::uint32_t> ids{ 1, 2, 3 };
	CHECK(PickFromIds({}, 4, true, 1).empty());
	CHECK(PickFromIds(ids, 5, false, 0) == std::vector<std::uint32_t>{ 1, 2, 3, 1, 2 });

	const auto shuffled = PickFromIds(ids, 6, true, 42);
	REQUIRE(shuffled.size() == 6);
	CHECK(std::set<std::uint32_t>(shuffled.begin(), shuffled.begin() + 3) == std::set<std::uint32_t>(ids.begin(), ids.end()));
	CHECK(std::set<std::uint32_t>(shuffled.begin() + 3, shuffled.end()) == std::set<std::uint32_t>(ids.begin(), ids.end()));
	CHECK(PickFromIds(ids, 6, true, 42) == shuffled);
	CHECK(PickFromIds(ids, 2, true, 9).size() == 2);
}

TEST_CASE("PickDistinctWigs favours physics hair and removes repeats", "[testbench]")
{
	const std::vector<SmpPhysics::WigCandidate> candidates{
		{ "Plain", "meshes/hair/plain.nif" },
		{ "bravo", "meshes/hair/HDT/bravo.nif" },
		{ "Alpha", "meshes/hair/smp/alpha.nif" },
		{ "ALPHA", "meshes/hair/smp/alpha2.nif" },
		{ "", "meshes/hair/smp/unnamed.nif" },
	};
	CHECK(SmpPhysics::PickDistinctWigs(candidates, 10) == std::vector<std::size_t>{ 2, 1, 0 });
	CHECK(SmpPhysics::PickDistinctWigs(candidates, 2) == std::vector<std::size_t>{ 2, 1 });
	CHECK(SmpPhysics::PickDistinctWigs(candidates, 0).empty());
}

TEST_CASE("Hair colours resolve by name ignoring case, spaces and underscores", "[testbench]")
{
	CHECK(SmpPhysics::ResolveHairColor("Black") == 0x000A0434u);
	CHECK(SmpPhysics::ResolveHairColor("light_blond") == 0x000A042Du);
	CHECK(SmpPhysics::ResolveHairColor("Dark Blond") == 0x000A042Cu);
	CHECK_FALSE(SmpPhysics::ResolveHairColor("Purple").has_value());
	for (const auto& color : SmpPhysics::TestCrowdHairColors())
		CHECK(SmpPhysics::ResolveHairColor(color.name) == color.formId);
}
