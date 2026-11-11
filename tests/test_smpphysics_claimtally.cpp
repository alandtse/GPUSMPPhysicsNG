// Unit tests for the claim outcome tally (src/smp/ClaimTally.cpp). Pure counting; no game or D3D dependency.

#include "smp/ClaimTally.h"

#include <catch2/catch_test_macros.hpp>

#include <string>

using SmpPhysics::ClaimTally;
using Kind = ClaimTally::Kind;

TEST_CASE("The tally separates attempts, accepts and declines per mesh kind", "[smpphysics][claimtally]")
{
	ClaimTally tally;
	tally.Record(Kind::Armor, 1, "a.xml", true, "");
	tally.Record(Kind::Armor, 2, "a.xml", false, "no skeleton");
	tally.Record(Kind::HeadPart, 3, "h.xml", false, "no skeleton");

	const auto snapshot = tally.Take();
	CHECK(snapshot.kinds[0].attempts == 2);
	CHECK(snapshot.kinds[0].accepts == 1);
	CHECK(snapshot.kinds[0].declines == 1);
	CHECK(snapshot.kinds[1].attempts == 1);
	CHECK(snapshot.kinds[1].declines == 1);
	REQUIRE(snapshot.reasons.size() == 1);
	CHECK(snapshot.reasons[0].declines == 2);
	CHECK(snapshot.reasons[0].byKind[0] == 1);
	CHECK(snapshot.reasons[0].byKind[1] == 1);
}

TEST_CASE("Quoted names and numbers do not split one reason into many", "[smpphysics][claimtally]")
{
	CHECK(ClaimTally::Normalize("bone 'NPC Hair 01' has no node below the skeleton") == "bone '?' has no node below the skeleton");
	CHECK(ClaimTally::Normalize("12 of 40 joints cannot be rooted") == "N of N joints cannot be rooted");

	ClaimTally tally;
	tally.Record(Kind::Armor, 1, "a.xml", false, "bone 'A' has no node below the skeleton");
	tally.Record(Kind::Armor, 2, "b.xml", false, "bone 'B' has no node below the skeleton");
	const auto snapshot = tally.Take();
	REQUIRE(snapshot.reasons.size() == 1);
	CHECK(snapshot.reasons[0].example == "bone 'A' has no node below the skeleton");
	CHECK(snapshot.reasons[0].sampleFiles.size() == 2);
}

TEST_CASE("A pair offered again counts once, by its latest outcome", "[smpphysics][claimtally]")
{
	ClaimTally tally;
	for (int retry = 0; retry < 5; ++retry)
		tally.Record(Kind::Armor, 7, "Hair.XML", false, "topology");
	tally.Record(Kind::Armor, 8, "hair.xml", false, "topology");

	auto snapshot = tally.Take();
	CHECK(snapshot.kinds[0].declines == 6);
	CHECK(snapshot.pairsDeclined == 2);
	REQUIRE(snapshot.reasons.size() == 1);
	CHECK(snapshot.reasons[0].declines == 6);
	CHECK(snapshot.reasons[0].pairs == 2);
	REQUIRE(snapshot.files.size() == 1);
	CHECK(snapshot.files[0].file == "hair.xml");

	tally.Record(Kind::Armor, 7, "hair.xml", true, "");
	snapshot = tally.Take();
	CHECK(snapshot.pairsAccepted == 1);
	CHECK(snapshot.pairsDeclined == 1);
	CHECK(snapshot.reasons[0].pairs == 1);
}

TEST_CASE("Reasons are ranked by the pairs they hold back", "[smpphysics][claimtally]")
{
	ClaimTally tally;
	tally.Record(Kind::Armor, 1, "a.xml", false, "rare");
	for (std::uint32_t actor = 10; actor < 13; ++actor)
		tally.Record(Kind::Armor, actor, "b.xml", false, "common");

	const auto snapshot = tally.Take();
	REQUIRE(snapshot.reasons.size() == 2);
	CHECK(snapshot.reasons[0].reason == "common");
	CHECK(snapshot.reasons[0].pairs == 3);
}

TEST_CASE("Storage is bounded and Reset clears it", "[smpphysics][claimtally]")
{
	ClaimTally tally;
	for (std::size_t index = 0; index < ClaimTally::kMaxReasons + 10; ++index)
		tally.Record(Kind::Armor, 1, "a.xml", false, "reason " + std::string(index + 1, 'x'));

	auto snapshot = tally.Take();
	CHECK(snapshot.reasons.size() == ClaimTally::kMaxReasons + 1);
	CHECK(snapshot.kinds[0].declines == ClaimTally::kMaxReasons + 10);

	for (std::uint32_t actor = 0; actor < ClaimTally::kMaxPairs + 5; ++actor)
		tally.Record(Kind::Armor, actor, "a.xml", true, "");
	snapshot = tally.Take();
	CHECK(snapshot.pairsTruncated);
	CHECK(snapshot.pairsAccepted + snapshot.pairsDeclined == ClaimTally::kMaxPairs);

	tally.Reset();
	snapshot = tally.Take();
	CHECK(snapshot.kinds[0].attempts == 0);
	CHECK(snapshot.reasons.empty());
	CHECK(snapshot.files.empty());
	CHECK_FALSE(snapshot.pairsTruncated);
}

TEST_CASE("DeclinedPairs lists the pairs whose latest outcome is a decline and skips actorless ones", "[smpphysics][claimtally]")
{
	ClaimTally tally;
	tally.Record(Kind::Armor, 1, "A.xml", false, "limit");
	tally.Record(Kind::Armor, 2, "b.xml", false, "limit");
	tally.Record(Kind::Armor, 2, "b.xml", true, "");
	tally.Record(Kind::Armor, 0, "c.xml", false, "limit");

	const auto declined = tally.DeclinedPairs();
	REQUIRE(declined.size() == 1);
	CHECK(declined[0].first == 1);
	CHECK(declined[0].second == "a.xml");
}
