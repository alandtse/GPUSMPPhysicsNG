// Unit tests for ordering a list by hair shape distance (src/smp/HairMatchOrder.cpp).

#include "smp/HairMatchOrder.h"

#include <catch2/catch_test_macros.hpp>

#include <limits>
#include <vector>

using namespace SmpPhysics;

TEST_CASE("Ranked items come first, closest to farthest, then the rest in their own order", "[smpphysics][hairmatchorder]")
{
	const std::vector<HairRank> ranked{ { 3, 1.4f }, { 1, 0.9f }, { 4, 1.1f } };
	const HairOrdering ordering = OrderByHairMatch(ranked, 6);

	CHECK(ordering.order == std::vector<int>{ 1, 4, 3, 0, 2, 5 });
	CHECK(ordering.badges.size() == 6);
}

TEST_CASE("Items within the tie band of the closest are marked as the closest group", "[smpphysics][hairmatchorder]")
{
	const std::vector<HairRank> ranked{ { 0, 0.86f }, { 1, 0.95f }, { 2, 1.01f }, { 3, 1.30f } };
	const HairOrdering ordering = OrderByHairMatch(ranked, 4);

	CHECK(ordering.closest == 3);
	CHECK(ordering.badges[0] == "closest 0.86");
	CHECK(ordering.badges[2] == "closest 1.01");
	CHECK(ordering.badges[3] == "1.30");
}

TEST_CASE("An unranked item has no badge", "[smpphysics][hairmatchorder]")
{
	const HairOrdering ordering = OrderByHairMatch(std::vector<HairRank>{ { 1, 0.5f } }, 3);

	CHECK(ordering.badges[0].empty());
	CHECK(ordering.badges[2].empty());
}

TEST_CASE("Nothing ranked leaves the natural order", "[smpphysics][hairmatchorder]")
{
	const HairOrdering ordering = OrderByHairMatch({}, 3);

	CHECK(ordering.order == std::vector<int>{ 0, 1, 2 });
	CHECK(ordering.closest == 0);
	CHECK(OrderByHairMatch({}, 0).order.empty());
}

TEST_CASE("Bad ranks are ignored", "[smpphysics][hairmatchorder]")
{
	const std::vector<HairRank> ranked{ { 9, 0.1f }, { 1, std::numeric_limits<float>::quiet_NaN() }, { 2, 0.7f }, { 2, 0.2f } };
	const HairOrdering ordering = OrderByHairMatch(ranked, 4);

	CHECK(ordering.order == std::vector<int>{ 2, 0, 1, 3 });
	CHECK(ordering.badges[2] == "closest 0.70");
}
