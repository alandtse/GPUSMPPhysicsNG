// Unit tests for the island size summary (src/smp/RigidGpuPacking.cpp).

#include "smp/RigidGpuPacking.h"

#include <catch2/catch_test_macros.hpp>

using namespace SmpPhysics::RigidGpu;

namespace
{
	IslandRecord Island(std::uint32_t a_constraints, std::uint32_t a_colors = 1)
	{
		IslandRecord island;
		island.range = { 0u, a_constraints, a_colors, 0u };
		return island;
	}
}

TEST_CASE("Islands fall into the size bucket their constraint count names", "[smpphysics][islands]")
{
	const IslandSizes sizes = SummariseIslands({ Island(1), Island(2), Island(4), Island(5), Island(8), Island(9), Island(16), Island(17), Island(32), Island(33), Island(64), Island(65), Island(500) });

	CHECK(sizes.islands == std::array<std::uint32_t, 7>{ 1, 2, 2, 2, 2, 2, 2 });
	CHECK(sizes.constraints[0] == 1);
	CHECK(sizes.constraints[1] == 6);
	CHECK(sizes.constraints[6] == 65 + 500);
	CHECK(sizes.total == 13);
	CHECK(sizes.largest == 500);
}

TEST_CASE("Empty islands are not counted and the most colours is kept", "[smpphysics][islands]")
{
	const IslandSizes sizes = SummariseIslands({ Island(0, 9), Island(3, 4), Island(3, 7) });

	CHECK(sizes.total == 2);
	CHECK(sizes.mostColors == 7);
	CHECK(SummariseIslands({}).total == 0);
}
