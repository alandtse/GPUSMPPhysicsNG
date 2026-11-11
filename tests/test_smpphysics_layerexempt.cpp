// Unit tests for ExemptSurfacesBehind (src/smp/SolverCommon.cpp). Pure data; no game or D3D dependency.

#include "smp/SolverCommon.h"

#include <catch2/catch_test_macros.hpp>

using namespace SmpPhysics;

namespace
{
	SphereProbe Probe(int a_bone, float a_behind, int a_surface)
	{
		SphereProbe probe;
		probe.bone = a_bone;
		probe.behind = a_behind;
		probe.behindSurface = a_surface;
		return probe;
	}
}

TEST_CASE("A bone authored behind a rig surface stops touching it", "[smpphysics][layerexempt]")
{
	std::vector<std::uint32_t> mask;

	const std::size_t exempted = ExemptSurfacesBehind({ Probe(1, 3.0f, 2), Probe(1, 2.0f, 2) }, 4, 3, mask);

	CHECK(exempted == 1);
	REQUIRE(mask.size() == 3);
	CHECK(mask[0] == 0xFFFFFFFFu);
	CHECK(mask[1] == 0xFFFFFFFBu);
}

TEST_CASE("A sphere in front of its surface, or behind one the limit excludes, keeps colliding", "[smpphysics][layerexempt]")
{
	std::vector<std::uint32_t> mask;

	const std::size_t exempted = ExemptSurfacesBehind({ Probe(0, 0.0f, 1), Probe(0, kProbeLostBehind, 1), Probe(0, 5.0f, 4), Probe(0, 5.0f, -1) }, 4, 2, mask);

	CHECK(exempted == 0);
	CHECK(mask.empty());
}

TEST_CASE("Each surface bit is cleared independently", "[smpphysics][layerexempt]")
{
	std::vector<std::uint32_t> mask(2, 0xFFFFFFFFu);

	ExemptSurfacesBehind({ Probe(0, 4.0f, 0), Probe(0, 4.0f, 3), Probe(1, 4.0f, 1) }, 8, 2, mask);

	CHECK(mask[0] == 0xFFFFFFF6u);
	CHECK(mask[1] == 0xFFFFFFFDu);
}
