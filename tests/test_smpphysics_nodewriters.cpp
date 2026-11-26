// Unit tests for the record of which claim last wrote each node (src/smp/NodeWriters.h).
// Pure logic; no game/D3D dependency.

#include "smp/NodeWriters.h"

#include <catch2/catch_test_macros.hpp>

#include <cstdint>
#include <map>
#include <set>
#include <vector>

using namespace SmpPhysics;

namespace
{
	struct Scene
	{
		std::map<std::uint64_t, std::vector<int>> claims;  ///< token -> indices into nodes
		int nodes[4]{};
		std::uint64_t serial{};
		int folds{};

		bool End(NodeWriters& a_writers)
		{
			return a_writers.EndFrame(
				serial, [&](std::uint64_t a_token) { return claims.contains(a_token); },
				[&](std::uint64_t a_token, auto&& a_visit) {
					++folds;
					for (const int node : claims.at(a_token))
						a_visit(&nodes[node]);
				});
		}
	};
}

TEST_CASE("The last claim to write a node in a frame holds it", "[smpphysics][nodewriters]")
{
	Scene scene;
	scene.claims = { { 7, { 0, 1 } }, { 9, { 1, 2 } } };
	scene.serial = 1;
	NodeWriters writers;
	writers.Wrote(7);
	writers.Wrote(9);
	REQUIRE(scene.End(writers));
	REQUIRE(*writers.Find(&scene.nodes[0]) == 7);
	REQUIRE(*writers.Find(&scene.nodes[1]) == 9);
	REQUIRE(*writers.Find(&scene.nodes[2]) == 9);
	REQUIRE(writers.Find(&scene.nodes[3]) == nullptr);
}

TEST_CASE("A frame with the same writers and claims is not folded again", "[smpphysics][nodewriters]")
{
	Scene scene;
	scene.claims = { { 7, { 0, 1 } }, { 9, { 1, 2 } } };
	scene.serial = 1;
	NodeWriters writers;
	writers.Wrote(7);
	writers.Wrote(9);
	scene.End(writers);
	const int folds = scene.folds;

	writers.Wrote(7);
	writers.Wrote(9);
	REQUIRE_FALSE(scene.End(writers));
	REQUIRE(scene.folds == folds);

	// The other order hands the shared node to the other claim.
	writers.Wrote(9);
	writers.Wrote(7);
	REQUIRE(scene.End(writers));
	REQUIRE(*writers.Find(&scene.nodes[1]) == 7);
}

TEST_CASE("A claim that stops writing keeps its nodes until another writes them or it is removed", "[smpphysics][nodewriters]")
{
	Scene scene;
	scene.claims = { { 7, { 0, 1 } }, { 9, { 1, 2 } } };
	scene.serial = 1;
	NodeWriters writers;
	writers.Wrote(7);
	writers.Wrote(9);
	scene.End(writers);

	writers.Wrote(7);
	REQUIRE(scene.End(writers));
	REQUIRE(*writers.Find(&scene.nodes[1]) == 7);
	REQUIRE(*writers.Find(&scene.nodes[2]) == 9);

	scene.claims.erase(9);
	++scene.serial;
	writers.Wrote(7);
	REQUIRE(scene.End(writers));
	REQUIRE(writers.Find(&scene.nodes[2]) == nullptr);
	REQUIRE(*writers.Find(&scene.nodes[0]) == 7);
}

TEST_CASE("A claim replaced under its token is folded again", "[smpphysics][nodewriters]")
{
	Scene scene;
	scene.claims = { { 7, { 0 } } };
	scene.serial = 1;
	NodeWriters writers;
	writers.Wrote(7);
	scene.End(writers);

	scene.claims[7] = { 3 };
	++scene.serial;
	writers.Wrote(7);
	REQUIRE(scene.End(writers));
	REQUIRE(*writers.Find(&scene.nodes[3]) == 7);
}
