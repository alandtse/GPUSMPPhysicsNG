// Unit tests for the collision debugging switches (src/smp/CollisionOverride.cpp). Pure data; no game or D3D dependency.

#include "smp/CollisionOverride.h"

#include <catch2/catch_test_macros.hpp>

using namespace SmpPhysics;

TEST_CASE("A collider switch exempts every sphere of the named piece only", "[smpphysics][collisionoverride]")
{
	std::vector<std::vector<BoneSphere>> spheres(3, std::vector<BoneSphere>(2));
	std::vector<std::uint32_t> mask;
	const std::vector<std::string> pieces{ "skirt", "tassel", "skirt" };

	const std::size_t changed = ApplyCollisionOverrides({ { "skirt", kOverrideColliders } }, pieces, {}, spheres, mask);

	CHECK(changed == 2);
	CHECK(spheres[0][1].exemptMask == 0xFFFFFFFFu);
	CHECK(spheres[1][0].exemptMask == 0u);
	CHECK(spheres[2][0].exemptMask == 0xFFFFFFFFu);
	CHECK(mask.empty());
}

TEST_CASE("A surface switch clears that surface's bit for the named piece, creating the mask when absent", "[smpphysics][collisionoverride]")
{
	std::vector<std::vector<BoneSphere>> spheres(2);
	std::vector<std::uint32_t> mask;
	const std::vector<std::string> pieces{ "tassel", "skirt" };
	const std::vector<std::string> surfaces{ "Skirtpart1", "Skirtpart2" };

	const std::size_t changed = ApplyCollisionOverrides({ { "tassel", "Skirtpart2" } }, pieces, surfaces, spheres, mask);

	CHECK(changed == 1);
	REQUIRE(mask.size() == 2);
	CHECK(mask[0] == 0xFFFFFFFDu);
	CHECK(mask[1] == 0xFFFFFFFFu);
}

TEST_CASE("A switch naming no piece or no surface changes nothing", "[smpphysics][collisionoverride]")
{
	std::vector<std::vector<BoneSphere>> spheres(1, std::vector<BoneSphere>(1));
	std::vector<std::uint32_t> mask;

	CHECK(ApplyCollisionOverrides({ { "missing", kOverrideColliders }, { "skirt", "missing" } }, { "skirt" }, { "Skirtpart1" }, spheres, mask) == 0);
	CHECK(spheres[0][0].exemptMask == 0u);
}

TEST_CASE("A surface switch clears that surface's bit on the named piece's cloth triangles", "[smpphysics][collisionoverride]")
{
	ClothTriangleSet cloth;
	cloth.triangles.resize(5);
	cloth.pieces = { { "skirt", 0u, 2u }, { "tassel", 2u, 3u } };
	const std::vector<std::string> surfaces{ "Body", "Legs" };

	CHECK(ApplyClothCollisionOverrides({ { "tassel", "Legs" }, { "tassel", kOverrideColliders }, { "veil", "Body" } }, surfaces, cloth) == 1);
	CHECK(cloth.triangles[1].surfaceMask == 0xFFFFFFFFu);
	for (std::size_t triangle = 2; triangle < 5; ++triangle)
		CHECK(cloth.triangles[triangle].surfaceMask == 0xFFFFFFFDu);
}
