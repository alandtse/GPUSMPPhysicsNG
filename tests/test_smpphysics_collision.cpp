// Unit tests for the analytic collision proxies and their contact response
// (src/smp/Collision.cpp and the diagnostic probe in SolverCommon.cpp).
// Pure maths; no game/D3D dependency.

#include "smp/Collision.h"
#include "smp/SkinnedSurface.h"
#include "smp/SolverCommon.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <string_view>
#include <vector>

using namespace SmpPhysics;
using Catch::Approx;

namespace
{

	constexpr std::string_view kChainXml = R"XML(
<system>
	<bone name="Head"><mass>0</mass></bone>
	<bone-default name="hair">
		<mass>0.1</mass>
		<inertia x="16" y="16" z="16"/>
		<angularDamping>0.75</angularDamping>
	</bone-default>
	<bone name="Hair01" template="hair"/>
	<bone name="Hair02" template="hair"/>
	<generic-constraint-default name="hair-joint">
		<frameInB>
			<basis x="0" y="0" z="0" w="1"/>
			<origin x="0" y="0" z="0"/>
		</frameInB>
		<linearLowerLimit x="0" y="0" z="0"/>
		<linearUpperLimit x="0" y="0" z="0"/>
		<angularLowerLimit x="-3" y="-3" z="-3"/>
		<angularUpperLimit x="3" y="3" z="3"/>
		<angularStiffness x="0" y="0" z="0"/>
		<angularDamping x="0" y="0" z="0"/>
	</generic-constraint-default>
	<constraint-group>
		<generic-constraint bodyA="Hair01" bodyB="Head" template="hair-joint"/>
		<generic-constraint bodyA="Hair02" bodyB="Hair01" template="hair-joint"/>
	</constraint-group>
</system>)XML";

	/** @brief A chain of two bones lying along +X from a stationary head, so gravity swings it down onto a plane. */
	struct Fixture
	{
		SmpRig rig;
		std::vector<Transform> rest;
		std::vector<Transform> kinematic;
		std::vector<std::vector<BoneSphere>> spheres;
	};

	Fixture MakeFixture()
	{
		Fixture fixture;
		const ParseResult result = ParseSmpXml(kChainXml);
		if (result.status == ParseStatus::Ok)
			fixture.rig = result.rig;
		fixture.rest = {
			{ Quat{}, Vec3{ 0.0f, 0.0f, 100.0f } },
			{ Quat{}, Vec3{ 20.0f, 0.0f, 100.0f } },
			{ Quat{}, Vec3{ 40.0f, 0.0f, 100.0f } }
		};
		fixture.kinematic = fixture.rest;
		fixture.spheres.resize(fixture.rig.bones.size());
		for (std::size_t bone = 1; bone < fixture.rig.bones.size(); ++bone)
			fixture.spheres[bone].push_back({ Vec3{}, 1.0f });
		return fixture;
	}

	SkinnedVertex WeightedVertex(const Vec3& a_position, int a_bone, float a_weight)
	{
		SkinnedVertex vertex;
		vertex.position = a_position;
		vertex.bone[0] = a_bone;
		vertex.weight[0] = a_weight;
		return vertex;
	}
}

TEST_CASE("Hair spheres are one per vertex of a dynamic bone's cluster", "[SmpPhysics]")
{
	std::vector<Transform> meshToBone(3);
	for (int bone = 0; bone < 3; ++bone)
		meshToBone[static_cast<std::size_t>(bone)].translation = { -10.0f * static_cast<float>(bone), 0.0f, 0.0f };
	const std::vector<bool> dynamic = { false, true, true };

	std::vector<SkinnedVertex> vertices;
	for (int index = 0; index < 10; ++index)
		vertices.push_back(WeightedVertex({ 20.0f, static_cast<float>(index), 0.0f }, 1, 1.0f));
	// A lightly weighted vertex still joins its bone, and one weighted to a kinematic bone joins nothing.
	vertices.push_back(WeightedVertex({ 100.0f, 0.0f, 0.0f }, 1, 0.4f));
	vertices.push_back(WeightedVertex({ -100.0f, 0.0f, 0.0f }, 0, 1.0f));

	const std::vector<std::vector<BoneSphere>> spheres = ReduceHairSpheres(vertices, meshToBone, dynamic, 0.01f);

	REQUIRE(spheres.size() == 3);
	REQUIRE(spheres[0].empty());
	REQUIRE(spheres[2].empty());
	REQUIRE(spheres[1].size() == 11);
	for (const BoneSphere& sphere : spheres[1]) {
		REQUIRE(sphere.radius == Approx(0.01f));
		REQUIRE(sphere.centerLocal.z == Approx(0.0f).margin(1e-4f));
	}
	// Bone 1's frame sits at x = 10, so it takes a vertex at x = 20 to x = 10.
	REQUIRE(spheres[1].front().centerLocal.x == Approx(10.0f).margin(1e-4f));

	// A vertex split over two dynamic bones is a sphere of each.
	SkinnedVertex split = WeightedVertex({ 20.0f, 0.0f, 0.0f }, 1, 0.5f);
	split.bone[1] = 2;
	split.weight[1] = 0.5f;
	const std::vector<std::vector<BoneSphere>> shared = ReduceHairSpheres({ split }, meshToBone, dynamic, 0.01f);
	REQUIRE(shared[1].size() == 1);
	REQUIRE(shared[2].size() == 1);
}

TEST_CASE("A cluster past the cap is thinned evenly", "[SmpPhysics]")
{
	const std::vector<Transform> meshToBone(1);
	const std::vector<bool> dynamic = { true };
	std::vector<SkinnedVertex> many;
	for (int index = 0; index < 2 * kMaxContactSpheresPerBone; ++index)
		many.push_back(WeightedVertex({ 0.0f, 0.0f, static_cast<float>(index) }, 0, 1.0f));

	const std::vector<std::vector<BoneSphere>> thinned = ReduceHairSpheres(many, meshToBone, dynamic, 0.01f);
	REQUIRE(thinned[0].size() == static_cast<std::size_t>(kMaxContactSpheresPerBone));
	REQUIRE(thinned[0].front().centerLocal.z == Approx(0.0f));
	REQUIRE(thinned[0].back().centerLocal.z == Approx(static_cast<float>(2 * kMaxContactSpheresPerBone - 2)));
}

TEST_CASE("Contacts of one bone merge by cubed depth into a single contact", "[SmpPhysics]")
{
	ContactMerge merge;
	Vec3 normal{};
	Vec3 position{};
	REQUIRE(merge.Result(normal, position) == 0.0f);

	merge.Add({ 1.0f, { 0.0f, 0.0f, 1.0f }, 3 }, { 0.0f, 0.0f, 0.0f });
	REQUIRE(merge.Result(normal, position) == Approx(1.0f));

	// Depth 3 weighs 9 against 1, so the mean depth is (27 + 1) / (9 + 1) and sits near the deeper contact.
	merge.Add({ 3.0f, { 0.0f, 0.0f, 1.0f }, 4 }, { 10.0f, 0.0f, 0.0f });
	REQUIRE(merge.Result(normal, position) == Approx(28.0f / 10.0f));
	REQUIRE(normal.z == Approx(1.0f));
	REQUIRE(position.x == Approx(9.0f));
	REQUIRE(merge.deepest == Approx(3.0f));
	REQUIRE(merge.collider == 4);

	// A touch that does not penetrate adds nothing.
	merge.Add({ 0.0f, { 0.0f, 0.0f, 1.0f }, 5 }, { 99.0f, 0.0f, 0.0f });
	REQUIRE(merge.Result(normal, position) == Approx(28.0f / 10.0f));
}

TEST_CASE("A contact against the merged direction is dropped, as hdtSMP64 does", "[SmpPhysics]")
{
	ContactMerge merge;
	merge.Add({ 1.0f, { 0.0f, 0.0f, 1.0f }, 3 }, { 0.0f, 0.0f, 0.0f });
	merge.Add({ 5.0f, { 0.0f, 0.0f, -1.0f }, 4 }, { 9.0f, 0.0f, 0.0f });

	Vec3 normal{};
	Vec3 position{};
	REQUIRE(merge.Result(normal, position) == Approx(1.0f));
	REQUIRE(normal.z == Approx(1.0f));
	REQUIRE(position.x == Approx(0.0f));
	REQUIRE(merge.deepest == Approx(1.0f));
}

TEST_CASE("Contacts of one key fill lane 0 exactly as the single merge does", "[SmpPhysics]")
{
	ContactMerge merge;
	ContactLanes lanes;
	const Contact contacts[] = { { 1.0f, { 0.0f, 0.6f, 0.8f }, 3 }, { 0.0f, { 0.0f, 0.0f, 1.0f }, 6 }, { 3.0f, { 0.0f, 0.0f, 1.0f }, 4 }, { 2.0f, { 0.0f, 0.0f, -1.0f }, 5 } };
	float offset = 0.0f;
	for (const Contact& contact : contacts) {
		const Vec3 position{ offset, 1.0f, -2.0f };
		merge.Add(contact, position);
		lanes.Add(contact, position, 7u);
		offset += 2.5f;
	}
	REQUIRE(lanes.used == 1u);
	REQUIRE(lanes.keys[0] == 7u);
	const ContactMerge& first = lanes.lanes[0];
	CHECK(first.weight == merge.weight);
	CHECK(first.normal.x == merge.normal.x);
	CHECK(first.normal.y == merge.normal.y);
	CHECK(first.normal.z == merge.normal.z);
	CHECK(first.position.x == merge.position.x);
	CHECK(first.position.y == merge.position.y);
	CHECK(first.position.z == merge.position.z);
	CHECK(first.deepest == merge.deepest);
	CHECK(first.collider == merge.collider);
	for (std::size_t lane = 1; lane < kContactLanes; ++lane) {
		CHECK(lanes.lanes[lane].weight == 0.0f);
		CHECK(lanes.lanes[lane].collider == -1);
	}
}

TEST_CASE("Contacts of different keys stay apart, and opposing ones no longer cancel", "[SmpPhysics]")
{
	ContactLanes lanes;
	lanes.Add({ 1.0f, { 0.0f, 0.0f, 1.0f }, 3 }, { 0.0f, 0.0f, 0.0f }, 1u);
	lanes.Add({ 5.0f, { 0.0f, 0.0f, -1.0f }, 4 }, { 9.0f, 0.0f, 0.0f }, 2u);
	lanes.Add({ 2.0f, { 0.0f, 0.0f, 1.0f }, 5 }, { 3.0f, 0.0f, 0.0f }, 1u);
	REQUIRE(lanes.used == 2u);

	Vec3 normal{};
	Vec3 position{};
	REQUIRE(lanes.lanes[0].Result(normal, position) == Approx(9.0f / 5.0f));
	REQUIRE(normal.z == Approx(1.0f));
	REQUIRE(lanes.lanes[0].collider == 5);
	REQUIRE(lanes.lanes[1].Result(normal, position) == Approx(5.0f));
	REQUIRE(normal.z == Approx(-1.0f));
	REQUIRE(position.x == Approx(9.0f));
	REQUIRE(lanes.lanes[1].collider == 4);
}

TEST_CASE("A key past the last lane joins the lane it is best aligned with", "[SmpPhysics]")
{
	ContactLanes lanes;
	float total = 0.0f;
	const auto add = [&lanes, &total](float a_depth, const Vec3& a_normal, std::uint32_t a_key) {
		lanes.Add({ a_depth, a_normal, static_cast<int>(a_key) }, { static_cast<float>(a_key), 0.0f, 0.0f }, a_key);
		total += a_depth * a_depth;
	};
	// The deepest lane has the longest normal sum, which must not win over the lane that actually faces the contact.
	add(4.0f, { 1.0f, 0.0f, 0.0f }, 10u);
	add(1.0f, { 0.0f, 1.0f, 0.0f }, 11u);
	add(1.0f, { 0.0f, 0.0f, 1.0f }, 12u);
	add(1.0f, { -1.0f, 0.0f, 0.0f }, 13u);
	REQUIRE(lanes.used == kContactLanes);

	add(2.0f, { 0.6f, 0.8f, 0.0f }, 14u);
	add(3.0f, { 0.0f, 0.0f, 1.0f }, 15u);
	CHECK(lanes.used == kContactLanes);
	CHECK(lanes.lanes[0].weight == Approx(16.0f));
	CHECK(lanes.lanes[1].weight == Approx(1.0f + 4.0f));
	CHECK(lanes.lanes[1].collider == 14);
	CHECK(lanes.lanes[2].weight == Approx(1.0f + 9.0f));
	CHECK(lanes.lanes[3].weight == Approx(1.0f));

	float kept = 0.0f;
	for (const ContactMerge& lane : lanes.lanes)
		kept += lane.weight;
	CHECK(kept == Approx(total));
}

TEST_CASE("Hair sphere radius is the shape margin, never zero", "[SmpPhysics]")
{
	const std::vector<Transform> meshToBone(1);
	const std::vector<bool> dynamic = { true };
	std::vector<SkinnedVertex> spread;
	for (int index = 0; index < 8; ++index)
		spread.push_back(WeightedVertex({ 0.0f, 0.0f, 100.0f * static_cast<float>(index) }, 0, 1.0f));

	const std::vector<std::vector<BoneSphere>> margined = ReduceHairSpheres(spread, meshToBone, dynamic, 0.8f);
	REQUIRE(margined[0].size() == 8);
	for (const BoneSphere& sphere : margined[0])
		REQUIRE(sphere.radius == Approx(0.8f));

	for (const BoneSphere& sphere : ReduceHairSpheres(spread, meshToBone, dynamic, 0.0f)[0])
		REQUIRE(sphere.radius > 0.0f);
}

TEST_CASE("A hair sphere carries its vertex's other bones, weights and blended margin", "[SmpPhysics]")
{
	std::vector<Transform> meshToBone(2);
	meshToBone[1].translation = { 0.0f, 0.0f, -5.0f };
	const std::vector<bool> dynamic = { true, false };
	VertexBlend blend;
	blend.ownerWeight = 0.75f;
	blend.flexible = 0.75f;
	blend.marginMultiplier = 2.0f;
	blend.others[0] = { 1, { 1.0f, 2.0f, 3.0f }, 0.25f };
	const std::vector<std::vector<BoneSphere>> spheres = ReduceHairSpheres({ WeightedVertex({ 1.0f, 2.0f, 3.0f }, 0, 1.0f) }, meshToBone, dynamic, 0.5f, {}, { blend });
	REQUIRE(spheres[0].size() == 1);
	const BoneSphere& sphere = spheres[0][0];
	CHECK(sphere.weight == Approx(0.75f));
	CHECK(sphere.flexible == Approx(0.75f));
	CHECK(sphere.radius == Approx(1.0f));
	REQUIRE(sphere.others[0].bone == 1);
	CHECK(sphere.others[0].weight == Approx(0.25f));
	CHECK(sphere.others[0].local.z == Approx(-2.0f));
	CHECK(sphere.others[1].bone == -1);
}

TEST_CASE("A head ellipsoid follows the vertex cloud's principal axes", "[SmpPhysics]")
{
	// A box 20 x 6 x 2 long, so its long axis is X.
	std::vector<Vec3> vertices;
	for (float x = -10.0f; x <= 10.0f; x += 1.0f) {
		for (float y = -3.0f; y <= 3.0f; y += 1.0f) {
			for (float z = -1.0f; z <= 1.0f; z += 1.0f)
				vertices.push_back({ x, y, z });
		}
	}

	const Collider collider = FitHeadEllipsoid(vertices, 2);
	REQUIRE(collider.type == Collider::Type::Ellipsoid);
	REQUIRE(collider.driverIndex == 2);
	REQUIRE(collider.center.x == Approx(0.0f).margin(1e-3f));
	REQUIRE(collider.center.y == Approx(0.0f).margin(1e-3f));
	REQUIRE(collider.center.z == Approx(0.0f).margin(1e-3f));
	// The long axis first, and no half-extent past the cloud's own extent on that axis.
	REQUIRE(collider.halfExtents.x > collider.halfExtents.y);
	REQUIRE(collider.halfExtents.y > collider.halfExtents.z);
	REQUIRE(collider.halfExtents.x <= 10.0f + 1e-4f);
	REQUIRE(collider.halfExtents.x == Approx(9.5f).margin(0.6f));
	REQUIRE(collider.halfExtents.y == Approx(3.0f).margin(0.1f));
	REQUIRE(IsFiniteVector(collider.halfExtents));

	// The orientation is the axes' rotation, so it maps the local long axis onto the cloud's.
	const Vec3 longAxis = collider.orientation.Rotate(Vec3{ 1.0f, 0.0f, 0.0f });
	REQUIRE(std::abs(longAxis.x) == Approx(1.0f).margin(1e-3f));

	const Collider fallback = FitHeadEllipsoid({ { 0.0f, 0.0f, 0.0f }, { 1.0f, 0.0f, 0.0f } }, 1);
	REQUIRE(fallback.halfExtents.x == Approx(kDefaultHeadRadius));
	REQUIRE(fallback.halfExtents.y == Approx(kDefaultHeadRadius));

	// A cloud with no spread at all still gives a usable proxy rather than a zero-sized one.
	const Vec3 repeated{ 4.0f, 4.0f, 4.0f };
	const Collider flat = FitHeadEllipsoid({ repeated, repeated, repeated, repeated }, 0);
	REQUIRE(flat.halfExtents.x > 0.0f);
	REQUIRE(IsFiniteVector(flat.halfExtents));
}

TEST_CASE("A segment radius follows the 90th percentile of the body vertices it selects", "[SmpPhysics]")
{
	const Vec3 endA{ 0.0f, 0.0f, 0.0f };
	const Vec3 endB{ 0.0f, 0.0f, -10.0f };
	constexpr float kBodyRadius = 5.0f;
	constexpr float kArmorRadius = 7.0f;
	constexpr float kDefaultRadius = 2.0f;

	// A cylinder of 200 vertices around the segment, each skinned to it.
	std::vector<SegmentVertex> cylinder;
	for (int ring = 0; ring < 20; ++ring) {
		for (int step = 0; step < 10; ++step) {
			const float angle = 0.31415927f * static_cast<float>(ring);
			const float z = -static_cast<float>(step);
			cylinder.push_back({ { kBodyRadius * std::cos(angle), kBodyRadius * std::sin(angle), z }, true });
		}
	}

	const SegmentFit measured = FitSegmentRadius(cylinder, endA, endB, kDefaultRadius);
	REQUIRE(measured.fitted);
	REQUIRE(measured.vertexCount == 200);
	REQUIRE(measured.radius == Approx(kBodyRadius + kBodyArmorMargin).margin(1e-3f));

	// One percent of the vertices thrown far out does not move the percentile away from the body's own surface.
	std::vector<SegmentVertex> strayed = cylinder;
	strayed.push_back({ { 60.0f, 0.0f, -5.0f }, true });
	strayed.push_back({ { -60.0f, 0.0f, -5.0f }, true });
	const SegmentFit robust = FitSegmentRadius(strayed, endA, endB, kDefaultRadius);
	REQUIRE(robust.fitted);
	REQUIRE(robust.radius == Approx(kBodyRadius + kBodyArmorMargin).margin(1e-3f));

	// A cuirass sits two units outside the naked body all round, so the capsule has to follow it out.
	std::vector<SegmentVertex> armoured;
	for (int ring = 0; ring < 20; ++ring) {
		const float angle = 0.31415927f * static_cast<float>(ring);
		const float radius = ring < 10 ? kBodyRadius : kArmorRadius;
		armoured.push_back({ { radius * std::cos(angle), radius * std::sin(angle), -5.0f }, true });
	}
	const SegmentFit shell = FitSegmentRadius(armoured, endA, endB, kDefaultRadius);
	REQUIRE(shell.fitted);
	REQUIRE(shell.radius == Approx(kArmorRadius + kBodyArmorMargin).margin(1e-3f));

	// Only a vertex skinned to the segment and projecting within the slack of its ends is measured.
	const std::vector<SegmentVertex> beyondReach = {
		{ { 0.0f, 0.0f, -5.0f }, false }, { { 0.0f, 0.0f, -40.0f }, true }, { { 3.0f, 0.0f, -11.0f }, true }
	};
	REQUIRE(FitSegmentRadius(beyondReach, endA, endB, kDefaultRadius).vertexCount == 1);

	// A segment nothing measured keeps its default and says so.
	const std::vector<SegmentVertex> sparse = { { { 1.0f, 0.0f, 0.0f }, true }, { { 2.0f, 0.0f, 0.0f }, true } };
	const SegmentFit untouched = FitSegmentRadius(sparse, endA, endB, kDefaultRadius);
	REQUIRE_FALSE(untouched.fitted);
	REQUIRE(untouched.radius == Approx(kDefaultRadius));
	REQUIRE(untouched.vertexCount == 2);
	REQUIRE(FitSegmentRadius({}, endA, endB, kDefaultRadius).radius == Approx(kDefaultRadius));

	// The default is also the floor, and no misread mesh may widen a capsule past the ceiling.
	const SegmentFit inside = FitSegmentRadius(cylinder, endA, endB, kBodyRadius + 10.0f);
	REQUIRE(inside.fitted);
	REQUIRE(inside.radius == Approx(kBodyRadius + 10.0f));
	const SegmentFit huge = FitSegmentRadius({ { { 500.0f, 0.0f, -5.0f }, true } }, endA, endB, kDefaultRadius);
	REQUIRE_FALSE(huge.fitted);
}

TEST_CASE("A vertex's dominant bone is its heaviest one above the weight threshold", "[SmpPhysics]")
{
	SkinnedVertex vertex = WeightedVertex({ 0.0f, 0.0f, 0.0f }, 0, 1.0f);
	REQUIRE(DominantBone(vertex, 3) == 0);

	vertex.bone[0] = 2;
	vertex.weight[0] = 0.4f;
	vertex.bone[1] = 1;
	vertex.weight[1] = 0.6f;
	REQUIRE(DominantBone(vertex, 3) == 1);

	// A weight at the threshold counts, and an index past the bone count does not.
	vertex.weight[1] = Util::kMinDominantBoneWeight;
	REQUIRE(DominantBone(vertex, 3) == 1);
	REQUIRE(DominantBone(vertex, 1) == -1);
	REQUIRE(DominantBone(WeightedVertex({}, 0, 0.4f), 3) == -1);
	REQUIRE(DominantBone(SkinnedVertex{}, 3) == -1);
}

TEST_CASE("Skinning blends a vertex between its bones' world transforms", "[SmpPhysics]")
{
	const std::vector<Transform> skinToBone = { Transform{}, Transform{} };
	std::vector<Transform> boneToWorld(2);
	boneToWorld[0].translation = { 0.0f, 0.0f, 0.0f };
	boneToWorld[1].translation = { 10.0f, 0.0f, 0.0f };

	SkinnedVertex vertex;
	vertex.position = { 2.0f, 0.0f, 0.0f };
	vertex.bone[0] = 0;
	vertex.weight[0] = 0.5f;
	vertex.bone[1] = 1;
	vertex.weight[1] = 0.5f;
	// Halfway between a bone at the origin and one at x = 10, both with mesh space and bone space aligned.
	Vec3 world = SkinVertexToWorld(vertex, boneToWorld, skinToBone);
	REQUIRE(world.x == Approx(7.0f).margin(1e-4f));
	REQUIRE(world.y == Approx(0.0f).margin(1e-4f));
	REQUIRE(world.z == Approx(0.0f).margin(1e-4f));

	// The bone's own world transform turns the vertex about its own origin before placing it, so the
	// second bone contributes (10, 2, 0) rather than the (12, 0, 0) an origin-first rotation would give.
	boneToWorld[1].rotation = FromAxisAngle(Vec3{ 0.0f, 0.0f, 1.0f }, 1.5707963f);
	world = SkinVertexToWorld(vertex, boneToWorld, skinToBone);
	REQUIRE(world.x == Approx(6.0f).margin(1e-4f));
	REQUIRE(world.y == Approx(1.0f).margin(1e-4f));

	// A slot with no weight, and one naming a bone that is not there, leave the rest of the blend alone.
	vertex.weight[1] = 0.0f;
	world = SkinVertexToWorld(vertex, boneToWorld, skinToBone);
	REQUIRE(world.x == Approx(2.0f).margin(1e-4f));
	REQUIRE(world.y == Approx(0.0f).margin(1e-4f));
	vertex.bone[0] = 9;
	REQUIRE(SkinVertexToWorld(vertex, boneToWorld, skinToBone).x == Approx(2.0f).margin(1e-4f));

	// Weights are half floats and rarely sum to one, so the blend normalises what it was given.
	vertex.bone[0] = 1;
	vertex.weight[0] = 0.25f;
	REQUIRE(SkinVertexToWorld(vertex, boneToWorld, skinToBone).x == Approx(10.0f).margin(1e-4f));
}

TEST_CASE("Collider queries give finite distances and outward normals", "[SmpPhysics]")
{
	const std::vector<Transform> drivers = { Transform{} };

	Collider ellipsoid = MakeDefaultHeadCollider(0);
	ellipsoid.halfExtents = { 4.0f, 6.0f, 8.0f };
	REQUIRE(QueryColliders({ ellipsoid }, drivers, Vec3{ 15.0f, 0.0f, 0.0f }, 1.0f).collider == -1);
	const Contact touching = QueryColliders({ ellipsoid }, drivers, Vec3{ 4.5f, 0.0f, 0.0f }, 1.0f);
	REQUIRE(touching.collider == 0);
	REQUIRE(touching.depth == Approx(0.5f).margin(1e-3f));
	REQUIRE(touching.normal.x == Approx(1.0f).margin(1e-4f));
	REQUIRE(touching.normal.y == Approx(0.0f).margin(1e-4f));
	// At the centre the radial direction is gone, but the normal still has to be a unit vector.
	const Contact centred = QueryColliders({ ellipsoid }, drivers, Vec3{ 0.0f, 0.0f, 0.0f }, 1.0f);
	REQUIRE(centred.depth == Approx(1.0f + 4.0f).margin(1e-3f));
	REQUIRE(Length(centred.normal) == Approx(1.0f).margin(1e-4f));
	REQUIRE(IsFiniteVector(centred.normal));

	Collider capsule = MakeDefaultCapsule(Vec3{ 0.0f, -5.0f, 0.0f }, Vec3{ 0.0f, 5.0f, 0.0f }, 2.0f, 0);
	REQUIRE(QueryColliders({ capsule }, drivers, Vec3{ 9.0f, 0.0f, 0.0f }, 1.0f).collider == -1);
	const Contact beside = QueryColliders({ capsule }, drivers, Vec3{ 2.5f, 0.0f, 0.0f }, 1.0f);
	REQUIRE(beside.depth == Approx(0.5f).margin(1e-4f));
	REQUIRE(beside.normal.x == Approx(1.0f).margin(1e-4f));
	// On the segment's own axis the offset has no direction, so the normal has to be synthesized.
	const Contact onAxis = QueryColliders({ capsule }, drivers, Vec3{ 0.0f, 0.0f, 0.0f }, 1.0f);
	REQUIRE(onAxis.depth == Approx(3.0f).margin(1e-4f));
	REQUIRE(Length(onAxis.normal) == Approx(1.0f).margin(1e-4f));
	REQUIRE(IsFiniteVector(onAxis.normal));

	const Collider plane = MakeGroundPlane(Vec3{ 0.0f, 0.0f, 20.0f }, Vec3{ 0.0f, 0.0f, 1.0f }, 0);
	{
		const Contact clear = QueryColliders({ plane }, drivers, Vec3{ 0.0f, 0.0f, 25.0f }, 1.0f);
		REQUIRE(clear.collider == -1);
		REQUIRE(clear.depth == Approx(0.0f));
	}
	const Contact onPlane = QueryColliders({ plane }, drivers, Vec3{ 0.0f, 0.0f, 20.5f }, 1.0f);
	REQUIRE(onPlane.depth == Approx(0.5f).margin(1e-4f));
	REQUIRE(onPlane.normal.z == Approx(1.0f).margin(1e-4f));
	REQUIRE(QueryColliders({ plane }, drivers, Vec3{ 0.0f, 0.0f, 19.0f }, 1.0f).depth == Approx(2.0f).margin(1e-4f));

	// A collider is expressed in its driver's frame, so moving the driver moves it.
	std::vector<Transform> moved = { Transform{} };
	moved[0].translation = { 0.0f, 0.0f, 10.0f };
	REQUIRE(QueryColliders({ plane }, moved, Vec3{ 0.0f, 0.0f, 31.0f }, 1.0f).depth == Approx(0.0f).margin(1e-4f));

	// A collider whose driver was not supplied is skipped rather than read out of range.
	Collider orphan = plane;
	orphan.driverIndex = 7;
	REQUIRE(QueryColliders({ orphan }, drivers, Vec3{ 0.0f, 0.0f, 20.0f }, 1.0f).collider == -1);
	REQUIRE(QueryColliders({ plane }, drivers, Vec3{ 0.0f, 0.0f, std::nanf("") }, 1.0f).collider == -1);
}

TEST_CASE("Rest exemptions mask the colliders a sphere is authored inside", "[SmpPhysics]")
{
	// A capsule along +Z of radius 5, and a head ellipsoid far above it, so a sphere can be inside one alone.
	Collider head = MakeDefaultHeadCollider(1);
	head.center = Vec3{ 0.0f, 0.0f, 50.0f };
	const std::vector<Collider> colliders = {
		MakeDefaultCapsule(Vec3{ 0.0f, 0.0f, -10.0f }, Vec3{ 0.0f, 0.0f, 10.0f }, 5.0f, 0),
		head
	};
	const std::vector<Transform> drivers = { Transform{}, Transform{} };
	const std::vector<Transform> boneRest = {
		{ Quat{}, Vec3{ 0.0f, 0.0f, -0.5f } },   // five units inside the capsule
		{ Quat{}, Vec3{ 5.95f, 0.0f, -0.5f } },  // only 0.05 deep, within the capsule's own allowance
		{ Quat{}, Vec3{ 10.0f, 0.0f, -0.5f } },  // clear of it
		{ Quat{}, Vec3{ 0.0f, 0.0f, 50.0f } }    // inside the ellipsoid, clear of the capsule
	};
	std::vector<std::vector<BoneSphere>> spheres(boneRest.size());
	for (std::vector<BoneSphere>& bone : spheres)
		bone.push_back({ Vec3{}, 1.0f });

	ComputeRestExemptions(spheres, boneRest, colliders, drivers);
	REQUIRE(spheres[0][0].exemptMask == 0x1u);  // the capsule owns bit 0
	REQUIRE(spheres[1][0].exemptMask == 0x0u);
	REQUIRE(spheres[2][0].exemptMask == 0x0u);
	REQUIRE(spheres[3][0].exemptMask == 0x2u);  // the ellipsoid owns bit 1

	// A proxy whose driver no transform covers is skipped, as QueryColliders skips it.
	std::vector<std::vector<BoneSphere>> orphaned(1);
	orphaned[0].push_back({ Vec3{}, 1.0f });
	ComputeRestExemptions(orphaned, { boneRest[0] }, { MakeGroundPlane(Vec3{}, Vec3{ 0.0f, 0.0f, 1.0f }, -1) }, drivers);
	REQUIRE(orphaned[0][0].exemptMask == 0x0u);

	// Only the first kMaximumExemptColliders proxies can be named, so a sphere is exempt from the first 32 of
	// them and from none at all when the only proxy it rests inside sits past that cap.
	const std::vector<Transform> above = { { Quat{}, Vec3{ 0.0f, 0.0f, 999.5f } } };
	std::vector<Collider> reachable;
	std::vector<Collider> beyondCap;
	for (std::size_t index = 0; index < kMaximumExemptColliders + 8; ++index) {
		reachable.push_back(MakeGroundPlane(Vec3{ 0.0f, 0.0f, index < kMaximumExemptColliders ? 1000.0f : -1000.0f }, Vec3{ 0.0f, 0.0f, 1.0f }, 0));
		beyondCap.push_back(MakeGroundPlane(Vec3{ 0.0f, 0.0f, index == 35 ? 1000.0f : -1000.0f }, Vec3{ 0.0f, 0.0f, 1.0f }, 0));
	}
	std::vector<std::vector<BoneSphere>> capped(1);
	capped[0].push_back({ Vec3{}, 1.0f });
	ComputeRestExemptions(capped, above, reachable, drivers);
	REQUIRE(capped[0][0].exemptMask == 0xFFFFFFFFu);

	std::vector<std::vector<BoneSphere>> uncapped(1);
	uncapped[0].push_back({ Vec3{}, 1.0f });
	ComputeRestExemptions(uncapped, above, beyondCap, drivers);
	REQUIRE(uncapped[0][0].exemptMask == 0x0u);
}

namespace
{
	/** @brief A flat quad at a_height facing +Z, skinned to driver 0 so it stays put under an identity driver. */
	SkinnedSurface MakeFloorSurface(float a_height)
	{
		SkinnedSurface surface;
		surface.name = "floor";
		for (const Vec3& corner : { Vec3{ -10.0f, -10.0f, a_height }, Vec3{ 60.0f, -10.0f, a_height }, Vec3{ 60.0f, 10.0f, a_height }, Vec3{ -10.0f, 10.0f, a_height } })
			surface.vertices.push_back(WeightedVertex(corner, 0, 1.0f));
		surface.indices = { 0, 1, 2, 0, 2, 3 };
		surface.skinToBone = { Transform{} };
		surface.boneDriver = { 0 };
		return surface;
	}
}

TEST_CASE("Spheres probed at supplied bone poses report their penetration of a surface", "[SmpPhysics]")
{
	const Fixture fixture = MakeFixture();
	const std::vector<Transform> drivers = { Transform{} };
	const std::vector<Collider> colliders = { MakeGroundPlane(Vec3{ 0.0f, 0.0f, -1000.0f }, Vec3{ 0.0f, 0.0f, 1.0f }, 0) };

	std::vector<Transform> worlds = fixture.rest;
	worlds[1].translation.z = 88.3f;
	worlds[2].translation.z = 95.0f;
	std::vector<SphereProbe> probes;
	ProbeSpheres(fixture.spheres, colliders, { MakeFloorSurface(88.0f) }, worlds, drivers, probes);
	REQUIRE(probes.size() == 2);
	for (const SphereProbe& probe : probes) {
		if (probe.bone == 1)
			REQUIRE(probe.depth == Approx(0.7f).margin(0.01f));
		else
			REQUIRE(probe.depth == Approx(0.0f).margin(1e-4f));
	}
}

TEST_CASE("A probe reports how far a sphere sits behind a one-sided face", "[SmpPhysics]")
{
	const Fixture fixture = MakeFixture();
	SkinnedSurface surface = MakeFloorSurface(88.0f);
	surface.contactPenetration = 0.4f;
	surface.deepPush = false;
	const std::vector<Transform> drivers = { Transform{} };
	const std::vector<Collider> colliders = { MakeGroundPlane(Vec3{ 0.0f, 0.0f, -1000.0f }, Vec3{ 0.0f, 0.0f, 1.0f }, 0) };

	std::vector<Transform> worlds = fixture.rest;
	worlds[1].translation.z = 90.0f;
	worlds[2].translation.z = 86.5f;
	std::vector<SphereProbe> probes;
	ProbeSpheres(fixture.spheres, colliders, { surface }, worlds, drivers, probes);
	REQUIRE(probes.size() == 2);
	for (const SphereProbe& probe : probes) {
		if (probe.bone == 1)
			REQUIRE(probe.behind == Approx(0.0f).margin(1e-4f));
		else
			REQUIRE(probe.behind == Approx(1.5f).margin(1e-3f));
	}
}

TEST_CASE("A probe finds no spheres when the bone poses do not match the sphere lists", "[SmpPhysics]")
{
	const Fixture fixture = MakeFixture();
	std::vector<SphereProbe> probes{ SphereProbe{} };
	ProbeSpheres(fixture.spheres, {}, {}, { Transform{} }, { Transform{} }, probes);
	REQUIRE(probes.empty());
	ProbeSpheres({}, {}, {}, {}, { Transform{} }, probes);
	REQUIRE(probes.empty());
}

TEST_CASE("Probing more spheres than the parallel threshold keeps each probe with its bone", "[SmpPhysics]")
{
	const std::size_t count = kParallelProbeSpheres + 37;
	std::vector<std::vector<BoneSphere>> spheres(count);
	std::vector<Transform> worlds(count);
	for (std::size_t bone = 0; bone < count; ++bone) {
		spheres[bone].push_back({ Vec3{}, 1.0f });
		worlds[bone].translation = Vec3{ static_cast<float>(bone % 10), static_cast<float>(bone % 7), 88.0f + static_cast<float>(bone % 5) * 0.1f };
	}
	std::vector<SphereProbe> probes;
	ProbeSpheres(spheres, {}, { MakeFloorSurface(88.0f) }, worlds, { Transform{} }, probes);

	REQUIRE(probes.size() == count);
	for (std::size_t bone = 0; bone < count; ++bone) {
		REQUIRE(probes[bone].bone == static_cast<int>(bone));
		REQUIRE(probes[bone].center.z == Approx(worlds[bone].translation.z));
		REQUIRE(probes[bone].depth == Approx(std::max(0.0f, 1.0f - (worlds[bone].translation.z - 88.0f))).margin(0.01f));
	}
}
