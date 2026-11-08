// Unit tests for the claim-time rest retarget (src/smp/RestRetarget.cpp) and the grid query it uses.

#include "smp/RestRetarget.h"
#include "smp/TriangleGrid.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <string_view>
#include <vector>

using namespace SmpPhysics;
using Catch::Approx;

namespace
{
	/** @brief Head at the top, two 20 unit bones, joints limited to a fifth of a radian like a real hair rig. */
	constexpr std::string_view kChainXml = R"XML(
<system>
	<bone name="Head"><mass>0</mass></bone>
	<bone-default name="hair">
		<mass>0.1</mass>
		<inertia x="16" y="16" z="16"/>
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
		<angularLowerLimit x="-0.02" y="-0.2" z="-0.2"/>
		<angularUpperLimit x="0.02" y="0.2" z="0.2"/>
		<angularStiffness x="44440" y="44440" z="44440"/>
	</generic-constraint-default>
	<constraint-group>
		<generic-constraint bodyA="Hair01" bodyB="Head" template="hair-joint"/>
		<generic-constraint bodyA="Hair02" bodyB="Hair01" template="hair-joint"/>
	</constraint-group>
</system>)XML";

	constexpr RetargetLoad kNoGravity{ 0.0f, 1.0f };
	constexpr RetargetLoad kGravity{ 687.7193f, 1.0f };
	constexpr float kBone = 20.0f;
	constexpr float kTilt = 0.3f;  ///< the tilted fixture leans each bone this far outward from vertical, in radians

	struct Fixture
	{
		SmpRig rig;
		std::vector<Transform> rest;
		std::vector<std::vector<BoneSphere>> spheres;
	};

	/**
	 * @brief A chain hanging from the head a unit off the body axis along +X, so a wall beside it has a side; a swing in X
	 *        is a turn about the joints' wide Y axis. With a_tilt each bone leans outward instead of hanging straight.
	 */
	Fixture MakeFixture(float a_tilt = 0.0f, bool a_capSprings = false, float a_bone = kBone)
	{
		Fixture fixture;
		const ParseResult result = ParseSmpXml(kChainXml);
		REQUIRE(result.status == ParseStatus::Ok);
		fixture.rig = result.rig;
		if (a_capSprings)
			ApplySpringLimits(fixture.rig, 1.0f / 60.0f);
		const Vec3 step{ a_bone * std::sin(a_tilt), 0.0f, -a_bone * std::cos(a_tilt) };
		const Vec3 head{ 1.0f, 0.0f, 100.0f };
		fixture.rest = { { Quat{}, head }, { Quat{}, head + step }, { Quat{}, head + step + step } };
		fixture.spheres.resize(3);
		fixture.spheres[1].push_back({ Vec3{}, 0.01f });
		fixture.spheres[2].push_back({ Vec3{}, 0.01f });
		return fixture;
	}

	SkinnedVertex Vertex(const Vec3& a_position)
	{
		SkinnedVertex vertex;
		vertex.position = a_position;
		vertex.bone[0] = 0;
		vertex.weight[0] = 1.0f;
		return vertex;
	}

	/** @brief A vertical wall at x = a_x facing +X, a worn plate standing beside the chain, with the body axis at x = 0. */
	SkinnedSurface MakeWall(float a_x)
	{
		SkinnedSurface surface;
		surface.name = "plate";
		for (const Vec3& corner : { Vec3{ a_x, -10.0f, 40.0f }, Vec3{ a_x, 10.0f, 40.0f }, Vec3{ a_x, 10.0f, 110.0f }, Vec3{ a_x, -10.0f, 110.0f } })
			surface.vertices.push_back(Vertex(corner));
		surface.indices = { 0, 1, 2, 0, 2, 3 };
		surface.skinToBone = { Transform{} };
		surface.boneDriver = { 0 };
		surface.contactPenetration = 0.4f;
		surface.deepPush = false;
		return surface;
	}

	const std::vector<Transform> kDrivers = { Transform{} };
	const Vec3 kAxis{ 0.0f, 0.0f, 100.0f };
}

TEST_CASE("A point is inside a shell when the ray out from the body axis exits it through an outward face", "[SmpPhysics]")
{
	std::vector<Vec3> positions;
	std::vector<std::uint32_t> indices;
	// Two outward walls (normal +X, away from the axis at x = 0) at x = 30 and x = 33, and a lining at x = 36 facing back toward the axis.
	const auto addSheet = [&](float a_x, bool a_outward) {
		const std::uint32_t base = static_cast<std::uint32_t>(positions.size());
		positions.insert(positions.end(), { { a_x, -5.0f, 0.0f }, { a_x, 5.0f, 0.0f }, { a_x, 5.0f, 10.0f }, { a_x, -5.0f, 10.0f } });
		if (a_outward)
			indices.insert(indices.end(), { base, base + 1, base + 2, base, base + 2, base + 3 });
		else
			indices.insert(indices.end(), { base, base + 2, base + 1, base, base + 3, base + 2 });
	};
	addSheet(30.0f, true);
	addSheet(33.0f, true);
	addSheet(36.0f, false);
	TriangleGrid grid;
	grid.Build(positions, indices, 3.0f);

	Vec3 normal{};
	REQUIRE(grid.RadialDepthInside({ 28.0f, 0.0f, 5.0f }, 10.0f, 0.4f, Vec3{}, &normal) == Approx(-2.0f));
	REQUIRE(normal.x == Approx(1.0f));
	REQUIRE(grid.RadialDepthInside({ 32.0f, 0.0f, 5.0f }, 10.0f, 0.4f, Vec3{}) == Approx(-1.0f));
	// Between the second wall and the lining the ray meets the lining first, the far plate across an air gap: not inside.
	REQUIRE_FALSE(std::isfinite(grid.RadialDepthInside({ 35.0f, 0.0f, 5.0f }, 10.0f, 0.4f, Vec3{})));
	// Just inside the first wall, within its allowance, the ray goes on and exits the second: that depth is reported.
	REQUIRE(grid.RadialDepthInside({ 29.8f, 0.0f, 5.0f }, 10.0f, 0.4f, Vec3{}) == Approx(-3.2f));
	// Within the allowance of the only exit, beside the sheets, and with the axis on the far side: none of these count.
	REQUIRE_FALSE(std::isfinite(grid.RadialDepthInside({ 32.8f, 0.0f, 5.0f }, 10.0f, 0.4f, Vec3{})));
	REQUIRE_FALSE(std::isfinite(grid.RadialDepthInside({ 28.0f, 20.0f, 5.0f }, 10.0f, 0.4f, Vec3{})));
	REQUIRE_FALSE(std::isfinite(grid.RadialDepthInside({ 28.0f, 0.0f, 5.0f }, 10.0f, 0.4f, Vec3{ 29.0f, 0.0f, 0.0f })));

	// A ledge over a point never lies on the horizontal ray: the same quad laid flat, with the point beneath it. With a
	// wall under its outer edge the point is measured against that wall, as hair fallen through a shoulder plate is.
	std::vector<Vec3> flat = { { -5.0f, -5.0f, 10.0f }, { 5.0f, -5.0f, 10.0f }, { 5.0f, 5.0f, 10.0f }, { -5.0f, 5.0f, 10.0f } };
	TriangleGrid ledge;
	ledge.Build(flat, { 0, 1, 2, 0, 2, 3 }, 3.0f);
	REQUIRE_FALSE(std::isfinite(ledge.RadialDepthInside({ 1.0f, 1.0f, 5.0f }, 10.0f, 0.4f, Vec3{})));
	flat.insert(flat.end(), { { 5.0f, -5.0f, 0.0f }, { 5.0f, 5.0f, 0.0f } });
	TriangleGrid shelf;
	shelf.Build(flat, { 0, 1, 2, 0, 2, 3, 4, 5, 2, 4, 2, 1 }, 3.0f);
	REQUIRE(shelf.RadialDepthInside({ 1.0f, 0.0f, 5.0f }, 10.0f, 0.4f, Vec3{}) == Approx(-4.0f));
}

TEST_CASE("A chain authored inside a plate is turned out through it within the joint limits", "[SmpPhysics]")
{
	const Fixture fixture = MakeFixture();
	const std::vector<SkinnedSurface> surfaces = { MakeWall(4.0f) };
	const RestRetarget retarget = RetargetRestToSurfaces(fixture.rig, fixture.rest, fixture.spheres, surfaces, kDrivers, kAxis, kNoGravity);

	REQUIRE(retarget.behindSpheres == 2);
	REQUIRE(retarget.residualSpheres == 0);
	REQUIRE(retarget.retargetedJoints >= 1);
	REQUIRE(retarget.maxAngle <= 0.2f + 1e-4f);
	REQUIRE(retarget.sagMaxMove == 0.0f);
	// Within the contact allowance of the plate counts as out: the runtime contact holds the last bit.
	REQUIRE(retarget.restPose[1].translation.x > 4.0f - 0.4f);
	REQUIRE(retarget.restPose[2].translation.x > 4.0f - 0.4f);
	for (std::size_t bone = 1; bone < 3; ++bone) {
		REQUIRE(std::fabs(retarget.angleOffsets[bone].x) <= 0.02f + 1e-4f);
		REQUIRE(std::fabs(retarget.angleOffsets[bone].y) <= 0.2f + 1e-4f);
		REQUIRE(std::fabs(retarget.angleOffsets[bone].z) <= 0.2f + 1e-4f);
	}
	// Bone lengths are preserved: joints turned, nothing stretched.
	REQUIRE(Length(retarget.restPose[2].translation - retarget.restPose[1].translation) == Approx(kBone).margin(1e-3f));
	REQUIRE(Length(retarget.restPose[1].translation - retarget.restPose[0].translation) == Approx(kBone).margin(1e-3f));
}

TEST_CASE("A plate too far out for even the widened swing to free most of the hair keeps the authored rest", "[SmpPhysics]")
{
	// With 8 unit bones the first joint can carry its bone at most 8 * sin(kRetargetMaxSwing), about 4.5 units, from
	// the axis, so only the second bone could be freed: half the hair, short of kRetargetMinimumFreed.
	const Fixture fixture = MakeFixture(0.0f, false, 8.0f);
	const std::vector<SkinnedSurface> surfaces = { MakeWall(7.0f) };
	const RestRetarget retarget = RetargetRestToSurfaces(fixture.rig, fixture.rest, fixture.spheres, surfaces, kDrivers, kAxis, kNoGravity);
	REQUIRE(retarget.behindSpheres == 2);
	REQUIRE(retarget.keptAuthored);
	REQUIRE(retarget.residualSpheres == 2);
	REQUIRE(retarget.retargetedJoints == 0);
	REQUIRE(retarget.maxBoneMove == 0.0f);
	for (std::size_t bone = 0; bone < 3; ++bone) {
		REQUIRE(retarget.restPose[bone].translation.x == fixture.rest[bone].translation.x);
		REQUIRE(retarget.angleOffsets[bone].y == 0.0f);
	}
}

TEST_CASE("A plate past the authored limits is still cleared, as the limits shape hair on the bare body only", "[SmpPhysics]")
{
	const Fixture fixture = MakeFixture();
	const std::vector<SkinnedSurface> surfaces = { MakeWall(9.0f) };
	const RestRetarget retarget = RetargetRestToSurfaces(fixture.rig, fixture.rest, fixture.spheres, surfaces, kDrivers, kAxis, kNoGravity);
	REQUIRE(retarget.behindSpheres == 2);
	REQUIRE(retarget.residualSpheres == 0);
	REQUIRE(retarget.maxAngle > 0.2f);
	REQUIRE(retarget.maxAngle <= kRetargetMaxSwing + 1e-4f);
	// Within the contact allowance of the plate counts as out: the runtime contact holds the last bit.
	REQUIRE(retarget.restPose[1].translation.x > 9.0f - 0.4f);
	REQUIRE(retarget.restPose[2].translation.x > 9.0f - 0.4f);
}

TEST_CASE("Hair in front of every wall, behind a lining, or against skin is left exactly as authored", "[SmpPhysics]")
{
	const Fixture fixture = MakeFixture();
	const auto unchanged = [&](const std::vector<SkinnedSurface>& a_surfaces) {
		const RestRetarget retarget = RetargetRestToSurfaces(fixture.rig, fixture.rest, fixture.spheres, a_surfaces, kDrivers, kAxis, kGravity);
		REQUIRE(retarget.behindSpheres == 0);
		REQUIRE(retarget.retargetedJoints == 0);
		REQUIRE(retarget.maxBoneMove == 0.0f);
		for (std::size_t bone = 0; bone < 3; ++bone) {
			REQUIRE(retarget.restPose[bone].translation.x == fixture.rest[bone].translation.x);
			REQUIRE(retarget.angleOffsets[bone].y == 0.0f);
		}
	};
	unchanged({});
	unchanged({ MakeWall(-2.0f) });
	// A surface that already reaches deep behind its faces, like the rig's own body shapes, is not retargeted against.
	SkinnedSurface deep = MakeWall(4.0f);
	deep.deepPush = true;
	unchanged({ deep });
	// Nor is the character's skin, which the hair was modelled against.
	SkinnedSurface skin = MakeWall(4.0f);
	skin.skin = true;
	unchanged({ skin });
}

TEST_CASE("Gravity sags a leaning chain toward the vertical as far as the capped springs and limits allow", "[SmpPhysics]")
{
	const Fixture fixture = MakeFixture(kTilt, true);
	const std::vector<Transform> still = PredictGravitySag(fixture.rig, fixture.rest, fixture.spheres, {}, kDrivers, kAxis, kNoGravity);
	for (std::size_t bone = 0; bone < 3; ++bone)
		REQUIRE(Length(still[bone].translation - fixture.rest[bone].translation) < 1e-3f);

	// Each joint settles where its capped spring balances the gravity torque of its own bone's lean: k * sag = m g L sin(lean - sag).
	const std::vector<Transform> sagged = PredictGravitySag(fixture.rig, fixture.rest, fixture.spheres, {}, kDrivers, kAxis, kGravity);
	const float stiffness = fixture.rig.joints[0].angularStiffness.y;
	REQUIRE(stiffness < 44440.0f);
	const auto settledLeanOf = [&](std::size_t a_bone) {
		const Vec3 settledLean = sagged[a_bone].translation - sagged[a_bone - 1].translation;
		REQUIRE(Length(settledLean) == Approx(kBone).margin(1e-3f));
		return std::atan2(settledLean.x, -settledLean.z);
	};
	const float weight = kGravity.gravity * fixture.rig.bones[1].mass * fixture.rig.bones[1].gravityFactor;
	const float firstLean = settledLeanOf(1);
	REQUIRE(firstLean < kTilt);
	REQUIRE(firstLean > 0.0f);
	REQUIRE(stiffness * (kTilt - firstLean) == Approx(weight * kBone * std::sin(firstLean)).epsilon(1e-2f));
	// The second bone leans by its parent's settled lean less its own sag, which balances its own weight the same way
	// against its own cap, lower because both bodies of that joint have inertia.
	const float secondStiffness = fixture.rig.joints[1].angularStiffness.y;
	REQUIRE(secondStiffness < stiffness);
	const float secondLean = settledLeanOf(2);
	REQUIRE(secondLean < firstLean);
	REQUIRE(secondStiffness * (firstLean - secondLean) == Approx(weight * kBone * std::sin(secondLean)).epsilon(1e-2f));
}

TEST_CASE("A chain the retarget would leave standing out from the body keeps its authored rest", "[SmpPhysics]")
{
	// Leaning 0.65 rad out, the chain hangs well; clearing a plate at x = 19 would take it to about 1.12 rad, hanging less than half.
	const Fixture fixture = MakeFixture(0.65f, false);
	const std::vector<SkinnedSurface> surfaces = { MakeWall(19.0f) };
	const RestRetarget retarget = RetargetRestToSurfaces(fixture.rig, fixture.rest, fixture.spheres, surfaces, kDrivers, kAxis, kNoGravity);
	REQUIRE(retarget.behindSpheres >= 1);
	REQUIRE(retarget.revertedChains == 1);
	REQUIRE(retarget.retargetedJoints == 0);
	for (std::size_t bone = 0; bone < 3; ++bone)
		REQUIRE(retarget.restPose[bone].translation.x == Approx(fixture.rest[bone].translation.x).margin(1e-3f));

	// The same chain with a plate it can clear while still hanging is turned as before.
	const RestRetarget cleared = RetargetRestToSurfaces(fixture.rig, fixture.rest, fixture.spheres, { MakeWall(15.0f) }, kDrivers, kAxis, kNoGravity);
	REQUIRE(cleared.revertedChains == 0);
	REQUIRE(cleared.retargetedJoints >= 1);
}

TEST_CASE("A bone sags only as far as the body under it lets it", "[SmpPhysics]")
{
	const Fixture fixture = MakeFixture(kTilt, true);
	const float free = PredictGravitySag(fixture.rig, fixture.rest, fixture.spheres, {}, kDrivers, kAxis, kGravity)[1].translation.x;
	// The body's skin stands between the authored lean and where the bone would sag to, so the bone comes to rest on it.
	SkinnedSurface body = MakeWall(6.0f);
	body.deepPush = true;
	REQUIRE(free < 6.0f);
	const std::vector<Transform> resting = PredictGravitySag(fixture.rig, fixture.rest, fixture.spheres, { body }, kDrivers, kAxis, kGravity);
	REQUIRE(resting[1].translation.x >= 6.0f - 0.05f);
	REQUIRE(resting[1].translation.x < fixture.rest[1].translation.x);
	// The skin of a worn body mesh supports the hair the same way, and a plate that does not reach deep behind its faces does not.
	SkinnedSurface skin = MakeWall(6.0f);
	skin.deepPush = false;
	skin.skin = true;
	REQUIRE(PredictGravitySag(fixture.rig, fixture.rest, fixture.spheres, { skin }, kDrivers, kAxis, kGravity)[1].translation.x >= 6.0f - 0.05f);
	SkinnedSurface plate = MakeWall(6.0f);
	plate.deepPush = false;
	REQUIRE(PredictGravitySag(fixture.rig, fixture.rest, fixture.spheres, { plate }, kDrivers, kAxis, kGravity)[1].translation.x == Approx(free));
}

TEST_CASE("Hair carried out over a plate hangs back down below it", "[SmpPhysics]")
{
	// A four bone chain hanging straight down beside a short plate that only its upper half is inside: the root joint
	// carries the whole chain out, and the joints below the plate turn back so the lower bones hang vertically again.
	const ParseResult parsed = ParseSmpXml(R"XML(
<system>
	<bone name="Head"><mass>0</mass></bone>
	<bone-default name="hair"><mass>0.1</mass><inertia x="16" y="16" z="16"/></bone-default>
	<bone name="Hair01" template="hair"/>
	<bone name="Hair02" template="hair"/>
	<bone name="Hair03" template="hair"/>
	<bone name="Hair04" template="hair"/>
	<generic-constraint-default name="hair-joint">
		<frameInB><basis x="0" y="0" z="0" w="1"/><origin x="0" y="0" z="0"/></frameInB>
		<linearLowerLimit x="0" y="0" z="0"/><linearUpperLimit x="0" y="0" z="0"/>
		<angularLowerLimit x="-0.02" y="-0.2" z="-0.2"/><angularUpperLimit x="0.02" y="0.2" z="0.2"/>
		<angularStiffness x="44440" y="44440" z="44440"/>
	</generic-constraint-default>
	<constraint-group>
		<generic-constraint bodyA="Hair01" bodyB="Head" template="hair-joint"/>
		<generic-constraint bodyA="Hair02" bodyB="Hair01" template="hair-joint"/>
		<generic-constraint bodyA="Hair03" bodyB="Hair02" template="hair-joint"/>
		<generic-constraint bodyA="Hair04" bodyB="Hair03" template="hair-joint"/>
	</constraint-group>
</system>)XML");
	REQUIRE(parsed.status == ParseStatus::Ok);
	std::vector<Transform> rest;
	std::vector<std::vector<BoneSphere>> spheres(5);
	for (int bone = 0; bone < 5; ++bone) {
		rest.push_back({ Quat{}, Vec3{ 1.0f, 0.0f, 100.0f - 10.0f * static_cast<float>(bone) } });
		if (bone > 0)
			spheres[static_cast<std::size_t>(bone)].push_back({ Vec3{}, 0.01f });
	}
	SkinnedSurface plate = MakeWall(4.0f);
	for (SkinnedVertex& vertex : plate.vertices)
		vertex.position.z = vertex.position.z < 100.0f ? 75.0f : 101.0f;
	const RestRetarget retarget = RetargetRestToSurfaces(parsed.rig, rest, spheres, { plate }, kDrivers, kAxis, kNoGravity);
	REQUIRE(retarget.behindSpheres == 2);
	REQUIRE(retarget.residualSpheres == 0);
	REQUIRE(retarget.restPose[2].translation.x > 4.0f - 0.4f);
	// The bones below the plate hang straight down again from where the chain cleared it.
	const Vec3 lower = retarget.restPose[4].translation - retarget.restPose[3].translation;
	REQUIRE(std::fabs(lower.x) < 0.3f);
	REQUIRE(retarget.restPose[4].translation.x < retarget.restPose[2].translation.x + 0.5f);
	REQUIRE(retarget.restPose[4].translation.z < retarget.restPose[3].translation.z);
}

TEST_CASE("Hair that sags into a plate is given a rest from which it settles just clear of it", "[SmpPhysics]")
{
	const Fixture fixture = MakeFixture(kTilt, true);
	// The authored lean clears the wall; the sagged first bone, about 5.2 units from the axis, is behind it.
	const float wall = 5.9f;
	const std::vector<SkinnedSurface> surfaces = { MakeWall(wall) };
	REQUIRE(fixture.rest[1].translation.x > wall + 0.4f);
	REQUIRE(PredictGravitySag(fixture.rig, fixture.rest, fixture.spheres, {}, kDrivers, kAxis, kGravity)[1].translation.x < wall - 0.4f);

	const RestRetarget retarget = RetargetRestToSurfaces(fixture.rig, fixture.rest, fixture.spheres, surfaces, kDrivers, kAxis, kGravity);
	REQUIRE(retarget.behindSpheres == 1);
	REQUIRE(retarget.residualSpheres == 0);
	REQUIRE(retarget.retargetedJoints >= 1);
	REQUIRE(retarget.sagMaxMove > 1.5f);
	// The rest leans further out than authored, about the first joint's wide axis only.
	REQUIRE(retarget.restPose[1].translation.x > fixture.rest[1].translation.x);
	REQUIRE(retarget.angleOffsets[1].x == 0.0f);
	REQUIRE(retarget.angleOffsets[1].y != 0.0f);
	// Settling the new rest under the same gravity lands the first bone at the wall's face, no longer inside it.
	const std::vector<Transform> settled = PredictGravitySag(fixture.rig, retarget.restPose, fixture.spheres, surfaces, kDrivers, kAxis, kGravity);
	REQUIRE(settled[1].translation.x >= wall - 0.05f);
	REQUIRE(settled[1].translation.x < wall + 1.0f);
}
