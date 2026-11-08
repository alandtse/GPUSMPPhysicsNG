// Unit tests for the CPU reference of SMP's rigid-body step
// (src/smp/RigidReference.cpp). Analytic expectations only.

#include "smp/RigidReference.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

using namespace SmpPhysics::RigidReference;
using Catch::Approx;

namespace
{
	constexpr float kStep = 1.0f / 240.0f;
	constexpr float kGravity = 9.8f;
	constexpr float kPi = 3.14159265358979323846f;

	/** @brief A dynamic body with an isotropic inertia, so the gyroscopic term stays zero. */
	Body DynamicBody(const Vec3& a_origin, float a_inverseMass, float a_inverseInertia)
	{
		Body body;
		body.worldTransform = MakeTransform(a_origin, Quat{});
		body.inverseMass = a_inverseMass;
		body.inverseInertiaLocal = { a_inverseInertia, a_inverseInertia, a_inverseInertia };
		return body;
	}

	Body KinematicBody(const Vec3& a_origin)
	{
		Body body;
		body.kinematic = true;
		body.worldTransform = MakeTransform(a_origin, Quat{});
		return body;
	}

	/** @brief A 6-DOF constraint with the three linear axes locked and the three angular axes free. */
	Generic6DofSpring2 LockedLinearConstraint(int a_bodyA, int a_bodyB)
	{
		Generic6DofSpring2 constraint;
		constraint.bodyA = a_bodyA;
		constraint.bodyB = a_bodyB;
		for (RotationalMotor& motor : constraint.angular) {
			motor.lower = 1.0f;
			motor.upper = -1.0f;
			motor.springEnabled = false;
		}
		return constraint;
	}

	float RelativeAngleZ(const Body& a_body)
	{
		return std::atan2(a_body.worldTransform.basis.row[0].y, a_body.worldTransform.basis.row[0].x);
	}
}

TEST_CASE("A free body under gravity follows Bullet's discrete step", "[smpphysics][rigid_reference]")
{
	constexpr int steps = 120;

	World world;
	world.bodies.push_back(DynamicBody({}, 1.0f, 2.5f));
	world.bodies[0].gravity = { 0.0f, 0.0f, -kGravity };

	world.bodies.push_back(DynamicBody({}, 1.0f, 2.5f));
	world.bodies[1].gravity = { 0.0f, 0.0f, -kGravity };
	world.bodies[1].linearDamping = 0.5f;
	world.bodies[1].angularDamping = 0.4f;

	world.BeginFrame();
	for (int step = 0; step < steps; ++step)
		world.Step(kStep);
	world.EndFrame();

	const float time = steps * kStep;
	const float undamped = -kGravity * kStep * kStep * 0.5f * steps * (steps + 1);
	CHECK(world.bodies[0].linearVelocity.z == Approx(-kGravity * time).margin(1.0e-4f));
	CHECK(world.bodies[0].worldTransform.origin.z == Approx(undamped).margin(1.0e-4f));
	// Bullet's semi-implicit order leaves the fall one half step beyond the continuous 0.5 g t^2:
	// the difference is 0.5 g dt t, about 1 cm here, which is what the tolerance states.
	CHECK(world.bodies[0].worldTransform.origin.z == Approx(-0.5f * kGravity * time * time).margin(2.0e-2f));

	const float damping = std::pow(1.0f - world.bodies[1].linearDamping, kStep);
	const float series = damping * (1.0f - std::pow(damping, static_cast<float>(steps))) / (1.0f - damping);
	const float dampedDistance = -kGravity * kStep * kStep / (1.0f - damping) * (static_cast<float>(steps) - series);
	CHECK(world.bodies[1].worldTransform.origin.z == Approx(dampedDistance).epsilon(0.005f));
	CHECK(world.bodies[1].worldTransform.origin.z > undamped);
}

TEST_CASE("A kinematic body is integrated from its interpolation velocity", "[smpphysics][rigid_reference]")
{
	constexpr int steps = 200;
	const Vec3 start{ 1.0f, 2.0f, 3.0f };
	const Vec3 velocity{ 2.0f, -1.0f, 0.5f };

	World world;
	world.bodies.push_back(KinematicBody(start));
	world.bodies[0].interpolationLinearVelocity = velocity;

	world.BeginFrame();
	for (int step = 0; step < steps; ++step)
		world.Step(kStep);
	world.EndFrame();

	const float time = steps * kStep;
	const Vec3& origin = world.bodies[0].worldTransform.origin;
	CHECK(origin.x == Approx(start.x + velocity.x * time).margin(1.0e-3f));
	CHECK(origin.y == Approx(start.y + velocity.y * time).margin(1.0e-3f));
	CHECK(origin.z == Approx(start.z + velocity.z * time).margin(1.0e-3f));
	CHECK(world.bodies[0].linearVelocity.x == Approx(0.0f).margin(1.0e-6f));
	CHECK(world.bodies[0].worldTransform.basis.row[0].x == Approx(1.0f).margin(1.0e-5f));
}

TEST_CASE("A pendulum holds its length and swings at the analytic period", "[smpphysics][rigid_reference]")
{
	constexpr float length = 1.0f;
	constexpr float startAngle = 0.05f;
	constexpr int steps = 1200;

	const Vec3 offset{ length * std::sin(startAngle), -length * std::cos(startAngle), 0.0f };

	World world;
	world.bodies.push_back(KinematicBody({}));
	world.bodies.push_back(DynamicBody(offset, 1.0f, 2.5f));
	world.bodies[1].gravity = { 0.0f, -kGravity, 0.0f };

	Generic6DofSpring2 constraint = LockedLinearConstraint(0, 1);
	constraint.frameB.origin = { -offset.x, -offset.y, -offset.z };
	world.constraints.push_back(constraint);

	std::vector<float> angles;
	angles.reserve(steps + 1);
	angles.push_back(startAngle);
	float worstLengthError = 0.0f;

	world.BeginFrame();
	for (int step = 0; step < steps; ++step) {
		world.Step(kStep);
		const Vec3& position = world.bodies[1].worldTransform.origin;
		angles.push_back(std::atan2(position.x, -position.y));
		worstLengthError = std::max(worstLengthError, std::fabs(std::sqrt(position.x * position.x + position.y * position.y) - length));
	}
	world.EndFrame();

	CHECK(worstLengthError < 0.01f * length);

	std::vector<float> crossings;
	for (std::size_t index = 1; index < angles.size() && crossings.size() < 2; ++index) {
		if ((angles[index - 1] > 0.0f) == (angles[index] > 0.0f))
			continue;
		const float fraction = angles[index - 1] / (angles[index - 1] - angles[index]);
		crossings.push_back(static_cast<float>(index - 1 + fraction) * kStep);
	}
	REQUIRE(crossings.size() == 2);

	// The bob is a rigid body pinned at the anchor, so it swings as a physical pendulum about the pin:
	// I = m L^2 + 1 / inverseInertia, and T = 2 pi sqrt(I / (m g L)).
	const float inverseInertia = world.bodies[1].inverseInertiaLocal.x;
	const float moment = length * length / world.bodies[1].inverseMass + 1.0f / inverseInertia;
	const float period = 2.0f * (crossings[1] - crossings[0]);
	const float analytic = 2.0f * kPi * std::sqrt(moment / (kGravity * length / world.bodies[1].inverseMass));
	CHECK(period == Approx(analytic).epsilon(0.01f));
}

TEST_CASE("A linear spring-damper settles at its configured offset", "[smpphysics][rigid_reference]")
{
	constexpr float equilibrium = 0.25f;
	constexpr int steps = 2400;

	World world;
	world.bodies.push_back(KinematicBody({}));
	world.bodies.push_back(DynamicBody({}, 1.0f, 2.5f));

	Generic6DofSpring2 constraint = LockedLinearConstraint(0, 1);
	constraint.linear.lower = { 1.0f, 1.0f, 1.0f };
	constraint.linear.upper = { -1.0f, -1.0f, -1.0f };
	constraint.linear.springEnabled = { true, false, false };
	constraint.linear.springStiffness = { 100.0f, 0.0f, 0.0f };
	constraint.linear.springDamping = { 2.0f, 0.0f, 0.0f };
	constraint.linear.equilibrium = { equilibrium, 0.0f, 0.0f };
	world.constraints.push_back(constraint);

	world.BeginFrame();
	for (int step = 0; step < steps; ++step)
		world.Step(kStep);
	world.EndFrame();

	const Vec3& offset = world.bodies[1].worldTransform.origin;
	CHECK(offset.x == Approx(equilibrium).margin(1.0e-3f));
	CHECK(offset.y == Approx(0.0f).margin(1.0e-3f));
	CHECK(offset.z == Approx(0.0f).margin(1.0e-3f));
	CHECK(world.bodies[1].linearVelocity.x == Approx(0.0f).margin(1.0e-2f));
}

TEST_CASE("An angular limit stops a rotation at the limit within the ERP slack", "[smpphysics][rigid_reference]")
{
	constexpr float lower = -0.1f;
	constexpr float upper = 0.5f;
	constexpr int steps = 960;

	World world;
	world.bodies.push_back(KinematicBody({}));
	world.bodies.push_back(DynamicBody({}, 1.0f, 2.5f));
	world.bodies[1].angularVelocity = { 0.0f, 0.0f, -4.0f };

	Generic6DofSpring2 constraint = LockedLinearConstraint(0, 1);
	constraint.angular[2].lower = lower;
	constraint.angular[2].upper = upper;
	world.constraints.push_back(constraint);

	world.BeginFrame();
	for (int step = 0; step < steps; ++step)
		world.Step(kStep);
	world.EndFrame();

	// Bullet extracts the Z angle as atan2(-m10, m00), so a body spinning negatively about Z raises it.
	const float angle = RelativeAngleZ(world.bodies[1]);
	CHECK(angle <= upper + 0.02f);
	CHECK(angle >= upper - 0.02f);
	CHECK(std::fabs(world.bodies[1].angularVelocity.z) < 1.0f);
}

TEST_CASE("A non-Hookean stiffness weakens the spring in proportion to its stretch", "[smpphysics][rigid_reference]")
{
	const auto stepFromOffset = [](float a_stiffnessFactor) {
		World world;
		world.bodies.push_back(KinematicBody({}));
		world.bodies.push_back(DynamicBody({ 0.5f, 0.0f, 0.0f }, 1.0f, 2.5f));

		Generic6DofSpring2 constraint = LockedLinearConstraint(0, 1);
		constraint.linear.lower = { -1.0f, 0.0f, 0.0f };
		constraint.linear.upper = { 1.0f, 0.0f, 0.0f };
		constraint.linear.springEnabled = { true, false, false };
		constraint.linear.springStiffness = { 100.0f, 0.0f, 0.0f };
		constraint.linear.springStiffnessLimited = { false, false, false };
		constraint.linear.nonHookeanStiffness = { a_stiffnessFactor, 0.0f, 0.0f };
		world.constraints.push_back(constraint);

		world.BeginFrame();
		world.Step(kStep);
		world.EndFrame();
		return world.bodies[1].linearVelocity.x;
	};

	const float hookean = stepFromOffset(0.0f);
	REQUIRE(hookean < 0.0f);
	CHECK(stepFromOffset(1.0f) == Approx(hookean * 0.5f).epsilon(0.02));
}

TEST_CASE("A non-Hookean damping weakens the damper in proportion to its stretch", "[smpphysics][rigid_reference]")
{
	const auto stepFromOffset = [](float a_dampingFactor) {
		World world;
		world.bodies.push_back(KinematicBody({}));
		world.bodies.push_back(DynamicBody({ 0.5f, 0.0f, 0.0f }, 1.0f, 2.5f));
		world.bodies[1].linearVelocity = { 1.0f, 0.0f, 0.0f };

		Generic6DofSpring2 constraint = LockedLinearConstraint(0, 1);
		constraint.linear.lower = { -1.0f, 0.0f, 0.0f };
		constraint.linear.upper = { 1.0f, 0.0f, 0.0f };
		constraint.linear.springEnabled = { true, false, false };
		constraint.linear.springDamping = { 2.0f, 0.0f, 0.0f };
		constraint.linear.springDampingLimited = { false, false, false };
		constraint.linear.nonHookeanDamping = { a_dampingFactor, 0.0f, 0.0f };
		world.constraints.push_back(constraint);

		world.BeginFrame();
		world.Step(kStep);
		world.EndFrame();
		return 1.0f - world.bodies[1].linearVelocity.x;
	};

	const float plain = stepFromOffset(0.0f);
	REQUIRE(plain > 0.0f);
	CHECK(stepFromOffset(1.0f) == Approx(plain * 0.5f).epsilon(0.02));
}
