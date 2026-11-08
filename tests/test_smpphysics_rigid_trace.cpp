// Replays recorded hdtSMP64 steps through the CPU reference of SMP's rigid-body step
// (src/smp/RigidReference.cpp) and compares each step's result with SMP's own.

#include "rigid_trace_loader.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <string>
#include <vector>

using namespace SmpPhysics::RigidReference;
using namespace SmpPhysics::RigidTrace;

namespace
{
	constexpr float kContactFreePositionTolerance = 1.0e-3f;
	constexpr float kContactFreeVelocityTolerance = 0.01f;
	constexpr float kContactPositionTolerance = 0.05f;
	constexpr float kContactVelocityTolerance = 1.0f;
	constexpr float kContactAngularVelocityTolerance = 0.1f;
	constexpr float kOutsideContactPositionTolerance = 0.015f;
	constexpr float kOutsideContactVelocityTolerance = 0.8f;
	constexpr float kOutsideContactAngularVelocityTolerance = 0.12f;

	/** @brief The worst error of one body class over a replay. */
	struct ErrorStats
	{
		float position{};
		float rotation{};
		float linearVelocity{};
		float angularVelocity{};
		std::size_t samples{};
	};

	/** @brief Errors of bodies whose island had no contact in the step, which the reference must reproduce, and of the rest. */
	struct ReplayResult
	{
		ErrorStats free;
		ErrorStats contacted;
	};

	void Accumulate(ErrorStats& a_stats, float a_position, float a_rotation, float a_linearVelocity, float a_angularVelocity)
	{
		a_stats.position = std::max(a_stats.position, a_position);
		a_stats.rotation = std::max(a_stats.rotation, a_rotation);
		a_stats.linearVelocity = std::max(a_stats.linearVelocity, a_linearVelocity);
		a_stats.angularVelocity = std::max(a_stats.angularVelocity, a_angularVelocity);
		++a_stats.samples;
	}

	/** @brief The island each body belongs to: bodies joined by a constraint whose two ends are both dynamic share one. */
	std::vector<std::size_t> ConstraintIslands(const World& a_world)
	{
		std::vector<std::size_t> parent(a_world.bodies.size());
		for (std::size_t index = 0; index < parent.size(); ++index)
			parent[index] = index;
		const auto find = [&parent](std::size_t a_index) {
			while (parent[a_index] != a_index)
				a_index = parent[a_index] = parent[parent[a_index]];
			return a_index;
		};
		for (const Generic6DofSpring2& constraint : a_world.constraints) {
			const auto a = static_cast<std::size_t>(constraint.bodyA);
			const auto b = static_cast<std::size_t>(constraint.bodyB);
			if (!a_world.bodies[a].kinematic && !a_world.bodies[b].kinematic)
				parent[find(a)] = find(b);
		}
		std::vector<std::size_t> islands(parent.size());
		for (std::size_t index = 0; index < parent.size(); ++index)
			islands[index] = find(index);
		return islands;
	}

	/** @brief Replays every recorded step from its recorded pre-step state and returns the worst error per body class. */
	ReplayResult ReplayTrace(const std::string& a_name)
	{
		const std::filesystem::path directory = TraceDirectory();
		const json header = json::parse(std::ifstream(directory / (a_name + ".json")));
		const std::vector<char> bytes = ReadFile(directory / (a_name + ".bin"));
		REQUIRE(header["version"].get<int>() >= kTraceVersion);
		World world = ReadWorld(header);
		const std::size_t bodyCount = world.bodies.size();
		const std::vector<StepView> steps = ReadSteps(bytes, bodyCount, header["steps"].get<std::size_t>(), ContactStride(header));

		const std::vector<std::size_t> islands = ConstraintIslands(world);
		ReplayResult result;
		for (const StepView& step : steps) {
			std::vector<bool> contacted(bodyCount, false);
			const char* contact = bytes.data() + step.contactOffset;
			for (std::uint32_t index = 0; index < step.contactCount; ++index, contact += ContactStride(header)) {
				std::int32_t bodies[2];
				std::memcpy(bodies, contact, sizeof(bodies));
				for (const std::int32_t body : bodies) {
					if (body >= 0)
						contacted[static_cast<std::size_t>(body)] = true;
				}
			}

			std::vector<bool> islandContacted(bodyCount, false);
			for (std::size_t body = 0; body < bodyCount; ++body) {
				if (contacted[body])
					islandContacted[islands[body]] = true;
			}

			ApplyPreState(world, FloatsAt(bytes, step.preOffset));
			world.contacts = ReadContacts(bytes, step, ContactStride(header));
			world.BeginFrame();
			world.Step(step.timeStep);
			world.EndFrame();

			const float* post = FloatsAt(bytes, step.postOffset);
			for (std::size_t body = 0; body < bodyCount; ++body) {
				const float* values = post + body * kPostFloats;
				const Body& state = world.bodies[body];
				Accumulate(islandContacted[islands[body]] ? result.contacted : result.free,
					Length(Sub(state.worldTransform.origin, { values[0], values[1], values[2] })),
					AngleBetween(RotationFromBasis(state.worldTransform.basis), { values[3], values[4], values[5], values[6] }),
					Length(Sub(state.linearVelocity, { values[7], values[8], values[9] })),
					Length(Sub(state.angularVelocity, { values[10], values[11], values[12] })));
			}
		}
		return result;
	}
}

TEST_CASE("The reference reproduces SMP's contact-free steps", "[smpphysics][rigid_reference][rigid_trace]")
{
	const char* name = GENERATE("walk_cloak_slice", "walk_gem_slice");
	const ReplayResult result = ReplayTrace(name);
	REQUIRE(result.free.samples > 0);

	INFO(name << " contact-free: position " << result.free.position << " linear velocity " << result.free.linearVelocity << " angular velocity " << result.free.angularVelocity << " over " << result.free.samples << " body steps");
	INFO(name << " in contact: position " << result.contacted.position << " linear velocity " << result.contacted.linearVelocity << " over " << result.contacted.samples << " body steps");
	CHECK(result.free.position < kContactFreePositionTolerance);
	CHECK(result.free.linearVelocity < kContactFreeVelocityTolerance);
}

TEST_CASE("The reference solves SMP's contact steps", "[smpphysics][rigid_reference][rigid_trace]")
{
	const ReplayResult result = ReplayTrace("walk_skirt_slice");
	REQUIRE(result.contacted.samples > 0);

	INFO("skirt in contact: position " << result.contacted.position << " linear velocity " << result.contacted.linearVelocity << " angular velocity " << result.contacted.angularVelocity << " over " << result.contacted.samples << " body steps");
	CHECK(result.contacted.position < kContactPositionTolerance);
	CHECK(result.contacted.linearVelocity < kContactVelocityTolerance);
	CHECK(result.contacted.angularVelocity < kContactAngularVelocityTolerance);
}

namespace
{
	/**
	 * @brief Runs the recorded kinematic motion with the dynamic bodies free from the first step on, and returns the worst
	 *        position error against SMP over every body that never touches anything.
	 * @details SMP re-centres the world on its bodies each frame, so a body carried over from the last step is moved by the
	 *          same shift the kinematic bodies show between the two recorded frames.
	 */
	float FreeRunError(const std::string& a_name)
	{
		const std::filesystem::path directory = TraceDirectory();
		const json header = json::parse(std::ifstream(directory / (a_name + ".json")));
		const std::vector<char> bytes = ReadFile(directory / (a_name + ".bin"));
		World world = ReadWorld(header);
		const std::size_t bodyCount = world.bodies.size();
		const std::vector<StepView> steps = ReadSteps(bytes, bodyCount, header["steps"].get<std::size_t>(), ContactStride(header));
		const std::vector<std::size_t> islands = ConstraintIslands(world);

		std::vector<bool> touched(bodyCount, false);
		for (const StepView& step : steps) {
			const char* contact = bytes.data() + step.contactOffset;
			for (std::uint32_t index = 0; index < step.contactCount; ++index, contact += ContactStride(header)) {
				std::int32_t pair[2];
				std::memcpy(pair, contact, sizeof(pair));
				for (const std::int32_t body : pair) {
					if (body >= 0)
						touched[islands[static_cast<std::size_t>(body)]] = true;
				}
			}
		}

		float worst = 0.0f;
		for (std::size_t stepIndex = 0; stepIndex < steps.size(); ++stepIndex) {
			const std::vector<Body> carried = world.bodies;
			ApplyPreState(world, FloatsAt(bytes, steps[stepIndex].preOffset));
			if (stepIndex > 0) {
				const float* lastPost = FloatsAt(bytes, steps[stepIndex - 1].postOffset);
				Vec3 shift{};
				for (std::size_t body = 0; body < bodyCount; ++body) {
					if (world.bodies[body].kinematic) {
						const float* values = lastPost + body * kPostFloats;
						shift = Sub(world.bodies[body].worldTransform.origin, { values[0], values[1], values[2] });
						break;
					}
				}
				for (std::size_t body = 0; body < bodyCount; ++body) {
					if (world.bodies[body].kinematic)
						continue;
					world.bodies[body] = carried[body];
					Vec3& origin = world.bodies[body].worldTransform.origin;
					origin = { origin.x + shift.x, origin.y + shift.y, origin.z + shift.z };
				}
			}
			world.BeginFrame();
			world.Step(steps[stepIndex].timeStep);
			world.EndFrame();

			const float* post = FloatsAt(bytes, steps[stepIndex].postOffset);
			for (std::size_t body = 0; body < bodyCount; ++body) {
				if (world.bodies[body].kinematic || touched[islands[body]])
					continue;
				const float* values = post + body * kPostFloats;
				worst = std::max(worst, Length(Sub(world.bodies[body].worldTransform.origin, { values[0], values[1], values[2] })));
			}
		}
		return worst;
	}
}

TEST_CASE("The reference tracks SMP over a free run of contact-free bodies", "[smpphysics][rigid_reference][rigid_trace]")
{
	CHECK(FreeRunError("walk_gem_slice") < kContactFreePositionTolerance);
}

TEST_CASE("The reference solves SMP's steps against bodies outside the trace", "[smpphysics][rigid_reference][rigid_trace]")
{
	const std::filesystem::path directory = TraceDirectory();
	const json header = json::parse(std::ifstream(directory / "wig_motion_slice.json"));
	const std::vector<char> bytes = ReadFile(directory / "wig_motion_slice.bin");
	const World base = ReadWorld(header);
	const std::size_t bodyCount = base.bodies.size();
	const std::vector<StepView> steps = ReadSteps(bytes, bodyCount, header["steps"].get<std::size_t>(), ContactStride(header));
	REQUIRE(!steps.empty());

	float position = 0.0f;
	float linearVelocity = 0.0f;
	float angularVelocity = 0.0f;
	std::size_t outsideContacts = 0;
	for (const StepView& step : steps) {
		World world = base;
		ApplyPreState(world, FloatsAt(bytes, step.preOffset));
		world.contacts = ReadContacts(bytes, step, ContactStride(header));
		AppendExternalContacts(world, bytes, step, ContactStride(header));
		outsideContacts += world.bodies.size() - bodyCount;
		world.BeginFrame();
		world.Step(step.timeStep);
		world.EndFrame();

		const float* post = FloatsAt(bytes, step.postOffset);
		for (std::size_t body = 0; body < bodyCount; ++body) {
			const float* values = post + body * kPostFloats;
			const Body& state = world.bodies[body];
			position = std::max(position, Length(Sub(state.worldTransform.origin, { values[0], values[1], values[2] })));
			linearVelocity = std::max(linearVelocity, Length(Sub(state.linearVelocity, { values[7], values[8], values[9] })));
			angularVelocity = std::max(angularVelocity, Length(Sub(state.angularVelocity, { values[10], values[11], values[12] })));
		}
	}

	WARN("wig outside contacts: position " << position << " linear velocity " << linearVelocity << " angular velocity " << angularVelocity << " over " << steps.size() << " steps, " << outsideContacts << " outside contacts");
	CHECK(outsideContacts > 0);
	CHECK(position < kOutsideContactPositionTolerance);
	CHECK(linearVelocity < kOutsideContactVelocityTolerance);
	CHECK(angularVelocity < kOutsideContactAngularVelocityTolerance);
}
