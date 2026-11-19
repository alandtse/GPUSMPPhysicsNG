// Runs the rigid-body compute kernels (RigidSolverCS.hlsl) on a WARP device and compares every body they produce
// with the CPU reference's result for the same recorded step.

#include "rigid_gpu_harness.h"
#include "rigid_trace_loader.h"
#include "smp/RigidGpuPacking.h"
#include "smp/RigidReference.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <string>
#include <vector>

using namespace SmpPhysics::RigidReference;
using namespace SmpPhysics::RigidTrace;
using namespace SmpPhysics::RigidGpu;
using namespace SmpPhysics::RigidGpuTest;

namespace
{
	constexpr float kPositionTolerance{ 1.0e-3f };
	constexpr float kVelocityTolerance{ 1.0e-2f };
	constexpr float kRotationTolerance{ 1.0e-4f };
	/**
	 * @brief Twice the coloured solve's worst measured deviation, over 140 recorded steps.
	 * @details The colour-major sweep reorders a solve that SMP stops at 16 iterations, so its result differs from the
	 *          stored-order one by whatever that reordering is worth before convergence; it is a deviation, not a defect.
	 */
	constexpr float kColoredPositionTolerance{ 1.5f };
	constexpr float kColoredVelocityTolerance{ 100.0f };
	constexpr float kColoredRotationTolerance{ 0.02f };
	/** @brief Twice the coloured kernel's worst measured gap from the reference swept in its own colour order. */
	constexpr float kOrderedPositionTolerance{ 2.0e-4f };
	constexpr float kOrderedVelocityTolerance{ 1.2e-2f };

	/**
	 * @brief The angle between two bases, in radians.
	 * @details 2*acos(dot) cannot resolve an angle below about 1e-4 in float32, so the quaternions are differenced
	 *          instead: 2*atan2(|q1 - q2|, |q1 + q2|) stays accurate down to rounding.
	 */
	float RotationError(const Mat3& a_produced, const Mat3& a_expected)
	{
		const Quat produced = RotationFromBasis(a_produced);
		const Quat expected = RotationFromBasis(a_expected);
		const float dot = produced.x * expected.x + produced.y * expected.y + produced.z * expected.z + produced.w * expected.w;
		const float sign = dot < 0.0f ? -1.0f : 1.0f;
		const float dx = produced.x - sign * expected.x;
		const float dy = produced.y - sign * expected.y;
		const float dz = produced.z - sign * expected.z;
		const float dw = produced.w - sign * expected.w;
		const float sx = produced.x + sign * expected.x;
		const float sy = produced.y + sign * expected.y;
		const float sz = produced.z + sign * expected.z;
		const float sw = produced.w + sign * expected.w;
		return 2.0f * std::atan2(std::sqrt(dx * dx + dy * dy + dz * dz + dw * dw), std::sqrt(sx * sx + sy * sy + sz * sz + sw * sw));
	}

	/** @brief The worst difference between the two sides of one step, and how many bodies were compared. */
	struct StepErrors
	{
		float position{};
		float rotation{};
		float linearVelocity{};
		float angularVelocity{};
		std::size_t bodies{};
	};

	/** @brief The worst gap between the coloured kernel and the reference solved in the packing's own sweep order. */
	struct OrderedErrors
	{
		float position{};
		float velocity{};
	};

	/** @brief The WARP device and its kernels, built once for every fixture the tests replay. */
	struct RigidKernels
	{
		GpuDevice gpu;
		Kernels kernels;
	};

	const RigidKernels& LoadKernels()
	{
		static const RigidKernels built = [] {
			RigidKernels value;
			value.gpu = CreateDevice(D3D_DRIVER_TYPE_WARP);
			value.kernels = CompileKernels(value.gpu.device.Get());
			return value;
		}();
		return built;
	}

	/** @brief One replayable trace fixture: its header, its recorded bytes, and the steps to replay. */
	struct Fixture
	{
		std::string name;
		World preStep;
		std::vector<char> bytes;
		std::vector<StepView> steps;
		std::size_t contactBytes{};
	};

	Fixture LoadFixture(const char* a_name)
	{
		Fixture fixture;
		fixture.name = a_name;
		const std::filesystem::path directory = TraceDirectory();
		const json header = json::parse(std::ifstream(directory / (fixture.name + ".json")));
		fixture.bytes = ReadFile(directory / (fixture.name + ".bin"));
		REQUIRE(header["version"].get<int>() >= kTraceVersion);

		fixture.contactBytes = ContactStride(header);
		fixture.preStep = ReadWorld(header);
		REQUIRE(fixture.preStep.bodies.size() > 0);
		fixture.steps = ReadSteps(fixture.bytes, fixture.preStep.bodies.size(), header["steps"].get<std::size_t>(), ContactStride(header));
		REQUIRE(!fixture.steps.empty());
		return fixture;
	}

	/** @brief The world one recorded step starts from, after BeginFrame applied the frame's forces. */
	World StepWorld(const Fixture& a_fixture, const StepView& a_step)
	{
		World world = a_fixture.preStep;
		ApplyPreState(world, FloatsAt(a_fixture.bytes, a_step.preOffset));
		world.BeginFrame();
		return world;
	}

	/** @brief StepWorld with the contacts SMP recorded for the step, which the reference and the kernels both solve. */
	World ContactStepWorld(const Fixture& a_fixture, const StepView& a_step)
	{
		World world = StepWorld(a_fixture, a_step);
		world.contacts = ReadContacts(a_fixture.bytes, a_step, a_fixture.contactBytes);
		return world;
	}

	/**
	 * @brief The same world with its constraints in the order the coloured solve sweeps them: island by island, each
	 *        island colour class by colour class.
	 * @details Islands share no dynamic body, so the reference's single global sweep reaches the same rows in the same
	 *          relative order; a step of this world is the exact result the coloured kernel is aiming at.
	 */
	World ColorOrderedWorld(const World& a_world, const StepData& a_packed)
	{
		World ordered = a_world;
		ordered.constraints.clear();
		for (std::size_t island = 0; island < a_packed.islands.size(); ++island) {
			const IslandRecord& record = a_packed.islands[island];
			for (std::uint32_t color = 0; color < record.range.z; ++color) {
				const std::uint32_t first = a_packed.colorOffsets[island * kMaxColors + color];
				const std::uint32_t last = color + 1 < record.range.z ? a_packed.colorOffsets[island * kMaxColors + color + 1] : record.range.y;
				for (std::uint32_t offset = first; offset < last; ++offset)
					ordered.constraints.push_back(a_world.constraints[a_packed.coloredConstraints[record.range.x + offset]]);
			}
		}
		return ordered;
	}

	/** @brief The islands' colour classes, as the packing recorded them. */
	struct ColorSummary
	{
		std::uint32_t maximum{};
		std::uint32_t islands{};
		double mean{};
	};

	ColorSummary SummarizeColors(const StepData& a_packed)
	{
		ColorSummary summary;
		summary.islands = static_cast<std::uint32_t>(a_packed.islands.size());
		std::uint32_t total = 0;
		for (const IslandRecord& island : a_packed.islands) {
			summary.maximum = std::max(summary.maximum, island.range.z);
			total += island.range.z;
		}
		summary.mean = a_packed.islands.empty() ? 0.0 : static_cast<double>(total) / static_cast<double>(a_packed.islands.size());
		return summary;
	}
}

TEST_CASE("The GPU rigid solver reproduces the CPU reference", "[smpphysics][rigid_gpu]")
{
	const Fixture fixture = LoadFixture(GENERATE("walk_cloak_slice", "walk_gem_slice"));
	const RigidKernels& kernels = LoadKernels();
	const std::size_t bodyCount = fixture.preStep.bodies.size();

	StepErrors worst;
	for (std::size_t stepIndex = 0; stepIndex < fixture.steps.size(); ++stepIndex) {
		const StepView& step = fixture.steps[stepIndex];
		const World world = StepWorld(fixture, step);
		World reference = world;
		reference.Step(step.timeStep);

		const StepData packed = PackStep(world, step.timeStep);
		INFO(fixture.name << " step " << stepIndex << ": a constraint asks for more rows than its slot holds");
		REQUIRE(packed.rowsFit);
		INFO(fixture.name << " step " << stepIndex << ": an island needs more colour classes than the packing holds");
		REQUIRE(packed.colorsFit);

		const std::vector<BodyRecord> integrated = RunStep(kernels.gpu, kernels.kernels, packed, kernels.kernels.solve.Get(),
			GroupCount(packed.islands.size()));

		World actual = world;
		UnpackBodies(integrated, actual);
		for (std::size_t body = 0; body < bodyCount; ++body) {
			const Body& expected = reference.bodies[body];
			const Body& produced = actual.bodies[body];
			const float position = Length(Sub(produced.worldTransform.origin, expected.worldTransform.origin));
			const float rotation = RotationError(produced.worldTransform.basis, expected.worldTransform.basis);
			const float linearVelocity = Length(Sub(produced.linearVelocity, expected.linearVelocity));
			const float angularVelocity = Length(Sub(produced.angularVelocity, expected.angularVelocity));
			INFO(fixture.name << " step " << stepIndex << " body " << body << ": position " << position << " rotation " << rotation
			                  << " linear velocity " << linearVelocity << " angular velocity " << angularVelocity);
			REQUIRE(std::isfinite(position));
			REQUIRE(std::isfinite(rotation));
			REQUIRE(position < kPositionTolerance);
			REQUIRE(rotation < kRotationTolerance);
			REQUIRE(linearVelocity < kVelocityTolerance);
			REQUIRE(angularVelocity < kVelocityTolerance);

			worst.position = std::max(worst.position, position);
			worst.rotation = std::max(worst.rotation, rotation);
			worst.linearVelocity = std::max(worst.linearVelocity, linearVelocity);
			worst.angularVelocity = std::max(worst.angularVelocity, angularVelocity);
			++worst.bodies;
		}
	}

	INFO(fixture.name << ": position " << worst.position << " rotation " << worst.rotation << " linear velocity "
	                  << worst.linearVelocity << " angular velocity " << worst.angularVelocity << " over " << worst.bodies << " body steps");
	CHECK(worst.bodies == bodyCount * fixture.steps.size());
}

TEST_CASE("The GPU rigid solver reproduces the CPU reference's contact rows", "[smpphysics][rigid_gpu][rigid_contacts]")
{
	const Fixture fixture = LoadFixture("walk_skirt_slice");
	const RigidKernels& kernels = LoadKernels();
	const std::size_t bodyCount = fixture.preStep.bodies.size();

	StepErrors worst;
	std::size_t contactSteps = 0;
	for (std::size_t stepIndex = 0; stepIndex < fixture.steps.size(); ++stepIndex) {
		const StepView& step = fixture.steps[stepIndex];
		const World world = ContactStepWorld(fixture, step);
		World reference = world;
		reference.Step(step.timeStep);

		const StepData packed = PackStep(world, step.timeStep);
		INFO("step " << stepIndex << ": a constraint asks for more rows than its slot holds");
		REQUIRE(packed.rowsFit);
		contactSteps += packed.contacts.empty() ? 0 : 1;

		const std::vector<BodyRecord> integrated = RunStep(kernels.gpu, kernels.kernels, packed, kernels.kernels.solve.Get(),
			GroupCount(packed.islands.size()));

		const std::vector<BodyRecord> colored = RunStep(kernels.gpu, kernels.kernels, packed, kernels.kernels.solveColored.Get(),
			static_cast<UINT>(packed.islands.size()));
		World coloredActual = world;
		UnpackBodies(colored, coloredActual);
		for (std::size_t body = 0; body < bodyCount; ++body) {
			const float position = Length(Sub(coloredActual.bodies[body].worldTransform.origin, reference.bodies[body].worldTransform.origin));
			INFO("step " << stepIndex << " body " << body << ": coloured position " << position);
			REQUIRE(std::isfinite(position));
			REQUIRE(position < kColoredPositionTolerance);
		}

		World actual = world;
		UnpackBodies(integrated, actual);
		for (std::size_t body = 0; body < bodyCount; ++body) {
			const Body& expected = reference.bodies[body];
			const Body& produced = actual.bodies[body];
			const float position = Length(Sub(produced.worldTransform.origin, expected.worldTransform.origin));
			const float rotation = RotationError(produced.worldTransform.basis, expected.worldTransform.basis);
			const float linearVelocity = Length(Sub(produced.linearVelocity, expected.linearVelocity));
			const float angularVelocity = Length(Sub(produced.angularVelocity, expected.angularVelocity));
			INFO("step " << stepIndex << " body " << body << " of " << packed.contacts.size() << " contacts: position " << position
			             << " rotation " << rotation << " linear velocity " << linearVelocity << " angular velocity " << angularVelocity);
			REQUIRE(std::isfinite(position));
			REQUIRE(position < kPositionTolerance);
			REQUIRE(rotation < kRotationTolerance);
			REQUIRE(linearVelocity < kVelocityTolerance);
			REQUIRE(angularVelocity < kVelocityTolerance);

			worst.position = std::max(worst.position, position);
			worst.rotation = std::max(worst.rotation, rotation);
			worst.linearVelocity = std::max(worst.linearVelocity, linearVelocity);
			worst.angularVelocity = std::max(worst.angularVelocity, angularVelocity);
			++worst.bodies;
		}
	}

	WARN("skirt contacts, GPU vs reference: position " << worst.position << " rotation " << worst.rotation << " linear velocity "
	                                                   << worst.linearVelocity << " angular velocity " << worst.angularVelocity << " over " << worst.bodies << " body steps, "
	                                                   << contactSteps << " steps with contacts");
	CHECK(contactSteps > 0);
}

TEST_CASE("The GPU rigid solver reproduces the CPU reference against bodies outside the trace", "[smpphysics][rigid_gpu][rigid_contacts]")
{
	const Fixture fixture = LoadFixture("wig_motion_slice");
	const RigidKernels& kernels = LoadKernels();
	const std::size_t bodyCount = fixture.preStep.bodies.size();

	StepErrors worst;
	float coloredPosition = 0.0f;
	for (std::size_t stepIndex = 0; stepIndex < fixture.steps.size(); ++stepIndex) {
		const StepView& step = fixture.steps[stepIndex];
		World world = ContactStepWorld(fixture, step);
		AppendExternalContacts(world, fixture.bytes, step, fixture.contactBytes);
		World reference = world;
		reference.Step(step.timeStep);

		const StepData packed = PackStep(world, step.timeStep);
		INFO("step " << stepIndex << ": a constraint asks for more rows than its slot holds");
		REQUIRE(packed.rowsFit);

		const std::vector<BodyRecord> integrated = RunStep(kernels.gpu, kernels.kernels, packed, kernels.kernels.solve.Get(),
			GroupCount(packed.islands.size()));
		World actual = world;
		UnpackBodies(integrated, actual);

		const std::vector<BodyRecord> colored = RunStep(kernels.gpu, kernels.kernels, packed, kernels.kernels.solveColored.Get(),
			static_cast<UINT>(packed.islands.size()));
		World coloredActual = world;
		UnpackBodies(colored, coloredActual);

		for (std::size_t body = 0; body < bodyCount; ++body) {
			const Body& expected = reference.bodies[body];
			const Body& produced = actual.bodies[body];
			const float position = Length(Sub(produced.worldTransform.origin, expected.worldTransform.origin));
			const float rotation = RotationError(produced.worldTransform.basis, expected.worldTransform.basis);
			const float linearVelocity = Length(Sub(produced.linearVelocity, expected.linearVelocity));
			const float angularVelocity = Length(Sub(produced.angularVelocity, expected.angularVelocity));
			INFO("step " << stepIndex << " body " << body << ": position " << position << " rotation " << rotation
			             << " linear velocity " << linearVelocity << " angular velocity " << angularVelocity);
			REQUIRE(std::isfinite(position));
			REQUIRE(position < kPositionTolerance);
			REQUIRE(rotation < kRotationTolerance);
			REQUIRE(linearVelocity < kVelocityTolerance);
			REQUIRE(angularVelocity < kVelocityTolerance);
			worst.position = std::max(worst.position, position);
			worst.rotation = std::max(worst.rotation, rotation);
			worst.linearVelocity = std::max(worst.linearVelocity, linearVelocity);
			worst.angularVelocity = std::max(worst.angularVelocity, angularVelocity);
			++worst.bodies;

			const float coloredGap = Length(Sub(coloredActual.bodies[body].worldTransform.origin, expected.worldTransform.origin));
			REQUIRE(std::isfinite(coloredGap));
			REQUIRE(coloredGap < kColoredPositionTolerance);
			coloredPosition = std::max(coloredPosition, coloredGap);
		}
	}

	WARN("wig outside contacts, GPU vs reference: position " << worst.position << " rotation " << worst.rotation << " linear velocity "
	                                                         << worst.linearVelocity << " angular velocity " << worst.angularVelocity << "; coloured position " << coloredPosition
	                                                         << " over " << worst.bodies << " body steps");
	CHECK(worst.bodies == bodyCount * fixture.steps.size());
}

TEST_CASE("The coloured GPU rigid solve stays near the CPU reference", "[smpphysics][rigid_gpu][rigid_colored]")
{
	const Fixture fixture = LoadFixture(GENERATE("walk_cloak_slice", "walk_gem_slice"));
	const RigidKernels& kernels = LoadKernels();
	const std::size_t bodyCount = fixture.preStep.bodies.size();

	StepErrors worst;
	OrderedErrors worstOrdered;
	ColorSummary colors;
	for (std::size_t stepIndex = 0; stepIndex < fixture.steps.size(); ++stepIndex) {
		const StepView& step = fixture.steps[stepIndex];
		const World world = StepWorld(fixture, step);
		World reference = world;
		reference.Step(step.timeStep);

		const StepData packed = PackStep(world, step.timeStep);
		INFO(fixture.name << " step " << stepIndex << ": an island needs more colour classes than the packing holds");
		REQUIRE(packed.colorsFit);
		colors = SummarizeColors(packed);

		const std::vector<BodyRecord> integrated = RunStep(kernels.gpu, kernels.kernels, packed,
			kernels.kernels.solveColored.Get(), static_cast<UINT>(packed.islands.size()));

		World ordered = ColorOrderedWorld(world, packed);
		ordered.Step(step.timeStep);

		World actual = world;
		UnpackBodies(integrated, actual);
		for (std::size_t body = 0; body < bodyCount; ++body) {
			const Body& expected = reference.bodies[body];
			const Body& produced = actual.bodies[body];
			const float position = Length(Sub(produced.worldTransform.origin, expected.worldTransform.origin));
			const float rotation = RotationError(produced.worldTransform.basis, expected.worldTransform.basis);
			const float linearVelocity = Length(Sub(produced.linearVelocity, expected.linearVelocity));
			const float angularVelocity = Length(Sub(produced.angularVelocity, expected.angularVelocity));
			INFO(fixture.name << " step " << stepIndex << " body " << body << ": position " << position << " rotation " << rotation
			                  << " linear velocity " << linearVelocity << " angular velocity " << angularVelocity
			                  << " over " << packed.islands.size() << " islands, " << colors.maximum << " colours");
			REQUIRE(std::isfinite(position));
			REQUIRE(std::isfinite(rotation));
			REQUIRE(position < kColoredPositionTolerance);
			REQUIRE(rotation < kColoredRotationTolerance);
			REQUIRE(linearVelocity < kColoredVelocityTolerance);
			REQUIRE(angularVelocity < kColoredVelocityTolerance);

			worst.position = std::max(worst.position, position);
			worst.rotation = std::max(worst.rotation, rotation);
			worst.linearVelocity = std::max(worst.linearVelocity, linearVelocity);
			worst.angularVelocity = std::max(worst.angularVelocity, angularVelocity);
			++worst.bodies;

			const float orderedPosition = Length(Sub(produced.worldTransform.origin, ordered.bodies[body].worldTransform.origin));
			const float orderedVelocity = Length(Sub(produced.linearVelocity, ordered.bodies[body].linearVelocity));
			INFO(fixture.name << " step " << stepIndex << " body " << body << " against the colour-ordered reference: position "
			                  << orderedPosition << " linear velocity " << orderedVelocity);
			REQUIRE(orderedPosition < kOrderedPositionTolerance);
			REQUIRE(orderedVelocity < kOrderedVelocityTolerance);
			worstOrdered.position = std::max(worstOrdered.position, orderedPosition);
			worstOrdered.velocity = std::max(worstOrdered.velocity, orderedVelocity);
		}

		const std::vector<BodyRecord> repeated = RunStep(kernels.gpu, kernels.kernels, packed,
			kernels.kernels.solveColored.Get(), static_cast<UINT>(packed.islands.size()));
		REQUIRE(repeated.size() == integrated.size());
		for (std::size_t body = 0; body < integrated.size(); ++body) {
			INFO(fixture.name << " step " << stepIndex << " body " << body << " is not deterministic");
			CHECK(std::memcmp(&repeated[body], &integrated[body], sizeof(BodyRecord)) == 0);
		}
	}

	WARN(fixture.name << " coloured vs stored order: position " << worst.position << " rotation " << worst.rotation << " linear velocity "
	                  << worst.linearVelocity << " angular velocity " << worst.angularVelocity << " over " << worst.bodies << " body steps, "
	                  << fixture.preStep.constraints.size() << " constraints in " << colors.islands << " islands, max " << colors.maximum
	                  << " colours, mean " << colors.mean);
	WARN(fixture.name << " coloured vs colour order: position " << worstOrdered.position << " velocity " << worstOrdered.velocity);
	CHECK(worst.bodies == bodyCount * fixture.steps.size());
}

TEST_CASE("The rigid packing colours each island's constraints", "[smpphysics][rigid_gpu][rigid_colored]")
{
	/** @brief A world of a_bodies chain of dynamic bodies joined end to end, in a_ring closing the last to the first. */
	const auto buildWorld = [](std::size_t a_bodies, bool a_ring) {
		World world;
		for (std::size_t index = 0; index < a_bodies; ++index) {
			Body body;
			body.inverseMass = 1.0f;
			world.bodies.push_back(body);
		}
		for (std::size_t index = 0; index + 1 < a_bodies; ++index) {
			Generic6DofSpring2 constraint;
			constraint.bodyA = static_cast<int>(index);
			constraint.bodyB = static_cast<int>(index + 1);
			world.constraints.push_back(constraint);
		}
		if (a_ring && a_bodies > 2) {
			Generic6DofSpring2 constraint;
			constraint.bodyA = static_cast<int>(a_bodies - 1);
			constraint.bodyB = 0;
			world.constraints.push_back(constraint);
		}
		return world;
	};

	const auto colorCount = [](const StepData& a_packed, std::size_t a_island) {
		return a_packed.islands[a_island].range.z;
	};

	SECTION("a chain of dynamic bodies takes two colours")
	{
		constexpr std::size_t kChainBodies{ 9 };
		const StepData packed = PackStep(buildWorld(kChainBodies, false), 1.0f / 60.0f);
		REQUIRE(packed.colorsFit);
		REQUIRE(packed.islands.size() == 1);
		CHECK(colorCount(packed, 0) == 2);
		CHECK(packed.coloredConstraints.size() == kChainBodies - 1);
	}

	SECTION("a ring of dynamic bodies takes three colours")
	{
		constexpr std::size_t kRingBodies{ 5 };
		const StepData packed = PackStep(buildWorld(kRingBodies, true), 1.0f / 60.0f);
		REQUIRE(packed.colorsFit);
		REQUIRE(packed.islands.size() == 1);
		CHECK(colorCount(packed, 0) == 3);
		CHECK(packed.coloredConstraints.size() == kRingBodies);
	}

	SECTION("a body held by more constraints than the old 32 classes still colours")
	{
		constexpr std::size_t kLeaves{ 40 };
		World world = buildWorld(kLeaves + 1, false);
		world.constraints.clear();
		for (std::size_t leaf = 1; leaf <= kLeaves; ++leaf) {
			Generic6DofSpring2 constraint;
			constraint.bodyA = static_cast<int>(leaf);
			constraint.bodyB = 0;
			world.constraints.push_back(constraint);
		}
		const StepData packed = PackStep(world, 1.0f / 60.0f);
		REQUIRE(packed.colorsFit);
		REQUIRE(packed.islands.size() == 1);
		CHECK(colorCount(packed, 0) == kLeaves);
		CHECK(packed.coloredConstraints.size() == kLeaves);
	}

	SECTION("an island needing more classes than the packing holds reports failure")
	{
		constexpr std::size_t kCliqueBodies{ kMaxColors + 1 };
		World world = buildWorld(kCliqueBodies, false);
		world.constraints.clear();
		for (std::size_t first = 0; first < kCliqueBodies; ++first) {
			for (std::size_t second = first + 1; second < kCliqueBodies; ++second) {
				Generic6DofSpring2 constraint;
				constraint.bodyA = static_cast<int>(first);
				constraint.bodyB = static_cast<int>(second);
				world.constraints.push_back(constraint);
			}
		}
		const StepData packed = PackStep(world, 1.0f / 60.0f);
		CHECK_FALSE(packed.colorsFit);
		CHECK(packed.coloredConstraints.empty());
		CHECK(packed.colorOffsets.empty());
	}

	SECTION("every recorded constraint is coloured exactly once and each colour is proper")
	{
		const Fixture fixture = LoadFixture(GENERATE("walk_cloak_slice", "walk_gem_slice"));
		const World world = StepWorld(fixture, fixture.steps.front());
		const StepData packed = PackStep(world, fixture.steps.front().timeStep);
		REQUIRE(packed.colorsFit);
		REQUIRE(packed.coloredConstraints.size() == packed.islandConstraints.size());
		REQUIRE(packed.colorOffsets.size() == packed.islands.size() * kMaxColors);

		std::vector<std::uint32_t> occurrences(world.constraints.size(), 0u);
		for (std::size_t island = 0; island < packed.islands.size(); ++island) {
			const IslandRecord& record = packed.islands[island];
			const std::uint32_t colorCount = record.range.z;
			REQUIRE(colorCount > 0);
			REQUIRE(packed.colorOffsets[island * kMaxColors] == 0u);

			std::vector<std::vector<std::uint32_t>> classes(colorCount);
			for (std::uint32_t color = 0; color < colorCount; ++color) {
				const std::uint32_t first = packed.colorOffsets[island * kMaxColors + color];
				const std::uint32_t last = color + 1 < colorCount ? packed.colorOffsets[island * kMaxColors + color + 1] : record.range.y;
				REQUIRE(first <= last);
				for (std::uint32_t offset = first; offset < last; ++offset)
					classes[color].push_back(packed.coloredConstraints[record.range.x + offset]);
			}
			REQUIRE(packed.colorOffsets[island * kMaxColors + colorCount - 1] <= record.range.y);

			for (const std::vector<std::uint32_t>& members : classes) {
				std::vector<std::uint32_t> bodies;
				for (const std::uint32_t constraint : members) {
					++occurrences[constraint];
					for (const int body : { world.constraints[constraint].bodyA, world.constraints[constraint].bodyB }) {
						if (body < 0 || static_cast<std::size_t>(body) >= world.bodies.size() || world.bodies[static_cast<std::size_t>(body)].kinematic)
							continue;
						INFO(fixture.name << " constraint " << constraint << " repeats dynamic body " << body << " in one colour");
						CHECK(std::find(bodies.begin(), bodies.end(), static_cast<std::uint32_t>(body)) == bodies.end());
						bodies.push_back(static_cast<std::uint32_t>(body));
					}
				}
			}
		}

		for (std::size_t constraint = 0; constraint < world.constraints.size(); ++constraint) {
			const Generic6DofSpring2& entry = world.constraints[constraint];
			const bool solvable = entry.enabled && entry.bodyA >= 0 && static_cast<std::size_t>(entry.bodyA) < world.bodies.size() &&
			                      entry.bodyB >= 0 && static_cast<std::size_t>(entry.bodyB) < world.bodies.size();
			if (!solvable) {
				CHECK(occurrences[constraint] == 0u);
				continue;
			}
			INFO(fixture.name << " constraint " << constraint << " appears " << occurrences[constraint] << " times");
			CHECK(occurrences[constraint] == 1u);
		}
	}
}

TEST_CASE("The GPU rigid solver reproduces non-Hookean springs, motors and servos", "[smpphysics][rigid_gpu]")
{
	const RigidKernels& kernels = LoadKernels();

	World start;
	Body anchor;
	anchor.kinematic = true;
	anchor.worldTransform = MakeTransform({}, Quat{});
	Body bob;
	bob.worldTransform = MakeTransform({ 0.4f, 0.1f, 0.0f }, Quat{ 0.0f, 0.0f, 0.1f, 0.995f });
	bob.inverseMass = 2.0f;
	bob.inverseInertiaLocal = { 5.0f, 5.0f, 5.0f };
	bob.linearVelocity = { 0.5f, 0.0f, 0.2f };
	bob.angularVelocity = { 0.0f, 0.3f, 0.0f };
	start.bodies = { anchor, bob };

	Generic6DofSpring2 constraint;
	constraint.bodyA = 1;
	constraint.bodyB = 0;
	constraint.linear.lower = { -1.0f, -1.0f, 0.0f };
	constraint.linear.upper = { 1.0f, 1.0f, 0.0f };
	constraint.linear.springEnabled = { true, true, false };
	constraint.linear.springStiffness = { 50.0f, 40.0f, 0.0f };
	constraint.linear.springDamping = { 2.0f, 1.5f, 0.0f };
	constraint.linear.nonHookeanStiffness = { 0.6f, 0.0f, 0.0f };
	constraint.linear.nonHookeanDamping = { 0.4f, 0.5f, 0.0f };
	constraint.linear.motorEnabled = { false, false, true };
	constraint.linear.targetVelocity = { 0.0f, 0.0f, 0.3f };
	constraint.linear.maxMotorForce = { 0.0f, 0.0f, 5.0f };
	for (std::size_t axis = 0; axis < 3; ++axis) {
		RotationalMotor& motor = constraint.angular[axis];
		motor.lower = -0.8f;
		motor.upper = 0.8f;
		motor.springEnabled = true;
		motor.springStiffness = 8.0f;
		motor.springDamping = 0.5f;
		motor.nonHookeanStiffness = 0.3f;
		motor.nonHookeanDamping = 0.2f;
	}
	constraint.angular[1].servoEnabled = true;
	constraint.angular[1].servoTarget = 0.2f;
	constraint.angular[1].targetVelocity = 0.5f;
	constraint.angular[1].maxMotorForce = 4.0f;
	start.constraints.push_back(constraint);
	start.accumulatedForce.assign(start.bodies.size(), {});

	World world = start;
	world.BeginFrame();
	World reference = world;
	reference.Step(1.0f / 60.0f);

	const StepData packed = PackStep(world, 1.0f / 60.0f);
	REQUIRE(packed.rowsFit);
	const std::vector<BodyRecord> integrated = RunStep(kernels.gpu, kernels.kernels, packed, kernels.kernels.solve.Get(),
		GroupCount(packed.islands.size()));
	World actual = world;
	UnpackBodies(integrated, actual);

	const Body& expected = reference.bodies[1];
	const Body& produced = actual.bodies[1];
	CHECK(Length(Sub(produced.worldTransform.origin, expected.worldTransform.origin)) < kPositionTolerance);
	CHECK(RotationError(produced.worldTransform.basis, expected.worldTransform.basis) < kRotationTolerance);
	CHECK(Length(Sub(produced.linearVelocity, expected.linearVelocity)) < kVelocityTolerance);
	CHECK(Length(Sub(produced.angularVelocity, expected.angularVelocity)) < kVelocityTolerance);

	World plain = world;
	for (RotationalMotor& motor : plain.constraints[0].angular) {
		motor.nonHookeanStiffness = 0.0f;
		motor.nonHookeanDamping = 0.0f;
	}
	plain.constraints[0].linear.nonHookeanStiffness = {};
	plain.constraints[0].linear.nonHookeanDamping = {};
	plain.Step(1.0f / 60.0f);
	CHECK(Length(Sub(plain.bodies[1].linearVelocity, expected.linearVelocity)) > 10.0f * kVelocityTolerance);
}
