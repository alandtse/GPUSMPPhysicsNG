#pragma once

// Shared loader for the recorded hdtSMP64 step traces under tests/fixtures/smp_traces: the recorded header, each
// step's pre- and post-step body state, and the geometry helpers a comparison needs.

#include "smp/RigidReference.h"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

namespace SmpPhysics::RigidTrace
{
	using nlohmann::json;

	inline constexpr std::size_t kStepHeaderBytes{ 20 };
	inline constexpr std::size_t kPreFloats{ 19 };
	inline constexpr std::size_t kPostFloats{ 13 };
	inline constexpr std::size_t kContactBytes{ 44 };
	inline constexpr std::size_t kContactBytesV3{ 68 };
	inline constexpr int kTraceVersion{ 2 };

	/** @brief One recorded step: its time step, and where its pre-state, post-state and body-pair contacts start. */
	struct StepView
	{
		float timeStep{};
		std::size_t preOffset{};
		std::size_t postOffset{};
		std::size_t contactOffset{};
		std::uint32_t contactCount{};
	};

	/** @brief The size of one recorded contact: version 3 appends the velocity of each body at the contact point. */
	inline std::size_t ContactStride(const json& a_header)
	{
		return a_header["version"].get<int>() >= 3 ? kContactBytesV3 : kContactBytes;
	}

	inline RigidReference::Vec3 ReadVec(const json& a_value)
	{
		return { a_value[0].get<float>(), a_value[1].get<float>(), a_value[2].get<float>() };
	}

	inline RigidReference::Transform ReadTransform(const json& a_value)
	{
		const json& rotation = a_value["rotation"];
		return RigidReference::MakeTransform(ReadVec(a_value["origin"]),
			{ rotation[0].get<float>(), rotation[1].get<float>(), rotation[2].get<float>(), rotation[3].get<float>() });
	}

	inline void ReadSolver(const json& a_header, RigidReference::SolverSettings& a_solver)
	{
		const json& solver = a_header["solver"];
		a_solver.iterations = solver["iterations"].get<int>();
		a_solver.tau = solver["tau"].get<float>();
		a_solver.damping = solver["damping"].get<float>();
		a_solver.friction = solver["friction"].get<float>();
		a_solver.restitution = solver["restitution"].get<float>();
		a_solver.erp = solver["erp"].get<float>();
		a_solver.erp2 = solver["erp2"].get<float>();
		a_solver.globalCfm = solver["globalCfm"].get<float>();
		a_solver.warmstartingFactor = solver["warmstartingFactor"].get<float>();
		a_solver.linearSlop = solver["linearSlop"].get<float>();
		a_solver.maxErrorReduction = solver["maxErrorReduction"].get<float>();
		a_solver.sor = solver["sor"].get<float>();
		a_solver.splitImpulse = solver["splitImpulse"].get<bool>();
		a_solver.splitImpulseTurnErp = solver["splitImpulseTurnErp"].get<float>();
		a_solver.solverMode = solver["solverMode"].get<int>();
	}

	inline RigidReference::RotationalMotor ReadMotor(const json& a_value)
	{
		RigidReference::RotationalMotor motor;
		motor.lower = a_value["lower"].get<float>();
		motor.upper = a_value["upper"].get<float>();
		motor.bounce = a_value["bounce"].get<float>();
		motor.stopErp = a_value["stopERP"].get<float>();
		motor.stopCfm = a_value["stopCFM"].get<float>();
		motor.springEnabled = a_value["springEnabled"].get<bool>();
		motor.springStiffness = a_value["springStiffness"].get<float>();
		motor.springStiffnessLimited = a_value["springStiffnessLimited"].get<bool>();
		motor.springDamping = a_value["springDamping"].get<float>();
		motor.springDampingLimited = a_value["springDampingLimited"].get<bool>();
		motor.equilibrium = a_value["equilibrium"].get<float>();
		motor.nonHookeanStiffness = a_value["nonHookeanStiffness"].get<float>();
		motor.nonHookeanDamping = a_value["nonHookeanDamping"].get<float>();
		motor.motorEnabled = a_value["motorEnabled"].get<bool>();
		motor.servoEnabled = a_value["servoEnabled"].get<bool>();
		return motor;
	}

	inline RigidReference::Generic6DofSpring2 ReadConstraint(const json& a_value)
	{
		RigidReference::Generic6DofSpring2 constraint;
		constraint.bodyA = a_value["bodyA"].get<int>();
		constraint.bodyB = a_value["bodyB"].get<int>();
		constraint.enabled = a_value["enabled"].get<bool>();
		constraint.frameA = ReadTransform(a_value["frameA"]);
		constraint.frameB = ReadTransform(a_value["frameB"]);
		constraint.rotationOrder = static_cast<RigidReference::RotateOrder>(a_value["rotationOrder"].get<int>());

		const json& linear = a_value["linear"];
		constraint.linear.lower = ReadVec(linear["lower"]);
		constraint.linear.upper = ReadVec(linear["upper"]);
		constraint.linear.stopErp = ReadVec(linear["stopERP"]);
		constraint.linear.stopCfm = ReadVec(linear["stopCFM"]);
		for (int axis = 0; axis < 3; ++axis)
			constraint.linear.springEnabled[static_cast<std::size_t>(axis)] = linear["springEnabled"][axis].get<bool>();
		constraint.linear.springStiffness = ReadVec(linear["springStiffness"]);
		constraint.linear.springDamping = ReadVec(linear["springDamping"]);
		constraint.linear.equilibrium = ReadVec(linear["equilibrium"]);
		constraint.linear.nonHookeanStiffness = ReadVec(linear["nonHookeanStiffness"]);
		constraint.linear.nonHookeanDamping = ReadVec(linear["nonHookeanDamping"]);

		for (std::size_t axis = 0; axis < 3; ++axis)
			constraint.angular[axis] = ReadMotor(a_value["angular"][axis]);
		return constraint;
	}

	inline RigidReference::World ReadWorld(const json& a_header)
	{
		RigidReference::World world;
		ReadSolver(a_header, world.solver);
		for (const json& entry : a_header["bodies"]) {
			RigidReference::Body body;
			body.kinematic = entry["kinematic"].get<bool>();
			body.inverseMass = entry["inverseMass"].get<float>();
			body.inverseInertiaLocal = ReadVec(entry["inverseInertiaLocal"]);
			body.linearDamping = entry["linearDamping"].get<float>();
			body.angularDamping = entry["angularDamping"].get<float>();
			// The trace stores the gravity acceleration; Bullet keeps it as a force, acceleration times mass.
			const float mass = body.inverseMass != 0.0f ? 1.0f / body.inverseMass : 0.0f;
			const RigidReference::Vec3 acceleration = ReadVec(entry["gravity"]);
			body.gravity = { acceleration.x * mass, acceleration.y * mass, acceleration.z * mass };
			world.bodies.push_back(body);
		}
		for (const json& entry : a_header["constraints"]) {
			if (entry["type"].get<std::string>() == "generic6dofSpring2")
				world.constraints.push_back(ReadConstraint(entry));
		}
		return world;
	}

	inline std::filesystem::path TraceDirectory()
	{
		return std::filesystem::path(SMPPHYSICS_FIXTURE_DIR) / "smp_traces";
	}

	inline std::vector<char> ReadFile(const std::filesystem::path& a_path)
	{
		std::ifstream stream(a_path, std::ios::binary);
		return { std::istreambuf_iterator<char>(stream), std::istreambuf_iterator<char>() };
	}

	inline const float* FloatsAt(const std::vector<char>& a_bytes, std::size_t a_offset)
	{
		return reinterpret_cast<const float*>(a_bytes.data() + a_offset);
	}

	/** @brief Walks the step records, recording where each step's state and contacts start. */
	inline std::vector<StepView> ReadSteps(const std::vector<char>& a_bytes, std::size_t a_bodyCount, std::size_t a_stepCount,
		std::size_t a_contactBytes = kContactBytes)
	{
		const std::size_t stateBytes = a_bodyCount * (kPreFloats + kPostFloats) * sizeof(float);
		std::vector<StepView> steps;
		steps.reserve(a_stepCount);
		std::size_t offset = 0;
		for (std::size_t step = 0; step < a_stepCount; ++step) {
			StepView view;
			view.timeStep = FloatsAt(a_bytes, offset + 4)[0];
			view.preOffset = offset + kStepHeaderBytes;
			view.postOffset = offset + kStepHeaderBytes + a_bodyCount * kPreFloats * sizeof(float);
			std::memcpy(&view.contactCount, a_bytes.data() + offset + kStepHeaderBytes + stateBytes, sizeof(view.contactCount));
			view.contactOffset = offset + kStepHeaderBytes + stateBytes + sizeof(view.contactCount);
			steps.push_back(view);
			offset = view.contactOffset + view.contactCount * a_contactBytes;
		}
		return steps;
	}

	/** @brief The step's recorded contacts between bodies of the trace; those against a body the trace left out are dropped. */
	inline std::vector<RigidReference::World::Contact> ReadContacts(const std::vector<char>& a_bytes, const StepView& a_step, std::size_t a_contactBytes)
	{
		std::vector<RigidReference::World::Contact> contacts;
		const char* record = a_bytes.data() + a_step.contactOffset;
		for (std::uint32_t index = 0; index < a_step.contactCount; ++index, record += a_contactBytes) {
			std::int32_t bodies[2];
			std::memcpy(bodies, record, sizeof(bodies));
			if (bodies[0] < 0 || bodies[1] < 0)
				continue;
			float values[9];
			std::memcpy(values, record + sizeof(bodies), sizeof(values));
			RigidReference::World::Contact contact;
			contact.bodyA = bodies[0];
			contact.bodyB = bodies[1];
			contact.pointOnB = { values[0], values[1], values[2] };
			contact.normalOnB = { values[3], values[4], values[5] };
			contact.distance = values[6];
			contact.friction = values[8];
			contacts.push_back(contact);
		}
		return contacts;
	}

	/**
	 * @brief Adds, for each contact against a body the trace left out, a kinematic stand-in at the contact point carrying
	 *        the velocity SMP recorded for that side, and the contact against it.
	 * @details A version 3 trace records both sides' velocity at the point, which is all the solver reads of a body it
	 *          cannot move. Call it on a world that holds only the trace's bodies: the stand-ins are appended after them.
	 */
	inline void AppendExternalContacts(RigidReference::World& a_world, const std::vector<char>& a_bytes, const StepView& a_step, std::size_t a_contactBytes)
	{
		if (a_contactBytes != kContactBytesV3)
			return;
		const char* record = a_bytes.data() + a_step.contactOffset;
		for (std::uint32_t index = 0; index < a_step.contactCount; ++index, record += a_contactBytes) {
			std::int32_t bodies[2];
			std::memcpy(bodies, record, sizeof(bodies));
			if ((bodies[0] < 0) == (bodies[1] < 0))
				continue;
			float values[15];
			std::memcpy(values, record + sizeof(bodies), sizeof(values));
			const int outside = bodies[0] < 0 ? 0 : 1;
			RigidReference::Body standIn;
			standIn.kinematic = true;
			standIn.worldTransform = RigidReference::MakeTransform({ values[0], values[1], values[2] }, { 0.0f, 0.0f, 0.0f, 1.0f });
			const RigidReference::Vec3 velocity{ values[9 + 3 * outside], values[10 + 3 * outside], values[11 + 3 * outside] };
			standIn.linearVelocity = velocity;
			standIn.interpolationLinearVelocity = velocity;
			RigidReference::World::Contact contact;
			contact.bodyA = bodies[0];
			contact.bodyB = bodies[1];
			(outside == 0 ? contact.bodyA : contact.bodyB) = static_cast<int>(a_world.bodies.size());
			contact.pointOnB = { values[0], values[1], values[2] };
			contact.normalOnB = { values[3], values[4], values[5] };
			contact.distance = values[6];
			contact.friction = values[8];
			a_world.bodies.push_back(standIn);
			a_world.contacts.push_back(contact);
		}
	}

	/** @brief Applies one step's recorded pre-step body state, which is what both sides of a comparison start from. */
	inline void ApplyPreState(RigidReference::World& a_world, const float* a_pre)
	{
		for (std::size_t body = 0; body < a_world.bodies.size(); ++body) {
			const float* values = a_pre + body * kPreFloats;
			RigidReference::Body& state = a_world.bodies[body];
			state.worldTransform = RigidReference::MakeTransform({ values[0], values[1], values[2] }, { values[3], values[4], values[5], values[6] });
			state.linearVelocity = { values[7], values[8], values[9] };
			state.angularVelocity = { values[10], values[11], values[12] };
			state.interpolationLinearVelocity = { values[13], values[14], values[15] };
			state.interpolationAngularVelocity = { values[16], values[17], values[18] };
		}
	}

	inline RigidReference::Vec3 Sub(const RigidReference::Vec3& a_lhs, const RigidReference::Vec3& a_rhs)
	{
		return { a_lhs.x - a_rhs.x, a_lhs.y - a_rhs.y, a_lhs.z - a_rhs.z };
	}

	inline float Length(const RigidReference::Vec3& a_value)
	{
		return std::sqrt(a_value.x * a_value.x + a_value.y * a_value.y + a_value.z * a_value.z);
	}

	/** @brief The angle, in radians, of the rotation taking a_lhs to a_rhs. */
	inline float AngleBetween(const RigidReference::Quat& a_lhs, const RigidReference::Quat& a_rhs)
	{
		const float dot = std::fabs(a_lhs.x * a_rhs.x + a_lhs.y * a_rhs.y + a_lhs.z * a_rhs.z + a_lhs.w * a_rhs.w);
		return 2.0f * std::acos(std::min(dot, 1.0f));
	}
}
