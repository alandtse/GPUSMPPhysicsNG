#include "RigidGpuPacking.h"

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstddef>

namespace SmpPhysics::RigidGpu
{
	namespace
	{
		constexpr float kInfinity{ 3.40282347e+38f };
		constexpr std::size_t kNoIsland{ static_cast<std::size_t>(-1) };

		Float4 Wide(const RigidReference::Vec3& a_value)
		{
			return { a_value.x, a_value.y, a_value.z, 0.0f };
		}

		Float4 WideRow(const RigidReference::Mat3& a_basis, std::size_t a_row)
		{
			const RigidReference::Vec3& row = a_basis.row[a_row];
			return { row.x, row.y, row.z, 0.0f };
		}

		bool IsBodyIndex(int a_index, std::size_t a_count) noexcept
		{
			return a_index >= 0 && static_cast<std::size_t>(a_index) < a_count;
		}

		/** @brief Follows a union-find chain to its root, halving the path as it goes. */
		std::size_t FindRoot(std::vector<std::size_t>& a_parent, std::size_t a_index) noexcept
		{
			while (a_parent[a_index] != a_index)
				a_index = a_parent[a_index] = a_parent[a_parent[a_index]];
			return a_index;
		}

		void AppendIsland(StepData& a_data, const RigidReference::World& a_world, const std::vector<std::uint32_t>& a_constraints)
		{
			if (a_constraints.empty())
				return;
			const auto contactCount = static_cast<std::uint32_t>(std::count_if(a_constraints.begin(), a_constraints.end(),
				[&a_world](std::uint32_t a_index) { return a_world.constraints[a_index].explicitContact != 0u || a_world.constraints[a_index].contactSlot != 0u; }));
			IslandRecord island;
			island.range = { static_cast<std::uint32_t>(a_data.islandConstraints.size()), static_cast<std::uint32_t>(a_constraints.size()), 0u, contactCount };
			a_data.islands.push_back(island);
			a_data.islandConstraints.insert(a_data.islandConstraints.end(), a_constraints.begin(), a_constraints.end());
		}

		/** @brief Splits the enabled constraints into the islands they solve in, each list in stored order. */
		void PackIslands(StepData& a_data, const RigidReference::World& a_world)
		{
			const std::size_t bodyCount = a_world.bodies.size();
			const auto isDynamic = [&a_world](std::size_t a_index) { return !a_world.bodies[a_index].kinematic; };

			std::vector<std::size_t> parent(bodyCount);
			for (std::size_t index = 0; index < bodyCount; ++index)
				parent[index] = index;
			for (const RigidReference::Generic6DofSpring2& constraint : a_world.constraints) {
				if (!constraint.enabled || !IsBodyIndex(constraint.bodyA, bodyCount) || !IsBodyIndex(constraint.bodyB, bodyCount))
					continue;
				const auto bodyA = static_cast<std::size_t>(constraint.bodyA);
				const auto bodyB = static_cast<std::size_t>(constraint.bodyB);
				if (isDynamic(bodyA) && isDynamic(bodyB))
					parent[FindRoot(parent, bodyA)] = FindRoot(parent, bodyB);
			}

			std::vector<std::size_t> islandOfRoot(bodyCount, kNoIsland);
			std::size_t islandCount = 0;
			for (std::size_t index = 0; index < bodyCount; ++index) {
				if (!isDynamic(index))
					continue;
				const std::size_t root = FindRoot(parent, index);
				if (islandOfRoot[root] == kNoIsland)
					islandOfRoot[root] = islandCount++;
			}

			std::vector<std::vector<std::uint32_t>> lists(islandCount);
			std::vector<std::uint32_t> staticConstraints;
			for (std::size_t index = 0; index < a_world.constraints.size(); ++index) {
				const RigidReference::Generic6DofSpring2& constraint = a_world.constraints[index];
				if (!constraint.enabled || !IsBodyIndex(constraint.bodyA, bodyCount) || !IsBodyIndex(constraint.bodyB, bodyCount))
					continue;
				const auto bodyA = static_cast<std::size_t>(constraint.bodyA);
				const auto bodyB = static_cast<std::size_t>(constraint.bodyB);
				const auto entry = static_cast<std::uint32_t>(index);
				if (isDynamic(bodyA))
					lists[islandOfRoot[FindRoot(parent, bodyA)]].push_back(entry);
				else if (isDynamic(bodyB))
					lists[islandOfRoot[FindRoot(parent, bodyB)]].push_back(entry);
				else
					staticConstraints.push_back(entry);
			}

			for (const std::vector<std::uint32_t>& list : lists)
				AppendIsland(a_data, a_world, list);
			AppendIsland(a_data, a_world, staticConstraints);
		}

		/**
		 * @brief Greedy-colours every island's constraints, in stored order, and writes them back colour-major.
		 * @details Two constraints may share a colour when they do not share a dynamic body: a kinematic body's
		 *          solver-body delta stays zero, so concurrent rows through it write back what they read.
		 * @return False when an island needs more than kMaxColors classes.
		 */
		bool PackColors(StepData& a_data, const RigidReference::World& a_world)
		{
			a_data.coloredConstraints.clear();
			a_data.colorOffsets.assign(a_data.islands.size() * kMaxColors, 0u);

			// Islands are disjoint in dynamic bodies, so one mask per body serves every island.
			std::vector<std::uint32_t> bodyColors(a_world.bodies.size(), 0u);
			std::vector<std::uint32_t> colors;

			for (std::size_t island = 0; island < a_data.islands.size(); ++island) {
				const std::uint32_t first = a_data.islands[island].range.x;
				const std::uint32_t count = a_data.islands[island].range.y;
				colors.assign(count, 0u);

				std::uint32_t colorCount = 0;
				for (std::uint32_t entry = 0; entry < count; ++entry) {
					const RigidReference::Generic6DofSpring2& constraint = a_world.constraints[a_data.islandConstraints[first + entry]];
					const auto bodyA = static_cast<std::size_t>(constraint.bodyA);
					const auto bodyB = static_cast<std::size_t>(constraint.bodyB);
					const bool dynamicA = !a_world.bodies[bodyA].kinematic;
					const bool dynamicB = !a_world.bodies[bodyB].kinematic;
					const std::uint32_t forbidden = (dynamicA ? bodyColors[bodyA] : 0u) | (dynamicB ? bodyColors[bodyB] : 0u);
					std::uint32_t color = 0;
					while (color < kMaxColors && (forbidden & (1u << color)) != 0u)
						++color;
					if (color == kMaxColors)
						return false;
					if (dynamicA)
						bodyColors[bodyA] |= 1u << color;
					if (dynamicB)
						bodyColors[bodyB] |= 1u << color;
					colors[entry] = color;
					colorCount = std::max(colorCount, color + 1u);
				}

				std::uint32_t offset = 0;
				for (std::uint32_t color = 0; color < colorCount; ++color) {
					a_data.colorOffsets[island * kMaxColors + color] = offset;
					for (std::uint32_t entry = 0; entry < count; ++entry) {
						if (colors[entry] != color)
							continue;
						a_data.coloredConstraints.push_back(a_data.islandConstraints[first + entry]);
						++offset;
					}
				}
				a_data.islands[island].range.z = colorCount;
			}
			return true;
		}
	}

	IslandSizes SummariseIslands(const std::vector<IslandRecord>& a_islands)
	{
		IslandSizes sizes;
		for (const IslandRecord& island : a_islands) {
			const std::uint32_t count = island.range.y;
			if (count == 0)
				continue;
			constexpr std::array<std::uint32_t, IslandSizes::kBuckets - 1> kLimits{ 1, 4, 8, 16, 32, 64 };
			const auto bucket = static_cast<std::size_t>(std::ranges::find_if(kLimits, [count](std::uint32_t a_limit) { return count <= a_limit; }) - kLimits.begin());
			++sizes.islands[bucket];
			sizes.constraints[bucket] += count;
			++sizes.total;
			sizes.largest = std::max(sizes.largest, count);
			sizes.mostColors = std::max(sizes.mostColors, island.range.z);
		}
		return sizes;
	}

	StepData PackStep(const RigidReference::World& a_packed, float a_timeStep)
	{
		StepData data;
		RigidReference::World extended;
		const bool hasContacts = !a_packed.contacts.empty();
		if (hasContacts) {
			extended = a_packed;
			for (const RigidReference::World::Contact& contact : a_packed.contacts) {
				if (!IsBodyIndex(contact.bodyA, a_packed.bodies.size()) || !IsBodyIndex(contact.bodyB, a_packed.bodies.size()))
					continue;
				ContactRecord record;
				record.bodies = { contact.bodyA, contact.bodyB, 0, 0 };
				record.pointDistance = { contact.pointOnB.x, contact.pointOnB.y, contact.pointOnB.z, contact.distance };
				record.normalFriction = { contact.normalOnB.x, contact.normalOnB.y, contact.normalOnB.z, contact.friction };
				data.contacts.push_back(record);
				RigidReference::Generic6DofSpring2 pseudo;
				pseudo.bodyA = contact.bodyA;
				pseudo.bodyB = contact.bodyB;
				pseudo.explicitContact = static_cast<std::uint32_t>(data.contacts.size());
				extended.constraints.push_back(pseudo);
			}
		}
		const RigidReference::World& a_world = hasContacts ? extended : a_packed;
		const std::size_t bodyCount = a_world.bodies.size();
		const std::size_t constraintCount = a_world.constraints.size();

		data.bodies.reserve(bodyCount);
		for (std::size_t index = 0; index < bodyCount; ++index) {
			const RigidReference::Body& body = a_world.bodies[index];
			BodyRecord record;
			record.inertia = { body.inverseInertiaLocal.x, body.inverseInertiaLocal.y, body.inverseInertiaLocal.z, body.inverseMass };
			record.damping = { body.linearDamping, body.angularDamping, 0.0f, 0.0f };
			record.flags = { body.kinematic ? 1u : 0u, 0u, 0u, 0u };
			record.gravity = Wide(body.gravity);
			record.basis0 = WideRow(body.worldTransform.basis, 0);
			record.basis1 = WideRow(body.worldTransform.basis, 1);
			record.basis2 = WideRow(body.worldTransform.basis, 2);
			record.origin = Wide(body.worldTransform.origin);
			record.linearVelocity = Wide(body.linearVelocity);
			record.angularVelocity = Wide(body.angularVelocity);
			record.interpolationLinearVelocity = Wide(body.interpolationLinearVelocity);
			record.interpolationAngularVelocity = Wide(body.interpolationAngularVelocity);
			record.accumulatedForce = index < a_world.accumulatedForce.size() ? Wide(a_world.accumulatedForce[index]) : Float4{};
			data.bodies.push_back(record);
		}

		data.constraints.reserve(constraintCount);
		for (const RigidReference::Generic6DofSpring2& constraint : a_world.constraints) {
			ConstraintRecord record;
			record.header = { static_cast<std::uint32_t>(constraint.bodyA), static_cast<std::uint32_t>(constraint.bodyB),
				constraint.enabled ? 1u : 0u, static_cast<std::uint32_t>(constraint.rotationOrder) };
			record.constraintFlags = { static_cast<std::uint32_t>(constraint.flags), constraint.contactSlot, constraint.explicitContact,
				std::bit_cast<std::uint32_t>(constraint.contactFriction) };
			record.frameABasis0 = WideRow(constraint.frameA.basis, 0);
			record.frameABasis1 = WideRow(constraint.frameA.basis, 1);
			record.frameABasis2 = WideRow(constraint.frameA.basis, 2);
			record.frameAOrigin = Wide(constraint.frameA.origin);
			record.frameBBasis0 = WideRow(constraint.frameB.basis, 0);
			record.frameBBasis1 = WideRow(constraint.frameB.basis, 1);
			record.frameBBasis2 = WideRow(constraint.frameB.basis, 2);
			record.frameBOrigin = Wide(constraint.frameB.origin);

			const RigidReference::LinearAxisLimits& linear = constraint.linear;
			record.lower = Wide(linear.lower);
			record.upper = Wide(linear.upper);
			record.stopErp = Wide(linear.stopErp);
			record.stopCfm = Wide(linear.stopCfm);
			record.springStiffness = Wide(linear.springStiffness);
			record.springDamping = Wide(linear.springDamping);
			record.equilibrium = Wide(linear.equilibrium);
			record.bounce = Wide(linear.bounce);
			record.servoTarget = Wide(linear.servoTarget);
			record.targetVelocity = Wide(linear.targetVelocity);
			record.maxMotorForce = Wide(linear.maxMotorForce);
			record.motorErp = Wide(linear.motorErp);
			record.motorCfm = Wide(linear.motorCfm);
			record.nonHookeanStiffness = Wide(linear.nonHookeanStiffness);
			record.nonHookeanDamping = Wide(linear.nonHookeanDamping);

			std::uint32_t linearFlags[3]{};
			for (std::size_t axis = 0; axis < 3; ++axis) {
				linearFlags[axis] = AxisFlags(linear.springEnabled[axis], linear.springStiffnessLimited[axis],
					linear.springDampingLimited[axis], linear.motorEnabled[axis], linear.servoEnabled[axis]);
			}
			record.linearFlags = { linearFlags[0], linearFlags[1], linearFlags[2], 0u };

			const bool producesRows = constraint.enabled && IsBodyIndex(constraint.bodyA, bodyCount) && IsBodyIndex(constraint.bodyB, bodyCount);
			for (std::size_t axis = 0; axis < 3; ++axis) {
				const RigidReference::RotationalMotor& motor = constraint.angular[axis];
				AngularMotorRecord& packed = record.angular[axis];
				packed.limits = { motor.lower, motor.upper, motor.bounce, motor.stopErp };
				packed.spring = { motor.stopCfm, motor.springStiffness, motor.springDamping, motor.equilibrium };
				packed.motor = { motor.motorErp, motor.motorCfm, motor.targetVelocity, motor.maxMotorForce };
				packed.servo = { motor.servoTarget, motor.nonHookeanStiffness, motor.nonHookeanDamping, 0.0f };
				packed.flags = { AxisFlags(motor.springEnabled, motor.springStiffnessLimited, motor.springDampingLimited,
									 motor.motorEnabled, motor.servoEnabled),
					0u, 0u, 0u };
				if (producesRows && motor.motorEnabled && motor.servoEnabled)
					data.rowsFit = false;
				if (producesRows && linear.motorEnabled[axis] && linear.servoEnabled[axis])
					data.rowsFit = false;
			}
			data.constraints.push_back(record);
		}

		data.solverBodies.resize(bodyCount);
		data.rows.assign(constraintCount * kRowsPerConstraint, RowRecord{});
		for (RowRecord& row : data.rows)
			row.limits = { -kInfinity, kInfinity, 0.0f, 0.0f };
		data.rowMasks.assign(constraintCount * kAxisSlots, 0u);

		PackIslands(data, a_world);
		data.colorsFit = PackColors(data, a_world);
		if (!data.colorsFit) {
			data.coloredConstraints.clear();
			data.colorOffsets.clear();
		}

		data.constants.bodyCount = static_cast<std::uint32_t>(bodyCount);
		data.constants.constraintCount = static_cast<std::uint32_t>(constraintCount);
		data.constants.islandCount = static_cast<std::uint32_t>(data.islands.size());
		data.constants.iterations = static_cast<std::uint32_t>(std::max(a_world.solver.iterations, 0));
		data.constants.timeStep = a_timeStep;
		data.constants.framesPerSecond = 1.0f / a_timeStep;
		data.constants.erp = a_world.solver.erp;
		data.constants.globalCfm = a_world.solver.globalCfm;
		data.constants.damping = a_world.solver.damping;
		data.constants.residualThreshold = a_world.solver.leastSquaresResidualThreshold;
		data.constants.axisSlots = kAxisSlots;
		return data;
	}

	void UnpackBodies(const std::vector<BodyRecord>& a_bodies, RigidReference::World& a_world)
	{
		const std::size_t count = std::min(a_bodies.size(), a_world.bodies.size());
		for (std::size_t index = 0; index < count; ++index) {
			const BodyRecord& record = a_bodies[index];
			RigidReference::Body& body = a_world.bodies[index];
			body.worldTransform.basis = RigidReference::Mat3{
				RigidReference::Vec3{ record.basis0.x, record.basis0.y, record.basis0.z },
				RigidReference::Vec3{ record.basis1.x, record.basis1.y, record.basis1.z },
				RigidReference::Vec3{ record.basis2.x, record.basis2.y, record.basis2.z },
			};
			body.worldTransform.origin = { record.origin.x, record.origin.y, record.origin.z };
			body.linearVelocity = { record.linearVelocity.x, record.linearVelocity.y, record.linearVelocity.z };
			body.angularVelocity = { record.angularVelocity.x, record.angularVelocity.y, record.angularVelocity.z };
		}
	}
}
