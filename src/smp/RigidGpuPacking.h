#pragma once

#include "smp/RigidReference.h"

#include <array>
#include <cstdint>
#include <vector>

/**
 * @brief The step-time records the rigid-body compute shaders read and write, packed from a RigidReference::World.
 *
 * Pure data and pure functions: no D3D11, no RE/SKSE, so a step can be packed and its result unpacked without a
 * device. The record layout is mirrored by RigidSolverCS.hlsl -- a field added here needs the matching field there,
 * in the same order.
 */
namespace SmpPhysics::RigidGpu
{
	struct Float4
	{
		float x{};
		float y{};
		float z{};
		float w{};
	};

	struct Uint4
	{
		std::uint32_t x{};
		std::uint32_t y{};
		std::uint32_t z{};
		std::uint32_t w{};
	};

	struct Int4
	{
		std::int32_t x{};
		std::int32_t y{};
		std::int32_t z{};
		std::int32_t w{};
	};

	/** @brief One body's properties, its pre-step state and the frame's force, as RigidPredict reads them. */
	struct BodyRecord
	{
		Float4 inertia{};  ///< xyz the body-frame inverse inertia, w the inverse mass
		Float4 damping{};  ///< x linear damping, y angular damping
		Uint4 flags{};     ///< x 1 when the body is kinematic
		Float4 gravity{};  ///< xyz the body's frame gravity force
		Float4 basis0{};   ///< xyz world basis row 0
		Float4 basis1{};   ///< xyz world basis row 1
		Float4 basis2{};   ///< xyz world basis row 2
		Float4 origin{};   ///< xyz world origin
		Float4 linearVelocity{};
		Float4 angularVelocity{};
		Float4 interpolationLinearVelocity{};
		Float4 interpolationAngularVelocity{};
		Float4 accumulatedForce{};  ///< xyz the force BeginFrame accumulated for the frame
		Float4 accumulatedTorque{};
		Float4 grabLocal{};   ///< xyz the held point in the body frame, w the pin's stiffness
		Float4 grabTarget{};  ///< xyz where the hand holds the point, w 1 for a pin
	};
	static_assert(sizeof(BodyRecord) == 256);

	/** @brief One angular axis's limits, springs and motor, as HairRigidSolver.hlsli's MotorSettings holds them. */
	struct AngularMotorRecord
	{
		Float4 limits{};  ///< x lower, y upper, z bounce, w stop ERP
		Float4 spring{};  ///< x stop CFM, y spring stiffness, z spring damping, w equilibrium
		Float4 motor{};   ///< x motor ERP, y motor CFM, z target velocity, w maximum motor force
		Float4 servo{};   ///< x servo target, y non-Hookean stiffness, z non-Hookean damping
		Uint4 flags{};    ///< x the axis's kAxis* bits
	};
	static_assert(sizeof(AngularMotorRecord) == 80);

	/** @brief One btGeneric6DofSpring2Constraint, as RigidSetup reads it. */
	struct ConstraintRecord
	{
		Uint4 header{};           ///< x body A, y body B, z 1 when enabled, w rotation order
		Uint4 constraintFlags{};  ///< x the per-axis ERP/CFM override bits, SMP's kSmpLinearOverrideFlags
		Float4 frameABasis0{};    ///< xyz basis row 0 of the joint frame in body A
		Float4 frameABasis1{};
		Float4 frameABasis2{};
		Float4 frameAOrigin{};  ///< xyz
		Float4 frameBBasis0{};
		Float4 frameBBasis1{};
		Float4 frameBBasis2{};
		Float4 frameBOrigin{};  ///< xyz
		Float4 lower{};
		Float4 upper{};
		Float4 stopErp{};
		Float4 stopCfm{};
		Float4 springStiffness{};
		Float4 springDamping{};
		Float4 equilibrium{};
		Float4 bounce{};
		Float4 servoTarget{};
		Float4 targetVelocity{};
		Float4 maxMotorForce{};
		Float4 motorErp{};
		Float4 motorCfm{};
		Float4 nonHookeanStiffness{};
		Float4 nonHookeanDamping{};
		Uint4 linearFlags{};  ///< xyz each linear axis's kAxis* bits
		std::array<AngularMotorRecord, 3> angular{};
	};
	static_assert(sizeof(ConstraintRecord) == 656);

	/** @brief btSolverBody's fields, which the solve advances one row at a time. */
	struct SolverBodyRecord
	{
		Float4 deltaLinearVelocity{};
		Float4 deltaAngularVelocity{};
		Float4 linearVelocity{};
		Float4 angularVelocity{};
		Float4 externalForceImpulse{};
		Float4 externalTorqueImpulse{};
		Float4 inverseMassVector{};
		Float4 pushVelocity{};  ///< the split-impulse pass's linear velocity, which moves the body without adding to its velocity
		Float4 turnVelocity{};
	};
	static_assert(sizeof(SolverBodyRecord) == 144);

	/** @brief One btSolverConstraint, at its constrained slot in the row buffer. */
	struct RowRecord
	{
		Float4 normal1{};
		Float4 relpos1CrossNormal{};
		Float4 normal2{};
		Float4 relpos2CrossNormal{};
		Float4 angularComponentA{};
		Float4 angularComponentB{};
		Float4 impulse{};  ///< x rhs, y cfm, z jacDiagABInv, w applied impulse
		Float4 limits{};   ///< x lower limit, y upper limit; a contact's normal row z split-impulse rhs, w applied push; its friction row z coefficient
		Int4 bodies{};     ///< x solver body A, y solver body B
	};
	static_assert(sizeof(RowRecord) == 144);

	/** @brief One recorded manifold point, as the shader's HairRigidExplicitContact. */
	struct ContactRecord
	{
		Int4 bodies{};            ///< x body A, y body B
		Float4 pointDistance{};   ///< xyz point on B, w distance
		Float4 normalFriction{};  ///< xyz normal on B, w friction coefficient
	};
	static_assert(sizeof(ContactRecord) == 48);

	/** @brief One island's slice of the constraint list, back to back. */
	struct IslandRecord
	{
		Uint4 range{};  ///< x first constraint index in the island list, y count, z colour classes the island's constraints take, w contact constraints
	};
	static_assert(sizeof(IslandRecord) == 16);

	/** @brief The step's constant buffer, as HairRigidParams. */
	struct StepConstants
	{
		std::uint32_t bodyCount{};
		std::uint32_t constraintCount{};
		std::uint32_t islandCount{};
		std::uint32_t iterations{};
		float timeStep{};
		float framesPerSecond{};
		float erp{};
		float globalCfm{};
		float damping{};
		float residualThreshold{};
		std::uint32_t axisSlots{};  ///< kAxisSlots, passed as a value so the shader's axis loop stays a loop
		std::uint32_t padding{};
	};
	static_assert(sizeof(StepConstants) == 48);

	/** @brief Rows one axis's slot holds: a stop pair, a non-servo motor, a servo motor and a spring. Mirrors the shader's kRowsPerAxis. */
	inline constexpr std::uint32_t kRowsPerAxis{ 5 };

	/** @brief Slots per constraint: the three angular axes in their processing order, then the three linear ones. */
	inline constexpr std::uint32_t kAxisSlots{ 6 };

	/** @brief Rows one constraint's slot holds. */
	inline constexpr std::uint32_t kRowsPerConstraint{ kRowsPerAxis * kAxisSlots };

	/** @brief The colour classes one island's constraints may take; a step that needs more falls back to the exact kernel. */
	inline constexpr std::uint32_t kMaxColors{ 32 };

	/** @brief Per-axis configuration bits, mirroring the shader's kAxis* constants. */
	inline constexpr std::uint32_t kAxisSpring{ 1u };
	inline constexpr std::uint32_t kAxisSpringStiffnessLimited{ 2u };
	inline constexpr std::uint32_t kAxisSpringDampingLimited{ 4u };
	inline constexpr std::uint32_t kAxisMotor{ 8u };
	inline constexpr std::uint32_t kAxisServo{ 16u };

	/** @brief One step's buffers, in the order the kernels read them. */
	struct StepData
	{
		std::vector<BodyRecord> bodies;
		std::vector<ConstraintRecord> constraints;
		std::vector<SolverBodyRecord> solverBodies;  ///< scratch, one per body, which RigidPredict fills
		std::vector<RowRecord> rows;                 ///< constraintCount * kRowsPerConstraint
		std::vector<std::uint32_t> rowMasks;         ///< constraintCount * kAxisSlots, one row-slot bit per entry, which RigidSetup fills
		std::vector<ContactRecord> contacts;         ///< the recorded contacts, which each carries a constraint after the joints
		std::vector<IslandRecord> islands;
		std::vector<std::uint32_t> islandConstraints;  ///< each island's constraint indices, back to back
		/** @brief Each island's constraint indices again, colour-major and then in stored order, for the coloured kernel. */
		std::vector<std::uint32_t> coloredConstraints;
		/** @brief islandCount * kMaxColors: each island's per-colour start offset inside its own slice of the two lists. */
		std::vector<std::uint32_t> colorOffsets;
		StepConstants constants;
		/** @brief False when a constraint needs more rows than its slot holds, which the reference cannot express either. */
		bool rowsFit{ true };
		/** @brief False when an island needs more than kMaxColors classes, which empties the two coloured lists. */
		bool colorsFit{ true };
	};

	/** @brief The bits one axis's spring, motor and servo settings pack into. */
	[[nodiscard]] constexpr std::uint32_t AxisFlags(bool a_spring, bool a_springStiffnessLimited, bool a_springDampingLimited,
		bool a_motor, bool a_servo) noexcept
	{
		return (a_spring ? kAxisSpring : 0u) | (a_springStiffnessLimited ? kAxisSpringStiffnessLimited : 0u) |
		       (a_springDampingLimited ? kAxisSpringDampingLimited : 0u) | (a_motor ? kAxisMotor : 0u) |
		       (a_servo ? kAxisServo : 0u);
	}

	/**
	 * @brief Packs one step: the bodies after BeginFrame, the constraint parameters, the solver scratch and the islands.
	 * @param a_world The world after BeginFrame, so accumulatedForce holds the frame's forces.
	 * @param a_timeStep The time step World::Step is given, which must be positive.
	 */
	[[nodiscard]] StepData PackStep(const RigidReference::World& a_world, float a_timeStep);

	/** @brief How the islands split by constraint count, to judge which solver kernel each size suits. */
	struct IslandSizes
	{
		static constexpr std::size_t kBuckets = 7;  ///< 1, 2-4, 5-8, 9-16, 17-32, 33-64, 65 and more constraints
		std::array<std::uint32_t, kBuckets> islands{};
		std::array<std::uint32_t, kBuckets> constraints{};
		std::uint32_t total{};
		std::uint32_t largest{};
		std::uint32_t mostColors{};
	};

	/** @brief Counts each island and its constraints into the size bucket its constraint count falls in. */
	[[nodiscard]] IslandSizes SummariseIslands(const std::vector<IslandRecord>& a_islands);
}
