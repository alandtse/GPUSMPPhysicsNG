#pragma once

#include <array>
#include <cstdint>
#include <vector>

namespace SmpPhysics::RigidReference
{
	/** @brief A three-component vector, as btVector3 keeps it. */
	struct Vec3
	{
		float x{};
		float y{};
		float z{};
	};

	/** @brief A rotation quaternion in Bullet's (x, y, z, w) order. */
	struct Quat
	{
		float x{};
		float y{};
		float z{};
		float w{ 1.0f };
	};

	/** @brief A 3x3 basis whose rows are btMatrix3x3's rows. */
	struct Mat3
	{
		Vec3 row[3]{ { 1.0f, 0.0f, 0.0f }, { 0.0f, 1.0f, 0.0f }, { 0.0f, 0.0f, 1.0f } };
	};

	/** @brief A rigid transform: basis and origin, as btTransform. */
	struct Transform
	{
		Mat3 basis{};
		Vec3 origin{};
	};

	/** @brief The basis a quaternion rotates with, as btMatrix3x3(const btQuaternion&) builds it. */
	[[nodiscard]] Mat3 BasisFromRotation(const Quat& a_rotation) noexcept;

	/** @brief The quaternion of a basis, as btMatrix3x3::getRotation computes it. */
	[[nodiscard]] Quat RotationFromBasis(const Mat3& a_basis) noexcept;

	/** @brief A transform from an origin and a rotation, the form the trace header stores. */
	[[nodiscard]] Transform MakeTransform(const Vec3& a_origin, const Quat& a_rotation) noexcept;

	/** @brief The Euler order btGeneric6DofSpring2Constraint extracts its angles with. */
	enum class RotateOrder : int
	{
		XYZ = 0,
		XZY,
		YXZ,
		YZX,
		ZXY,
		ZYX
	};

	/**
	 * @brief One rigid body with the state and properties a recorded trace header carries.
	 * @details centerOfMassOffset is recorded for trace parity only: Bullet keeps a rigid body's centre-of-mass
	 *          transform equal to its world transform, so the field is the identity and is not applied.
	 */
	struct Body
	{
		bool kinematic{};
		float inverseMass{};
		Vec3 inverseInertiaLocal{};
		float linearDamping{};
		float angularDamping{};
		/** @brief btRigidBody's friction, whose product with the other body's makes a contact's combined friction. */
		float friction{ 0.5f };
		/** @brief The body's gravity force for one frame, already scaled by its bone's gravity factor. */
		Vec3 gravity{};
		Transform centerOfMassOffset{};
		Transform worldTransform{};
		Vec3 linearVelocity{};
		Vec3 angularVelocity{};
		/** @brief The motion a kinematic body is integrated with; a dynamic body ignores both. */
		Vec3 interpolationLinearVelocity{};
		Vec3 interpolationAngularVelocity{};
	};

	/** @brief Bullet's default friction, which hdtSMP64's collision-mesh bodies keep because their files never set one. */
	inline constexpr float kSurfaceBodyFriction = 0.5f;

	/**
	 * @brief The trace header's "linear" block: btTranslationalLimitMotor2's limits and springs.
	 * @details springStiffnessLimited, springDampingLimited, bounce and the motor fields are not in the trace
	 *          header; they hold SMP's template defaults, which every recorded rig uses.
	 */
	struct LinearAxisLimits
	{
		Vec3 lower{};
		Vec3 upper{};
		Vec3 stopErp{ 0.2f, 0.2f, 0.2f };
		Vec3 stopCfm{};
		std::array<bool, 3> springEnabled{};
		Vec3 springStiffness{};
		Vec3 springDamping{};
		Vec3 equilibrium{};
		Vec3 nonHookeanStiffness{};
		Vec3 nonHookeanDamping{};
		std::array<bool, 3> springStiffnessLimited{ true, true, true };
		std::array<bool, 3> springDampingLimited{ true, true, true };
		Vec3 bounce{};
		std::array<bool, 3> motorEnabled{};
		std::array<bool, 3> servoEnabled{};
		Vec3 servoTarget{};
		Vec3 targetVelocity{};
		Vec3 maxMotorForce{};
		Vec3 motorErp{ 0.9f, 0.9f, 0.9f };
		Vec3 motorCfm{};
	};

	/**
	 * @brief One angular axis of a trace header's "angular" array: a btRotationalLimitMotor2.
	 * @details motorErp, motorCfm, targetVelocity, maxMotorForce and servoTarget are not in the trace header;
	 *          they hold SMP's template defaults, which every recorded rig uses.
	 */
	struct RotationalMotor
	{
		float lower{ 1.0f };
		float upper{ -1.0f };
		float bounce{};
		float stopErp{ 0.2f };
		float stopCfm{};
		bool springEnabled{};
		float springStiffness{};
		bool springStiffnessLimited{ true };
		float springDamping{};
		bool springDampingLimited{ true };
		float equilibrium{};
		float nonHookeanStiffness{};
		float nonHookeanDamping{};
		bool motorEnabled{};
		bool servoEnabled{};
		float motorErp{ 0.9f };
		float motorCfm{};
		float targetVelocity{};
		float maxMotorForce{ 6.0f };
		float servoTarget{};
	};

	/**
	 * @brief The per-axis ERP/CFM override bits SMP's constraint factory sets on the three linear axes.
	 * @details The trace header does not record m_flags; SMP calls setParam for axes 0..2 only, and writes the
	 *          angular motors' fields directly, so every recorded constraint carries exactly these bits.
	 */
	inline constexpr int kSmpLinearOverrideFlags{ 0x0fff };

	/** @brief One btGeneric6DofSpring2Constraint joining two bodies, as the trace header writes it. */
	struct Generic6DofSpring2
	{
		int bodyA{ -1 };
		int bodyB{ -1 };
		bool enabled{ true };
		Transform frameA{};
		Transform frameB{};
		RotateOrder rotationOrder{ RotateOrder::XYZ };
		LinearAxisLimits linear{};
		std::array<RotationalMotor, 3> angular{};
		int flags{ kSmpLinearOverrideFlags };
		/** @brief Nonzero marks a GPU-only contact row for bodyA: one more than the GPU solver contact slot that supplies its normal. */
		std::uint32_t contactSlot{};
		/** @brief The combined friction coefficient of a contactSlot row; zero adds no friction row. */
		float contactFriction{};
		/** @brief Nonzero marks a GPU-only contact row between bodyA and bodyB: one more than the index of the recorded World::Contact it solves. */
		std::uint32_t explicitContact{};
	};

	/**
	 * @brief The solver settings SMP's world is constructed with, as the trace header writes them.
	 * @details A step with no contacts and no friction rows only reads iterations, damping, erp, globalCfm,
	 *          splitImpulse and leastSquaresResidualThreshold; the remaining fields are carried for trace parity.
	 */
	struct SolverSettings
	{
		int iterations{ 16 };
		float tau{ 0.6f };
		float damping{ 1.0f };
		float friction{};
		float restitution{};
		float erp{ 0.2f };
		float erp2{ 0.15f };
		float globalCfm{ 0.001f };
		float warmstartingFactor{ 0.85f };
		float linearSlop{};
		float maxErrorReduction{ 20.0f };
		float sor{ 1.0f };
		bool splitImpulse{ true };
		float splitImpulseTurnErp{ 0.1f };
		int solverMode{ 256 };
		/** @brief The residual below which a step's iterations stop early; SMP's constructor value, not in the trace. */
		float leastSquaresResidualThreshold{ 0.0001f };
	};

	/**
	 * @brief A set of rigid bodies joined by 6-DOF spring constraints, stepped as hdtSMP64 steps them.
	 * @details Gravity is applied as a force by BeginFrame and held across that frame's Step calls until
	 *          EndFrame clears it, matching SMP's stepSimulation. Constraints are solved as a single island in
	 *          the order they are stored in, which is the order a recorded trace lists them in.
	 */
	struct World
	{
		/** @brief A manifold point between two bodies of the world, as SMP's narrow phase leaves it for the solver. */
		struct Contact
		{
			int bodyA{ -1 };
			int bodyB{ -1 };
			Vec3 pointOnB{};
			Vec3 normalOnB{};
			float distance{};  ///< Negative while the bodies penetrate.
			float friction{};  ///< The combined friction coefficient; zero adds no friction row.
		};

		std::vector<Body> bodies;
		std::vector<Generic6DofSpring2> constraints;
		/** @brief The contacts the next Step solves with the constraints; the caller refreshes them every step. */
		std::vector<Contact> contacts;
		SolverSettings solver;
		/** @brief One accumulated force per body, applied once per frame and cleared by EndFrame. */
		std::vector<Vec3> accumulatedForce;

		/** @brief Applies each body's gravity as a force for the frame, as SMP's applyGravity does. */
		void BeginFrame();

		/** @brief One fixed internal step: damp and predict, solve the constraints, integrate transforms. */
		void Step(float a_timeStep);

		/** @brief Clears the forces accumulated this frame. */
		void EndFrame();
	};
}
