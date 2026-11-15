#ifndef __HAIR_RIGID_SOLVER_DEPENDENCY_HLSL__
#define __HAIR_RIGID_SOLVER_DEPENDENCY_HLSL__

// HLSL counterpart of RigidReference.cpp (hdtSMP64's Bullet 3.25 step); names and operation order match so the two diff.
//
// FXC cannot index a struct's array member at runtime, so arrays are named fields with accessors, and an inout
// parameter is stored back whole.

namespace HairRigidSolver
{
	static const float kEpsilon = 1.19209290e-7f;
	static const float kInfinity = 3.40282347e+38f;
	/** @brief The open low end of a solver row's limit range. */
	static const float kNegInfinity = -3.40282347e+38f;
	static const float kPi = 3.14159265358979323846f;
	static const float kTwoPi = 2.0f * kPi;
	static const float kHalfPi = kPi * 0.5f;
	static const float kAngularMotionThreshold = 0.5f * kHalfPi;
	static const float kAngularTaylorAngle = 0.001f;
	static const float kAngularTaylorCoefficient = 0.020833333333f;
	static const float kVelocityLimit = 1.0e9f;
	static const float kRotationViolationThreshold = 1.0e-3f;
	static const float kSpringFrequencyFraction = 0.25f;
	static const float kFuzzyEpsilon = 1.19209290e-7f;
	static const float kSpringStiffnessDivisor = 16.0f;

	/** @brief btGeneric6DofSpring2Constraint's BT_6DOF_FLAGS_* per-axis bits. */
	static const int kFlagCfmStop = 1;
	static const int kFlagErpStop = 2;
	static const int kFlagCfmMotor = 4;
	static const int kFlagErpMotor = 8;
	static const int kFlagAxisShift = 4;

	/** @brief Per-axis configuration bits of the packed records, which the reference holds as separate bools. */
	static const uint kAxisSpring = 1u;
	static const uint kAxisSpringStiffnessLimited = 2u;
	static const uint kAxisSpringDampingLimited = 4u;
	static const uint kAxisMotor = 8u;
	static const uint kAxisServo = 16u;

	/** @brief Euler orders btGeneric6DofSpring2Constraint extracts its angles with, in the reference's enum order. */
	static const int kRotateOrderXYZ = 0;
	static const int kRotateOrderXZY = 1;
	static const int kRotateOrderYXZ = 2;
	static const int kRotateOrderYZX = 3;
	static const int kRotateOrderZXY = 4;
	static const int kRotateOrderZYX = 5;

	/** @brief One row slot's bit in AxisRows::present, in the order the solver walks the slots. */
	static const uint kRowLimitLower = 1u;
	static const uint kRowLimitUpper = 2u;
	static const uint kRowMotor = 4u;
	static const uint kRowServo = 8u;
	static const uint kRowSpring = 16u;

	/** @brief Rows one axis's slot holds: a stop pair, a non-servo motor, a servo motor and a spring. */
	static const int kRowsPerAxis = 5;

	/** @brief Slots per constraint: the three angular axes in their processing order, then the three linear ones. */
	static const int kAxisSlots = 6;

	/** @brief Rows one constraint's slot holds. */
	static const int kRowsPerConstraint = kRowsPerAxis * kAxisSlots;

	/** @brief The colour classes one island's constraints may take, mirroring RigidGpuPacking.h's kMaxColors. */
	static const uint kMaxColors = 32u;

	/** @brief A rotation quaternion in Bullet's (x, y, z, w) order. */
	struct Quat
	{
		float x;
		float y;
		float z;
		float w;
	};

	/** @brief A 3x3 basis whose rows are btMatrix3x3's rows. */
	struct Mat3
	{
		float3 row0;
		float3 row1;
		float3 row2;
	};

	/** @brief A rigid transform: basis and origin, as btTransform. */
	struct Transform
	{
		Mat3 basis;
		float3 origin;
	};

	/** @brief One rigid body, as the reference's Body keeps it for a step. */
	struct Body
	{
		bool kinematic;
		float inverseMass;
		float3 inverseInertiaLocal;
		float linearDamping;
		float angularDamping;
		float3 gravity;
		Transform worldTransform;
		float3 linearVelocity;
		float3 angularVelocity;
		float3 interpolationLinearVelocity;
		float3 interpolationAngularVelocity;
	};

	/** @brief One axis's limits and springs, as the reference's LinearAxisLimits holds the fields the step reads. */
	struct LinearLimits
	{
		float3 lower;
		float3 upper;
		float3 stopErp;
		float3 stopCfm;
		float3 springStiffness;
		float3 springDamping;
		float3 equilibrium;
		float3 bounce;
		float3 servoTarget;
		float3 targetVelocity;
		float3 maxMotorForce;
		float3 motorErp;
		float3 motorCfm;
		float3 nonHookeanStiffness;
		float3 nonHookeanDamping;
		uint3 flags;
	};

	/** @brief One angular axis, as the reference's RotationalMotor holds the fields the step reads. */
	struct MotorSettings
	{
		float lower;
		float upper;
		float bounce;
		float stopErp;
		float stopCfm;
		float springStiffness;
		float springDamping;
		float equilibrium;
		float motorErp;
		float motorCfm;
		float targetVelocity;
		float maxMotorForce;
		float servoTarget;
		float nonHookeanStiffness;
		float nonHookeanDamping;
		uint flags;
	};

	/** @brief One btGeneric6DofSpring2Constraint's configuration, as the step reads it. */
	struct ConstraintConfig
	{
		Mat3 frameABasis;
		float3 frameAOrigin;
		Mat3 frameBBasis;
		float3 frameBOrigin;
		int rotationOrder;
		int flags;
		LinearLimits linearLimits;
		MotorSettings angular0;
		MotorSettings angular1;
		MotorSettings angular2;
	};

	/** @brief The solver settings a step reads. */
	struct SolverSettings
	{
		float erp;
		float globalCfm;
		float damping;
	};

	/** @brief btSolverBody: the velocity deltas the rows accumulate, plus the step's external impulses. */
	struct SolverBody
	{
		float3 deltaLinearVelocity;
		float3 deltaAngularVelocity;
		float3 linearVelocity;
		float3 angularVelocity;
		float3 externalForceImpulse;
		float3 externalTorqueImpulse;
		float3 inverseMassVector;
	};

	/** @brief One body's state at the start of the solve, as btSequentialImpulseConstraintSolver::setup sees it. */
	struct BodyState
	{
		Transform transform;
		float3 linearVelocity;
		float3 angularVelocity;
		Mat3 inverseInertiaWorld;
		float inverseMass;
	};

	/** @brief One btSolverConstraint. */
	struct SolverRow
	{
		float3 normal1;
		float3 relpos1CrossNormal;
		float3 normal2;
		float3 relpos2CrossNormal;
		float3 angularComponentA;
		float3 angularComponentB;
		float rhs;
		float cfm;
		float jacDiagABInv;
		float appliedImpulse;
		float lowerLimit;
		float upperLimit;
		int solverBodyA;
		int solverBodyB;
	};

	/** @brief A solver row carrying the reference's defaults: no impulse, and limits open on both sides. */
	SolverRow ZeroRow()
	{
		SolverRow result;
		result.normal1 = 0.0f;
		result.relpos1CrossNormal = 0.0f;
		result.normal2 = 0.0f;
		result.relpos2CrossNormal = 0.0f;
		result.angularComponentA = 0.0f;
		result.angularComponentB = 0.0f;
		result.rhs = 0.0f;
		result.cfm = 0.0f;
		result.jacDiagABInv = 0.0f;
		result.appliedImpulse = 0.0f;
		result.lowerLimit = kNegInfinity;
		result.upperLimit = kInfinity;
		result.solverBodyA = 0;
		result.solverBodyB = 0;
		return result;
	}

	/** @brief btSolverBody value-initialised, the state a step starts every body from. */
	SolverBody ZeroSolverBody()
	{
		SolverBody result;
		result.deltaLinearVelocity = 0.0f;
		result.deltaAngularVelocity = 0.0f;
		result.linearVelocity = 0.0f;
		result.angularVelocity = 0.0f;
		result.externalForceImpulse = 0.0f;
		result.externalTorqueImpulse = 0.0f;
		result.inverseMassVector = 0.0f;
		return result;
	}

	/** @brief The runtime fields of a btRotationalLimitMotor2, rebuilt for every step. */
	struct MotorState
	{
		float position;
		float error;
		float errorHi;
		int limit;
	};

	/** @brief A working copy of btRotationalLimitMotor2, the shape GetLimitMotorInfo2 consumes. */
	struct LimitMotor
	{
		float loLimit;
		float hiLimit;
		float bounce;
		float stopErp;
		float stopCfm;
		float motorErp;
		float motorCfm;
		bool enableMotor;
		float targetVelocity;
		float maxMotorForce;
		bool servoMotor;
		float servoTarget;
		bool enableSpring;
		float springStiffness;
		bool springStiffnessLimited;
		float springDamping;
		bool springDampingLimited;
		float equilibriumPoint;
		float nonHookeanDamping;
		float nonHookeanStiffness;
		float currentLimitError;
		float currentLimitErrorHi;
		float currentPosition;
		int currentLimit;
	};

	/** @brief The runtime fields of a btGeneric6DofSpring2Constraint for one step. */
	struct Joint
	{
		Transform calculatedA;
		Transform calculatedB;
		float3 axisAngleDiff;
		float3 axis0;
		float3 axis1;
		float3 axis2;
		float3 linearDiff;
		float3 linearError;
		float3 linearErrorHi;
		int linearLimit0;
		int linearLimit1;
		int linearLimit2;
		MotorState angular0;
		MotorState angular1;
		MotorState angular2;
		float factA;
		float factB;
		bool hasStaticBody;
	};

	/** @brief A motor state cleared, as the reference's Joint() default-initialises every axis. */
	MotorState ZeroMotorState()
	{
		MotorState result;
		result.position = 0.0f;
		result.error = 0.0f;
		result.errorHi = 0.0f;
		result.limit = 0;
		return result;
	}

	/** @brief One axis's rows, at fixed slots in the order the solver walks them. */
	struct AxisRows
	{
		SolverRow limitLower;  ///< the single row of a locked axis, or the first row of a limited one
		SolverRow limitUpper;  ///< the second row of a limited axis
		SolverRow motor;
		SolverRow servo;
		SolverRow spring;
		uint present;  ///< one bit per slot that holds a row
	};

	/** @brief An empty axis block. */
	AxisRows ZeroAxisRows()
	{
		const SolverRow empty = ZeroRow();
		AxisRows result;
		result.limitLower = empty;
		result.limitUpper = empty;
		result.motor = empty;
		result.servo = empty;
		result.spring = empty;
		result.present = 0u;
		return result;
	}

	/** @brief A joint with every runtime field cleared, as the reference's Joint() default-initialises it. */
	Joint ZeroJoint()
	{
		Joint result;
		result.calculatedA.basis.row0 = 0.0f;
		result.calculatedA.basis.row1 = 0.0f;
		result.calculatedA.basis.row2 = 0.0f;
		result.calculatedA.origin = 0.0f;
		result.calculatedB.basis.row0 = 0.0f;
		result.calculatedB.basis.row1 = 0.0f;
		result.calculatedB.basis.row2 = 0.0f;
		result.calculatedB.origin = 0.0f;
		result.axisAngleDiff = 0.0f;
		result.axis0 = 0.0f;
		result.axis1 = 0.0f;
		result.axis2 = 0.0f;
		result.linearDiff = 0.0f;
		result.linearError = 0.0f;
		result.linearErrorHi = 0.0f;
		result.linearLimit0 = 0;
		result.linearLimit1 = 0;
		result.linearLimit2 = 0;
		result.angular0 = ZeroMotorState();
		result.angular1 = ZeroMotorState();
		result.angular2 = ZeroMotorState();
		result.factA = 0.0f;
		result.factB = 0.0f;
		result.hasStaticBody = false;
		return result;
	}

	float3 GetAxis(in Joint a_joint, in int a_axis)
	{
		return a_axis == 0 ? a_joint.axis0 : (a_axis == 1 ? a_joint.axis1 : a_joint.axis2);
	}

	Joint SetAxis(in Joint a_joint, in int a_axis, in float3 a_value)
	{
		if (a_axis == 0)
			a_joint.axis0 = a_value;
		else if (a_axis == 1)
			a_joint.axis1 = a_value;
		else
			a_joint.axis2 = a_value;
		return a_joint;
	}

	int GetLinearLimit(in Joint a_joint, in int a_axis)
	{
		return a_axis == 0 ? a_joint.linearLimit0 : (a_axis == 1 ? a_joint.linearLimit1 : a_joint.linearLimit2);
	}

	Joint SetLinearLimit(in Joint a_joint, in int a_axis, in int a_value)
	{
		if (a_axis == 0)
			a_joint.linearLimit0 = a_value;
		else if (a_axis == 1)
			a_joint.linearLimit1 = a_value;
		else
			a_joint.linearLimit2 = a_value;
		return a_joint;
	}

	MotorState GetAngularState(in Joint a_joint, in int a_axis)
	{
		if (a_axis == 0)
			return a_joint.angular0;
		if (a_axis == 1)
			return a_joint.angular1;
		return a_joint.angular2;
	}

	Joint SetAngularState(in Joint a_joint, in int a_axis, in MotorState a_value)
	{
		if (a_axis == 0)
			a_joint.angular0 = a_value;
		else if (a_axis == 1)
			a_joint.angular1 = a_value;
		else
			a_joint.angular2 = a_value;
		return a_joint;
	}

	MotorSettings GetMotorSettings(in ConstraintConfig a_config, in int a_axis)
	{
		if (a_axis == 0)
			return a_config.angular0;
		if (a_axis == 1)
			return a_config.angular1;
		return a_config.angular2;
	}

	/** @brief Whether a packed per-axis flag word carries a bit. */
	bool HasFlag(in uint a_flags, in int a_bit)
	{
		return (a_flags & (uint)a_bit) != 0u;
	}

	/** @brief One component of a vector, as the reference's Component indexes a Vec3. */
	float Component(in float3 a_value, in int a_axis)
	{
		return a_axis == 0 ? a_value.x : (a_axis == 1 ? a_value.y : a_value.z);
	}

	/** @brief One component of a packed per-axis flag triple. */
	uint Component(in uint3 a_value, in int a_axis)
	{
		return a_axis == 0 ? a_value.x : (a_axis == 1 ? a_value.y : a_value.z);
	}

	/** @brief The vector with one component replaced, as the reference's SetComponent writes one. */
	float3 SetComponent(in float3 a_value, in int a_axis, in float a_component)
	{
		float3 result = a_value;
		if (a_axis == 0)
			result.x = a_component;
		else if (a_axis == 1)
			result.y = a_component;
		else
			result.z = a_component;
		return result;
	}

	float Dot(in float3 a_left, in float3 a_right)
	{
		return a_left.x * a_right.x + a_left.y * a_right.y + a_left.z * a_right.z;
	}

	float3 Cross(in float3 a_left, in float3 a_right)
	{
		return float3(a_left.y * a_right.z - a_left.z * a_right.y,
			a_left.z * a_right.x - a_left.x * a_right.z,
			a_left.x * a_right.y - a_left.y * a_right.x);
	}

	float Length(in float3 a_value)
	{
		return sqrt(Dot(a_value, a_value));
	}

	float3 Normalized(in float3 a_value)
	{
		return a_value / Length(a_value);
	}

	float3 GetRow(in Mat3 a_matrix, in int a_row)
	{
		return a_row == 0 ? a_matrix.row0 : (a_row == 1 ? a_matrix.row1 : a_matrix.row2);
	}

	/** @brief One component of a basis row, as the reference's GetMatrixElem indexes a Mat3. */
	float GetMatrixElem(in Mat3 a_matrix, in int a_index)
	{
		return Component(GetRow(a_matrix, a_index % 3), a_index / 3);
	}

	float3 Column(in Mat3 a_matrix, in int a_column)
	{
		return float3(Component(a_matrix.row0, a_column), Component(a_matrix.row1, a_column), Component(a_matrix.row2, a_column));
	}

	float3 MatVec(in Mat3 a_matrix, in float3 a_value)
	{
		return float3(Dot(a_matrix.row0, a_value), Dot(a_matrix.row1, a_value), Dot(a_matrix.row2, a_value));
	}

	Mat3 MatMul(in Mat3 a_left, in Mat3 a_right)
	{
		const float3 column0 = Column(a_right, 0);
		const float3 column1 = Column(a_right, 1);
		const float3 column2 = Column(a_right, 2);
		Mat3 result;
		result.row0 = float3(Dot(a_left.row0, column0), Dot(a_left.row0, column1), Dot(a_left.row0, column2));
		result.row1 = float3(Dot(a_left.row1, column0), Dot(a_left.row1, column1), Dot(a_left.row1, column2));
		result.row2 = float3(Dot(a_left.row2, column0), Dot(a_left.row2, column1), Dot(a_left.row2, column2));
		return result;
	}

	Mat3 MatAdd(in Mat3 a_left, in Mat3 a_right)
	{
		Mat3 result;
		result.row0 = a_left.row0 + a_right.row0;
		result.row1 = a_left.row1 + a_right.row1;
		result.row2 = a_left.row2 + a_right.row2;
		return result;
	}

	Mat3 MatSub(in Mat3 a_left, in Mat3 a_right)
	{
		Mat3 result;
		result.row0 = a_left.row0 - a_right.row0;
		result.row1 = a_left.row1 - a_right.row1;
		result.row2 = a_left.row2 - a_right.row2;
		return result;
	}

	Mat3 MatScale(in Mat3 a_matrix, in float a_scale)
	{
		Mat3 result;
		result.row0 = a_matrix.row0 * a_scale;
		result.row1 = a_matrix.row1 * a_scale;
		result.row2 = a_matrix.row2 * a_scale;
		return result;
	}

	Mat3 Transposed(in Mat3 a_matrix)
	{
		Mat3 result;
		result.row0 = float3(a_matrix.row0.x, a_matrix.row1.x, a_matrix.row2.x);
		result.row1 = float3(a_matrix.row0.y, a_matrix.row1.y, a_matrix.row2.y);
		result.row2 = float3(a_matrix.row0.z, a_matrix.row1.z, a_matrix.row2.z);
		return result;
	}

	Mat3 ScaledRows(in Mat3 a_matrix, in float3 a_scale)
	{
		Mat3 result;
		result.row0 = a_matrix.row0 * a_scale;
		result.row1 = a_matrix.row1 * a_scale;
		result.row2 = a_matrix.row2 * a_scale;
		return result;
	}

	float Cofactor(in Mat3 a_matrix, in int a_row1, in int a_column1, in int a_row2, in int a_column2)
	{
		return Component(GetRow(a_matrix, a_row1), a_column1) * Component(GetRow(a_matrix, a_row2), a_column2) -
		       Component(GetRow(a_matrix, a_row1), a_column2) * Component(GetRow(a_matrix, a_row2), a_column1);
	}

	Mat3 Inverted(in Mat3 a_matrix)
	{
		const float3 cofactor = float3(Cofactor(a_matrix, 1, 1, 2, 2), Cofactor(a_matrix, 1, 2, 2, 0), Cofactor(a_matrix, 1, 0, 2, 1));
		const float determinant = Dot(a_matrix.row0, cofactor);
		const float scale = 1.0f / determinant;
		Mat3 result;
		result.row0 = float3(cofactor.x * scale, Cofactor(a_matrix, 0, 2, 2, 1) * scale, Cofactor(a_matrix, 0, 1, 1, 2) * scale);
		result.row1 = float3(cofactor.y * scale, Cofactor(a_matrix, 0, 0, 2, 2) * scale, Cofactor(a_matrix, 0, 2, 1, 0) * scale);
		result.row2 = float3(cofactor.z * scale, Cofactor(a_matrix, 0, 1, 2, 0) * scale, Cofactor(a_matrix, 0, 0, 1, 1) * scale);
		return result;
	}

	Transform Compose(in Transform a_left, in Transform a_right)
	{
		Transform result;
		result.basis = MatMul(a_left.basis, a_right.basis);
		result.origin = a_left.origin + MatVec(a_left.basis, a_right.origin);
		return result;
	}

	Mat3 SkewSymmetric(in float3 a_value)
	{
		Mat3 result;
		result.row0 = float3(0.0f, -a_value.z, a_value.y);
		result.row1 = float3(a_value.z, 0.0f, -a_value.x);
		result.row2 = float3(-a_value.y, a_value.x, 0.0f);
		return result;
	}

	/** @brief btMatrix3x3::solve33, a Cramer solve with the determinant kept when it is not invertible. */
	float3 Solve33(in Mat3 a_matrix, in float3 a_value)
	{
		const float3 column1 = Column(a_matrix, 0);
		const float3 column2 = Column(a_matrix, 1);
		const float3 column3 = Column(a_matrix, 2);
		float determinant = Dot(column1, Cross(column2, column3));
		if (abs(determinant) > kEpsilon)
			determinant = 1.0f / determinant;
		return float3(determinant * Dot(a_value, Cross(column2, column3)),
			determinant * Dot(column1, Cross(a_value, column3)),
			determinant * Dot(column1, Cross(column2, a_value)));
	}

	Quat QuatMul(in Quat a_left, in Quat a_right)
	{
		Quat result;
		result.x = a_left.w * a_right.x + a_left.x * a_right.w + a_left.y * a_right.z - a_left.z * a_right.y;
		result.y = a_left.w * a_right.y + a_left.y * a_right.w + a_left.z * a_right.x - a_left.x * a_right.z;
		result.z = a_left.w * a_right.z + a_left.z * a_right.w + a_left.x * a_right.y - a_left.y * a_right.x;
		result.w = a_left.w * a_right.w - a_left.x * a_right.x - a_left.y * a_right.y - a_left.z * a_right.z;
		return result;
	}

	Quat Conjugate(in Quat a_rotation)
	{
		Quat result;
		result.x = -a_rotation.x;
		result.y = -a_rotation.y;
		result.z = -a_rotation.z;
		result.w = a_rotation.w;
		return result;
	}

	float QuatLength2(in Quat a_rotation)
	{
		return a_rotation.x * a_rotation.x + a_rotation.y * a_rotation.y + a_rotation.z * a_rotation.z + a_rotation.w * a_rotation.w;
	}

	Quat QuatSafeNormalize(in Quat a_rotation)
	{
		const float length2 = QuatLength2(a_rotation);
		if (length2 <= kEpsilon)
			return a_rotation;
		const float inverseLength = 1.0f / sqrt(length2);
		Quat result;
		result.x = a_rotation.x * inverseLength;
		result.y = a_rotation.y * inverseLength;
		result.z = a_rotation.z * inverseLength;
		result.w = a_rotation.w * inverseLength;
		return result;
	}

	float3 QuatRotate(in Quat a_rotation, in float3 a_value)
	{
		Quat value;
		value.x = a_value.x;
		value.y = a_value.y;
		value.z = a_value.z;
		value.w = 0.0f;
		const Quat rotated = QuatMul(QuatMul(a_rotation, value), Conjugate(a_rotation));
		return float3(rotated.x, rotated.y, rotated.z);
	}

	/** @brief The basis a quaternion rotates with, as btMatrix3x3(const btQuaternion&) builds it. */
	Mat3 BasisFromRotation(in Quat a_rotation)
	{
		const float d = a_rotation.x * a_rotation.x + a_rotation.y * a_rotation.y + a_rotation.z * a_rotation.z + a_rotation.w * a_rotation.w;
		const float s = 2.0f / d;
		const float xs = a_rotation.x * s;
		const float ys = a_rotation.y * s;
		const float zs = a_rotation.z * s;
		const float wx = a_rotation.w * xs;
		const float wy = a_rotation.w * ys;
		const float wz = a_rotation.w * zs;
		const float xx = a_rotation.x * xs;
		const float xy = a_rotation.x * ys;
		const float xz = a_rotation.x * zs;
		const float yy = a_rotation.y * ys;
		const float yz = a_rotation.y * zs;
		const float zz = a_rotation.z * zs;
		Mat3 basis;
		basis.row0 = float3(1.0f - (yy + zz), xy - wz, xz + wy);
		basis.row1 = float3(xy + wz, 1.0f - (xx + zz), yz - wx);
		basis.row2 = float3(xz - wy, yz + wx, 1.0f - (xx + yy));
		return basis;
	}

	/** @brief The quaternion of a basis, as btMatrix3x3::getRotation computes it. */
	Quat RotationFromBasis(in Mat3 a_basis)
	{
		const float trace = a_basis.row0.x + a_basis.row1.y + a_basis.row2.z;
		float x = 0.0f;
		float y = 0.0f;
		float z = 0.0f;
		float w = 0.0f;
		if (trace > 0.0f) {
			float s = sqrt(trace + 1.0f);
			w = s * 0.5f;
			s = 0.5f / s;
			x = (a_basis.row2.y - a_basis.row1.z) * s;
			y = (a_basis.row0.z - a_basis.row2.x) * s;
			z = (a_basis.row1.x - a_basis.row0.y) * s;
		} else {
			const float diagonal0 = a_basis.row0.x;
			const float diagonal1 = a_basis.row1.y;
			const float diagonal2 = a_basis.row2.z;
			const int i = diagonal0 < diagonal1 ? (diagonal1 < diagonal2 ? 2 : 1) : (diagonal0 < diagonal2 ? 2 : 0);
			float s;
			if (i == 0) {
				s = sqrt(diagonal0 - diagonal1 - diagonal2 + 1.0f);
				x = s * 0.5f;
				s = 0.5f / s;
				w = (a_basis.row2.y - a_basis.row1.z) * s;
				y = (a_basis.row1.x + a_basis.row0.y) * s;
				z = (a_basis.row2.x + a_basis.row0.z) * s;
			} else if (i == 1) {
				s = sqrt(diagonal1 - diagonal2 - diagonal0 + 1.0f);
				y = s * 0.5f;
				s = 0.5f / s;
				w = (a_basis.row0.z - a_basis.row2.x) * s;
				z = (a_basis.row2.y + a_basis.row1.z) * s;
				x = (a_basis.row0.y + a_basis.row1.x) * s;
			} else {
				s = sqrt(diagonal2 - diagonal0 - diagonal1 + 1.0f);
				z = s * 0.5f;
				s = 0.5f / s;
				w = (a_basis.row1.x - a_basis.row0.y) * s;
				x = (a_basis.row0.z + a_basis.row2.x) * s;
				y = (a_basis.row1.z + a_basis.row2.y) * s;
			}
		}
		Quat result;
		result.x = x;
		result.y = y;
		result.z = z;
		result.w = w;
		return result;
	}

	/** @brief Cephes atanf; the HLSL intrinsic is too coarse for the joint limit angles Bullet extracts. */
	float PreciseAtan(float a_x)
	{
		const float ax = abs(a_x);
		float y = 0.0f;
		float x = ax;
		if (ax > 2.414213562373095f) {
			y = kHalfPi;
			x = -1.0f / ax;
		} else if (ax > 0.4142135623730951f) {
			y = 0.7853981633974483f;
			x = (ax - 1.0f) / (ax + 1.0f);
		}
		const float z = x * x;
		y += (((8.05374449538e-2f * z - 1.38776856032e-1f) * z + 1.99777106478e-1f) * z - 3.33329491539e-1f) * z * x + x;
		return a_x < 0.0f ? -y : y;
	}

	/** @brief std::atan2 for the joint Euler extraction. */
	float PreciseAtan2(float a_y, float a_x)
	{
		if (a_x == 0.0f)
			return a_y > 0.0f ? kHalfPi : (a_y < 0.0f ? -kHalfPi : 0.0f);
		const float angle = PreciseAtan(a_y / a_x);
		if (a_x > 0.0f)
			return angle;
		return a_y >= 0.0f ? angle + kPi : angle - kPi;
	}

	/** @brief std::asin for the joint Euler extraction. */
	float PreciseAsin(float a_x)
	{
		return PreciseAtan2(a_x, sqrt((1.0f - a_x) * (1.0f + a_x)));
	}

	float3 MatrixToEulerXYZ(in Mat3 a_matrix)
	{
		float3 xyz = 0.0f;
		const float fi = GetMatrixElem(a_matrix, 2);
		if (fi < 1.0f) {
			if (fi > -1.0f) {
				xyz.x = PreciseAtan2(-GetMatrixElem(a_matrix, 5), GetMatrixElem(a_matrix, 8));
				xyz.y = PreciseAsin(GetMatrixElem(a_matrix, 2));
				xyz.z = PreciseAtan2(-GetMatrixElem(a_matrix, 1), GetMatrixElem(a_matrix, 0));
				return xyz;
			}
			xyz.x = -PreciseAtan2(GetMatrixElem(a_matrix, 3), GetMatrixElem(a_matrix, 4));
			xyz.y = -kHalfPi;
			xyz.z = 0.0f;
			return xyz;
		}
		xyz.x = PreciseAtan2(GetMatrixElem(a_matrix, 3), GetMatrixElem(a_matrix, 4));
		xyz.y = kHalfPi;
		xyz.z = 0.0f;
		return xyz;
	}

	float3 MatrixToEulerXZY(in Mat3 a_matrix)
	{
		float3 xyz = 0.0f;
		const float fi = GetMatrixElem(a_matrix, 1);
		if (fi < 1.0f) {
			if (fi > -1.0f) {
				xyz.x = PreciseAtan2(GetMatrixElem(a_matrix, 7), GetMatrixElem(a_matrix, 4));
				xyz.y = PreciseAtan2(GetMatrixElem(a_matrix, 2), GetMatrixElem(a_matrix, 0));
				xyz.z = PreciseAsin(-GetMatrixElem(a_matrix, 1));
				return xyz;
			}
			xyz.x = -PreciseAtan2(-GetMatrixElem(a_matrix, 6), GetMatrixElem(a_matrix, 8));
			xyz.y = 0.0f;
			xyz.z = kHalfPi;
			return xyz;
		}
		xyz.x = PreciseAtan2(-GetMatrixElem(a_matrix, 6), GetMatrixElem(a_matrix, 8));
		xyz.y = 0.0f;
		xyz.z = -kHalfPi;
		return xyz;
	}

	float3 MatrixToEulerYXZ(in Mat3 a_matrix)
	{
		float3 xyz = 0.0f;
		const float fi = GetMatrixElem(a_matrix, 5);
		if (fi < 1.0f) {
			if (fi > -1.0f) {
				xyz.x = PreciseAsin(-GetMatrixElem(a_matrix, 5));
				xyz.y = PreciseAtan2(GetMatrixElem(a_matrix, 2), GetMatrixElem(a_matrix, 8));
				xyz.z = PreciseAtan2(GetMatrixElem(a_matrix, 3), GetMatrixElem(a_matrix, 4));
				return xyz;
			}
			xyz.x = kHalfPi;
			xyz.y = -PreciseAtan2(-GetMatrixElem(a_matrix, 1), GetMatrixElem(a_matrix, 0));
			xyz.z = 0.0f;
			return xyz;
		}
		xyz.x = -kHalfPi;
		xyz.y = PreciseAtan2(-GetMatrixElem(a_matrix, 1), GetMatrixElem(a_matrix, 0));
		xyz.z = 0.0f;
		return xyz;
	}

	float3 MatrixToEulerYZX(in Mat3 a_matrix)
	{
		float3 xyz = 0.0f;
		const float fi = GetMatrixElem(a_matrix, 3);
		if (fi < 1.0f) {
			if (fi > -1.0f) {
				xyz.x = PreciseAtan2(-GetMatrixElem(a_matrix, 5), GetMatrixElem(a_matrix, 4));
				xyz.y = PreciseAtan2(-GetMatrixElem(a_matrix, 6), GetMatrixElem(a_matrix, 0));
				xyz.z = PreciseAsin(GetMatrixElem(a_matrix, 3));
				return xyz;
			}
			xyz.x = 0.0f;
			xyz.y = -PreciseAtan2(GetMatrixElem(a_matrix, 7), GetMatrixElem(a_matrix, 8));
			xyz.z = -kHalfPi;
			return xyz;
		}
		xyz.x = 0.0f;
		xyz.y = PreciseAtan2(GetMatrixElem(a_matrix, 7), GetMatrixElem(a_matrix, 8));
		xyz.z = kHalfPi;
		return xyz;
	}

	float3 MatrixToEulerZXY(in Mat3 a_matrix)
	{
		float3 xyz = 0.0f;
		const float fi = GetMatrixElem(a_matrix, 7);
		if (fi < 1.0f) {
			if (fi > -1.0f) {
				xyz.x = PreciseAsin(GetMatrixElem(a_matrix, 7));
				xyz.y = PreciseAtan2(-GetMatrixElem(a_matrix, 6), GetMatrixElem(a_matrix, 8));
				xyz.z = PreciseAtan2(-GetMatrixElem(a_matrix, 1), GetMatrixElem(a_matrix, 4));
				return xyz;
			}
			xyz.x = -kHalfPi;
			xyz.y = 0.0f;
			xyz.z = -PreciseAtan2(GetMatrixElem(a_matrix, 2), GetMatrixElem(a_matrix, 0));
			return xyz;
		}
		xyz.x = kHalfPi;
		xyz.y = 0.0f;
		xyz.z = PreciseAtan2(GetMatrixElem(a_matrix, 2), GetMatrixElem(a_matrix, 0));
		return xyz;
	}

	float3 MatrixToEulerZYX(in Mat3 a_matrix)
	{
		float3 xyz = 0.0f;
		const float fi = GetMatrixElem(a_matrix, 6);
		if (fi < 1.0f) {
			if (fi > -1.0f) {
				xyz.x = PreciseAtan2(GetMatrixElem(a_matrix, 7), GetMatrixElem(a_matrix, 8));
				xyz.y = PreciseAsin(-GetMatrixElem(a_matrix, 6));
				xyz.z = PreciseAtan2(GetMatrixElem(a_matrix, 3), GetMatrixElem(a_matrix, 0));
				return xyz;
			}
			xyz.x = 0.0f;
			xyz.y = kHalfPi;
			xyz.z = -PreciseAtan2(GetMatrixElem(a_matrix, 1), GetMatrixElem(a_matrix, 2));
			return xyz;
		}
		xyz.x = 0.0f;
		xyz.y = -kHalfPi;
		xyz.z = PreciseAtan2(-GetMatrixElem(a_matrix, 1), -GetMatrixElem(a_matrix, 2));
		return xyz;
	}

	float NormalizeAngle(in float a_angle)
	{
		float angle = fmod(a_angle, kTwoPi);
		if (angle < -kPi)
			return angle + kTwoPi;
		if (angle > kPi)
			return angle - kTwoPi;
		return angle;
	}

	/** @brief btAdjustAngleToLimits, which picks the 2-pi-shifted angle nearest the limits. */
	float AdjustAngleToLimits(in float a_angle, in float a_lower, in float a_upper)
	{
		if (a_lower >= a_upper)
			return a_angle;
		if (a_angle < a_lower) {
			const float diffLower = abs(NormalizeAngle(a_lower - a_angle));
			const float diffUpper = abs(NormalizeAngle(a_upper - a_angle));
			return diffLower < diffUpper ? a_angle : a_angle + kTwoPi;
		}
		if (a_angle > a_upper) {
			const float diffUpper = abs(NormalizeAngle(a_angle - a_upper));
			const float diffLower = abs(NormalizeAngle(a_angle - a_lower));
			return diffLower < diffUpper ? a_angle - kTwoPi : a_angle;
		}
		return a_angle;
	}

	/** @brief btTypedConstraint::getMotorFactor, the motor's approach ramp into a limit. */
	float GetMotorFactor(in float a_position, in float a_lower, in float a_upper, in float a_velocity, in float a_timeFactor)
	{
		if (a_lower > a_upper)
			return 1.0f;
		if (a_lower == a_upper)
			return 0.0f;
		const float deltaMax = a_velocity / a_timeFactor;
		if (deltaMax < 0.0f) {
			if (a_position >= a_lower && a_position < (a_lower - deltaMax))
				return (a_lower - a_position) / deltaMax;
			return a_position < a_lower ? 0.0f : 1.0f;
		}
		if (deltaMax > 0.0f) {
			if (a_position <= a_upper && a_position > (a_upper - deltaMax))
				return (a_upper - a_position) / deltaMax;
			return a_position > a_upper ? 0.0f : 1.0f;
		}
		return 0.0f;
	}

	/** @brief btTransformUtil::integrateTransform's exponential map, including its angular motion clamp. */
	Transform IntegrateTransform(in Transform a_current, in float3 a_linearVelocity, in float3 a_angularVelocity, in float a_timeStep)
	{
		Transform predicted;
		predicted.origin = a_current.origin + a_linearVelocity * a_timeStep;
		const float angle2 = Dot(a_angularVelocity, a_angularVelocity);
		float angle = 0.0f;
		if (angle2 > kEpsilon)
			angle = sqrt(angle2);
		if (angle * a_timeStep > kAngularMotionThreshold)
			angle = kAngularMotionThreshold / a_timeStep;
		float3 axis;
		if (angle < kAngularTaylorAngle)
			axis = a_angularVelocity * (0.5f * a_timeStep - (a_timeStep * a_timeStep * a_timeStep) * kAngularTaylorCoefficient * angle * angle);
		else
			axis = a_angularVelocity * (sin(0.5f * angle * a_timeStep) / angle);
		Quat delta;
		delta.x = axis.x;
		delta.y = axis.y;
		delta.z = axis.z;
		delta.w = cos(angle * a_timeStep * 0.5f);
		const Quat predictedRotation = QuatSafeNormalize(QuatMul(delta, RotationFromBasis(a_current.basis)));
		predicted.basis = a_current.basis;
		if (QuatLength2(predictedRotation) > kEpsilon)
			predicted.basis = BasisFromRotation(predictedRotation);
		return predicted;
	}

	/** @brief btRigidBody::updateInertiaTensor, basis * diag(inverse inertia) * basis transpose. */
	Mat3 InverseWorldInertia(in Body a_body)
	{
		return MatMul(ScaledRows(a_body.worldTransform.basis, a_body.inverseInertiaLocal), Transposed(a_body.worldTransform.basis));
	}

	/** @brief btRigidBody::computeGyroscopicImpulseImplicit_Body: one Newton step in body space, as an impulse. */
	float3 GyroscopicImpulse(in Body a_body, in float a_timeStep)
	{
		float3 localInertia = 0.0f;
		for (int axis = 0; axis < 3; ++axis) {
			const float inverse = Component(a_body.inverseInertiaLocal, axis);
			localInertia = SetComponent(localInertia, axis, inverse != 0.0f ? 1.0f / inverse : 0.0f);
		}
		Mat3 inertia;
		inertia.row0 = float3(localInertia.x, 0.0f, 0.0f);
		inertia.row1 = float3(0.0f, localInertia.y, 0.0f);
		inertia.row2 = float3(0.0f, 0.0f, localInertia.z);

		const Quat rotation = RotationFromBasis(a_body.worldTransform.basis);
		float3 bodyAngular = QuatRotate(Conjugate(rotation), a_body.angularVelocity);
		const float3 residual = Cross(bodyAngular, MatVec(inertia, bodyAngular)) * a_timeStep;
		const Mat3 jacobian = MatAdd(inertia,
			MatScale(MatSub(MatMul(SkewSymmetric(bodyAngular), inertia), SkewSymmetric(MatVec(inertia, bodyAngular))), a_timeStep));
		bodyAngular = bodyAngular - Solve33(jacobian, residual);
		return QuatRotate(rotation, bodyAngular) - a_body.angularVelocity;
	}

	/** @brief btGeneric6DofSpring2Constraint::calculateLinearInfo and testLimitValue. */
	Joint TestLinearLimitValue(in Joint a_joint, in ConstraintConfig a_config, in int a_axis, in float a_value)
	{
		Joint joint = a_joint;
		const float lower = Component(a_config.linearLimits.lower, a_axis);
		const float upper = Component(a_config.linearLimits.upper, a_axis);
		if (lower > upper) {
			joint.linearError = SetComponent(joint.linearError, a_axis, 0.0f);
			joint = SetLinearLimit(joint, a_axis, 0);
		} else if (lower == upper) {
			joint.linearError = SetComponent(joint.linearError, a_axis, a_value - lower);
			joint = SetLinearLimit(joint, a_axis, 3);
		} else {
			joint.linearError = SetComponent(joint.linearError, a_axis, a_value - lower);
			joint.linearErrorHi = SetComponent(joint.linearErrorHi, a_axis, a_value - upper);
			joint = SetLinearLimit(joint, a_axis, 4);
		}
		return joint;
	}

	Joint CalculateLinearInfo(in Joint a_joint, in ConstraintConfig a_config)
	{
		Joint joint = a_joint;
		const float3 difference = joint.calculatedB.origin - joint.calculatedA.origin;
		joint.linearDiff = MatVec(Inverted(joint.calculatedA.basis), difference);
		for (int axis = 0; axis < 3; ++axis)
			joint = TestLinearLimitValue(joint, a_config, axis, Component(joint.linearDiff, axis));
		return joint;
	}

	/** @brief btGeneric6DofSpring2Constraint::calculateAngleInfo. */
	Joint CalculateAngleInfo(in Joint a_joint, in ConstraintConfig a_config)
	{
		Joint joint = a_joint;
		const Mat3 relative = MatMul(Inverted(joint.calculatedA.basis), joint.calculatedB.basis);
		if (a_config.rotationOrder == kRotateOrderXYZ)
			joint.axisAngleDiff = MatrixToEulerXYZ(relative);
		else if (a_config.rotationOrder == kRotateOrderXZY)
			joint.axisAngleDiff = MatrixToEulerXZY(relative);
		else if (a_config.rotationOrder == kRotateOrderYXZ)
			joint.axisAngleDiff = MatrixToEulerYXZ(relative);
		else if (a_config.rotationOrder == kRotateOrderYZX)
			joint.axisAngleDiff = MatrixToEulerYZX(relative);
		else if (a_config.rotationOrder == kRotateOrderZXY)
			joint.axisAngleDiff = MatrixToEulerZXY(relative);
		else
			joint.axisAngleDiff = MatrixToEulerZYX(relative);

		if (a_config.rotationOrder == kRotateOrderXYZ || a_config.rotationOrder == kRotateOrderZYX) {
			const float3 axis0 = Column(joint.calculatedB.basis, 0);
			const float3 axis2 = Column(joint.calculatedA.basis, 2);
			joint.axis1 = Cross(axis2, axis0);
			joint.axis0 = Cross(joint.axis1, axis2);
			joint.axis2 = Cross(axis0, joint.axis1);
		} else if (a_config.rotationOrder == kRotateOrderXZY || a_config.rotationOrder == kRotateOrderYZX) {
			const float3 axis0 = Column(joint.calculatedB.basis, 0);
			const float3 axis1 = Column(joint.calculatedA.basis, 1);
			joint.axis2 = Cross(axis0, axis1);
			joint.axis0 = Cross(axis1, joint.axis2);
			joint.axis1 = Cross(joint.axis2, axis0);
		} else {
			const float3 axis1 = Column(joint.calculatedB.basis, 1);
			const float3 axis2 = Column(joint.calculatedA.basis, 2);
			joint.axis0 = Cross(axis1, axis2);
			joint.axis1 = Cross(axis2, joint.axis0);
			joint.axis2 = Cross(joint.axis0, axis1);
		}

		for (int index = 0; index < 3; ++index)
			joint = SetAxis(joint, index, Normalized(GetAxis(joint, index)));
		return joint;
	}

	/** @brief btGeneric6DofSpring2Constraint::testAngularLimitMotor. */
	Joint TestAngularLimitMotor(in Joint a_joint, in ConstraintConfig a_config, in int a_axis)
	{
		Joint joint = a_joint;
		const MotorSettings motor = GetMotorSettings(a_config, a_axis);
		const float angle = AdjustAngleToLimits(Component(joint.axisAngleDiff, a_axis), motor.lower, motor.upper);
		MotorState state = ZeroMotorState();
		state.position = angle;
		if (motor.lower > motor.upper) {
			state.limit = 0;
			state.error = 0.0f;
		} else if (motor.lower == motor.upper) {
			state.error = angle - motor.lower;
			state.limit = 3;
		} else {
			state.error = angle - motor.lower;
			state.errorHi = angle - motor.upper;
			state.limit = 4;
		}
		joint = SetAngularState(joint, a_axis, state);
		return joint;
	}

	/** @brief btGeneric6DofSpring2Constraint::calculateTransforms. */
	Joint CalculateTransforms(in Joint a_joint, in ConstraintConfig a_config, in BodyState a_bodyA, in BodyState a_bodyB)
	{
		Joint joint = a_joint;
		Transform frameA;
		frameA.basis = a_config.frameABasis;
		frameA.origin = a_config.frameAOrigin;
		Transform frameB;
		frameB.basis = a_config.frameBBasis;
		frameB.origin = a_config.frameBOrigin;
		joint.calculatedA = Compose(a_bodyA.transform, frameA);
		joint.calculatedB = Compose(a_bodyB.transform, frameB);
		joint = CalculateLinearInfo(joint, a_config);
		joint = CalculateAngleInfo(joint, a_config);
		joint.hasStaticBody = a_bodyA.inverseMass < kEpsilon || a_bodyB.inverseMass < kEpsilon;
		const float inverseMassSum = a_bodyA.inverseMass + a_bodyB.inverseMass;
		joint.factA = inverseMassSum > 0.0f ? a_bodyB.inverseMass / inverseMassSum : 0.5f;
		joint.factB = 1.0f - joint.factA;
		return joint;
	}

	/** @brief btGeneric6DofSpring2Constraint::getInfo1, row counting. */
	int CountRows(in Joint a_joint, in ConstraintConfig a_config)
	{
		int rows = 0;
		for (int axis = 0; axis < 3; ++axis) {
			if (GetLinearLimit(a_joint, axis) == 4)
				rows += 2;
			else if (GetLinearLimit(a_joint, axis) != 0)
				rows += 1;
			if (HasFlag(Component(a_config.linearLimits.flags, axis), (int)kAxisMotor))
				rows += 1;
			if (HasFlag(Component(a_config.linearLimits.flags, axis), (int)kAxisSpring))
				rows += 1;
		}
		for (int axis = 0; axis < 3; ++axis) {
			if (GetAngularState(a_joint, axis).limit == 4)
				rows += 2;
			else if (GetAngularState(a_joint, axis).limit != 0)
				rows += 1;
			if (HasFlag(GetMotorSettings(a_config, axis).flags, (int)kAxisMotor))
				rows += 1;
			if (HasFlag(GetMotorSettings(a_config, axis).flags, (int)kAxisSpring))
				rows += 1;
		}
		return rows;
	}

	/** @brief btGeneric6DofSpring2Constraint::calculateJacobi. */
	SolverRow CalculateJacobi(in Joint a_joint, in SolverRow a_row, in float3 a_axis, in bool a_rotational, in bool a_rotAllowed,
		in BodyState a_bodyA, in BodyState a_bodyB)
	{
		SolverRow row = a_row;
		if (a_rotational) {
			row.relpos1CrossNormal = a_axis;
			row.relpos2CrossNormal = a_axis * -1.0f;
			return row;
		}

		row.normal1 = a_axis;
		row.normal2 = a_axis * -1.0f;
		const float3 relativeB = a_joint.calculatedB.origin - a_bodyB.transform.origin;
		const float3 relativeA = a_joint.calculatedA.origin - a_bodyA.transform.origin;
		float3 crossA = Cross(relativeA, a_axis);
		float3 crossB = Cross(relativeB, a_axis);
		if (a_joint.hasStaticBody && !a_rotAllowed) {
			crossA = crossA * a_joint.factA;
			crossB = crossB * a_joint.factB;
		}
		row.relpos1CrossNormal = crossA;
		row.relpos2CrossNormal = crossB * -1.0f;
		return row;
	}

	/**
	 * @brief btGeneric6DofSpring2Constraint::get_limit_motor_info2: the stop, motor and spring rows of one axis.
	 * @details An earlier axis's spring row zeroes the constraint's row 0 cfm, which the later axes read back.
	 */
	AxisRows GetLimitMotorInfo2(in AxisRows a_rows, in Joint a_joint, in LimitMotor a_limot, in float3 a_axis,
		in bool a_rotational, in bool a_rotAllowed, in BodyState a_bodyA, in BodyState a_bodyB, in float a_framesPerSecond)
	{
		AxisRows rows = a_rows;
		const SolverRow empty = ZeroRow();
		SolverRow row;

		if (a_limot.currentLimit == 4) {
			const float velocity = a_rotational ?
			                           Dot(a_bodyA.angularVelocity, a_axis) - Dot(a_bodyB.angularVelocity, a_axis) :
			                           Dot(a_bodyA.linearVelocity, a_axis) - Dot(a_bodyB.linearVelocity, a_axis);

			row = CalculateJacobi(a_joint, empty, a_axis, a_rotational, a_rotAllowed, a_bodyA, a_bodyB);
			row.rhs = a_limot.stopErp * a_limot.currentLimitError * (a_rotational ? -1.0f : 1.0f) * a_framesPerSecond;
			if (a_rotational) {
				if (row.rhs - velocity * a_limot.stopErp > 0.0f) {
					const float bounceError = -a_limot.bounce * velocity;
					if (bounceError > row.rhs)
						row.rhs = bounceError;
				}
			} else {
				if (row.rhs - velocity * a_limot.stopErp < 0.0f) {
					const float bounceError = -a_limot.bounce * velocity;
					if (bounceError < row.rhs)
						row.rhs = bounceError;
				}
			}
			row.lowerLimit = a_rotational ? 0.0f : kNegInfinity;
			row.upperLimit = a_rotational ? kInfinity : 0.0f;
			row.cfm = a_limot.stopCfm;
			rows.limitLower = row;
			rows.present |= kRowLimitLower;

			row = CalculateJacobi(a_joint, empty, a_axis, a_rotational, a_rotAllowed, a_bodyA, a_bodyB);
			row.rhs = a_limot.stopErp * a_limot.currentLimitErrorHi * (a_rotational ? -1.0f : 1.0f) * a_framesPerSecond;
			if (a_rotational) {
				if (row.rhs - velocity * a_limot.stopErp < 0.0f) {
					const float bounceError = -a_limot.bounce * velocity;
					if (bounceError < row.rhs)
						row.rhs = bounceError;
				}
			} else {
				if (row.rhs - velocity * a_limot.stopErp > 0.0f) {
					const float bounceError = -a_limot.bounce * velocity;
					if (bounceError > row.rhs)
						row.rhs = bounceError;
				}
			}
			row.lowerLimit = a_rotational ? kNegInfinity : 0.0f;
			row.upperLimit = a_rotational ? 0.0f : kInfinity;
			row.cfm = a_limot.stopCfm;
			rows.limitUpper = row;
			rows.present |= kRowLimitUpper;
		} else if (a_limot.currentLimit == 3) {
			row = CalculateJacobi(a_joint, empty, a_axis, a_rotational, a_rotAllowed, a_bodyA, a_bodyB);
			row.rhs = a_limot.stopErp * a_limot.currentLimitError * (a_rotational ? -1.0f : 1.0f) * a_framesPerSecond;
			row.lowerLimit = kNegInfinity;
			row.upperLimit = kInfinity;
			row.cfm = a_limot.stopCfm;
			rows.limitLower = row;
			rows.present |= kRowLimitLower;
		}

		if (a_limot.enableMotor && !a_limot.servoMotor) {
			row = CalculateJacobi(a_joint, empty, a_axis, a_rotational, a_rotAllowed, a_bodyA, a_bodyB);
			const float targetVelocity = a_rotational ? a_limot.targetVelocity : -a_limot.targetVelocity;
			const float motorFactor = GetMotorFactor(a_limot.currentPosition, a_limot.loLimit, a_limot.hiLimit,
				targetVelocity, a_framesPerSecond * a_limot.motorErp);
			row.rhs = motorFactor * a_limot.targetVelocity;
			row.lowerLimit = -a_limot.maxMotorForce / a_framesPerSecond;
			row.upperLimit = a_limot.maxMotorForce / a_framesPerSecond;
			row.cfm = a_limot.motorCfm;
			rows.motor = row;
			rows.present |= kRowMotor;
		}

		if (a_limot.enableMotor && a_limot.servoMotor) {
			float error = a_limot.currentPosition - a_limot.servoTarget;
			float currentTarget = a_limot.servoTarget;
			if (a_rotational) {
				if (error > kPi) {
					error -= kTwoPi;
					currentTarget += kTwoPi;
				}
				if (error < -kPi) {
					error += kTwoPi;
					currentTarget -= kTwoPi;
				}
			}

			row = CalculateJacobi(a_joint, empty, a_axis, a_rotational, a_rotAllowed, a_bodyA, a_bodyB);
			const float targetVelocity = error < 0.0f ? -a_limot.targetVelocity : a_limot.targetVelocity;
			const float targetParameter = -targetVelocity;
			float motorFactor = 0.0f;
			if (error != 0.0f) {
				float lowLimit;
				float hiLimit;
				if (a_limot.loLimit > a_limot.hiLimit) {
					lowLimit = error > 0.0f ? currentTarget : kNegInfinity;
					hiLimit = error < 0.0f ? currentTarget : kInfinity;
				} else {
					lowLimit = error > 0.0f && currentTarget > a_limot.loLimit ? currentTarget : a_limot.loLimit;
					hiLimit = error < 0.0f && currentTarget < a_limot.hiLimit ? currentTarget : a_limot.hiLimit;
				}
				motorFactor = GetMotorFactor(a_limot.currentPosition, lowLimit, hiLimit, targetParameter, a_framesPerSecond * a_limot.motorErp);
			}
			row.rhs = motorFactor * targetVelocity * (a_rotational ? -1.0f : 1.0f);
			row.lowerLimit = -a_limot.maxMotorForce / a_framesPerSecond;
			row.upperLimit = a_limot.maxMotorForce / a_framesPerSecond;
			row.cfm = a_limot.motorCfm;
			rows.servo = row;
			rows.present |= kRowServo;
		}

		if (a_limot.enableSpring) {
			const float error = a_limot.currentPosition - a_limot.equilibriumPoint;
			row = CalculateJacobi(a_joint, empty, a_axis, a_rotational, a_rotAllowed, a_bodyA, a_bodyB);
			const float timeStep = 1.0f / a_framesPerSecond;
			float damping = a_limot.springDamping;
			float stiffness = a_limot.springStiffness;
			const float position = a_limot.currentPosition;
			const float equilibrium = a_limot.equilibriumPoint;
			float velocity;
			if (a_rotational) {
				velocity = Dot(a_bodyA.angularVelocity, a_axis) - Dot(a_bodyB.angularVelocity, a_axis);
			} else {
				const float3 tangentVelocityA = Cross(a_bodyA.angularVelocity, a_joint.calculatedA.origin - a_bodyA.transform.origin);
				const float3 tangentVelocityB = Cross(a_bodyB.angularVelocity, a_joint.calculatedB.origin - a_bodyB.transform.origin);
				velocity = Dot(a_bodyA.linearVelocity + tangentVelocityA, a_axis) -
				           Dot(a_bodyB.linearVelocity + tangentVelocityB, a_axis);
			}
			const float dampingFactor = a_limot.nonHookeanDamping;
			const float stiffnessFactor = a_limot.nonHookeanStiffness;
			if (abs(error) >= kFuzzyEpsilon && (abs(dampingFactor) >= kFuzzyEpsilon || abs(stiffnessFactor) >= kFuzzyEpsilon)) {
				const float range = position < equilibrium ? equilibrium - a_limot.loLimit : a_limot.hiLimit - equilibrium;
				const float fraction = abs(range) >= kFuzzyEpsilon ? abs(error) / range : 0.0f;
				const float curve = clamp(fraction, 0.0f, 1.0f);
				damping *= 1.0f - dampingFactor * curve;
				stiffness *= 1.0f - stiffnessFactor * curve;
			}
			float massA = 1.0f / a_bodyA.inverseMass;
			float massB = 1.0f / a_bodyB.inverseMass;
			if (a_rotational) {
				const float radiusA = Dot(a_joint.calculatedA.origin - a_bodyA.transform.origin,
					a_joint.calculatedA.origin - a_bodyA.transform.origin);
				const float radiusB = Dot(a_joint.calculatedB.origin - a_bodyB.transform.origin,
					a_joint.calculatedB.origin - a_bodyB.transform.origin);
				if (a_bodyA.inverseMass)
					massA = massA * radiusA + 1.0f / Length(MatVec(a_bodyA.inverseInertiaWorld, a_axis));
				if (a_bodyB.inverseMass)
					massB = massB * radiusB + 1.0f / Length(MatVec(a_bodyB.inverseInertiaWorld, a_axis));
			}
			float mass;
			if (a_bodyA.inverseMass == 0.0f)
				mass = massB;
			else if (a_bodyB.inverseMass == 0.0f)
				mass = massA;
			else
				mass = massA * massB / (massA + massB);
			const float angularFrequency = sqrt(stiffness / mass);

			if (a_limot.springStiffnessLimited && kSpringFrequencyFraction < angularFrequency * timeStep)
				stiffness = 1.0f / timeStep / timeStep / kSpringStiffnessDivisor * mass;
			if (a_limot.springDampingLimited && damping * timeStep > mass)
				damping = mass / timeStep;
			const float springForce = stiffness * error * timeStep;
			const float dampingForce = -damping * velocity * (a_rotational ? -1.0f : 1.0f) * timeStep;
			const float force = springForce + dampingForce;

			row.rhs = velocity + force / mass * (a_rotational ? -1.0f : 1.0f);
			const float minimumForce = force < dampingForce ? force : dampingForce;
			const float maximumForce = force < dampingForce ? dampingForce : force;
			if (!a_rotational) {
				row.lowerLimit = minimumForce > 0.0f ? 0.0f : minimumForce;
				row.upperLimit = maximumForce < 0.0f ? 0.0f : maximumForce;
			} else {
				row.lowerLimit = -maximumForce > 0.0f ? 0.0f : -maximumForce;
				row.upperLimit = -minimumForce < 0.0f ? 0.0f : -minimumForce;
			}
			row.cfm = 0.0f;
			rows.spring = row;
			rows.present |= kRowSpring;
		}

		return rows;
	}

	bool IsLimitViolated(in MotorState a_motor)
	{
		return a_motor.limit == 1 || a_motor.limit == 2 ||
		       (a_motor.limit == 3 && (a_motor.error < -kRotationViolationThreshold || a_motor.error > kRotationViolationThreshold)) ||
		       (a_motor.limit == 4 && (a_motor.error < -kRotationViolationThreshold || a_motor.errorHi > kRotationViolationThreshold));
	}

	/** @brief The cfm of the constraint's row 0 after a block of rows was written, or the running value when it wrote none. */
	float BlockRowCfm(in float a_rowCfm, in AxisRows a_block)
	{
		if ((a_block.present & kRowLimitLower) != 0u)
			return a_block.limitLower.cfm;
		if ((a_block.present & kRowLimitUpper) != 0u)
			return a_block.limitUpper.cfm;
		if ((a_block.present & kRowMotor) != 0u)
			return a_block.motor.cfm;
		if ((a_block.present & kRowServo) != 0u)
			return a_block.servo.cfm;
		if ((a_block.present & kRowSpring) != 0u)
			return a_block.spring.cfm;
		return a_rowCfm;
	}

	/**
	 * @brief One of getInfo2's six axes, in the order the solver walks them: the three angular axes, then the three linear ones.
	 * @details btGeneric6DofSpring2Constraint builds the angular rows and then the linear ones. The caller walks the slots in
	 *          one rolled loop, so the compiled shader holds a single copy of this builder, and writes each block out as it
	 *          returns: the only state that crosses slots is the row 0 cfm. An axis with no rows returns an empty block.
	 */
	AxisRows BuildAxisRows(in int a_slot, in Joint a_joint, in ConstraintConfig a_config,
		in BodyState a_bodyA, in BodyState a_bodyB, in SolverSettings a_solver, in float a_framesPerSecond, in float a_rowCfm)
	{
		int order[3] = { 0, 1, 2 };
		if (a_config.rotationOrder == kRotateOrderXZY) {
			order[0] = 0;
			order[1] = 2;
			order[2] = 1;
		} else if (a_config.rotationOrder == kRotateOrderYXZ) {
			order[0] = 1;
			order[1] = 0;
			order[2] = 2;
		} else if (a_config.rotationOrder == kRotateOrderYZX) {
			order[0] = 1;
			order[1] = 2;
			order[2] = 0;
		} else if (a_config.rotationOrder == kRotateOrderZXY) {
			order[0] = 2;
			order[1] = 0;
			order[2] = 1;
		} else if (a_config.rotationOrder == kRotateOrderZYX) {
			order[0] = 2;
			order[1] = 1;
			order[2] = 0;
		}

		const bool rotational = a_slot < 3;
		const int axis = rotational ? order[a_slot % 3] : (a_slot - 3);
		LimitMotor limot;
		bool neighboursViolated = false;
		if (rotational) {
			const MotorSettings motor = GetMotorSettings(a_config, axis);
			const MotorState state = GetAngularState(a_joint, axis);
			if (state.limit == 0 && !HasFlag(motor.flags, (int)kAxisMotor) && !HasFlag(motor.flags, (int)kAxisSpring))
				return ZeroAxisRows();

			limot.loLimit = motor.lower;
			limot.hiLimit = motor.upper;
			limot.bounce = motor.bounce;
			limot.stopErp = motor.stopErp;
			limot.stopCfm = motor.stopCfm;
			limot.motorErp = motor.motorErp;
			limot.motorCfm = motor.motorCfm;
			limot.enableMotor = HasFlag(motor.flags, (int)kAxisMotor);
			limot.targetVelocity = motor.targetVelocity;
			limot.maxMotorForce = motor.maxMotorForce;
			limot.servoMotor = HasFlag(motor.flags, (int)kAxisServo);
			limot.servoTarget = motor.servoTarget;
			limot.enableSpring = HasFlag(motor.flags, (int)kAxisSpring);
			limot.springStiffness = motor.springStiffness;
			limot.springStiffnessLimited = HasFlag(motor.flags, (int)kAxisSpringStiffnessLimited);
			limot.springDamping = motor.springDamping;
			limot.springDampingLimited = HasFlag(motor.flags, (int)kAxisSpringDampingLimited);
			limot.equilibriumPoint = motor.equilibrium;
			limot.nonHookeanDamping = motor.nonHookeanDamping;
			limot.nonHookeanStiffness = motor.nonHookeanStiffness;
			limot.currentPosition = state.position;
			limot.currentLimit = state.limit;
			limot.currentLimitError = state.error;
			limot.currentLimitErrorHi = state.errorHi;

			const int flags = a_config.flags >> ((axis + 3) * kFlagAxisShift);
			if (!(flags & kFlagCfmStop))
				limot.stopCfm = a_rowCfm;
			if (!(flags & kFlagErpStop))
				limot.stopErp = a_solver.erp;
			if (!(flags & kFlagCfmMotor))
				limot.motorCfm = a_rowCfm;
			if (!(flags & kFlagErpMotor))
				limot.motorErp = a_solver.erp;
		} else {
			const LinearLimits limits = a_config.linearLimits;
			if (GetLinearLimit(a_joint, axis) == 0 && !HasFlag(Component(limits.flags, axis), (int)kAxisMotor) &&
				!HasFlag(Component(limits.flags, axis), (int)kAxisSpring))
				return ZeroAxisRows();

			limot.bounce = Component(limits.bounce, axis);
			limot.currentLimit = GetLinearLimit(a_joint, axis);
			limot.currentPosition = Component(a_joint.linearDiff, axis);
			limot.currentLimitError = Component(a_joint.linearError, axis);
			limot.currentLimitErrorHi = Component(a_joint.linearErrorHi, axis);
			limot.enableMotor = HasFlag(Component(limits.flags, axis), (int)kAxisMotor);
			limot.servoMotor = HasFlag(Component(limits.flags, axis), (int)kAxisServo);
			limot.servoTarget = Component(limits.servoTarget, axis);
			limot.enableSpring = HasFlag(Component(limits.flags, axis), (int)kAxisSpring);
			limot.springStiffness = Component(limits.springStiffness, axis);
			limot.springStiffnessLimited = HasFlag(Component(limits.flags, axis), (int)kAxisSpringStiffnessLimited);
			limot.springDamping = Component(limits.springDamping, axis);
			limot.springDampingLimited = HasFlag(Component(limits.flags, axis), (int)kAxisSpringDampingLimited);
			limot.equilibriumPoint = Component(limits.equilibrium, axis);
			limot.nonHookeanDamping = Component(limits.nonHookeanDamping, axis);
			limot.nonHookeanStiffness = Component(limits.nonHookeanStiffness, axis);
			limot.hiLimit = Component(limits.upper, axis);
			limot.loLimit = Component(limits.lower, axis);
			limot.maxMotorForce = Component(limits.maxMotorForce, axis);
			limot.targetVelocity = Component(limits.targetVelocity, axis);

			const int flags = a_config.flags >> (axis * kFlagAxisShift);
			limot.stopCfm = (flags & kFlagCfmStop) ? Component(limits.stopCfm, axis) : a_rowCfm;
			limot.stopErp = (flags & kFlagErpStop) ? Component(limits.stopErp, axis) : a_solver.erp;
			limot.motorCfm = (flags & kFlagCfmMotor) ? Component(limits.motorCfm, axis) : a_rowCfm;
			limot.motorErp = (flags & kFlagErpMotor) ? Component(limits.motorErp, axis) : a_solver.erp;

			neighboursViolated = IsLimitViolated(GetAngularState(a_joint, (axis + 1) % 3)) &&
			                     IsLimitViolated(GetAngularState(a_joint, (axis + 2) % 3));
		}

		const float3 rowAxis = rotational ? GetAxis(a_joint, axis) : Column(a_joint.calculatedA.basis, axis);
		return GetLimitMotorInfo2(ZeroAxisRows(), a_joint, limot, rowAxis, rotational,
			!neighboursViolated, a_bodyA, a_bodyB, a_framesPerSecond);
	}

	/** @brief btSequentialImpulseConstraintSolver::convertJoint's per-row finalisation. */
	SolverRow FinalizeRow(in SolverRow a_row, in BodyState a_bodyA, in BodyState a_bodyB,
		in SolverBody a_solverBodyA, in SolverBody a_solverBodyB, in float a_damping)
	{
		SolverRow row = a_row;
		row.angularComponentA = MatVec(a_bodyA.inverseInertiaWorld, row.relpos1CrossNormal);
		row.angularComponentB = MatVec(a_bodyB.inverseInertiaWorld, row.relpos2CrossNormal);
		const float3 linearA = row.normal1 * a_bodyA.inverseMass;
		const float3 linearB = row.normal2 * a_bodyB.inverseMass;
		const float3 angularA = MatVec(a_bodyA.inverseInertiaWorld, row.relpos1CrossNormal);
		const float3 angularB = MatVec(a_bodyB.inverseInertiaWorld, row.relpos2CrossNormal);
		const float sum = Dot(linearA, row.normal1) + Dot(angularA, row.relpos1CrossNormal) +
		                  Dot(linearB, row.normal2) + Dot(angularB, row.relpos2CrossNormal);
		row.jacDiagABInv = abs(sum) > kEpsilon ? 1.0f / sum : 0.0f;

		const float velocityA = Dot(row.normal1, a_bodyA.linearVelocity + a_solverBodyA.externalForceImpulse) +
		                        Dot(row.relpos1CrossNormal, a_bodyA.angularVelocity + a_solverBodyA.externalTorqueImpulse);
		const float velocityB = Dot(row.normal2, a_bodyB.linearVelocity + a_solverBodyB.externalForceImpulse) +
		                        Dot(row.relpos2CrossNormal, a_bodyB.angularVelocity + a_solverBodyB.externalTorqueImpulse);
		const float velocityError = -(velocityA + velocityB) * a_damping;
		row.rhs = row.rhs * row.jacDiagABInv + velocityError * row.jacDiagABInv;
		row.appliedImpulse = 0.0f;
		return row;
	}

	/** @brief btSequentialImpulseConstraintSolver's scalar gResolveSingleConstraintRowGeneric. */
	float ResolveRow(inout SolverRow a_row, inout SolverBody a_bodyA, inout SolverBody a_bodyB)
	{
		SolverRow row = a_row;
		SolverBody bodyA = a_bodyA;
		SolverBody bodyB = a_bodyB;

		float deltaImpulse = row.rhs - row.appliedImpulse * row.cfm;
		const float deltaVelocityA = Dot(row.normal1, bodyA.deltaLinearVelocity) +
		                             Dot(row.relpos1CrossNormal, bodyA.deltaAngularVelocity);
		const float deltaVelocityB = Dot(row.normal2, bodyB.deltaLinearVelocity) +
		                             Dot(row.relpos2CrossNormal, bodyB.deltaAngularVelocity);
		deltaImpulse -= deltaVelocityA * row.jacDiagABInv;
		deltaImpulse -= deltaVelocityB * row.jacDiagABInv;

		const float sum = row.appliedImpulse + deltaImpulse;
		if (sum < row.lowerLimit) {
			deltaImpulse = row.lowerLimit - row.appliedImpulse;
			row.appliedImpulse = row.lowerLimit;
		} else if (sum > row.upperLimit) {
			deltaImpulse = row.upperLimit - row.appliedImpulse;
			row.appliedImpulse = row.upperLimit;
		} else {
			row.appliedImpulse = sum;
		}

		bodyA.deltaLinearVelocity = bodyA.deltaLinearVelocity + (row.normal1 * bodyA.inverseMassVector) * deltaImpulse;
		bodyA.deltaAngularVelocity = bodyA.deltaAngularVelocity + row.angularComponentA * deltaImpulse;
		bodyB.deltaLinearVelocity = bodyB.deltaLinearVelocity + (row.normal2 * bodyB.inverseMassVector) * deltaImpulse;
		bodyB.deltaAngularVelocity = bodyB.deltaAngularVelocity + row.angularComponentB * deltaImpulse;

		a_row = row;
		a_bodyA = bodyA;
		a_bodyB = bodyB;
		return deltaImpulse * (1.0f / row.jacDiagABInv);
	}

	float SetMax(in float a_value, in float a_limit)
	{
		return a_value < a_limit ? a_limit : a_value;
	}

	float SetMin(in float a_value, in float a_limit)
	{
		return a_value < a_limit ? a_value : a_limit;
	}

	float3 ClampVelocity(in float3 a_value)
	{
		return float3(SetMin(SetMax(a_value.x, -kVelocityLimit), kVelocityLimit),
			SetMin(SetMax(a_value.y, -kVelocityLimit), kVelocityLimit),
			SetMin(SetMax(a_value.z, -kVelocityLimit), kVelocityLimit));
	}
}

#endif
