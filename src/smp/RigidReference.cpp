#include "RigidReference.h"

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace SmpPhysics::RigidReference
{
	Mat3 BasisFromRotation(const Quat& a_rotation) noexcept
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
		basis.row[0] = { 1.0f - (yy + zz), xy - wz, xz + wy };
		basis.row[1] = { xy + wz, 1.0f - (xx + zz), yz - wx };
		basis.row[2] = { xz - wy, yz + wx, 1.0f - (xx + yy) };
		return basis;
	}

	Quat RotationFromBasis(const Mat3& a_basis) noexcept
	{
		const float trace = a_basis.row[0].x + a_basis.row[1].y + a_basis.row[2].z;
		float values[4] = {};
		if (trace > 0.0f) {
			float s = std::sqrt(trace + 1.0f);
			values[3] = s * 0.5f;
			s = 0.5f / s;
			values[0] = (a_basis.row[2].y - a_basis.row[1].z) * s;
			values[1] = (a_basis.row[0].z - a_basis.row[2].x) * s;
			values[2] = (a_basis.row[1].x - a_basis.row[0].y) * s;
		} else {
			const float diagonal[3] = { a_basis.row[0].x, a_basis.row[1].y, a_basis.row[2].z };
			const int i = diagonal[0] < diagonal[1] ? (diagonal[1] < diagonal[2] ? 2 : 1) : (diagonal[0] < diagonal[2] ? 2 : 0);
			const int j = (i + 1) % 3;
			const int k = (i + 2) % 3;
			const float element[3][3] = {
				{ a_basis.row[0].x, a_basis.row[0].y, a_basis.row[0].z },
				{ a_basis.row[1].x, a_basis.row[1].y, a_basis.row[1].z },
				{ a_basis.row[2].x, a_basis.row[2].y, a_basis.row[2].z }
			};
			float s = std::sqrt(diagonal[i] - diagonal[j] - diagonal[k] + 1.0f);
			values[i] = s * 0.5f;
			s = 0.5f / s;
			values[3] = (element[k][j] - element[j][k]) * s;
			values[j] = (element[j][i] + element[i][j]) * s;
			values[k] = (element[k][i] + element[i][k]) * s;
		}
		return { values[0], values[1], values[2], values[3] };
	}

	Transform MakeTransform(const Vec3& a_origin, const Quat& a_rotation) noexcept
	{
		Transform transform;
		transform.basis = BasisFromRotation(a_rotation);
		transform.origin = a_origin;
		return transform;
	}

	namespace
	{
		constexpr float kEpsilon{ 1.19209290e-7f };
		constexpr float kInfinity{ 3.40282347e+38f };

		bool IsFuzzyZero(float a_value) noexcept
		{
			return std::fabs(a_value) < kEpsilon;
		}
		constexpr float kPi{ 3.14159265358979323846f };
		constexpr float kTwoPi{ 2.0f * kPi };
		constexpr float kHalfPi{ kPi * 0.5f };
		constexpr float kAngularMotionThreshold{ 0.5f * kHalfPi };
		constexpr float kAngularTaylorAngle{ 0.001f };
		constexpr float kAngularTaylorCoefficient{ 0.020833333333f };
		constexpr float kVelocityLimit{ 1.0e9f };
		constexpr float kRotationViolationThreshold{ 1.0e-3f };
		constexpr float kSpringFrequencyFraction{ 0.25f };
		constexpr float kSpringStiffnessDivisor{ 16.0f };

		float Component(const Vec3& a_value, int a_axis) noexcept
		{
			return a_axis == 0 ? a_value.x : (a_axis == 1 ? a_value.y : a_value.z);
		}

		void SetComponent(Vec3& a_value, int a_axis, float a_component) noexcept
		{
			if (a_axis == 0)
				a_value.x = a_component;
			else if (a_axis == 1)
				a_value.y = a_component;
			else
				a_value.z = a_component;
		}

		Vec3 Add(const Vec3& a_left, const Vec3& a_right) noexcept
		{
			return { a_left.x + a_right.x, a_left.y + a_right.y, a_left.z + a_right.z };
		}

		Vec3 Sub(const Vec3& a_left, const Vec3& a_right) noexcept
		{
			return { a_left.x - a_right.x, a_left.y - a_right.y, a_left.z - a_right.z };
		}

		Vec3 Scale(const Vec3& a_value, float a_scale) noexcept
		{
			return { a_value.x * a_scale, a_value.y * a_scale, a_value.z * a_scale };
		}

		Vec3 Multiply(const Vec3& a_left, const Vec3& a_right) noexcept
		{
			return { a_left.x * a_right.x, a_left.y * a_right.y, a_left.z * a_right.z };
		}

		Vec3 Divide(const Vec3& a_value, float a_divisor) noexcept
		{
			return { a_value.x / a_divisor, a_value.y / a_divisor, a_value.z / a_divisor };
		}

		float Dot(const Vec3& a_left, const Vec3& a_right) noexcept
		{
			return a_left.x * a_right.x + a_left.y * a_right.y + a_left.z * a_right.z;
		}

		Vec3 Cross(const Vec3& a_left, const Vec3& a_right) noexcept
		{
			return { a_left.y * a_right.z - a_left.z * a_right.y,
				a_left.z * a_right.x - a_left.x * a_right.z,
				a_left.x * a_right.y - a_left.y * a_right.x };
		}

		float Length(const Vec3& a_value) noexcept
		{
			return std::sqrt(Dot(a_value, a_value));
		}

		Vec3 Normalized(const Vec3& a_value) noexcept
		{
			return Divide(a_value, Length(a_value));
		}

		Vec3 Column(const Mat3& a_matrix, int a_column) noexcept
		{
			return { Component(a_matrix.row[0], a_column), Component(a_matrix.row[1], a_column), Component(a_matrix.row[2], a_column) };
		}

		Vec3 MatVec(const Mat3& a_matrix, const Vec3& a_value) noexcept
		{
			return { Dot(a_matrix.row[0], a_value), Dot(a_matrix.row[1], a_value), Dot(a_matrix.row[2], a_value) };
		}

		Mat3 MatMul(const Mat3& a_left, const Mat3& a_right) noexcept
		{
			Mat3 result;
			for (int row = 0; row < 3; ++row) {
				for (int column = 0; column < 3; ++column)
					SetComponent(result.row[row], column, Dot(a_left.row[row], Column(a_right, column)));
			}
			return result;
		}

		Mat3 MatAdd(const Mat3& a_left, const Mat3& a_right) noexcept
		{
			Mat3 result;
			for (int row = 0; row < 3; ++row)
				result.row[row] = Add(a_left.row[row], a_right.row[row]);
			return result;
		}

		Mat3 MatSub(const Mat3& a_left, const Mat3& a_right) noexcept
		{
			Mat3 result;
			for (int row = 0; row < 3; ++row)
				result.row[row] = Sub(a_left.row[row], a_right.row[row]);
			return result;
		}

		Mat3 MatScale(const Mat3& a_matrix, float a_scale) noexcept
		{
			Mat3 result;
			for (int row = 0; row < 3; ++row)
				result.row[row] = Scale(a_matrix.row[row], a_scale);
			return result;
		}

		Mat3 Transposed(const Mat3& a_matrix) noexcept
		{
			Mat3 result;
			for (int row = 0; row < 3; ++row) {
				for (int column = 0; column < 3; ++column)
					SetComponent(result.row[row], column, Component(a_matrix.row[column], row));
			}
			return result;
		}

		Mat3 ScaledRows(const Mat3& a_matrix, const Vec3& a_scale) noexcept
		{
			Mat3 result;
			for (int row = 0; row < 3; ++row)
				result.row[row] = Multiply(a_matrix.row[row], a_scale);
			return result;
		}

		float Cofactor(const Mat3& a_matrix, int a_row1, int a_column1, int a_row2, int a_column2) noexcept
		{
			return Component(a_matrix.row[a_row1], a_column1) * Component(a_matrix.row[a_row2], a_column2) -
			       Component(a_matrix.row[a_row1], a_column2) * Component(a_matrix.row[a_row2], a_column1);
		}

		Mat3 Inverted(const Mat3& a_matrix) noexcept
		{
			const Vec3 cofactor{ Cofactor(a_matrix, 1, 1, 2, 2), Cofactor(a_matrix, 1, 2, 2, 0), Cofactor(a_matrix, 1, 0, 2, 1) };
			const float determinant = Dot(a_matrix.row[0], cofactor);
			const float scale = 1.0f / determinant;
			Mat3 result;
			result.row[0] = { cofactor.x * scale, Cofactor(a_matrix, 0, 2, 2, 1) * scale, Cofactor(a_matrix, 0, 1, 1, 2) * scale };
			result.row[1] = { cofactor.y * scale, Cofactor(a_matrix, 0, 0, 2, 2) * scale, Cofactor(a_matrix, 0, 2, 1, 0) * scale };
			result.row[2] = { cofactor.z * scale, Cofactor(a_matrix, 0, 1, 2, 0) * scale, Cofactor(a_matrix, 0, 0, 1, 1) * scale };
			return result;
		}

		Transform Compose(const Transform& a_left, const Transform& a_right) noexcept
		{
			Transform result;
			result.basis = MatMul(a_left.basis, a_right.basis);
			result.origin = Add(a_left.origin, MatVec(a_left.basis, a_right.origin));
			return result;
		}

		Mat3 SkewSymmetric(const Vec3& a_value) noexcept
		{
			Mat3 result;
			result.row[0] = { 0.0f, -a_value.z, a_value.y };
			result.row[1] = { a_value.z, 0.0f, -a_value.x };
			result.row[2] = { -a_value.y, a_value.x, 0.0f };
			return result;
		}

		/** @brief btMatrix3x3::solve33, a Cramer solve with the determinant kept when it is not invertible. */
		Vec3 Solve33(const Mat3& a_matrix, const Vec3& a_value) noexcept
		{
			const Vec3 column1 = Column(a_matrix, 0);
			const Vec3 column2 = Column(a_matrix, 1);
			const Vec3 column3 = Column(a_matrix, 2);
			float determinant = Dot(column1, Cross(column2, column3));
			if (std::fabs(determinant) > kEpsilon)
				determinant = 1.0f / determinant;
			return { determinant * Dot(a_value, Cross(column2, column3)),
				determinant * Dot(column1, Cross(a_value, column3)),
				determinant * Dot(column1, Cross(column2, a_value)) };
		}

		Quat QuatMul(const Quat& a_left, const Quat& a_right) noexcept
		{
			return { a_left.w * a_right.x + a_left.x * a_right.w + a_left.y * a_right.z - a_left.z * a_right.y,
				a_left.w * a_right.y + a_left.y * a_right.w + a_left.z * a_right.x - a_left.x * a_right.z,
				a_left.w * a_right.z + a_left.z * a_right.w + a_left.x * a_right.y - a_left.y * a_right.x,
				a_left.w * a_right.w - a_left.x * a_right.x - a_left.y * a_right.y - a_left.z * a_right.z };
		}

		Quat Conjugate(const Quat& a_rotation) noexcept
		{
			return { -a_rotation.x, -a_rotation.y, -a_rotation.z, a_rotation.w };
		}

		float QuatLength2(const Quat& a_rotation) noexcept
		{
			return a_rotation.x * a_rotation.x + a_rotation.y * a_rotation.y + a_rotation.z * a_rotation.z + a_rotation.w * a_rotation.w;
		}

		Quat QuatSafeNormalize(const Quat& a_rotation) noexcept
		{
			const float length2 = QuatLength2(a_rotation);
			if (length2 <= kEpsilon)
				return a_rotation;
			const float inverseLength = 1.0f / std::sqrt(length2);
			return { a_rotation.x * inverseLength, a_rotation.y * inverseLength, a_rotation.z * inverseLength, a_rotation.w * inverseLength };
		}

		Vec3 QuatRotate(const Quat& a_rotation, const Vec3& a_value) noexcept
		{
			const Quat rotated = QuatMul(QuatMul(a_rotation, Quat{ a_value.x, a_value.y, a_value.z, 0.0f }), Conjugate(a_rotation));
			return { rotated.x, rotated.y, rotated.z };
		}

		float GetMatrixElem(const Mat3& a_matrix, int a_index) noexcept
		{
			return Component(a_matrix.row[a_index % 3], a_index / 3);
		}

		void MatrixToEulerXYZ(const Mat3& a_matrix, Vec3& a_xyz) noexcept
		{
			const float fi = GetMatrixElem(a_matrix, 2);
			if (fi < 1.0f) {
				if (fi > -1.0f) {
					a_xyz.x = std::atan2(-GetMatrixElem(a_matrix, 5), GetMatrixElem(a_matrix, 8));
					a_xyz.y = std::asin(GetMatrixElem(a_matrix, 2));
					a_xyz.z = std::atan2(-GetMatrixElem(a_matrix, 1), GetMatrixElem(a_matrix, 0));
					return;
				}
				a_xyz.x = -std::atan2(GetMatrixElem(a_matrix, 3), GetMatrixElem(a_matrix, 4));
				a_xyz.y = -kHalfPi;
				a_xyz.z = 0.0f;
				return;
			}
			a_xyz.x = std::atan2(GetMatrixElem(a_matrix, 3), GetMatrixElem(a_matrix, 4));
			a_xyz.y = kHalfPi;
			a_xyz.z = 0.0f;
		}

		void MatrixToEulerXZY(const Mat3& a_matrix, Vec3& a_xyz) noexcept
		{
			const float fi = GetMatrixElem(a_matrix, 1);
			if (fi < 1.0f) {
				if (fi > -1.0f) {
					a_xyz.x = std::atan2(GetMatrixElem(a_matrix, 7), GetMatrixElem(a_matrix, 4));
					a_xyz.y = std::atan2(GetMatrixElem(a_matrix, 2), GetMatrixElem(a_matrix, 0));
					a_xyz.z = std::asin(-GetMatrixElem(a_matrix, 1));
					return;
				}
				a_xyz.x = -std::atan2(-GetMatrixElem(a_matrix, 6), GetMatrixElem(a_matrix, 8));
				a_xyz.y = 0.0f;
				a_xyz.z = kHalfPi;
				return;
			}
			a_xyz.x = std::atan2(-GetMatrixElem(a_matrix, 6), GetMatrixElem(a_matrix, 8));
			a_xyz.y = 0.0f;
			a_xyz.z = -kHalfPi;
		}

		void MatrixToEulerYXZ(const Mat3& a_matrix, Vec3& a_xyz) noexcept
		{
			const float fi = GetMatrixElem(a_matrix, 5);
			if (fi < 1.0f) {
				if (fi > -1.0f) {
					a_xyz.x = std::asin(-GetMatrixElem(a_matrix, 5));
					a_xyz.y = std::atan2(GetMatrixElem(a_matrix, 2), GetMatrixElem(a_matrix, 8));
					a_xyz.z = std::atan2(GetMatrixElem(a_matrix, 3), GetMatrixElem(a_matrix, 4));
					return;
				}
				a_xyz.x = kHalfPi;
				a_xyz.y = -std::atan2(-GetMatrixElem(a_matrix, 1), GetMatrixElem(a_matrix, 0));
				a_xyz.z = 0.0f;
				return;
			}
			a_xyz.x = -kHalfPi;
			a_xyz.y = std::atan2(-GetMatrixElem(a_matrix, 1), GetMatrixElem(a_matrix, 0));
			a_xyz.z = 0.0f;
		}

		void MatrixToEulerYZX(const Mat3& a_matrix, Vec3& a_xyz) noexcept
		{
			const float fi = GetMatrixElem(a_matrix, 3);
			if (fi < 1.0f) {
				if (fi > -1.0f) {
					a_xyz.x = std::atan2(-GetMatrixElem(a_matrix, 5), GetMatrixElem(a_matrix, 4));
					a_xyz.y = std::atan2(-GetMatrixElem(a_matrix, 6), GetMatrixElem(a_matrix, 0));
					a_xyz.z = std::asin(GetMatrixElem(a_matrix, 3));
					return;
				}
				a_xyz.x = 0.0f;
				a_xyz.y = -std::atan2(GetMatrixElem(a_matrix, 7), GetMatrixElem(a_matrix, 8));
				a_xyz.z = -kHalfPi;
				return;
			}
			a_xyz.x = 0.0f;
			a_xyz.y = std::atan2(GetMatrixElem(a_matrix, 7), GetMatrixElem(a_matrix, 8));
			a_xyz.z = kHalfPi;
		}

		void MatrixToEulerZXY(const Mat3& a_matrix, Vec3& a_xyz) noexcept
		{
			const float fi = GetMatrixElem(a_matrix, 7);
			if (fi < 1.0f) {
				if (fi > -1.0f) {
					a_xyz.x = std::asin(GetMatrixElem(a_matrix, 7));
					a_xyz.y = std::atan2(-GetMatrixElem(a_matrix, 6), GetMatrixElem(a_matrix, 8));
					a_xyz.z = std::atan2(-GetMatrixElem(a_matrix, 1), GetMatrixElem(a_matrix, 4));
					return;
				}
				a_xyz.x = -kHalfPi;
				a_xyz.y = 0.0f;
				a_xyz.z = -std::atan2(GetMatrixElem(a_matrix, 2), GetMatrixElem(a_matrix, 0));
				return;
			}
			a_xyz.x = kHalfPi;
			a_xyz.y = 0.0f;
			a_xyz.z = std::atan2(GetMatrixElem(a_matrix, 2), GetMatrixElem(a_matrix, 0));
		}

		void MatrixToEulerZYX(const Mat3& a_matrix, Vec3& a_xyz) noexcept
		{
			const float fi = GetMatrixElem(a_matrix, 6);
			if (fi < 1.0f) {
				if (fi > -1.0f) {
					a_xyz.x = std::atan2(GetMatrixElem(a_matrix, 7), GetMatrixElem(a_matrix, 8));
					a_xyz.y = std::asin(-GetMatrixElem(a_matrix, 6));
					a_xyz.z = std::atan2(GetMatrixElem(a_matrix, 3), GetMatrixElem(a_matrix, 0));
					return;
				}
				a_xyz.x = 0.0f;
				a_xyz.y = kHalfPi;
				a_xyz.z = -std::atan2(GetMatrixElem(a_matrix, 1), GetMatrixElem(a_matrix, 2));
				return;
			}
			a_xyz.x = 0.0f;
			a_xyz.y = -kHalfPi;
			a_xyz.z = std::atan2(-GetMatrixElem(a_matrix, 1), -GetMatrixElem(a_matrix, 2));
		}

		float NormalizeAngle(float a_angle) noexcept
		{
			a_angle = std::fmod(a_angle, kTwoPi);
			if (a_angle < -kPi)
				return a_angle + kTwoPi;
			if (a_angle > kPi)
				return a_angle - kTwoPi;
			return a_angle;
		}

		/** @brief btAdjustAngleToLimits, which picks the 2-pi-shifted angle nearest the limits. */
		float AdjustAngleToLimits(float a_angle, float a_lower, float a_upper) noexcept
		{
			if (a_lower >= a_upper)
				return a_angle;
			if (a_angle < a_lower) {
				const float diffLower = std::fabs(NormalizeAngle(a_lower - a_angle));
				const float diffUpper = std::fabs(NormalizeAngle(a_upper - a_angle));
				return diffLower < diffUpper ? a_angle : a_angle + kTwoPi;
			}
			if (a_angle > a_upper) {
				const float diffUpper = std::fabs(NormalizeAngle(a_angle - a_upper));
				const float diffLower = std::fabs(NormalizeAngle(a_angle - a_lower));
				return diffLower < diffUpper ? a_angle - kTwoPi : a_angle;
			}
			return a_angle;
		}

		/** @brief btTypedConstraint::getMotorFactor, the motor's approach ramp into a limit. */
		float GetMotorFactor(float a_position, float a_lower, float a_upper, float a_velocity, float a_timeFactor) noexcept
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
		void IntegrateTransform(const Transform& a_current, const Vec3& a_linearVelocity, const Vec3& a_angularVelocity,
			float a_timeStep, Transform& a_predicted) noexcept
		{
			a_predicted.origin = Add(a_current.origin, Scale(a_linearVelocity, a_timeStep));
			const float angle2 = Dot(a_angularVelocity, a_angularVelocity);
			float angle = 0.0f;
			if (angle2 > kEpsilon)
				angle = std::sqrt(angle2);
			if (angle * a_timeStep > kAngularMotionThreshold)
				angle = kAngularMotionThreshold / a_timeStep;
			Vec3 axis;
			if (angle < kAngularTaylorAngle)
				axis = Scale(a_angularVelocity, 0.5f * a_timeStep - (a_timeStep * a_timeStep * a_timeStep) * kAngularTaylorCoefficient * angle * angle);
			else
				axis = Scale(a_angularVelocity, std::sin(0.5f * angle * a_timeStep) / angle);
			const Quat delta{ axis.x, axis.y, axis.z, std::cos(angle * a_timeStep * 0.5f) };
			const Quat predicted = QuatSafeNormalize(QuatMul(delta, RotationFromBasis(a_current.basis)));
			a_predicted.basis = QuatLength2(predicted) > kEpsilon ? BasisFromRotation(predicted) : a_current.basis;
		}

		/** @brief btRigidBody::updateInertiaTensor, basis * diag(inverse inertia) * basis transpose. */
		Mat3 InverseWorldInertia(const Body& a_body) noexcept
		{
			return MatMul(ScaledRows(a_body.worldTransform.basis, a_body.inverseInertiaLocal), Transposed(a_body.worldTransform.basis));
		}

		/** @brief btRigidBody::computeGyroscopicImpulseImplicit_Body: one Newton step in body space, as an impulse. */
		Vec3 GyroscopicImpulse(const Body& a_body, float a_timeStep) noexcept
		{
			Vec3 localInertia;
			for (int axis = 0; axis < 3; ++axis) {
				const float inverse = Component(a_body.inverseInertiaLocal, axis);
				SetComponent(localInertia, axis, inverse != 0.0f ? 1.0f / inverse : 0.0f);
			}
			Mat3 inertia;
			inertia.row[0] = { localInertia.x, 0.0f, 0.0f };
			inertia.row[1] = { 0.0f, localInertia.y, 0.0f };
			inertia.row[2] = { 0.0f, 0.0f, localInertia.z };

			const Quat rotation = RotationFromBasis(a_body.worldTransform.basis);
			Vec3 bodyAngular = QuatRotate(Conjugate(rotation), a_body.angularVelocity);
			const Vec3 residual = Scale(Cross(bodyAngular, MatVec(inertia, bodyAngular)), a_timeStep);
			const Mat3 jacobian = MatAdd(inertia,
				MatScale(MatSub(MatMul(SkewSymmetric(bodyAngular), inertia), SkewSymmetric(MatVec(inertia, bodyAngular))), a_timeStep));
			bodyAngular = Sub(bodyAngular, Solve33(jacobian, residual));
			return Sub(QuatRotate(rotation, bodyAngular), a_body.angularVelocity);
		}

		struct SolverBody
		{
			Vec3 deltaLinearVelocity{};
			Vec3 deltaAngularVelocity{};
			Vec3 linearVelocity{};
			Vec3 angularVelocity{};
			Vec3 externalForceImpulse{};
			Vec3 externalTorqueImpulse{};
			Vec3 inverseMassVector{};
		};

		struct BodyState
		{
			Transform transform{};
			Vec3 linearVelocity{};
			Vec3 angularVelocity{};
			Mat3 inverseInertiaWorld{};
			float inverseMass{};
		};

		struct SolverRow
		{
			Vec3 normal1{};
			Vec3 relpos1CrossNormal{};
			Vec3 normal2{};
			Vec3 relpos2CrossNormal{};
			Vec3 angularComponentA{};
			Vec3 angularComponentB{};
			float rhs{};
			float cfm{};
			float jacDiagABInv{};
			float appliedImpulse{};
			float lowerLimit{ -kInfinity };
			float upperLimit{ kInfinity };
			int solverBodyA{};
			int solverBodyB{};
		};

		/** @brief The runtime fields of a btRotationalLimitMotor2, rebuilt for every step. */
		struct MotorState
		{
			float position{};
			float error{};
			float errorHi{};
			int limit{};
		};

		/** @brief The runtime fields of a btGeneric6DofSpring2Constraint for one step. */
		struct Joint
		{
			const Generic6DofSpring2* config{};
			Transform calculatedA{};
			Transform calculatedB{};
			Vec3 axisAngleDiff{};
			Vec3 axis[3]{};
			Vec3 linearDiff{};
			Vec3 linearError{};
			Vec3 linearErrorHi{};
			std::array<int, 3> linearLimit{};
			std::array<MotorState, 3> angular{};
			float factA{};
			float factB{};
			bool hasStaticBody{};
		};

		/** @brief A working copy of btRotationalLimitMotor2, the shape get_limit_motor_info2 consumes. */
		struct LimitMotor
		{
			float loLimit{ 1.0f };
			float hiLimit{ -1.0f };
			float bounce{};
			float stopErp{ 0.2f };
			float stopCfm{};
			float motorErp{ 0.9f };
			float motorCfm{};
			bool enableMotor{};
			float targetVelocity{};
			float maxMotorForce{};
			bool servoMotor{};
			float servoTarget{};
			bool enableSpring{};
			float springStiffness{};
			bool springStiffnessLimited{};
			float springDamping{};
			bool springDampingLimited{};
			float equilibriumPoint{};
			float nonHookeanDamping{};
			float nonHookeanStiffness{};
			float currentLimitError{};
			float currentLimitErrorHi{};
			float currentPosition{};
			int currentLimit{};
		};

		// btGeneric6DofSpring2Constraint::calculateLinearInfo and testLimitValue.
		void TestLinearLimitValue(Joint& a_joint, int a_axis, float a_value) noexcept
		{
			const float lower = Component(a_joint.config->linear.lower, a_axis);
			const float upper = Component(a_joint.config->linear.upper, a_axis);
			if (lower > upper) {
				SetComponent(a_joint.linearError, a_axis, 0.0f);
				a_joint.linearLimit[a_axis] = 0;
			} else if (lower == upper) {
				SetComponent(a_joint.linearError, a_axis, a_value - lower);
				a_joint.linearLimit[a_axis] = 3;
			} else {
				SetComponent(a_joint.linearError, a_axis, a_value - lower);
				SetComponent(a_joint.linearErrorHi, a_axis, a_value - upper);
				a_joint.linearLimit[a_axis] = 4;
			}
		}

		void CalculateLinearInfo(Joint& a_joint) noexcept
		{
			const Vec3 difference = Sub(a_joint.calculatedB.origin, a_joint.calculatedA.origin);
			a_joint.linearDiff = MatVec(Inverted(a_joint.calculatedA.basis), difference);
			for (int axis = 0; axis < 3; ++axis)
				TestLinearLimitValue(a_joint, axis, Component(a_joint.linearDiff, axis));
		}

		// btGeneric6DofSpring2Constraint::calculateAngleInfo.
		void CalculateAngleInfo(Joint& a_joint) noexcept
		{
			const Mat3 relative = MatMul(Inverted(a_joint.calculatedA.basis), a_joint.calculatedB.basis);
			switch (a_joint.config->rotationOrder) {
			case RotateOrder::XYZ:
				MatrixToEulerXYZ(relative, a_joint.axisAngleDiff);
				break;
			case RotateOrder::XZY:
				MatrixToEulerXZY(relative, a_joint.axisAngleDiff);
				break;
			case RotateOrder::YXZ:
				MatrixToEulerYXZ(relative, a_joint.axisAngleDiff);
				break;
			case RotateOrder::YZX:
				MatrixToEulerYZX(relative, a_joint.axisAngleDiff);
				break;
			case RotateOrder::ZXY:
				MatrixToEulerZXY(relative, a_joint.axisAngleDiff);
				break;
			case RotateOrder::ZYX:
				MatrixToEulerZYX(relative, a_joint.axisAngleDiff);
				break;
			}

			switch (a_joint.config->rotationOrder) {
			case RotateOrder::XYZ:
			case RotateOrder::ZYX:
				{
					const Vec3 axis0 = Column(a_joint.calculatedB.basis, 0);
					const Vec3 axis2 = Column(a_joint.calculatedA.basis, 2);
					a_joint.axis[1] = Cross(axis2, axis0);
					a_joint.axis[0] = Cross(a_joint.axis[1], axis2);
					a_joint.axis[2] = Cross(axis0, a_joint.axis[1]);
					break;
				}
			case RotateOrder::XZY:
			case RotateOrder::YZX:
				{
					const Vec3 axis0 = Column(a_joint.calculatedB.basis, 0);
					const Vec3 axis1 = Column(a_joint.calculatedA.basis, 1);
					a_joint.axis[2] = Cross(axis0, axis1);
					a_joint.axis[0] = Cross(axis1, a_joint.axis[2]);
					a_joint.axis[1] = Cross(a_joint.axis[2], axis0);
					break;
				}
			case RotateOrder::YXZ:
			case RotateOrder::ZXY:
				{
					const Vec3 axis1 = Column(a_joint.calculatedB.basis, 1);
					const Vec3 axis2 = Column(a_joint.calculatedA.basis, 2);
					a_joint.axis[0] = Cross(axis1, axis2);
					a_joint.axis[1] = Cross(axis2, a_joint.axis[0]);
					a_joint.axis[2] = Cross(a_joint.axis[0], axis1);
					break;
				}
			}

			for (Vec3& axis : a_joint.axis)
				axis = Normalized(axis);
		}

		// btGeneric6DofSpring2Constraint::testAngularLimitMotor.
		void TestAngularLimitMotor(Joint& a_joint, int a_axis) noexcept
		{
			const RotationalMotor& motor = a_joint.config->angular[a_axis];
			const float angle = AdjustAngleToLimits(Component(a_joint.axisAngleDiff, a_axis), motor.lower, motor.upper);
			MotorState& state = a_joint.angular[a_axis];
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
		}

		// btGeneric6DofSpring2Constraint::calculateTransforms.
		void CalculateTransforms(Joint& a_joint, const BodyState& a_bodyA, const BodyState& a_bodyB) noexcept
		{
			a_joint.calculatedA = Compose(a_bodyA.transform, a_joint.config->frameA);
			a_joint.calculatedB = Compose(a_bodyB.transform, a_joint.config->frameB);
			CalculateLinearInfo(a_joint);
			CalculateAngleInfo(a_joint);
			a_joint.hasStaticBody = a_bodyA.inverseMass < kEpsilon || a_bodyB.inverseMass < kEpsilon;
			const float inverseMassSum = a_bodyA.inverseMass + a_bodyB.inverseMass;
			a_joint.factA = inverseMassSum > 0.0f ? a_bodyB.inverseMass / inverseMassSum : 0.5f;
			a_joint.factB = 1.0f - a_joint.factA;
		}

		// btGeneric6DofSpring2Constraint::getInfo1, row counting.
		int CountRows(const Joint& a_joint) noexcept
		{
			int rows = 0;
			for (int axis = 0; axis < 3; ++axis) {
				if (a_joint.linearLimit[axis] == 4)
					rows += 2;
				else if (a_joint.linearLimit[axis] != 0)
					rows += 1;
				if (a_joint.config->linear.motorEnabled[axis])
					rows += 1;
				if (a_joint.config->linear.springEnabled[axis])
					rows += 1;
			}
			for (int axis = 0; axis < 3; ++axis) {
				if (a_joint.angular[axis].limit == 4)
					rows += 2;
				else if (a_joint.angular[axis].limit != 0)
					rows += 1;
				if (a_joint.config->angular[axis].motorEnabled)
					rows += 1;
				if (a_joint.config->angular[axis].springEnabled)
					rows += 1;
			}
			return rows;
		}

		// btGeneric6DofSpring2Constraint::calculateJacobi.
		void CalculateJacobi(const Joint& a_joint, SolverRow& a_row, const Vec3& a_axis, bool a_rotational, bool a_rotAllowed,
			const BodyState& a_bodyA, const BodyState& a_bodyB) noexcept
		{
			if (a_rotational) {
				a_row.relpos1CrossNormal = a_axis;
				a_row.relpos2CrossNormal = Scale(a_axis, -1.0f);
				return;
			}

			a_row.normal1 = a_axis;
			a_row.normal2 = Scale(a_axis, -1.0f);
			const Vec3 relativeB = Sub(a_joint.calculatedB.origin, a_bodyB.transform.origin);
			const Vec3 relativeA = Sub(a_joint.calculatedA.origin, a_bodyA.transform.origin);
			Vec3 crossA = Cross(relativeA, a_axis);
			Vec3 crossB = Cross(relativeB, a_axis);
			if (a_joint.hasStaticBody && !a_rotAllowed) {
				crossA = Scale(crossA, a_joint.factA);
				crossB = Scale(crossB, a_joint.factB);
			}
			a_row.relpos1CrossNormal = crossA;
			a_row.relpos2CrossNormal = Scale(crossB, -1.0f);
		}

		/**
		 * @brief btGeneric6DofSpring2Constraint::get_limit_motor_info2: the stop, motor and spring rows of one axis.
		 * @details a_row is the constraint's first solver row and a_localRow its next free index; an earlier axis's
		 *          spring row zeroes row 0's cfm, so the later axes read the cfm this function has already written.
		 */
		int GetLimitMotorInfo2(Joint& a_joint, const LimitMotor& a_limot, SolverRow* a_row, int a_localRow, const Vec3& a_axis,
			bool a_rotational, bool a_rotAllowed, const BodyState& a_bodyA, const BodyState& a_bodyB,
			float a_framesPerSecond) noexcept
		{
			int count = 0;

			if (a_limot.currentLimit == 4) {
				const float velocity = a_rotational ?
				                           Dot(a_bodyA.angularVelocity, a_axis) - Dot(a_bodyB.angularVelocity, a_axis) :
				                           Dot(a_bodyA.linearVelocity, a_axis) - Dot(a_bodyB.linearVelocity, a_axis);

				CalculateJacobi(a_joint, a_row[a_localRow], a_axis, a_rotational, a_rotAllowed, a_bodyA, a_bodyB);
				a_row[a_localRow].rhs = a_limot.stopErp * a_limot.currentLimitError * (a_rotational ? -1.0f : 1.0f) * a_framesPerSecond;
				if (a_rotational) {
					if (a_row[a_localRow].rhs - velocity * a_limot.stopErp > 0.0f) {
						const float bounceError = -a_limot.bounce * velocity;
						if (bounceError > a_row[a_localRow].rhs)
							a_row[a_localRow].rhs = bounceError;
					}
				} else {
					if (a_row[a_localRow].rhs - velocity * a_limot.stopErp < 0.0f) {
						const float bounceError = -a_limot.bounce * velocity;
						if (bounceError < a_row[a_localRow].rhs)
							a_row[a_localRow].rhs = bounceError;
					}
				}
				a_row[a_localRow].lowerLimit = a_rotational ? 0.0f : -kInfinity;
				a_row[a_localRow].upperLimit = a_rotational ? kInfinity : 0.0f;
				a_row[a_localRow].cfm = a_limot.stopCfm;
				++a_localRow;
				++count;

				CalculateJacobi(a_joint, a_row[a_localRow], a_axis, a_rotational, a_rotAllowed, a_bodyA, a_bodyB);
				a_row[a_localRow].rhs = a_limot.stopErp * a_limot.currentLimitErrorHi * (a_rotational ? -1.0f : 1.0f) * a_framesPerSecond;
				if (a_rotational) {
					if (a_row[a_localRow].rhs - velocity * a_limot.stopErp < 0.0f) {
						const float bounceError = -a_limot.bounce * velocity;
						if (bounceError < a_row[a_localRow].rhs)
							a_row[a_localRow].rhs = bounceError;
					}
				} else {
					if (a_row[a_localRow].rhs - velocity * a_limot.stopErp > 0.0f) {
						const float bounceError = -a_limot.bounce * velocity;
						if (bounceError > a_row[a_localRow].rhs)
							a_row[a_localRow].rhs = bounceError;
					}
				}
				a_row[a_localRow].lowerLimit = a_rotational ? -kInfinity : 0.0f;
				a_row[a_localRow].upperLimit = a_rotational ? 0.0f : kInfinity;
				a_row[a_localRow].cfm = a_limot.stopCfm;
				++a_localRow;
				++count;
			} else if (a_limot.currentLimit == 3) {
				CalculateJacobi(a_joint, a_row[a_localRow], a_axis, a_rotational, a_rotAllowed, a_bodyA, a_bodyB);
				a_row[a_localRow].rhs = a_limot.stopErp * a_limot.currentLimitError * (a_rotational ? -1.0f : 1.0f) * a_framesPerSecond;
				a_row[a_localRow].lowerLimit = -kInfinity;
				a_row[a_localRow].upperLimit = kInfinity;
				a_row[a_localRow].cfm = a_limot.stopCfm;
				++a_localRow;
				++count;
			}

			if (a_limot.enableMotor && !a_limot.servoMotor) {
				CalculateJacobi(a_joint, a_row[a_localRow], a_axis, a_rotational, a_rotAllowed, a_bodyA, a_bodyB);
				const float targetVelocity = a_rotational ? a_limot.targetVelocity : -a_limot.targetVelocity;
				const float motorFactor = GetMotorFactor(a_limot.currentPosition, a_limot.loLimit, a_limot.hiLimit,
					targetVelocity, a_framesPerSecond * a_limot.motorErp);
				a_row[a_localRow].rhs = motorFactor * a_limot.targetVelocity;
				a_row[a_localRow].lowerLimit = -a_limot.maxMotorForce / a_framesPerSecond;
				a_row[a_localRow].upperLimit = a_limot.maxMotorForce / a_framesPerSecond;
				a_row[a_localRow].cfm = a_limot.motorCfm;
				++a_localRow;
				++count;
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

				CalculateJacobi(a_joint, a_row[a_localRow], a_axis, a_rotational, a_rotAllowed, a_bodyA, a_bodyB);
				const float targetVelocity = error < 0.0f ? -a_limot.targetVelocity : a_limot.targetVelocity;
				const float targetParameter = -targetVelocity;
				float motorFactor = 0.0f;
				if (error != 0.0f) {
					float lowLimit;
					float hiLimit;
					if (a_limot.loLimit > a_limot.hiLimit) {
						lowLimit = error > 0.0f ? currentTarget : -kInfinity;
						hiLimit = error < 0.0f ? currentTarget : kInfinity;
					} else {
						lowLimit = error > 0.0f && currentTarget > a_limot.loLimit ? currentTarget : a_limot.loLimit;
						hiLimit = error < 0.0f && currentTarget < a_limot.hiLimit ? currentTarget : a_limot.hiLimit;
					}
					motorFactor = GetMotorFactor(a_limot.currentPosition, lowLimit, hiLimit, targetParameter, a_framesPerSecond * a_limot.motorErp);
				}
				a_row[a_localRow].rhs = motorFactor * targetVelocity * (a_rotational ? -1.0f : 1.0f);
				a_row[a_localRow].lowerLimit = -a_limot.maxMotorForce / a_framesPerSecond;
				a_row[a_localRow].upperLimit = a_limot.maxMotorForce / a_framesPerSecond;
				a_row[a_localRow].cfm = a_limot.motorCfm;
				++a_localRow;
				++count;
			}

			if (a_limot.enableSpring) {
				const float error = a_limot.currentPosition - a_limot.equilibriumPoint;
				CalculateJacobi(a_joint, a_row[a_localRow], a_axis, a_rotational, a_rotAllowed, a_bodyA, a_bodyB);
				const float timeStep = 1.0f / a_framesPerSecond;
				float damping = a_limot.springDamping;
				float stiffness = a_limot.springStiffness;
				const float position = a_limot.currentPosition;
				const float equilibrium = a_limot.equilibriumPoint;
				float velocity;
				if (a_rotational) {
					velocity = Dot(a_bodyA.angularVelocity, a_axis) - Dot(a_bodyB.angularVelocity, a_axis);
				} else {
					const Vec3 tangentVelocityA = Cross(a_bodyA.angularVelocity, Sub(a_joint.calculatedA.origin, a_bodyA.transform.origin));
					const Vec3 tangentVelocityB = Cross(a_bodyB.angularVelocity, Sub(a_joint.calculatedB.origin, a_bodyB.transform.origin));
					velocity = Dot(Add(a_bodyA.linearVelocity, tangentVelocityA), a_axis) -
					           Dot(Add(a_bodyB.linearVelocity, tangentVelocityB), a_axis);
				}
				const float dampingFactor = a_limot.nonHookeanDamping;
				const float stiffnessFactor = a_limot.nonHookeanStiffness;
				if (!IsFuzzyZero(error) && (!IsFuzzyZero(dampingFactor) || !IsFuzzyZero(stiffnessFactor))) {
					const float range = position < equilibrium ? equilibrium - a_limot.loLimit : a_limot.hiLimit - equilibrium;
					const float fraction = !IsFuzzyZero(range) ? std::fabs(error) / range : 0.0f;
					const float curve = std::clamp(fraction, 0.0f, 1.0f);
					damping *= 1.0f - dampingFactor * curve;
					stiffness *= 1.0f - stiffnessFactor * curve;
				}
				float massA = 1.0f / a_bodyA.inverseMass;
				float massB = 1.0f / a_bodyB.inverseMass;
				if (a_rotational) {
					const float radiusA = Dot(Sub(a_joint.calculatedA.origin, a_bodyA.transform.origin),
						Sub(a_joint.calculatedA.origin, a_bodyA.transform.origin));
					const float radiusB = Dot(Sub(a_joint.calculatedB.origin, a_bodyB.transform.origin),
						Sub(a_joint.calculatedB.origin, a_bodyB.transform.origin));
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
				const float angularFrequency = std::sqrt(stiffness / mass);

				if (a_limot.springStiffnessLimited && kSpringFrequencyFraction < angularFrequency * timeStep)
					stiffness = 1.0f / timeStep / timeStep / kSpringStiffnessDivisor * mass;
				if (a_limot.springDampingLimited && damping * timeStep > mass)
					damping = mass / timeStep;
				const float springForce = stiffness * error * timeStep;
				const float dampingForce = -damping * velocity * (a_rotational ? -1.0f : 1.0f) * timeStep;
				const float force = springForce + dampingForce;

				a_row[a_localRow].rhs = velocity + force / mass * (a_rotational ? -1.0f : 1.0f);
				const float minimumForce = force < dampingForce ? force : dampingForce;
				const float maximumForce = force < dampingForce ? dampingForce : force;
				if (!a_rotational) {
					a_row[a_localRow].lowerLimit = minimumForce > 0.0f ? 0.0f : minimumForce;
					a_row[a_localRow].upperLimit = maximumForce < 0.0f ? 0.0f : maximumForce;
				} else {
					a_row[a_localRow].lowerLimit = -maximumForce > 0.0f ? 0.0f : -maximumForce;
					a_row[a_localRow].upperLimit = -minimumForce < 0.0f ? 0.0f : -minimumForce;
				}
				a_row[a_localRow].cfm = 0.0f;
				++a_localRow;
				++count;
			}

			return count;
		}

		// btGeneric6DofSpring2Constraint's BT_6DOF_FLAGS_* per-axis bits.
		constexpr int kFlagCfmStop{ 1 };
		constexpr int kFlagErpStop{ 2 };
		constexpr int kFlagCfmMotor{ 4 };
		constexpr int kFlagErpMotor{ 8 };
		constexpr int kFlagAxisShift{ 4 };

		bool IsLimitViolated(const MotorState& a_motor) noexcept
		{
			return a_motor.limit == 1 || a_motor.limit == 2 ||
			       (a_motor.limit == 3 && (a_motor.error < -kRotationViolationThreshold || a_motor.error > kRotationViolationThreshold)) ||
			       (a_motor.limit == 4 && (a_motor.error < -kRotationViolationThreshold || a_motor.errorHi > kRotationViolationThreshold));
		}

		// btGeneric6DofSpring2Constraint::setAngularLimits.
		int SetAngularLimits(Joint& a_joint, SolverRow* a_row, int a_rowOffset, const BodyState& a_bodyA, const BodyState& a_bodyB,
			const SolverSettings& a_solver, float a_framesPerSecond) noexcept
		{
			int order[3] = { 0, 1, 2 };
			switch (a_joint.config->rotationOrder) {
			case RotateOrder::XYZ:
				order[0] = 0;
				order[1] = 1;
				order[2] = 2;
				break;
			case RotateOrder::XZY:
				order[0] = 0;
				order[1] = 2;
				order[2] = 1;
				break;
			case RotateOrder::YXZ:
				order[0] = 1;
				order[1] = 0;
				order[2] = 2;
				break;
			case RotateOrder::YZX:
				order[0] = 1;
				order[1] = 2;
				order[2] = 0;
				break;
			case RotateOrder::ZXY:
				order[0] = 2;
				order[1] = 0;
				order[2] = 1;
				break;
			case RotateOrder::ZYX:
				order[0] = 2;
				order[1] = 1;
				order[2] = 0;
				break;
			}

			int row = a_rowOffset;
			for (int index = 0; index < 3; ++index) {
				const int axis = order[index];
				const RotationalMotor& motor = a_joint.config->angular[axis];
				const MotorState& state = a_joint.angular[axis];
				if (state.limit == 0 && !motor.motorEnabled && !motor.springEnabled)
					continue;

				LimitMotor limot;
				limot.loLimit = motor.lower;
				limot.hiLimit = motor.upper;
				limot.bounce = motor.bounce;
				limot.stopErp = motor.stopErp;
				limot.stopCfm = motor.stopCfm;
				limot.motorErp = motor.motorErp;
				limot.motorCfm = motor.motorCfm;
				limot.enableMotor = motor.motorEnabled;
				limot.targetVelocity = motor.targetVelocity;
				limot.maxMotorForce = motor.maxMotorForce;
				limot.servoMotor = motor.servoEnabled;
				limot.servoTarget = motor.servoTarget;
				limot.enableSpring = motor.springEnabled;
				limot.springStiffness = motor.springStiffness;
				limot.springStiffnessLimited = motor.springStiffnessLimited;
				limot.springDamping = motor.springDamping;
				limot.springDampingLimited = motor.springDampingLimited;
				limot.equilibriumPoint = motor.equilibrium;
				limot.nonHookeanDamping = motor.nonHookeanDamping;
				limot.nonHookeanStiffness = motor.nonHookeanStiffness;
				limot.currentPosition = state.position;
				limot.currentLimit = state.limit;
				limot.currentLimitError = state.error;
				limot.currentLimitErrorHi = state.errorHi;

				const int flags = a_joint.config->flags >> ((axis + 3) * kFlagAxisShift);
				if (!(flags & kFlagCfmStop))
					limot.stopCfm = a_row[0].cfm;
				if (!(flags & kFlagErpStop))
					limot.stopErp = a_solver.erp;
				if (!(flags & kFlagCfmMotor))
					limot.motorCfm = a_row[0].cfm;
				if (!(flags & kFlagErpMotor))
					limot.motorErp = a_solver.erp;

				row += GetLimitMotorInfo2(a_joint, limot, a_row, row, a_joint.axis[axis], true, false,
					a_bodyA, a_bodyB, a_framesPerSecond);
			}
			return row;
		}

		// btGeneric6DofSpring2Constraint::setLinearLimits, which re-uses the rotational motor's rows.
		int SetLinearLimits(Joint& a_joint, SolverRow* a_row, int a_rowOffset, const BodyState& a_bodyA, const BodyState& a_bodyB,
			const SolverSettings& a_solver, float a_framesPerSecond) noexcept
		{
			const LinearAxisLimits& limits = a_joint.config->linear;
			int row = a_rowOffset;
			for (int axis = 0; axis < 3; ++axis) {
				if (a_joint.linearLimit[axis] == 0 && !limits.motorEnabled[axis] && !limits.springEnabled[axis])
					continue;

				LimitMotor limot;
				limot.bounce = Component(limits.bounce, axis);
				limot.currentLimit = a_joint.linearLimit[axis];
				limot.currentPosition = Component(a_joint.linearDiff, axis);
				limot.currentLimitError = Component(a_joint.linearError, axis);
				limot.currentLimitErrorHi = Component(a_joint.linearErrorHi, axis);
				limot.enableMotor = limits.motorEnabled[axis];
				limot.servoMotor = limits.servoEnabled[axis];
				limot.servoTarget = Component(limits.servoTarget, axis);
				limot.enableSpring = limits.springEnabled[axis];
				limot.springStiffness = Component(limits.springStiffness, axis);
				limot.springStiffnessLimited = limits.springStiffnessLimited[axis];
				limot.springDamping = Component(limits.springDamping, axis);
				limot.springDampingLimited = limits.springDampingLimited[axis];
				limot.equilibriumPoint = Component(limits.equilibrium, axis);
				limot.nonHookeanDamping = Component(limits.nonHookeanDamping, axis);
				limot.nonHookeanStiffness = Component(limits.nonHookeanStiffness, axis);
				limot.hiLimit = Component(limits.upper, axis);
				limot.loLimit = Component(limits.lower, axis);
				limot.maxMotorForce = Component(limits.maxMotorForce, axis);
				limot.targetVelocity = Component(limits.targetVelocity, axis);

				const int flags = a_joint.config->flags >> (axis * kFlagAxisShift);
				limot.stopCfm = (flags & kFlagCfmStop) ? Component(limits.stopCfm, axis) : a_row[0].cfm;
				limot.stopErp = (flags & kFlagErpStop) ? Component(limits.stopErp, axis) : a_solver.erp;
				limot.motorCfm = (flags & kFlagCfmMotor) ? Component(limits.motorCfm, axis) : a_row[0].cfm;
				limot.motorErp = (flags & kFlagErpMotor) ? Component(limits.motorErp, axis) : a_solver.erp;

				int rotAllowed = 1;
				if (IsLimitViolated(a_joint.angular[(axis + 1) % 3]) && IsLimitViolated(a_joint.angular[(axis + 2) % 3]))
					rotAllowed = 0;

				row += GetLimitMotorInfo2(a_joint, limot, a_row, row, Column(a_joint.calculatedA.basis, axis), false,
					rotAllowed != 0, a_bodyA, a_bodyB, a_framesPerSecond);
			}
			return row;
		}

		// btGeneric6DofSpring2Constraint::getInfo2: angular rows first, then the linear ones.
		void GetInfo2(Joint& a_joint, SolverRow* a_row, const BodyState& a_bodyA, const BodyState& a_bodyB,
			const SolverSettings& a_solver, float a_framesPerSecond) noexcept
		{
			const int row = SetAngularLimits(a_joint, a_row, 0, a_bodyA, a_bodyB, a_solver, a_framesPerSecond);
			SetLinearLimits(a_joint, a_row, row, a_bodyA, a_bodyB, a_solver, a_framesPerSecond);
		}

		// btSequentialImpulseConstraintSolver::convertJoint's per-row finalisation.
		void FinalizeRows(SolverRow* a_row, int a_count, const BodyState& a_bodyA, const BodyState& a_bodyB,
			const SolverBody& a_solverBodyA, const SolverBody& a_solverBodyB, float a_damping) noexcept
		{
			for (int index = 0; index < a_count; ++index) {
				SolverRow& row = a_row[index];
				row.angularComponentA = MatVec(a_bodyA.inverseInertiaWorld, row.relpos1CrossNormal);
				row.angularComponentB = MatVec(a_bodyB.inverseInertiaWorld, row.relpos2CrossNormal);
				const Vec3 linearA = Scale(row.normal1, a_bodyA.inverseMass);
				const Vec3 linearB = Scale(row.normal2, a_bodyB.inverseMass);
				const Vec3 angularA = MatVec(a_bodyA.inverseInertiaWorld, row.relpos1CrossNormal);
				const Vec3 angularB = MatVec(a_bodyB.inverseInertiaWorld, row.relpos2CrossNormal);
				const float sum = Dot(linearA, row.normal1) + Dot(angularA, row.relpos1CrossNormal) +
				                  Dot(linearB, row.normal2) + Dot(angularB, row.relpos2CrossNormal);
				row.jacDiagABInv = std::fabs(sum) > kEpsilon ? 1.0f / sum : 0.0f;

				const float velocityA = Dot(row.normal1, Add(a_bodyA.linearVelocity, a_solverBodyA.externalForceImpulse)) +
				                        Dot(row.relpos1CrossNormal, Add(a_bodyA.angularVelocity, a_solverBodyA.externalTorqueImpulse));
				const float velocityB = Dot(row.normal2, Add(a_bodyB.linearVelocity, a_solverBodyB.externalForceImpulse)) +
				                        Dot(row.relpos2CrossNormal, Add(a_bodyB.angularVelocity, a_solverBodyB.externalTorqueImpulse));
				const float velocityError = -(velocityA + velocityB) * a_damping;
				row.rhs = row.rhs * row.jacDiagABInv + velocityError * row.jacDiagABInv;
				row.appliedImpulse = 0.0f;
			}
		}

		// btSequentialImpulseConstraintSolver's scalar gResolveSingleConstraintRowGeneric.
		float ResolveRow(SolverRow& a_row, SolverBody& a_bodyA, SolverBody& a_bodyB) noexcept
		{
			float deltaImpulse = a_row.rhs - a_row.appliedImpulse * a_row.cfm;
			const float deltaVelocityA = Dot(a_row.normal1, a_bodyA.deltaLinearVelocity) +
			                             Dot(a_row.relpos1CrossNormal, a_bodyA.deltaAngularVelocity);
			const float deltaVelocityB = Dot(a_row.normal2, a_bodyB.deltaLinearVelocity) +
			                             Dot(a_row.relpos2CrossNormal, a_bodyB.deltaAngularVelocity);
			deltaImpulse -= deltaVelocityA * a_row.jacDiagABInv;
			deltaImpulse -= deltaVelocityB * a_row.jacDiagABInv;

			const float sum = a_row.appliedImpulse + deltaImpulse;
			if (sum < a_row.lowerLimit) {
				deltaImpulse = a_row.lowerLimit - a_row.appliedImpulse;
				a_row.appliedImpulse = a_row.lowerLimit;
			} else if (sum > a_row.upperLimit) {
				deltaImpulse = a_row.upperLimit - a_row.appliedImpulse;
				a_row.appliedImpulse = a_row.upperLimit;
			} else {
				a_row.appliedImpulse = sum;
			}

			a_bodyA.deltaLinearVelocity = Add(a_bodyA.deltaLinearVelocity,
				Scale(Multiply(a_row.normal1, a_bodyA.inverseMassVector), deltaImpulse));
			a_bodyA.deltaAngularVelocity = Add(a_bodyA.deltaAngularVelocity, Scale(a_row.angularComponentA, deltaImpulse));
			a_bodyB.deltaLinearVelocity = Add(a_bodyB.deltaLinearVelocity,
				Scale(Multiply(a_row.normal2, a_bodyB.inverseMassVector), deltaImpulse));
			a_bodyB.deltaAngularVelocity = Add(a_bodyB.deltaAngularVelocity, Scale(a_row.angularComponentB, deltaImpulse));
			return deltaImpulse * (1.0f / a_row.jacDiagABInv);
		}

		bool IsBodyIndex(int a_index, std::size_t a_count) noexcept
		{
			return a_index >= 0 && static_cast<std::size_t>(a_index) < a_count;
		}

		// hdtSkinnedMeshWorld::predictUnconstraintMotion, over btRigidBody::applyDamping.
		void PredictUnconstraintMotion(std::vector<Body>& a_bodies, float a_timeStep)
		{
			for (Body& body : a_bodies) {
				if (body.kinematic)
					continue;
				body.linearVelocity = Scale(body.linearVelocity, std::pow(1.0f - body.linearDamping, a_timeStep));
				body.angularVelocity = Scale(body.angularVelocity, std::pow(1.0f - body.angularDamping, a_timeStep));
				Transform predicted;
				IntegrateTransform(body.worldTransform, body.linearVelocity, body.angularVelocity, a_timeStep, predicted);
			}
		}

		constexpr float kSplitImpulsePenetrationThreshold{ -0.04f };

		/** @brief The velocity-and-position split of one contact row, which only the split-impulse pass reads. */
		struct ContactPush
		{
			float rhsPenetration{};
			float appliedPushImpulse{};
			bool hasFriction{};
			float friction{};
			SolverRow frictionRow{};
		};

		// btSequentialImpulseConstraintSolver::setupContactConstraint for a frictionless contact without restitution.
		SolverRow BuildContactRow(const World::Contact& a_contact, const std::vector<BodyState>& a_states,
			const std::vector<SolverBody>& a_solverBodies, const SolverSettings& a_solver, float a_timeStep, ContactPush& a_push)
		{
			const auto indexA = static_cast<std::size_t>(a_contact.bodyA);
			const auto indexB = static_cast<std::size_t>(a_contact.bodyB);
			const BodyState& stateA = a_states[indexA];
			const BodyState& stateB = a_states[indexB];
			const SolverBody& bodyA = a_solverBodies[indexA];
			const SolverBody& bodyB = a_solverBodies[indexB];
			const Vec3& normal = a_contact.normalOnB;
			const float invTimeStep = 1.0f / a_timeStep;
			const float cfm = a_solver.globalCfm * invTimeStep;

			const Vec3 pointOnA = Add(a_contact.pointOnB, Scale(normal, a_contact.distance));
			const Vec3 relPos1 = Sub(pointOnA, stateA.transform.origin);
			const Vec3 relPos2 = Sub(a_contact.pointOnB, stateB.transform.origin);
			const Vec3 torqueAxis0 = Cross(relPos1, normal);
			const Vec3 torqueAxis1 = Cross(relPos2, normal);

			SolverRow row;
			row.angularComponentA = MatVec(stateA.inverseInertiaWorld, torqueAxis0);
			row.angularComponentB = MatVec(stateB.inverseInertiaWorld, Scale(torqueAxis1, -1.0f));
			const float denom0 = stateA.inverseMass + Dot(normal, Cross(row.angularComponentA, relPos1));
			const float denom1 = stateB.inverseMass + Dot(normal, Cross(Scale(row.angularComponentB, -1.0f), relPos2));
			row.jacDiagABInv = a_solver.sor / (denom0 + denom1 + cfm);
			row.normal1 = normal;
			row.relpos1CrossNormal = torqueAxis0;
			row.normal2 = Scale(normal, -1.0f);
			row.relpos2CrossNormal = Scale(torqueAxis1, -1.0f);

			const float vel1Dotn = Dot(row.normal1, Add(bodyA.linearVelocity, bodyA.externalForceImpulse)) +
			                       Dot(row.relpos1CrossNormal, Add(bodyA.angularVelocity, bodyA.externalTorqueImpulse));
			const float vel2Dotn = Dot(row.normal2, Add(bodyB.linearVelocity, bodyB.externalForceImpulse)) +
			                       Dot(row.relpos2CrossNormal, Add(bodyB.angularVelocity, bodyB.externalTorqueImpulse));
			const float penetration = a_contact.distance + a_solver.linearSlop;
			float velocityError = -(vel1Dotn + vel2Dotn);
			float positionalError = 0.0f;
			if (penetration > 0.0f)
				velocityError -= penetration * invTimeStep;
			else
				positionalError = -penetration * a_solver.erp2 * invTimeStep;
			const float penetrationImpulse = positionalError * row.jacDiagABInv;
			const float velocityImpulse = velocityError * row.jacDiagABInv;
			if (!a_solver.splitImpulse || penetration > kSplitImpulsePenetrationThreshold) {
				row.rhs = penetrationImpulse + velocityImpulse;
			} else {
				row.rhs = velocityImpulse;
				a_push.rhsPenetration = penetrationImpulse;
			}
			row.cfm = cfm * row.jacDiagABInv;
			row.lowerLimit = 0.0f;
			row.upperLimit = 1.0e10f;
			row.solverBodyA = a_contact.bodyA;
			row.solverBodyB = a_contact.bodyB;
			return row;
		}

		// btPlaneSpace1: two unit vectors perpendicular to a_normal and to each other.
		Vec3 PlaneSpace(const Vec3& a_normal) noexcept
		{
			constexpr float kSqrt12 = 0.7071067811865475244008443621048490f;
			if (std::fabs(a_normal.z) > kSqrt12) {
				const float a = a_normal.y * a_normal.y + a_normal.z * a_normal.z;
				const float k = 1.0f / std::sqrt(a);
				return { 0.0f, -a_normal.z * k, a_normal.y * k };
			}
			const float a = a_normal.x * a_normal.x + a_normal.y * a_normal.y;
			const float k = 1.0f / std::sqrt(a);
			return { -a_normal.y * k, a_normal.x * k, 0.0f };
		}

		// setupFrictionConstraint with one friction direction taken from the contact's relative velocity.
		SolverRow BuildFrictionRow(const World::Contact& a_contact, const std::vector<BodyState>& a_states,
			const std::vector<SolverBody>& a_solverBodies, const SolverSettings& a_solver)
		{
			const auto indexA = static_cast<std::size_t>(a_contact.bodyA);
			const auto indexB = static_cast<std::size_t>(a_contact.bodyB);
			const BodyState& stateA = a_states[indexA];
			const BodyState& stateB = a_states[indexB];
			const SolverBody& bodyA = a_solverBodies[indexA];
			const SolverBody& bodyB = a_solverBodies[indexB];
			const Vec3& normal = a_contact.normalOnB;
			const Vec3 pointOnA = Add(a_contact.pointOnB, Scale(normal, a_contact.distance));
			const Vec3 relPos1 = Sub(pointOnA, stateA.transform.origin);
			const Vec3 relPos2 = Sub(a_contact.pointOnB, stateB.transform.origin);

			const Vec3 vel1 = Add(Add(bodyA.linearVelocity, bodyA.externalForceImpulse),
				Cross(Add(bodyA.angularVelocity, bodyA.externalTorqueImpulse), relPos1));
			const Vec3 vel2 = Add(Add(bodyB.linearVelocity, bodyB.externalForceImpulse),
				Cross(Add(bodyB.angularVelocity, bodyB.externalTorqueImpulse), relPos2));
			const Vec3 vel = Sub(vel1, vel2);
			const float relVel = Dot(normal, vel);
			Vec3 direction = Sub(vel, Scale(normal, relVel));
			const float lateral2 = Dot(direction, direction);
			constexpr float kSimdEpsilon = 1.192092896e-07f;
			direction = lateral2 > kSimdEpsilon ? Scale(direction, 1.0f / std::sqrt(lateral2)) : PlaneSpace(normal);

			SolverRow row;
			row.normal1 = direction;
			row.relpos1CrossNormal = Cross(relPos1, direction);
			row.angularComponentA = MatVec(stateA.inverseInertiaWorld, row.relpos1CrossNormal);
			row.normal2 = Scale(direction, -1.0f);
			row.relpos2CrossNormal = Cross(relPos2, row.normal2);
			row.angularComponentB = MatVec(stateB.inverseInertiaWorld, row.relpos2CrossNormal);
			const float denom0 = stateA.inverseMass + Dot(direction, Cross(row.angularComponentA, relPos1));
			const float denom1 = stateB.inverseMass + Dot(direction, Cross(Scale(row.angularComponentB, -1.0f), relPos2));
			row.jacDiagABInv = a_solver.sor / (denom0 + denom1);
			const float vel1Dotn = Dot(row.normal1, Add(bodyA.linearVelocity, bodyA.externalForceImpulse)) + Dot(row.relpos1CrossNormal, bodyA.angularVelocity);
			const float vel2Dotn = Dot(row.normal2, Add(bodyB.linearVelocity, bodyB.externalForceImpulse)) + Dot(row.relpos2CrossNormal, bodyB.angularVelocity);
			row.rhs = -(vel1Dotn + vel2Dotn) * row.jacDiagABInv;
			row.cfm = 0.0f;
			row.solverBodyA = a_contact.bodyA;
			row.solverBodyB = a_contact.bodyB;
			return row;
		}

		// btSequentialImpulseConstraintSolver::solveGroup: the joint rows, then the contact rows, each in stored order.
		void SolveConstraints(World& a_world, float a_timeStep)
		{
			const std::size_t bodyCount = a_world.bodies.size();
			if (bodyCount == 0)
				return;

			std::vector<BodyState> states(bodyCount);
			std::vector<SolverBody> solverBodies(bodyCount);
			for (std::size_t index = 0; index < bodyCount; ++index) {
				const Body& body = a_world.bodies[index];
				BodyState& state = states[index];
				state.transform = body.worldTransform;
				state.linearVelocity = body.linearVelocity;
				state.angularVelocity = body.angularVelocity;
				state.inverseInertiaWorld = InverseWorldInertia(body);
				state.inverseMass = body.inverseMass;

				SolverBody& solverBody = solverBodies[index];
				solverBody.inverseMassVector = { body.inverseMass, body.inverseMass, body.inverseMass };
				solverBody.linearVelocity = body.linearVelocity;
				solverBody.angularVelocity = body.angularVelocity;
				const Vec3 force = index < a_world.accumulatedForce.size() ? a_world.accumulatedForce[index] : Vec3{};
				solverBody.externalForceImpulse = Scale(force, body.inverseMass * a_timeStep);
			}

			for (std::size_t index = 0; index < bodyCount; ++index) {
				if (a_world.bodies[index].inverseMass == 0.0f)
					continue;
				solverBodies[index].externalTorqueImpulse = Add(solverBodies[index].externalTorqueImpulse,
					GyroscopicImpulse(a_world.bodies[index], a_timeStep));
			}

			std::vector<Joint> joints;
			std::vector<int> rowCounts;
			joints.reserve(a_world.constraints.size());
			rowCounts.reserve(a_world.constraints.size());
			int totalRows = 0;
			for (const Generic6DofSpring2& config : a_world.constraints) {
				Joint joint;
				joint.config = &config;
				int count = 0;
				if (config.enabled && IsBodyIndex(config.bodyA, bodyCount) && IsBodyIndex(config.bodyB, bodyCount)) {
					CalculateTransforms(joint, states[static_cast<std::size_t>(config.bodyA)],
						states[static_cast<std::size_t>(config.bodyB)]);
					for (int axis = 0; axis < 3; ++axis)
						TestAngularLimitMotor(joint, axis);
					count = CountRows(joint);
				}
				joints.push_back(joint);
				rowCounts.push_back(count);
				totalRows += count;
			}

			std::vector<SolverRow> rows(static_cast<std::size_t>(totalRows));
			const float framesPerSecond = 1.0f / a_timeStep;
			std::size_t base = 0;
			for (std::size_t index = 0; index < joints.size(); ++index) {
				const int count = rowCounts[index];
				if (count == 0)
					continue;
				Joint& joint = joints[index];
				const std::size_t bodyA = static_cast<std::size_t>(joint.config->bodyA);
				const std::size_t bodyB = static_cast<std::size_t>(joint.config->bodyB);
				SolverRow* first = &rows[base];
				for (int row = 0; row < count; ++row) {
					first[row].solverBodyA = joint.config->bodyA;
					first[row].solverBodyB = joint.config->bodyB;
				}
				first->cfm = a_world.solver.globalCfm;
				GetInfo2(joint, first, states[bodyA], states[bodyB], a_world.solver, framesPerSecond);
				FinalizeRows(first, count, states[bodyA], states[bodyB], solverBodies[bodyA], solverBodies[bodyB], a_world.solver.damping);
				base += static_cast<std::size_t>(count);
			}

			const std::size_t jointRowCount = rows.size();
			std::vector<ContactPush> pushes;
			pushes.reserve(a_world.contacts.size());
			for (const World::Contact& contact : a_world.contacts) {
				if (!IsBodyIndex(contact.bodyA, bodyCount) || !IsBodyIndex(contact.bodyB, bodyCount))
					continue;
				ContactPush push;
				rows.push_back(BuildContactRow(contact, states, solverBodies, a_world.solver, a_timeStep, push));
				if (contact.friction > 0.0f) {
					push.hasFriction = true;
					push.friction = contact.friction;
					push.frictionRow = BuildFrictionRow(contact, states, solverBodies, a_world.solver);
				}
				pushes.push_back(push);
			}

			const int iterations = a_world.solver.iterations;
			const auto resolve = [&solverBodies](SolverRow& a_row) {
				const float residual = ResolveRow(a_row, solverBodies[static_cast<std::size_t>(a_row.solverBodyA)],
					solverBodies[static_cast<std::size_t>(a_row.solverBodyB)]);
				return residual * residual;
			};
			for (int iteration = 0; iteration < iterations; ++iteration) {
				float leastSquaresResidual = 0.0f;
				for (std::size_t index = 0; index < jointRowCount; ++index)
					leastSquaresResidual = std::max(leastSquaresResidual, resolve(rows[index]));
				for (std::size_t contact = 0; contact < pushes.size(); ++contact) {
					SolverRow& normalRow = rows[jointRowCount + contact];
					leastSquaresResidual = std::max(leastSquaresResidual, resolve(normalRow));
					ContactPush& push = pushes[contact];
					if (push.hasFriction && normalRow.appliedImpulse > 0.0f) {
						push.frictionRow.lowerLimit = -(push.friction * normalRow.appliedImpulse);
						push.frictionRow.upperLimit = push.friction * normalRow.appliedImpulse;
						leastSquaresResidual = std::max(leastSquaresResidual, resolve(push.frictionRow));
					}
				}
				if (leastSquaresResidual <= a_world.solver.leastSquaresResidualThreshold || iteration >= iterations - 1)
					break;
			}

			// solveGroupCacheFriendlySplitImpulseIterations: the penetration part of each contact moves bodies without adding velocity.
			std::vector<Vec3> pushVelocity(bodyCount);
			std::vector<Vec3> turnVelocity(bodyCount);
			if (a_world.solver.splitImpulse) {
				for (int iteration = 0; iteration < iterations; ++iteration) {
					float leastSquaresResidual = 0.0f;
					for (std::size_t contact = 0; contact < pushes.size(); ++contact) {
						ContactPush& push = pushes[contact];
						if (push.rhsPenetration == 0.0f)
							continue;
						SolverRow& row = rows[jointRowCount + contact];
						const auto indexA = static_cast<std::size_t>(row.solverBodyA);
						const auto indexB = static_cast<std::size_t>(row.solverBodyB);
						float deltaImpulse = push.rhsPenetration - push.appliedPushImpulse * row.cfm;
						deltaImpulse -= (Dot(row.normal1, pushVelocity[indexA]) + Dot(row.relpos1CrossNormal, turnVelocity[indexA])) * row.jacDiagABInv;
						deltaImpulse -= (Dot(row.normal2, pushVelocity[indexB]) + Dot(row.relpos2CrossNormal, turnVelocity[indexB])) * row.jacDiagABInv;
						const float sum = push.appliedPushImpulse + deltaImpulse;
						if (sum < row.lowerLimit) {
							deltaImpulse = row.lowerLimit - push.appliedPushImpulse;
							push.appliedPushImpulse = row.lowerLimit;
						} else if (sum > row.upperLimit) {
							deltaImpulse = row.upperLimit - push.appliedPushImpulse;
							push.appliedPushImpulse = row.upperLimit;
						} else {
							push.appliedPushImpulse = sum;
						}
						pushVelocity[indexA] = Add(pushVelocity[indexA], Scale(Multiply(row.normal1, solverBodies[indexA].inverseMassVector), deltaImpulse));
						turnVelocity[indexA] = Add(turnVelocity[indexA], Scale(row.angularComponentA, deltaImpulse));
						pushVelocity[indexB] = Add(pushVelocity[indexB], Scale(Multiply(row.normal2, solverBodies[indexB].inverseMassVector), deltaImpulse));
						turnVelocity[indexB] = Add(turnVelocity[indexB], Scale(row.angularComponentB, deltaImpulse));
						const float residual = deltaImpulse * (1.0f / row.jacDiagABInv);
						leastSquaresResidual = std::max(leastSquaresResidual, residual * residual);
					}
					if (leastSquaresResidual <= a_world.solver.leastSquaresResidualThreshold || iteration >= iterations - 1)
						break;
				}
			}

			// btSequentialImpulseConstraintSolver::writeBackBodies.
			for (std::size_t index = 0; index < bodyCount; ++index) {
				SolverBody& solverBody = solverBodies[index];
				solverBody.linearVelocity = Add(solverBody.linearVelocity, solverBody.deltaLinearVelocity);
				solverBody.angularVelocity = Add(solverBody.angularVelocity, solverBody.deltaAngularVelocity);
				Body& body = a_world.bodies[index];
				if (a_world.solver.splitImpulse && Dot(pushVelocity[index], pushVelocity[index]) + Dot(turnVelocity[index], turnVelocity[index]) != 0.0f) {
					Transform pushed;
					IntegrateTransform(body.worldTransform, pushVelocity[index], Scale(turnVelocity[index], a_world.solver.splitImpulseTurnErp),
						a_timeStep, pushed);
					body.worldTransform = pushed;
				}
				body.linearVelocity = Add(solverBody.linearVelocity, solverBody.externalForceImpulse);
				body.angularVelocity = Add(solverBody.angularVelocity, solverBody.externalTorqueImpulse);
			}
		}

		float SetMax(float a_value, float a_limit) noexcept
		{
			return a_value < a_limit ? a_limit : a_value;
		}

		float SetMin(float a_value, float a_limit) noexcept
		{
			return a_value < a_limit ? a_value : a_limit;
		}

		Vec3 ClampVelocity(const Vec3& a_value) noexcept
		{
			return { SetMin(SetMax(a_value.x, -kVelocityLimit), kVelocityLimit),
				SetMin(SetMax(a_value.y, -kVelocityLimit), kVelocityLimit),
				SetMin(SetMax(a_value.z, -kVelocityLimit), kVelocityLimit) };
		}

		// hdtSkinnedMeshWorld::integrateTransforms, then btDiscreteDynamicsWorld::integrateTransforms.
		void IntegrateTransforms(std::vector<Body>& a_bodies, float a_timeStep)
		{
			for (Body& body : a_bodies) {
				if (!body.kinematic)
					continue;
				Transform predicted;
				IntegrateTransform(body.worldTransform, body.interpolationLinearVelocity, body.interpolationAngularVelocity,
					a_timeStep, predicted);
				body.worldTransform = predicted;
			}

			for (Body& body : a_bodies) {
				body.linearVelocity = ClampVelocity(body.linearVelocity);
				body.angularVelocity = ClampVelocity(body.angularVelocity);
			}

			for (Body& body : a_bodies) {
				if (body.kinematic)
					continue;
				Transform predicted;
				IntegrateTransform(body.worldTransform, body.linearVelocity, body.angularVelocity, a_timeStep, predicted);
				body.worldTransform = predicted;
			}
		}
	}

	void World::BeginFrame()
	{
		accumulatedForce.assign(bodies.size(), Vec3{});
		for (std::size_t index = 0; index < bodies.size(); ++index) {
			if (!bodies[index].kinematic)
				accumulatedForce[index] = Add(accumulatedForce[index], bodies[index].gravity);
		}
	}

	void World::Step(float a_timeStep)
	{
		if (a_timeStep <= 0.0f)
			return;
		PredictUnconstraintMotion(bodies, a_timeStep);
		SolveConstraints(*this, a_timeStep);
		IntegrateTransforms(bodies, a_timeStep);
	}

	void World::EndFrame()
	{
		accumulatedForce.assign(bodies.size(), Vec3{});
	}
}
