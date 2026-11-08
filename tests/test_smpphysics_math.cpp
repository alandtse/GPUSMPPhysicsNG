// Unit tests for the rotation-matrix conversions in
// src/smp/Math.h. Pure maths; no game/D3D dependency.

#include "smp/Math.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <limits>

using namespace SmpPhysics;
using Catch::Approx;

namespace
{
	constexpr float kPi = 3.14159265358979323846f;
	constexpr float kMatrixMargin = 1e-5f;
	constexpr float kRotationMargin = 1e-5f;

	/** Compares quaternions as rotations, so q and -q compare equal. */
	void RequireSameRotation(const Quat& a_left, const Quat& a_right)
	{
		const Quat left = a_left.Normalized();
		const Quat right = a_right.Normalized();
		const float cosine = left.w * right.w + left.x * right.x + left.y * right.y + left.z * right.z;
		REQUIRE(std::abs(cosine) == Approx(1.0f).margin(kRotationMargin));
	}

	void RequireSameMatrix(const float (&a_left)[3][3], const float (&a_right)[3][3])
	{
		for (int row = 0; row < 3; ++row) {
			for (int column = 0; column < 3; ++column)
				REQUIRE(a_left[row][column] == Approx(a_right[row][column]).margin(kMatrixMargin));
		}
	}

	void RequireFiniteQuaternion(const Quat& a_value)
	{
		REQUIRE(std::isfinite(a_value.w));
		REQUIRE(std::isfinite(a_value.x));
		REQUIRE(std::isfinite(a_value.y));
		REQUIRE(std::isfinite(a_value.z));
	}

	/** R * v for a column-vector rotation matrix. */
	Vec3 Multiply(const float (&a_matrix)[3][3], const Vec3& a_vector)
	{
		return { a_matrix[0][0] * a_vector.x + a_matrix[0][1] * a_vector.y + a_matrix[0][2] * a_vector.z,
			a_matrix[1][0] * a_vector.x + a_matrix[1][1] * a_vector.y + a_matrix[1][2] * a_vector.z,
			a_matrix[2][0] * a_vector.x + a_matrix[2][1] * a_vector.y + a_matrix[2][2] * a_vector.z };
	}

	const Quat kSampleRotations[] = {
		{ 1.0f, 0.0f, 0.0f, 0.0f },
		FromAxisAngle({ 1.0f, 0.0f, 0.0f }, 0.9f),
		FromAxisAngle({ 0.0f, 1.0f, 0.0f }, -2.4f),
		FromAxisAngle({ 0.0f, 0.0f, 1.0f }, 3.0f),
		FromAxisAngle({ 0.3f, -0.5f, 0.81f }, 1.7f),
		FromAxisAngle({ -0.2f, 0.6f, 0.3f }, kPi),
		FromEulerXYZ({ 0.4f, -0.9f, 2.1f })
	};

	const Vec3 kSampleVectors[] = {
		{ 1.0f, 0.0f, 0.0f },
		{ 0.0f, 1.0f, 0.0f },
		{ 0.0f, 0.0f, 1.0f },
		{ 0.5f, -1.25f, 2.0f },
		{ -3.0f, 0.75f, 0.0f }
	};
}

TEST_CASE("Quarter turns about each axis give the expected matrices and quaternions", "[SmpPhysics]")
{
	constexpr float quarterTurn = kPi * 0.5f;

	float matrixX[3][3] = {};
	ToRotationMatrix(FromAxisAngle({ 1.0f, 0.0f, 0.0f }, quarterTurn), matrixX);
	const float expectedX[3][3] = {
		{ 1.0f, 0.0f, 0.0f },
		{ 0.0f, 0.0f, -1.0f },
		{ 0.0f, 1.0f, 0.0f }
	};
	RequireSameMatrix(matrixX, expectedX);

	float matrixY[3][3] = {};
	ToRotationMatrix(FromAxisAngle({ 0.0f, 1.0f, 0.0f }, quarterTurn), matrixY);
	const float expectedY[3][3] = {
		{ 0.0f, 0.0f, 1.0f },
		{ 0.0f, 1.0f, 0.0f },
		{ -1.0f, 0.0f, 0.0f }
	};
	RequireSameMatrix(matrixY, expectedY);

	float matrixZ[3][3] = {};
	ToRotationMatrix(FromAxisAngle({ 0.0f, 0.0f, 1.0f }, quarterTurn), matrixZ);
	const float expectedZ[3][3] = {
		{ 0.0f, -1.0f, 0.0f },
		{ 1.0f, 0.0f, 0.0f },
		{ 0.0f, 0.0f, 1.0f }
	};
	RequireSameMatrix(matrixZ, expectedZ);

	constexpr float halfRootTwo = 0.70710678f;
	RequireSameRotation(FromRotationMatrix(expectedX), Quat{ halfRootTwo, halfRootTwo, 0.0f, 0.0f });
	RequireSameRotation(FromRotationMatrix(expectedY), Quat{ halfRootTwo, 0.0f, halfRootTwo, 0.0f });
	RequireSameRotation(FromRotationMatrix(expectedZ), Quat{ halfRootTwo, 0.0f, 0.0f, halfRootTwo });
	RequireSameRotation(FromRotationMatrix(expectedX), FromAxisAngle({ 1.0f, 0.0f, 0.0f }, quarterTurn));
	RequireSameRotation(FromRotationMatrix(expectedY), FromAxisAngle({ 0.0f, 1.0f, 0.0f }, quarterTurn));
	RequireSameRotation(FromRotationMatrix(expectedZ), FromAxisAngle({ 0.0f, 0.0f, 1.0f }, quarterTurn));
}

TEST_CASE("Quaternion to matrix to quaternion round trips", "[SmpPhysics]")
{
	for (const Quat& rotation : kSampleRotations) {
		float matrix[3][3] = {};
		ToRotationMatrix(rotation, matrix);
		RequireSameRotation(FromRotationMatrix(matrix), rotation);
	}
}

TEST_CASE("Matrix to quaternion to matrix round trips", "[SmpPhysics]")
{
	for (const Quat& rotation : kSampleRotations) {
		float matrix[3][3] = {};
		ToRotationMatrix(rotation, matrix);

		float roundTrip[3][3] = {};
		ToRotationMatrix(FromRotationMatrix(matrix), roundTrip);
		RequireSameMatrix(roundTrip, matrix);
	}
}

TEST_CASE("Half turns convert without producing NaN", "[SmpPhysics]")
{
	const Vec3 axes[] = {
		{ 1.0f, 0.0f, 0.0f },
		{ 0.0f, 1.0f, 0.0f },
		{ 0.0f, 0.0f, 1.0f },
		{ 0.3f, -0.6f, 0.74f }
	};

	for (const Vec3& axis : axes) {
		const Quat halfTurn = FromAxisAngle(axis, kPi);
		float matrix[3][3] = {};
		ToRotationMatrix(halfTurn, matrix);

		// A half turn is where the trace-based divisor vanishes, so this is the case that
		// a naive conversion turns into 0/0.
		REQUIRE(matrix[0][0] + matrix[1][1] + matrix[2][2] == Approx(-1.0f).margin(1e-4f));

		const Quat recovered = FromRotationMatrix(matrix);
		RequireFiniteQuaternion(recovered);
		RequireSameRotation(recovered, halfTurn);
	}
}

TEST_CASE("A non-finite matrix converts to the identity", "[SmpPhysics]")
{
	float notANumber[3][3] = {
		{ 1.0f, 0.0f, 0.0f },
		{ 0.0f, 1.0f, 0.0f },
		{ 0.0f, 0.0f, std::numeric_limits<float>::quiet_NaN() }
	};
	RequireSameRotation(FromRotationMatrix(notANumber), Quat{});

	float infinite[3][3] = {
		{ std::numeric_limits<float>::infinity(), 0.0f, 0.0f },
		{ 0.0f, 1.0f, 0.0f },
		{ 0.0f, 0.0f, 1.0f }
	};
	RequireSameRotation(FromRotationMatrix(infinite), Quat{});
}

TEST_CASE("A rotation matrix rotates a vector exactly like Quat::Rotate", "[SmpPhysics]")
{
	for (const Quat& rotation : kSampleRotations) {
		float matrix[3][3] = {};
		ToRotationMatrix(rotation, matrix);

		for (const Vec3& vector : kSampleVectors) {
			const Vec3 byMatrix = Multiply(matrix, vector);
			const Vec3 byQuaternion = rotation.Rotate(vector);
			REQUIRE(byMatrix.x == Approx(byQuaternion.x).margin(1e-4f));
			REQUIRE(byMatrix.y == Approx(byQuaternion.y).margin(1e-4f));
			REQUIRE(byMatrix.z == Approx(byQuaternion.z).margin(1e-4f));
		}
	}
}
