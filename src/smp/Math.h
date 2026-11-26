#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>

namespace SmpPhysics
{
	/** @brief A per-collider or per-surface mask with every bit set. */
	inline constexpr std::uint32_t kAllMaskBits = 0xFFFFFFFFu;
	/** @brief Surfaces a per-bone surface mask can name, one bit each. */
	inline constexpr std::size_t kMaskedSurfaceLimit = 32;
	/** @brief Length below which a vector is treated as having no usable direction. */
	inline constexpr float kDirectionEpsilon = 1e-8f;
	/** @brief Cosine above which Slerp falls back to a normalized lerp, because the sine normalizer vanishes on nearly parallel inputs. */
	inline constexpr float kSlerpLinearThreshold = 0.9995f;
	/** @brief Largest-squared-component below which FromRotationMatrix treats a matrix as degenerate, because its 1/sqrt divisor would otherwise reach infinity. */
	inline constexpr float kMatrixQuaternionEpsilon = 1e-6f;

	struct Vec3
	{
		float x{};
		float y{};
		float z{};
	};

	struct Quat
	{
		float w{ 1.0f };
		float x{};
		float y{};
		float z{};

		/** @brief Unit quaternion, or the identity when the input has no usable magnitude. */
		[[nodiscard]] Quat Normalized() const noexcept;
		/** @brief Rotates a vector by this quaternion. Assumes a unit quaternion. */
		[[nodiscard]] Vec3 Rotate(const Vec3& a_value) const noexcept;
	};

	struct Transform
	{
		Quat rotation{};
		Vec3 translation{};

		/** @brief Left inverse, so that Inverse(t) * t is the identity. */
		[[nodiscard]] Transform Inverse() const noexcept;
		[[nodiscard]] Vec3 TransformPoint(const Vec3& a_point) const noexcept;
	};

	inline Vec3 operator+(const Vec3& a_left, const Vec3& a_right) noexcept
	{
		return { a_left.x + a_right.x, a_left.y + a_right.y, a_left.z + a_right.z };
	}

	inline Vec3 operator-(const Vec3& a_left, const Vec3& a_right) noexcept
	{
		return { a_left.x - a_right.x, a_left.y - a_right.y, a_left.z - a_right.z };
	}

	inline Vec3 operator-(const Vec3& a_value) noexcept
	{
		return { -a_value.x, -a_value.y, -a_value.z };
	}

	inline Vec3 operator*(const Vec3& a_value, float a_scale) noexcept
	{
		return { a_value.x * a_scale, a_value.y * a_scale, a_value.z * a_scale };
	}

	inline Vec3 operator*(float a_scale, const Vec3& a_value) noexcept
	{
		return a_value * a_scale;
	}

	inline Vec3 operator/(const Vec3& a_value, float a_divisor) noexcept
	{
		return { a_value.x / a_divisor, a_value.y / a_divisor, a_value.z / a_divisor };
	}

	[[nodiscard]] inline float Dot(const Vec3& a_left, const Vec3& a_right) noexcept
	{
		return a_left.x * a_right.x + a_left.y * a_right.y + a_left.z * a_right.z;
	}

	[[nodiscard]] inline Vec3 Cross(const Vec3& a_left, const Vec3& a_right) noexcept
	{
		return {
			a_left.y * a_right.z - a_left.z * a_right.y,
			a_left.z * a_right.x - a_left.x * a_right.z,
			a_left.x * a_right.y - a_left.y * a_right.x
		};
	}

	[[nodiscard]] inline float Length(const Vec3& a_value) noexcept
	{
		return std::sqrt(Dot(a_value, a_value));
	}

	/** @brief One of the three coordinate axes, for code that indexes axes rather than naming them. */
	[[nodiscard]] inline Vec3 UnitAxis(int a_axis) noexcept
	{
		switch (a_axis) {
		case 0:
			return { 1.0f, 0.0f, 0.0f };
		case 1:
			return { 0.0f, 1.0f, 0.0f };
		default:
			return { 0.0f, 0.0f, 1.0f };
		}
	}

	/** @brief Unit vector, or zero when the input is too short to have a direction. */
	[[nodiscard]] inline Vec3 Normalized(const Vec3& a_value) noexcept
	{
		const float length = Length(a_value);
		if (!(length > kDirectionEpsilon))
			return {};
		return a_value / length;
	}

	inline Quat operator*(const Quat& a_left, const Quat& a_right) noexcept
	{
		return {
			a_left.w * a_right.w - a_left.x * a_right.x - a_left.y * a_right.y - a_left.z * a_right.z,
			a_left.w * a_right.x + a_left.x * a_right.w + a_left.y * a_right.z - a_left.z * a_right.y,
			a_left.w * a_right.y - a_left.x * a_right.z + a_left.y * a_right.w + a_left.z * a_right.x,
			a_left.w * a_right.z + a_left.x * a_right.y - a_left.y * a_right.x + a_left.z * a_right.w
		};
	}

	inline Quat operator+(const Quat& a_left, const Quat& a_right) noexcept
	{
		return { a_left.w + a_right.w, a_left.x + a_right.x, a_left.y + a_right.y, a_left.z + a_right.z };
	}

	inline Quat operator*(const Quat& a_value, float a_scale) noexcept
	{
		return { a_value.w * a_scale, a_value.x * a_scale, a_value.y * a_scale, a_value.z * a_scale };
	}

	[[nodiscard]] inline Quat Conjugate(const Quat& a_value) noexcept
	{
		return { a_value.w, -a_value.x, -a_value.y, -a_value.z };
	}

	[[nodiscard]] inline Quat FromAxisAngle(const Vec3& a_axis, float a_angle) noexcept
	{
		const Vec3 direction = Normalized(a_axis);
		if (direction.x == 0.0f && direction.y == 0.0f && direction.z == 0.0f)
			return {};
		const float half = a_angle * 0.5f;
		const float sine = std::sin(half);
		return { std::cos(half), direction.x * sine, direction.y * sine, direction.z * sine };
	}

	/** @brief Rotation vector of a quaternion, taking the shorter of the two equivalent rotations. */
	[[nodiscard]] inline Vec3 ToRotationVector(const Quat& a_rotation) noexcept
	{
		Quat value = a_rotation.Normalized();
		if (value.w < 0.0f)
			value = { -value.w, -value.x, -value.y, -value.z };
		const float axisLength = std::sqrt(value.x * value.x + value.y * value.y + value.z * value.z);
		if (!(axisLength > kDirectionEpsilon))
			return {};
		const float angle = 2.0f * std::atan2(axisLength, value.w);
		const float scale = angle / axisLength;
		return { value.x * scale, value.y * scale, value.z * scale };
	}

	[[nodiscard]] inline Quat Slerp(const Quat& a_from, const Quat& a_to, float a_amount) noexcept
	{
		const Quat start = a_from.Normalized();
		Quat end = a_to.Normalized();
		float cosine = start.w * end.w + start.x * end.x + start.y * end.y + start.z * end.z;
		if (cosine < 0.0f) {
			end = { -end.w, -end.x, -end.y, -end.z };
			cosine = -cosine;
		}
		if (cosine > kSlerpLinearThreshold)
			return (start * (1.0f - a_amount) + end * a_amount).Normalized();
		const float angle = std::acos(std::clamp(cosine, -1.0f, 1.0f));
		const float sine = std::sin(angle);
		return (start * (std::sin((1.0f - a_amount) * angle) / sine) + end * (std::sin(a_amount * angle) / sine))
		    .Normalized();
	}

	inline Quat Quat::Normalized() const noexcept
	{
		const float length = std::sqrt(w * w + x * x + y * y + z * z);
		if (!(length > kDirectionEpsilon))
			return {};
		return { w / length, x / length, y / length, z / length };
	}

	inline Vec3 Quat::Rotate(const Vec3& a_value) const noexcept
	{
		const Vec3 axis{ x, y, z };
		const Vec3 twiceCross = Cross(axis, a_value) * 2.0f;
		return a_value + twiceCross * w + Cross(axis, twiceCross);
	}

	inline Transform Transform::Inverse() const noexcept
	{
		const Quat inverseRotation = Conjugate(rotation.Normalized());
		return { inverseRotation, -inverseRotation.Rotate(translation) };
	}

	inline Vec3 Transform::TransformPoint(const Vec3& a_point) const noexcept
	{
		return translation + rotation.Rotate(a_point);
	}

	/** @brief A point through a skin transform whose rotated part is scaled: the bone's world scale times the bind scale, which a Transform cannot hold. */
	[[nodiscard]] inline Vec3 ScaledSkinPoint(const Transform& a_skin, const Vec3& a_point, float a_scale) noexcept
	{
		return a_skin.translation + (a_skin.TransformPoint(a_point) - a_skin.translation) * a_scale;
	}

	/** @brief Composes so that (a * b).TransformPoint(p) equals a.TransformPoint(b.TransformPoint(p)). */
	inline Transform operator*(const Transform& a_left, const Transform& a_right) noexcept
	{
		return { a_left.rotation * a_right.rotation,
			a_left.translation + a_left.rotation.Rotate(a_right.translation) };
	}

	/** @brief Euler angles of R = Rx(x) * Ry(y) * Rz(z), unique only while |y| < pi/2. */
	[[nodiscard]] inline Vec3 ToEulerXYZ(const Quat& a_rotation) noexcept
	{
		const Quat value = a_rotation.Normalized();
		const float r00 = 1.0f - 2.0f * (value.y * value.y + value.z * value.z);
		const float r01 = 2.0f * (value.x * value.y - value.w * value.z);
		const float r02 = 2.0f * (value.x * value.z + value.w * value.y);
		const float r12 = 2.0f * (value.y * value.z - value.w * value.x);
		const float r22 = 1.0f - 2.0f * (value.x * value.x + value.y * value.y);
		return { std::atan2(-r12, r22), std::asin(std::clamp(r02, -1.0f, 1.0f)), std::atan2(-r01, r00) };
	}

	/** @brief Quaternion of R = Rx(x) * Ry(y) * Rz(z). */
	[[nodiscard]] inline Quat FromEulerXYZ(const Vec3& a_angles) noexcept
	{
		const Quat rotationX{ std::cos(a_angles.x * 0.5f), std::sin(a_angles.x * 0.5f), 0.0f, 0.0f };
		const Quat rotationY{ std::cos(a_angles.y * 0.5f), 0.0f, std::sin(a_angles.y * 0.5f), 0.0f };
		const Quat rotationZ{ std::cos(a_angles.z * 0.5f), 0.0f, 0.0f, std::sin(a_angles.z * 0.5f) };
		return (rotationX * rotationY * rotationZ).Normalized();
	}

	/**
	 * @brief Unit quaternion of a rotation matrix in the column-vector convention, where R * v rotates v.
	 * @details Picks the largest of the four trace-based divisors, which stays positive for every
	 *          rotation including the 180 degree case where the trace is -1.
	 * @return The identity for a matrix with a non-finite entry or one too degenerate to normalise.
	 */
	[[nodiscard]] inline Quat FromRotationMatrix(const float (&a_matrix)[3][3]) noexcept
	{
		for (int row = 0; row < 3; ++row) {
			for (int column = 0; column < 3; ++column) {
				if (!std::isfinite(a_matrix[row][column]))
					return {};
			}
		}

		const float trace = a_matrix[0][0] + a_matrix[1][1] + a_matrix[2][2];
		if (trace + 1.0f > kMatrixQuaternionEpsilon) {
			const float scale = std::sqrt(trace + 1.0f) * 2.0f;
			return Quat{ scale * 0.25f,
				(a_matrix[2][1] - a_matrix[1][2]) / scale,
				(a_matrix[0][2] - a_matrix[2][0]) / scale,
				(a_matrix[1][0] - a_matrix[0][1]) / scale }
			    .Normalized();
		}

		const float xSquared = 1.0f + a_matrix[0][0] - a_matrix[1][1] - a_matrix[2][2];
		if (xSquared > kMatrixQuaternionEpsilon) {
			const float scale = std::sqrt(xSquared) * 2.0f;
			return Quat{ (a_matrix[2][1] - a_matrix[1][2]) / scale,
				scale * 0.25f,
				(a_matrix[0][1] + a_matrix[1][0]) / scale,
				(a_matrix[0][2] + a_matrix[2][0]) / scale }
			    .Normalized();
		}

		const float ySquared = 1.0f + a_matrix[1][1] - a_matrix[0][0] - a_matrix[2][2];
		if (ySquared > kMatrixQuaternionEpsilon) {
			const float scale = std::sqrt(ySquared) * 2.0f;
			return Quat{ (a_matrix[0][2] - a_matrix[2][0]) / scale,
				(a_matrix[0][1] + a_matrix[1][0]) / scale,
				scale * 0.25f,
				(a_matrix[1][2] + a_matrix[2][1]) / scale }
			    .Normalized();
		}

		const float zSquared = 1.0f + a_matrix[2][2] - a_matrix[0][0] - a_matrix[1][1];
		if (!(zSquared > kMatrixQuaternionEpsilon))
			return {};
		const float scale = std::sqrt(zSquared) * 2.0f;
		return Quat{ (a_matrix[1][0] - a_matrix[0][1]) / scale,
			(a_matrix[0][2] + a_matrix[2][0]) / scale,
			(a_matrix[1][2] + a_matrix[2][1]) / scale,
			scale * 0.25f }
		    .Normalized();
	}

	/** @brief Rotation matrix of a unit quaternion in the column-vector convention, where R * v equals Quat::Rotate(v). The quaternion is normalised first. */
	inline void ToRotationMatrix(const Quat& a_rotation, float (&a_matrix)[3][3]) noexcept
	{
		const Quat value = a_rotation.Normalized();
		const float xx = value.x * value.x;
		const float yy = value.y * value.y;
		const float zz = value.z * value.z;
		const float xy = value.x * value.y;
		const float xz = value.x * value.z;
		const float yz = value.y * value.z;
		const float wx = value.w * value.x;
		const float wy = value.w * value.y;
		const float wz = value.w * value.z;

		a_matrix[0][0] = 1.0f - 2.0f * (yy + zz);
		a_matrix[0][1] = 2.0f * (xy - wz);
		a_matrix[0][2] = 2.0f * (xz + wy);
		a_matrix[1][0] = 2.0f * (xy + wz);
		a_matrix[1][1] = 1.0f - 2.0f * (xx + zz);
		a_matrix[1][2] = 2.0f * (yz - wx);
		a_matrix[2][0] = 2.0f * (xz - wy);
		a_matrix[2][1] = 2.0f * (yz + wx);
		a_matrix[2][2] = 1.0f - 2.0f * (xx + yy);
	}

	[[nodiscard]] inline bool IsFiniteVector(const Vec3& a_value) noexcept
	{
		return std::isfinite(a_value.x) && std::isfinite(a_value.y) && std::isfinite(a_value.z);
	}

	[[nodiscard]] inline bool IsFiniteTransform(const Transform& a_transform) noexcept
	{
		return std::isfinite(a_transform.rotation.w) && std::isfinite(a_transform.rotation.x) &&
		       std::isfinite(a_transform.rotation.y) && std::isfinite(a_transform.rotation.z) &&
		       IsFiniteVector(a_transform.translation);
	}

	/** @brief Thread groups needed to cover a_threads threads at a_threadsPerGroup threads each. */
	[[nodiscard]] constexpr std::uint32_t DispatchGroups(std::uint32_t a_threads, std::uint32_t a_threadsPerGroup) noexcept
	{
		return (a_threads + a_threadsPerGroup - 1u) / a_threadsPerGroup;
	}
}
