#pragma once
#include <algorithm>
#include <cmath>
#include <concepts>
#include <cstdint>
#include <string_view>

namespace Util
{
	/**
	 * @brief Clamps a floating-point value to [min, max] if finite, otherwise returns fallback default.
	 */
	template <std::floating_point T>
	[[nodiscard]] constexpr T ClampFinite(T a_value, T a_min, T a_max, T a_default) noexcept
	{
		return std::isfinite(a_value) ? std::clamp(a_value, a_min, a_max) : a_default;
	}

	/** @brief FNV-1a 64-bit offset basis and prime. */
	inline constexpr std::uint64_t kFnv1a64Basis = 14695981039346656037ull;
	inline constexpr std::uint64_t kFnv1a64Prime = 1099511628211ull;

	/** @brief 64-bit FNV-1a of the bytes of @p a_text; pass a previous result as @p a_seed to hash several strings in sequence. */
	[[nodiscard]] constexpr std::uint64_t Fnv1a64(std::string_view a_text, std::uint64_t a_seed = kFnv1a64Basis) noexcept
	{
		std::uint64_t hash = a_seed;
		for (const char c : a_text) {
			hash ^= static_cast<unsigned char>(c);
			hash *= kFnv1a64Prime;
		}
		return hash;
	}
}
