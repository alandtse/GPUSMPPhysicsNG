#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <optional>

namespace Util
{
	/** @brief Bone influences a skinned vertex carries. */
	inline constexpr std::size_t kSkinInfluences = 4;

	/** @brief Bytes a skinned vertex reserves for its four half-float weights and four palette indices. */
	inline constexpr std::uint32_t kSkinVertexBlockBytes = 12;

	/**
	 * @brief Converts an IEEE half float to a float.
	 * @details Infinity and NaN come back as 0, so a corrupt weight cannot poison a blend.
	 */
	[[nodiscard]] inline float HalfToFloat(std::uint16_t a_half) noexcept
	{
		const bool negative = (a_half & 0x8000u) != 0;
		const std::uint32_t exponent = (a_half >> 10) & 0x1Fu;
		const std::uint32_t mantissa = a_half & 0x3FFu;
		if (exponent == 0x1Fu)
			return 0.0f;
		const float magnitude = exponent == 0 ? std::ldexp(static_cast<float>(mantissa), -24) : std::ldexp(1.0f + static_cast<float>(mantissa) / 1024.0f, static_cast<int>(exponent) - 15);
		return negative ? -magnitude : magnitude;
	}

	/** @brief Weight a vertex's strongest bone needs to own the vertex. */
	inline constexpr float kMinDominantBoneWeight = 0.5f;

	/**
	 * @brief The skin bone that owns a vertex: the strongest influence with a valid bone that reaches kMinDominantBoneWeight.
	 * @details Each influence is checked before it is compared. Equal weights go to the lower bone index, so the answer does not
	 *          depend on the order the exporter put the four slots in. Works on any bone and weight arrays of four.
	 * @param a_bone Four skin bone indices; negative values and values not below a_boneCount are skipped.
	 * @param a_weight Four weights, one per index; NaN never qualifies.
	 * @param a_boneCount How many bones the indices are valid within; must stay below the caller's invalid-bone sentinel.
	 * @return The owning bone, or nothing when no valid influence reaches the minimum.
	 */
	template <class Bones, class Weights>
	[[nodiscard]] std::optional<std::uint32_t> DominantSkinBone(const Bones& a_bone, const Weights& a_weight, std::uint32_t a_boneCount) noexcept
	{
		std::optional<std::uint32_t> owner;
		float ownerWeight = 0.0f;
		for (std::size_t slot = 0; slot < kSkinInfluences; ++slot) {
			const auto bone = static_cast<std::int64_t>(a_bone[slot]);
			const float weight = a_weight[slot];
			if (bone < 0 || bone >= static_cast<std::int64_t>(a_boneCount) || !(weight >= kMinDominantBoneWeight))
				continue;
			const auto index = static_cast<std::uint32_t>(bone);
			if (!owner || weight > ownerWeight || (weight == ownerWeight && index < *owner)) {
				owner = index;
				ownerWeight = weight;
			}
		}
		return owner;
	}

	/** @brief A skinned vertex's weights and the palette slots they name, before the slots are mapped to skeleton bones. */
	struct SkinVertexBlock
	{
		std::array<std::uint8_t, kSkinInfluences> paletteSlot{};
		std::array<float, kSkinInfluences> weight{};
	};

	/**
	 * @brief Reads the skinning block of one vertex: four half-float weights, then four palette slot bytes.
	 * @param a_block First byte of the block, kSkinVertexBlockBytes long.
	 */
	[[nodiscard]] inline SkinVertexBlock ReadSkinVertexBlock(const std::uint8_t* a_block) noexcept
	{
		SkinVertexBlock result;
		for (std::size_t slot = 0; slot < kSkinInfluences; ++slot) {
			std::uint16_t half;
			std::memcpy(&half, a_block + slot * sizeof(half), sizeof(half));
			result.weight[slot] = HalfToFloat(half);
			result.paletteSlot[slot] = a_block[4 * sizeof(half) + slot];
		}
		return result;
	}
}
