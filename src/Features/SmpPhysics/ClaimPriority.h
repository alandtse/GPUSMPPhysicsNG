#pragma once

#include <cstdint>

namespace SmpPhysics
{
	/** @brief Who a claim belongs to; the budget fills with lower tiers first. */
	enum class ClaimTier : std::uint8_t
	{
		Player,
		Follower,
		Other
	};

	struct RankedClaim
	{
		ClaimTier tier{ ClaimTier::Other };
		float distance{};  ///< Camera distance, already biased for hysteresis.
	};

	/** @brief Orders the claims the budget keeps: the player's, then the followers', then the rest, nearest first within each. */
	[[nodiscard]] constexpr bool RanksBefore(const RankedClaim& a_left, const RankedClaim& a_right) noexcept
	{
		return a_left.tier != a_right.tier ? a_left.tier < a_right.tier : a_left.distance < a_right.distance;
	}
}
