#pragma once

#include "engine/ReadyWeaponGate.h"

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>

namespace SmpPhysics::ReadyWeaponGate
{
	inline constexpr std::size_t kGripAxis = 2;          ///< The controller axis that carries grip squeeze on Oculus-style controllers.
	inline constexpr float kGripAxisClosedAbove = 0.6f;  ///< Squeeze past which the grip counts as closed when the button bit is not set.

	/** @brief Whether a grip is closed, from its button bit or its squeeze axis. */
	[[nodiscard]] constexpr bool IsGripClosed(bool a_buttonPressed, float a_squeeze) noexcept
	{
		return a_buttonPressed || a_squeeze > kGripAxisClosedAbove;
	}

	/** @brief Remembers, per hand, whether the press the game last saw was swallowed, so the release matches it. */
	class PressTracker
	{
	public:
		/** @brief A press from a_hand (0 left, 1 right); true when its bit is in a_heldHands, in which case its release is swallowed too. */
		bool Press(std::size_t a_hand, std::uint32_t a_heldHands) noexcept
		{
			if (a_hand >= swallowed.size())
				return false;
			const std::uint32_t bit = a_hand == 0 ? kLeftHand : kRightHand;
			const bool swallow = (a_heldHands & bit) != 0;
			swallowed[a_hand].store(swallow, std::memory_order_relaxed);
			return swallow;
		}

		/** @brief Any later event from a_hand: true while its press was swallowed; a_isUp ends that. */
		bool Follow(std::size_t a_hand, bool a_isUp) noexcept
		{
			if (a_hand >= swallowed.size())
				return false;
			const bool swallow = swallowed[a_hand].load(std::memory_order_relaxed);
			if (a_isUp)
				swallowed[a_hand].store(false, std::memory_order_relaxed);
			return swallow;
		}

	private:
		std::array<std::atomic<bool>, 2> swallowed{};
	};
}
