#pragma once

#include "engine/ReadyWeaponGate.h"

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>

namespace SmpPhysics::ReadyWeaponGate
{
	inline constexpr float kGripClosedAbove = 0.6f;  ///< Squeeze past which a grip counts as closed; a digital press reads 1.

	/** @brief Whether a grip event's value is a closed grip. */
	[[nodiscard]] constexpr bool IsGripValueClosed(float a_value) noexcept
	{
		return a_value > kGripClosedAbove;
	}

	/**
	 * @brief Whether each grip is closed, kept from the raw controller events the game's input manager raises, whatever the grip is mapped to.
	 * @details A grip that was down when a menu opened never sends its release, so a menu reads as open and forgets the press.
	 */
	class GripLatch
	{
	public:
		/** @brief A press (true) or release (false) of a_hand's grip, 0 left and 1 right. */
		void Set(std::size_t a_hand, bool a_down) noexcept
		{
			if (a_hand < closed.size())
				closed[a_hand].store(a_down, std::memory_order_relaxed);
		}

		[[nodiscard]] bool IsClosed(std::size_t a_hand, bool a_menuOpen) noexcept
		{
			if (a_hand >= closed.size())
				return false;
			if (a_menuOpen)
				closed[a_hand].store(false, std::memory_order_relaxed);
			return closed[a_hand].load(std::memory_order_relaxed);
		}

	private:
		std::array<std::atomic<bool>, 2> closed{};
	};

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
