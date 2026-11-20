#pragma once

#include <cstdint>

namespace SmpPhysics::ReadyWeaponGate
{
	/** @brief Bit for the left hand in the mask SetHeldHands takes. */
	inline constexpr std::uint32_t kLeftHand = 1u << 0;
	/** @brief Bit for the right hand. */
	inline constexpr std::uint32_t kRightHand = 1u << 1;

	/**
	 * @brief Hooks the player's Ready Weapon handler so a grip that is closing on hair does not also draw or sheathe the weapon.
	 * @details VR only and safe to call more than once. The grip press is a Ready Weapon button event; while a hand is in the mask, that
	 *          hand's press is swallowed, and so is its release, so the game never sees half of a press.
	 */
	void Install();

	/** @brief The hands whose grip is meant for hair this frame: a bit per hand. Call every frame, from any thread. */
	void SetHeldHands(std::uint32_t a_mask) noexcept;

	/** @brief Whether a controller's grip is squeezed, from the press and release events the gate sees; open while a menu is up. */
	[[nodiscard]] bool IsGripDown(bool a_isLeft) noexcept;

	/** @brief How many Ready Weapon events the gate has swallowed since the game started. */
	[[nodiscard]] std::uint32_t SwallowedCount() noexcept;
}
