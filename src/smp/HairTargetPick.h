// SPDX-License-Identifier: GPL-3.0-or-later WITH LicenseRef-Modding-Exception

#pragma once

#include <cstdint>

namespace SmpPhysics
{
	/** @brief Where the character whose hair is matched and swapped came from. */
	enum class HairTargetSource
	{
		Player,
		Console,   ///< The reference selected with prid.
		Crosshair  ///< The character last looked at while no menu was open.
	};

	/**
	 * @brief Which character the hair tools act on, following what the user selects.
	 * @details A form ID of 0 means the player. A new console selection is taken at once unless the user pinned a source; one that has not
	 *          changed is not, so a selection made long ago never takes the choice back.
	 */
	struct HairTargetPick
	{
		HairTargetSource source{ HairTargetSource::Player };
		std::uint32_t formID{};
		bool pinned{};
		std::uint32_t console{};
		std::uint32_t crosshair{};
	};

	/** @brief Takes a new console selection, 0 for none; a_pick follows it unless pinned. */
	void UpdatePick(HairTargetPick& a_pick, std::uint32_t a_console);

	/** @brief Remembers the character the crosshair is on; none, as when a menu opens, keeps the last. */
	void NoteCrosshair(HairTargetPick& a_pick, std::uint32_t a_crosshair);

	/** @brief Pins a_source as the target; false, changing nothing, when it has no character to pin. */
	bool PinPick(HairTargetPick& a_pick, HairTargetSource a_source);

	/** @brief Lets the console selection set the target again. */
	void UnpinPick(HairTargetPick& a_pick);

	/** @brief Forgets a character that is gone; a target that was it falls back to the player. */
	void ForgetPick(HairTargetPick& a_pick, std::uint32_t a_formID);
}
