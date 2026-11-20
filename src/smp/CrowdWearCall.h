// SPDX-License-Identifier: GPL-3.0-or-later WITH LicenseRef-Modding-Exception

#pragma once

#include <optional>
#include <string_view>

namespace SmpPhysics
{
	/** @brief The kinds of item the target can wear. */
	enum class TargetItem
	{
		Armor,
		Cloak,
		Wig
	};

	/** @brief A test crowd command and the arguments it takes; an argument that is not set is left out of the call. */
	struct CrowdWearCall
	{
		std::string_view command;
		std::optional<int> index;
		std::optional<bool> enabled;
	};

	/**
	 * @brief The crowd command that puts a_item on every test character, or takes it off.
	 * @param a_index The item's entry in its own list, past any leading "None"; ignored when taking off.
	 * @details Armor comes off by putting the crowd's outfits back, a cloak by index -1 and a wig by its own command. The settings menu and devbench send these same commands.
	 */
	[[nodiscard]] CrowdWearCall CrowdWearCommand(TargetItem a_item, bool a_takeOff, int a_index);
}
