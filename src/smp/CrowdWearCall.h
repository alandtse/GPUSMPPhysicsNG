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
		Wig,
		HairPart
	};

	/** @brief Who the item pickers' Put on and Take off reach; the values are the Apply to box's entries. */
	enum class ApplyTo
	{
		Crowd = 0,   ///< Every test character.
		Target = 1,  ///< The one character chosen under Target.
		Nobody = 2   ///< Nobody: the lists only browse.
	};

	/** @brief The scope a box entry names; no box means the target, and an entry that is not one of the three means nobody, so a stray value never dresses anyone. */
	[[nodiscard]] constexpr ApplyTo ApplyToOf(std::optional<int> a_pick) noexcept
	{
		if (!a_pick)
			return ApplyTo::Target;
		switch (*a_pick) {
		case static_cast<int>(ApplyTo::Crowd):
			return ApplyTo::Crowd;
		case static_cast<int>(ApplyTo::Target):
			return ApplyTo::Target;
		default:
			return ApplyTo::Nobody;
		}
	}

	/** @brief The names of the crowd commands that dress and strip the test crowd: registered under these, and sent by the settings menu. */
	namespace CrowdWearCommands
	{
		inline constexpr std::string_view kStrip = "stripTestCrowd";
		inline constexpr std::string_view kArmor = "setTestCrowdArmor";
		inline constexpr std::string_view kCloak = "setTestCrowdCloak";
		inline constexpr std::string_view kWig = "setTestCrowdWig";
		inline constexpr std::string_view kTakeOffWig = "takeOffTestCrowdWig";
		inline constexpr std::string_view kHairPart = "setTestCrowdHairPart";
		inline constexpr std::string_view kRestoreHairParts = "restoreTestCrowdHairParts";
	}

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
