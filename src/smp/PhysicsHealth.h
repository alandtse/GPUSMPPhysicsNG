// SPDX-License-Identifier: GPL-3.0-or-later WITH LicenseRef-Modding-Exception

#pragma once

#include <span>
#include <string_view>

namespace SmpPhysics
{
	/** @brief The reason a physics file that does not exist is given; hdtSMP64 skips such a file without a word. */
	inline constexpr std::string_view kMissingFileReason = "the physics file could not be opened";

	/** @brief How an item's physics files stand, for colouring the armor and cloak lists. */
	enum class Health
	{
		None,      ///< No physics file is known for the item.
		Working,   ///< Every file parsed as written.
		Repaired,  ///< A file had a fault this plugin repaired and used anyway.
		Broken     ///< A file is unreadable, unsupported or was declined for its own fault.
	};

	/** @brief The worse of two, with None giving way to anything. */
	[[nodiscard]] Health Worse(Health a_left, Health a_right);

	/**
	 * @brief Classifies one physics file.
	 * @param a_parsedOk The file opened and parsed as a supported rig.
	 * @param a_parseReason Why it did not, when it did not.
	 * @param a_repaired The repair log holds a problem for it.
	 * @param a_declineReason Why a claim for it was declined, empty when none was.
	 */
	[[nodiscard]] Health ClassifyFile(bool a_parsedOk, std::string_view a_parseReason, bool a_repaired, std::string_view a_declineReason);

	/** @brief One of an item's physics files, as classified and whether it does not exist. */
	struct FileHealth
	{
		Health health{};
		bool missing{};
	};

	/**
	 * @brief The health of an item from all its files.
	 * @details A model can name several physics files, one per physics object, and hdtSMP64 skips a name with no file, so
	 *          missing files are ignored while another file is there. An item whose files are all missing has no physics.
	 */
	[[nodiscard]] Health ClassifyItem(std::span<const FileHealth> a_files);
}
