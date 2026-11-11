// SPDX-License-Identifier: GPL-3.0-or-later WITH LicenseRef-Modding-Exception

#pragma once

#include <string_view>

namespace SmpPhysics
{
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
}
