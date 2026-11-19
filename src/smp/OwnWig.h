// SPDX-License-Identifier: GPL-3.0-or-later WITH LicenseRef-Modding-Exception

#pragma once

#include <cstdint>
#include <optional>

namespace SmpPhysics
{
	/** @brief The wig a character wore before the tool changed it (0 for none), and the wig the tool put on. */
	struct OwnWig
	{
		std::uint32_t natural = 0;
		std::uint32_t ours = 0;
	};

	/**
	 * @brief The wig that is a character's own look: the one it wore before the tool changed it, while the tool's wig is still the one worn.
	 * @param a_record What the tool recorded for the character, if anything.
	 * @param a_worn The wig the character wears now, 0 for none. A different wig than the tool's means the game or the user changed it since, so the record is stale.
	 */
	[[nodiscard]] constexpr std::uint32_t NaturalWigId(const std::optional<OwnWig>& a_record, std::uint32_t a_worn) noexcept
	{
		return a_record && a_record->ours == a_worn ? a_record->natural : a_worn;
	}
}
