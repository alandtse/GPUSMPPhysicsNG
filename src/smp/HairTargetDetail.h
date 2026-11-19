// SPDX-License-Identifier: GPL-3.0-or-later WITH LicenseRef-Modding-Exception

#pragma once

#include <cstddef>
#include <string>
#include <vector>

namespace SmpPhysics
{
	/** @brief An armor a character wears in a hair or head slot. */
	struct WornPiece
	{
		std::string name;
		bool hairSlot{};
		bool headSlot{};
	};

	/** @brief What the hair target wears on its head and which hair its ranking matched. */
	struct HairTargetDetail
	{
		std::string hairPart;  ///< The editor ID of its hair head part, empty for none.
		std::string hairPartModel;
		std::size_t extraParts{};
		std::vector<WornPiece> worn;
		std::string matchedName;  ///< The wig or hair head part the ranking compared, empty before a ranking.
		bool matchedByWig{};
		std::string matchedModel;
		std::size_t triangles{};
	};

	/** @brief A few lines for the hover tip of the target: what was matched, its mesh, the hair head part and the worn pieces. */
	[[nodiscard]] std::string FormatHairTargetDetail(const HairTargetDetail& a_detail);
}
