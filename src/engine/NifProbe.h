// SPDX-License-Identifier: GPL-3.0-or-later WITH LicenseRef-Modding-Exception

#pragma once

namespace NifProbe
{
	/** @brief Starts reading the leading bytes of every body armor's and cloak's models on a worker thread, timing it. Game thread only. */
	void Start();

	[[nodiscard]] bool Running();

	/** @brief Counts up each time a scan finishes, so a reader can tell its cached answers are stale. */
	[[nodiscard]] std::uint32_t Generation();

	/** @brief Armors and cloaks scanned so far, and how many there are. */
	[[nodiscard]] std::pair<std::size_t, std::size_t> Progress();

	/** @brief The physics xml paths the last scan found in an armor's models, empty when none. */
	[[nodiscard]] std::vector<std::string> XmlPaths(RE::FormID a_armor);

	/** @brief The physics xml files on disk (loose) that no model or default-map entry names, and the names with no file. */
	[[nodiscard]] json XmlAuditResult();

	/** @brief Progress, timings and the models found carrying physics extra data. */
	[[nodiscard]] json Result();
}
