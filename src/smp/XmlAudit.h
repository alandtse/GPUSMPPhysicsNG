// SPDX-License-Identifier: GPL-3.0-or-later WITH LicenseRef-Modding-Exception

#pragma once

#include <string>
#include <string_view>
#include <vector>

namespace SmpPhysics
{
	/** @brief A path as compared: lower case, backslashes, and any leading Data folder removed. */
	[[nodiscard]] std::string NormalizeXmlPath(std::string_view a_path);

	/** @brief The file= value of every <map> in an hdtSMP64 defaultBBPs.xml, as written. */
	[[nodiscard]] std::vector<std::string> ParseDefaultBbpFiles(std::string_view a_xml);

	struct XmlAudit
	{
		std::vector<std::string> orphans;   ///< On disk, but no model and no default-map entry names it.
		std::vector<std::string> dangling;  ///< Named by a model or the default map, but not on disk.
	};

	/** @brief Compares the physics xml files found on disk with the paths models and the default map name; both lists come back sorted. */
	[[nodiscard]] XmlAudit AuditXmlFiles(const std::vector<std::string>& a_onDisk, const std::vector<std::string>& a_referenced);
}
