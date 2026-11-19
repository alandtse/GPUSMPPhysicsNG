// SPDX-License-Identifier: GPL-3.0-or-later WITH LicenseRef-Modding-Exception

#pragma once

#include <algorithm>
#include <cctype>
#include <string>
#include <string_view>

namespace SmpPhysics
{
	/** @brief A path with its case and slash style removed, so the same file written two ways compares equal. */
	[[nodiscard]] inline std::string PathKey(std::string_view a_path)
	{
		std::string key(a_path);
		std::ranges::transform(key, key.begin(), [](unsigned char a_c) { return a_c == '/' ? '\\' : static_cast<char>(std::tolower(a_c)); });
		return key;
	}
}
