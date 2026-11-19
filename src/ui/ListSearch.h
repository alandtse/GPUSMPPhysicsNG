// SPDX-License-Identifier: GPL-3.0-or-later WITH LicenseRef-Modding-Exception

#pragma once

#include "Utils/StringUtils.h"

#include <algorithm>
#include <cstddef>
#include <string>
#include <string_view>

namespace SettingsUi
{
	/** @brief What a list's filter box searches for one item: its name and physics file paths, lower case, so a filter matches either. */
	[[nodiscard]] inline std::string SearchText(std::string_view a_name, std::string_view a_files)
	{
		return Util::ToLowerAscii(a_name) + '\n' + Util::ToLowerAscii(a_files);
	}

	/** @brief How much of a scan is done, from 0 to 1; 0 while its size is unknown. */
	[[nodiscard]] inline float ScanFraction(std::size_t a_done, std::size_t a_total)
	{
		return a_total ? std::min(1.0f, static_cast<float>(a_done) / static_cast<float>(a_total)) : 0.0f;
	}
}
