// Header-only string helpers with no PCH, RE or SKSE dependency.

#pragma once

#include <algorithm>
#include <cctype>
#include <string>
#include <string_view>

namespace Util
{
	/**
	 * @brief Lowercases the ASCII letters in @p a_str, returning a new string.
	 *
	 * The C++ standard library has no whole-string lowercase, so this wraps the
	 * canonical std::tolower idiom. Locale-independent for ASCII; suitable for
	 * case-insensitive matching of asset paths.
	 *
	 * @param a_str Input text.
	 * @return A lowercased copy of @p a_str.
	 */
	inline std::string ToLowerAscii(std::string_view a_str)
	{
		std::string result;
		result.reserve(a_str.size());
		for (char c : a_str)
			result.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(c))));
		return result;
	}

	/** @brief Case-insensitive equality for two strings. */
	inline bool IEquals(std::string_view a, std::string_view b)
	{
		return a.size() == b.size() && std::equal(a.begin(), a.end(), b.begin(), [](char ca, char cb) {
			return std::tolower(static_cast<unsigned char>(ca)) == std::tolower(static_cast<unsigned char>(cb));
		});
	}
}
