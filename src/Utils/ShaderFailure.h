// Header-only text helpers for reporting a failed shader compile, with no D3D dependency.

#pragma once

#include <format>
#include <string>
#include <string_view>

namespace Util
{
	/**
	 * @brief The first error line of a D3DCompile message, without the folder of the file it names.
	 * @details Falls back to the first non-empty line when none contains " error ", and to an empty string for an empty message.
	 */
	[[nodiscard]] inline std::string FirstShaderErrorLine(std::string_view a_message)
	{
		std::string_view fallback;
		while (!a_message.empty()) {
			const std::size_t end = a_message.find('\n');
			std::string_view line = a_message.substr(0, end);
			a_message = end == std::string_view::npos ? std::string_view{} : a_message.substr(end + 1);
			while (!line.empty() && (line.back() == '\r' || line.back() == ' '))
				line.remove_suffix(1);
			if (line.empty())
				continue;
			if (fallback.empty())
				fallback = line;
			const std::size_t marker = line.find(": error ");
			if (marker == std::string_view::npos)
				continue;
			const std::size_t folder = line.substr(0, marker).find_last_of("\\/");
			return std::string(folder == std::string_view::npos ? line : line.substr(folder + 1));
		}
		return std::string(fallback);
	}

	/** @brief One menu line for a failed compile: "<file> (<entry>): <detail>". */
	[[nodiscard]] inline std::string DescribeShaderFailure(std::string_view a_file, std::string_view a_entry, std::string_view a_detail)
	{
		return std::format("{} ({}): {}", a_file, a_entry, a_detail.empty() ? std::string_view("compile failed") : a_detail);
	}
}
