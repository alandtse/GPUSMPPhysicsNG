// Header-only text helpers for reporting a failed shader compile, with no D3D dependency.

#pragma once

#include <format>
#include <string>
#include <string_view>
#include <vector>

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

	/** @brief The menu's alert lines for failed compiles: a heading that says what to do, then one line per failure; empty when none failed. */
	[[nodiscard]] inline std::vector<std::string> ShaderAlertLines(const std::vector<std::string>& a_failures)
	{
		std::vector<std::string> lines;
		if (a_failures.empty())
			return lines;
		lines.reserve(a_failures.size() + 1);
		lines.emplace_back("Shader compilation failed. Features that use the shaders listed below are unavailable. Check that this release's Shaders folder is installed. If the problem persists, include these errors and the plugin log in your bug report.");
		for (const std::string& failure : a_failures)
			lines.push_back("  - " + failure);
		return lines;
	}
}
