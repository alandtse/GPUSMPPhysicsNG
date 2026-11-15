// Header-only wording of the settings page's status block, with no game or ImGui dependency.

#pragma once

#include <cctype>
#include <cstddef>
#include <format>
#include <string>
#include <string_view>

namespace SettingsUi
{
	/** @brief What the status block says about the hdtSMP64 link: the plugin's own reason when it has one, with what to do about a failed link. */
	[[nodiscard]] inline std::string LinkStatusText(bool a_linked, std::string_view a_linkStatus)
	{
		if (a_linkStatus.empty())
			return "hdtSMP64 was not found. Install it and load it before this plugin.";
		std::string text(a_linkStatus);
		if (a_linked)
			text.front() = static_cast<char>(std::toupper(static_cast<unsigned char>(text.front())));
		else
			text += ". Install the hdtSMP64 build named on the mod page, then restart the game.";
		return text;
	}

	/** @brief The whole status block: link state, managed files, solver time, and shader progress while the startup compile runs. */
	[[nodiscard]] inline std::string StatusText(bool a_linked, std::string_view a_linkStatus, std::size_t a_claimCount, double a_stepMilliseconds, bool a_shadersRunning, std::size_t a_shadersDone, std::size_t a_shadersTotal)
	{
		std::string text = std::format("{}\nManaged physics files: {}   Simulation time per frame: {:.3f} ms", LinkStatusText(a_linked, a_linkStatus), a_claimCount, a_stepMilliseconds);
		if (a_shadersRunning)
			text += std::format("\nPreparing shaders: {} of {} done. Claimed hair keeps the engine pose until they are ready.", a_shadersDone, a_shadersTotal);
		return text;
	}
}
