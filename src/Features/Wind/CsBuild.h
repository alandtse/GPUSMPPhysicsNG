#pragma once

#include <string_view>

namespace CsBuild
{
	/** @brief True for Community Shaders Expanded, whose FileVersion carries a "CSX" provenance tag; its interface revision 4 is not the wind API. */
	[[nodiscard]] inline bool IsExpanded(std::string_view a_fileVersion)
	{
		return a_fileVersion.find("CSX") != std::string_view::npos;
	}
}
