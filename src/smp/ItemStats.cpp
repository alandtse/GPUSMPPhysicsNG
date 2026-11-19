// SPDX-License-Identifier: GPL-3.0-or-later WITH LicenseRef-Modding-Exception

#include "smp/ItemStats.h"

#include <algorithm>
#include <array>
#include <format>

namespace SmpPhysics
{
	namespace
	{
		constexpr std::array<const char*, 4> kHealthNames = { "no physics", "works", "repaired", "broken" };

		std::string LineOf(const FileStats& a_file)
		{
			std::string line = std::format("{}: {}", a_file.file, kHealthNames[static_cast<std::size_t>(a_file.health)]);
			if (!a_file.read)
				return line + ", not read";
			line += std::format(", {} bones ({} dynamic), {} joints", a_file.bones, a_file.dynamic, a_file.joints);
			if (a_file.claim) {
				const ClaimTrim& claim = *a_file.claim;
				line += claim.keptLeast == claim.keptMost ? std::format("; claimed keeping {} bones", claim.keptMost) : std::format("; claimed keeping {}-{} bones", claim.keptLeast, claim.keptMost);
			}
			return line;
		}
	}

	std::string FormatItemStats(std::span<const FileStats> a_files, std::size_t a_maxFiles)
	{
		if (a_files.empty())
			return "No physics file found.";
		std::string text;
		const std::size_t shown = std::min(a_files.size(), a_maxFiles);
		for (std::size_t index = 0; index < shown; ++index)
			text += (index ? "\n" : "") + LineOf(a_files[index]);
		if (a_files.size() > shown)
			text += std::format("\nand {} more", a_files.size() - shown);
		return text;
	}
}
