// SPDX-License-Identifier: GPL-3.0-or-later WITH LicenseRef-Modding-Exception

#pragma once

#include "smp/PhysicsHealth.h"

#include <algorithm>
#include <cstddef>
#include <optional>
#include <span>
#include <string>

namespace SmpPhysics
{
	/** @brief How many of a physics file's bones its claims kept: a file shared by several meshes keeps a different share on each. */
	struct ClaimTrim
	{
		std::size_t keptLeast{};
		std::size_t keptMost{};

		/** @brief Counts a claim that kept a_kept bones. */
		void Add(std::size_t a_kept) noexcept
		{
			keptLeast = keptLeast ? std::min(keptLeast, a_kept) : a_kept;
			keptMost = std::max(keptMost, a_kept);
		}
	};

	/** @brief What is known of one physics file an item names. */
	struct FileStats
	{
		std::string file;
		Health health{};
		bool missing{};  ///< Nothing exists at the path; the line says so instead of giving a health.
		bool read{};     ///< The file parsed; the counts below mean something.
		std::size_t bones{};
		std::size_t dynamic{};
		std::size_t joints{};
		std::optional<ClaimTrim> claim;
	};

	/** @brief One short line per file, for a hover tip: its name, health, size and what the last claim kept; at most a_maxFiles lines. */
	[[nodiscard]] std::string FormatItemStats(std::span<const FileStats> a_files, std::size_t a_maxFiles = 4);
}
