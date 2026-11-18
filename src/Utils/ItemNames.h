#pragma once

#include "Utils/StringUtils.h"

#include <algorithm>
#include <cstdint>
#include <format>
#include <span>
#include <string>
#include <unordered_map>
#include <vector>

namespace Util
{
	/** @brief An item as a list shows it: its name and the form ID that tells apart items sharing a name. */
	struct NamedItem
	{
		std::string name;
		std::uint32_t formId{};
	};

	/** @brief The names of a_items in order, with " [formId]" added to every name more than one item has. */
	[[nodiscard]] inline std::vector<std::string> UniqueNames(std::span<const NamedItem> a_items)
	{
		std::unordered_map<std::string, std::size_t> uses;
		for (const NamedItem& item : a_items)
			++uses[item.name];
		std::vector<std::string> names;
		names.reserve(a_items.size());
		for (const NamedItem& item : a_items)
			names.push_back(uses[item.name] > 1 ? std::format("{} [{:08X}]", item.name, item.formId) : item.name);
		return names;
	}

	/** @brief The index of the first of a_names that contains a_part, ignoring case; -1 for none or an empty a_part. */
	[[nodiscard]] inline int IndexOfNamePart(std::span<const std::string> a_names, const std::string& a_part)
	{
		if (a_part.empty())
			return -1;
		const std::string part = ToLowerAscii(a_part);
		const auto match = std::ranges::find_if(a_names, [&part](const std::string& a_name) { return ToLowerAscii(a_name).find(part) != std::string::npos; });
		return match != a_names.end() ? static_cast<int>(match - a_names.begin()) : -1;
	}
}
