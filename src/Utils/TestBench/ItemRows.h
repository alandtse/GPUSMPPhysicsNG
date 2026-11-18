#pragma once

#include "Utils/ItemNames.h"
#include "Utils/StringUtils.h"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <string>
#include <vector>

namespace TestBench
{
	inline constexpr int kDefaultListLimit = 100;  ///< Entries a list query returns when it gives no limit.

	/** @brief The entry a command's args pick: args.name (part of an entry's name, first match) when given, else args.index; -1 for none. */
	[[nodiscard]] inline int ItemIndexFromArgs(const nlohmann::json& a_args, const std::vector<std::string>& a_names)
	{
		const std::string name = a_args.value("name", std::string{});
		return name.empty() ? a_args.value("index", -1) : Util::IndexOfNamePart(a_names, name);
	}

	/**
	 * @brief The rows of a devbench list of wearable items: index, form ID and name, for the names that contain args.filter, up to args.limit.
	 * @param a_names The names of a_forms, in the same order.
	 */
	[[nodiscard]] inline nlohmann::json ItemRows(const std::vector<std::string>& a_names, const std::vector<RE::TESObjectARMO*>& a_forms, const nlohmann::json& a_args)
	{
		const std::string filter = Util::ToLowerAscii(a_args.value("filter", std::string{}));
		const auto limit = static_cast<std::size_t>(std::max(a_args.value("limit", kDefaultListLimit), 1));
		nlohmann::json rows = nlohmann::json::array();
		for (std::size_t index = 0; index < a_names.size() && index < a_forms.size() && rows.size() < limit; ++index) {
			if (filter.empty() || Util::ToLowerAscii(a_names[index]).find(filter) != std::string::npos)
				rows.push_back({ { "index", index }, { "formId", a_forms[index]->GetFormID() }, { "name", a_names[index] } });
		}
		return rows;
	}
}
