#include "Features/SmpPhysics/ClaimOverrides.h"

#include <algorithm>
#include <cctype>

namespace SmpPhysics
{
	namespace
	{
		std::string LowerAscii(std::string_view a_text)
		{
			std::string lowered(a_text);
			std::ranges::transform(lowered, lowered.begin(), [](unsigned char a_char) { return static_cast<char>(std::tolower(a_char)); });
			return lowered;
		}
	}

	std::string ClaimOverrides::NormalizeFile(std::string_view a_path)
	{
		std::string file = LowerAscii(a_path);
		std::ranges::replace(file, '\\', '/');
		return file;
	}

	std::string ClaimOverrides::NormalizeBase(std::string_view a_base)
	{
		return LowerAscii(a_base);
	}

	bool ClaimOverrides::Matches(const OverrideRule& a_rule, std::string_view a_base, std::string_view a_file)
	{
		return a_rule.file == NormalizeFile(a_file) && (a_rule.base.empty() || a_rule.base == NormalizeBase(a_base));
	}

	bool ClaimOverrides::Add(OverrideRule a_rule)
	{
		a_rule.file = NormalizeFile(a_rule.file);
		a_rule.base = NormalizeBase(a_rule.base);
		if (a_rule.file.empty() || rules.size() >= kMaxRules || std::ranges::find(rules, a_rule) != rules.end())
			return false;
		rules.push_back(std::move(a_rule));
		return true;
	}

	bool ClaimOverrides::Remove(std::size_t a_index)
	{
		if (a_index >= rules.size())
			return false;
		rules.erase(rules.begin() + static_cast<std::ptrdiff_t>(a_index));
		return true;
	}

	bool ClaimOverrides::Blocks(std::string_view a_base, std::string_view a_file) const
	{
		return std::ranges::any_of(rules, [&](const OverrideRule& a_rule) { return Matches(a_rule, a_base, a_file); });
	}

	nlohmann::json ClaimOverrides::ToJson() const
	{
		nlohmann::json rulesJson = nlohmann::json::array();
		for (const OverrideRule& rule : rules) {
			nlohmann::json entry = { { "file", rule.file } };
			if (!rule.base.empty())
				entry["base"] = rule.base;
			rulesJson.push_back(std::move(entry));
		}
		return { { "keepWithHdtSmp64", std::move(rulesJson) } };
	}

	void ClaimOverrides::FromJson(const nlohmann::json& a_json)
	{
		rules.clear();
		if (!a_json.is_object())
			return;
		const auto found = a_json.find("keepWithHdtSmp64");
		if (found == a_json.end() || !found->is_array())
			return;
		for (const nlohmann::json& entry : *found) {
			if (!entry.is_object() || !entry.contains("file") || !entry["file"].is_string())
				continue;
			OverrideRule rule;
			rule.file = entry["file"].get<std::string>();
			if (entry.contains("base") && entry["base"].is_string())
				rule.base = entry["base"].get<std::string>();
			Add(std::move(rule));
		}
	}
}
