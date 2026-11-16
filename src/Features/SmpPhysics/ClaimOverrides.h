#pragma once

#include <nlohmann/json.hpp>

#include <cstddef>
#include <string>
#include <string_view>
#include <vector>

namespace SmpPhysics
{
	/** @brief Keeps one physics file with hdtSMP64: for every character when base is empty, else only for the character type base names. */
	struct OverrideRule
	{
		std::string base;  ///< Base form as "plugin.esp|0xLOCALID", lower case; empty for every character.
		std::string file;  ///< Physics file path, lower case with forward slashes.

		bool operator==(const OverrideRule&) const = default;
	};

	/** @brief Rules the user saved for physics files the GPU must not claim; they survive saves because they name base forms and files, never a reference. */
	class ClaimOverrides
	{
	public:
		/** @brief Lower-cases a path and uses forward slashes so the same file always compares equal. */
		[[nodiscard]] static std::string NormalizeFile(std::string_view a_path);
		[[nodiscard]] static std::string NormalizeBase(std::string_view a_base);
		[[nodiscard]] static bool Matches(const OverrideRule& a_rule, std::string_view a_base, std::string_view a_file);

		/** @return false when the rule has no file or is already present. */
		bool Add(OverrideRule a_rule);
		bool Remove(std::size_t a_index);
		[[nodiscard]] bool Blocks(std::string_view a_base, std::string_view a_file) const;
		[[nodiscard]] const std::vector<OverrideRule>& Rules() const { return rules; }

		[[nodiscard]] nlohmann::json ToJson() const;
		/** @brief Replaces the rules with the well-formed entries of a_json, skipping the rest. */
		void FromJson(const nlohmann::json& a_json);

		/** @brief The most rules kept, so a damaged file cannot grow the list without bound. */
		static constexpr std::size_t kMaxRules = 512;

	private:
		std::vector<OverrideRule> rules;
	};
}
