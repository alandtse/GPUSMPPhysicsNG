#pragma once

#include <string_view>

namespace TestBench
{
	/** @brief What the crowd filter needs to know about a character's base. */
	struct CharacterTraits
	{
		bool childRace = false;         ///< The base's race is a child race.
		std::string_view raceEditorId;  ///< Editor ID of the base's race.
	};

	/** @brief Which kinds of character a random spawn leaves out. */
	struct CharacterFilter
	{
		bool excludeChildren = true;
		bool excludeElderly = true;
	};

	/** @brief Whether a race editor ID names an elderly race ("ElderRace" and "OldPeopleRace" style names), ignoring case. */
	[[nodiscard]] bool IsElderlyRace(std::string_view a_raceEditorId);

	/**
	 * @brief Whether a model path belongs to a cloak or cape: a folder or file name starting with "cloak" or "cape", ignoring case.
	 * @details Stormcloak armor and "landscape" do not count, since the word must start a path segment and "stormcloaks" does not.
	 */
	[[nodiscard]] bool IsCloakModelPath(std::string_view a_modelPath);

	/** @brief Whether a character passes the filter: not a child when children are excluded, not elderly when the elderly are. */
	[[nodiscard]] bool PassesFilter(const CharacterTraits& a_traits, const CharacterFilter& a_filter);
}
