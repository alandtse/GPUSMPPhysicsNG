#include "CrowdFilter.h"

#include <algorithm>
#include <array>
#include <cctype>

namespace TestBench
{
	namespace
	{
		constexpr std::array<std::string_view, 2> kElderlyRaceMarkers{ "elder", "oldpeople" };
		constexpr std::array<std::string_view, 2> kCloakSegmentPrefixes{ "cloak", "cape" };

		bool ContainsIgnoringCase(std::string_view a_text, std::string_view a_part)
		{
			return std::search(a_text.begin(), a_text.end(), a_part.begin(), a_part.end(), [](char a_left, char a_right) {
				return std::tolower(static_cast<unsigned char>(a_left)) == std::tolower(static_cast<unsigned char>(a_right));
			}) != a_text.end();
		}
	}

	bool IsElderlyRace(std::string_view a_raceEditorId)
	{
		return std::ranges::any_of(kElderlyRaceMarkers, [&](std::string_view a_marker) { return ContainsIgnoringCase(a_raceEditorId, a_marker); });
	}

	bool IsCloakModelPath(std::string_view a_modelPath)
	{
		while (!a_modelPath.empty()) {
			const std::size_t end = a_modelPath.find_first_of("/\\");
			const std::string_view segment = a_modelPath.substr(0, end);
			if (std::ranges::any_of(kCloakSegmentPrefixes, [&](std::string_view a_prefix) { return segment.size() >= a_prefix.size() && ContainsIgnoringCase(segment.substr(0, a_prefix.size()), a_prefix); }))
				return true;
			if (end == std::string_view::npos)
				break;
			a_modelPath.remove_prefix(end + 1);
		}
		return false;
	}

	bool PassesFilter(const CharacterTraits& a_traits, const CharacterFilter& a_filter)
	{
		if (a_filter.excludeChildren && a_traits.childRace)
			return false;
		return !(a_filter.excludeElderly && IsElderlyRace(a_traits.raceEditorId));
	}
}
