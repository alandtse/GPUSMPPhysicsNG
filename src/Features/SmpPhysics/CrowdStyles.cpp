#include "CrowdStyles.h"

#include "Utils/StringUtils.h"

#include <algorithm>
#include <cctype>
#include <unordered_set>

namespace SmpPhysics
{
	namespace
	{
		bool MentionsPhysics(const std::string& a_lowerModel)
		{
			return a_lowerModel.find("hdt") != std::string::npos || a_lowerModel.find("smp") != std::string::npos;
		}
	}

	std::vector<std::size_t> PickDistinctWigs(const std::vector<WigCandidate>& a_candidates, std::size_t a_count)
	{
		struct Ranked
		{
			std::size_t index;
			std::string name;
			bool physics;
		};
		std::vector<Ranked> ranked;
		std::unordered_set<std::string> seen;
		for (std::size_t index = 0; index < a_candidates.size(); ++index) {
			std::string name = Util::ToLowerAscii(a_candidates[index].name);
			if (name.empty() || !seen.insert(name).second)
				continue;
			ranked.push_back({ index, std::move(name), MentionsPhysics(Util::ToLowerAscii(a_candidates[index].model)) });
		}
		std::ranges::sort(ranked, [](const Ranked& a_left, const Ranked& a_right) {
			if (a_left.physics != a_right.physics)
				return a_left.physics;
			return a_left.name < a_right.name;
		});

		std::vector<std::size_t> picked;
		for (const Ranked& entry : ranked) {
			if (picked.size() >= a_count)
				break;
			picked.push_back(entry.index);
		}
		return picked;
	}

	const std::vector<HairColorChoice>& TestCrowdHairColors()
	{
		static const std::vector<HairColorChoice> colors{
			{ "Black", 0x000A0434 },
			{ "BlueBlack", 0x000A0435 },
			{ "Chestnut", 0x000A0430 },
			{ "Auburn", 0x000A0431 },
			{ "DarkBlond", 0x000A042C },
			{ "LightBlond", 0x000A042D },
			{ "BrightGrey", 0x000A0438 },
		};
		return colors;
	}

	std::optional<std::uint32_t> ResolveHairColor(std::string_view a_name)
	{
		const auto squash = [](std::string_view a_text) {
			std::string squashed;
			for (const char letter : a_text) {
				if (letter != ' ' && letter != '_')
					squashed.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(letter))));
			}
			return squashed;
		};
		const std::string wanted = squash(a_name);
		for (const HairColorChoice& color : TestCrowdHairColors()) {
			if (squash(color.name) == wanted)
				return color.formId;
		}
		return std::nullopt;
	}
}
