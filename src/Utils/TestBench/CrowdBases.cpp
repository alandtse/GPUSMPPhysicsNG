#include "CrowdBases.h"

#include <algorithm>
#include <random>

namespace TestBench
{
	namespace
	{
		constexpr std::uint32_t kLydia = 0x000A2C8E;
		constexpr std::uint32_t kLisette = 0x00013297;
		constexpr std::uint32_t kHulda = 0x00013BA3;
		constexpr std::uint32_t kYsolda = 0x00013BAB;
		constexpr std::uint32_t kIrileth = 0x00013BB8;
		constexpr std::uint32_t kCarlottaValentia = 0x00013B99;
		constexpr std::uint32_t kAdrianneAvenicci = 0x00013BB9;
		constexpr std::uint32_t kAlfhildBattleBorn = 0x00013BB0;
		constexpr std::uint32_t kFraliaGrayMane = 0x00013B9C;
		constexpr std::uint32_t kOlfinaGrayMane = 0x00013B9B;
		constexpr std::uint32_t kArcadia = 0x00013BA4;
	}

	const std::vector<BasePreset>& BasePresets()
	{
		static const std::vector<BaseChoice> lydiaOnly{ { kLydia, "Lydia" } };
		static const std::vector<BaseChoice> mixedWomen{
			{ kLydia, "Lydia" },
			{ kLisette, "Lisette" },
			{ kHulda, "Hulda" },
			{ kYsolda, "Ysolda" },
			{ kIrileth, "Irileth" },
			{ kCarlottaValentia, "Carlotta Valentia" },
			{ kAdrianneAvenicci, "Adrianne Avenicci" },
			{ kAlfhildBattleBorn, "Alfhild Battle-Born" },
			{ kFraliaGrayMane, "Fralia Gray-Mane" },
			{ kOlfinaGrayMane, "Olfina Gray-Mane" },
			{ kArcadia, "Arcadia" },
		};
		static const std::vector<BasePreset> presets{
			{ "Lydia only", lydiaOnly },
			{ "Mixed women", mixedWomen },
		};
		return presets;
	}

	std::vector<std::uint32_t> PickBases(const BasePreset& a_preset, std::size_t a_count, bool a_randomOrder, std::uint32_t a_seed)
	{
		std::vector<std::uint32_t> ids;
		ids.reserve(a_preset.bases.size());
		for (const BaseChoice& base : a_preset.bases)
			ids.push_back(base.formId);
		return PickFromIds(ids, a_count, a_randomOrder, a_seed);
	}

	std::vector<std::uint32_t> PickFromIds(const std::vector<std::uint32_t>& a_ids, std::size_t a_count, bool a_randomOrder, std::uint32_t a_seed)
	{
		std::vector<std::uint32_t> picked;
		if (a_ids.empty())
			return picked;
		picked.reserve(a_count);
		const std::size_t size = a_ids.size();
		if (!a_randomOrder) {
			for (std::size_t index = 0; index < a_count; ++index)
				picked.push_back(a_ids[index % size]);
			return picked;
		}
		std::mt19937 generator{ a_seed };
		std::vector<std::uint32_t> deck;
		while (picked.size() < a_count) {
			deck = a_ids;
			std::shuffle(deck.begin(), deck.end(), generator);
			const std::size_t take = std::min(size, a_count - picked.size());
			picked.insert(picked.end(), deck.begin(), deck.begin() + static_cast<std::ptrdiff_t>(take));
		}
		return picked;
	}
}
