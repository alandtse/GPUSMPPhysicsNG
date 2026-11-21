#include "TestCrowdLayout.h"

#include "Utils/StringUtils.h"

#include <algorithm>

namespace TestBench
{
	TestCrowdSlot TestCrowdSlotFor(std::size_t a_index) noexcept
	{
		const std::size_t row = a_index / kTestCrowdColumns;
		const std::size_t column = a_index % kTestCrowdColumns;
		return { kTestCrowdFirstRow + static_cast<float>(row) * kTestCrowdSpacing,
			(static_cast<float>(column) - static_cast<float>(kTestCrowdColumns - 1) * 0.5f) * kTestCrowdSpacing };
	}

	bool IsIdleEndEvent(std::string_view a_tag) noexcept
	{
		return a_tag == "IdleStop";
	}

	std::optional<std::size_t> PickDefaultIdle(const std::vector<std::string>& a_editorIds)
	{
		std::optional<std::size_t> anyCicero;
		for (std::size_t index = 0; index < a_editorIds.size(); ++index) {
			const std::string lower = Util::ToLowerAscii(a_editorIds[index]);
			if (lower.find("cicero") == std::string::npos)
				continue;
			if (lower.find("dance") != std::string::npos)
				return index;
			if (!anyCicero)
				anyCicero = index;
		}
		return anyCicero;
	}

	std::vector<std::uint32_t> HousecarlPool(std::uint32_t a_rayya, const std::vector<std::uint32_t>& a_followers)
	{
		std::vector<std::uint32_t> pool(kSkyrimHousecarlBases.begin(), kSkyrimHousecarlBases.end());
		if (a_rayya != 0)
			pool.push_back(a_rayya);
		for (const std::uint32_t follower : a_followers) {
			if (std::ranges::find(pool, follower) == pool.end())
				pool.push_back(follower);
		}
		return pool;
	}

	std::uint32_t BaseForSlot(std::size_t a_index, const std::vector<std::uint32_t>& a_bases, std::uint32_t a_fallback) noexcept
	{
		return a_bases.empty() ? a_fallback : a_bases[a_index % a_bases.size()];
	}
}
