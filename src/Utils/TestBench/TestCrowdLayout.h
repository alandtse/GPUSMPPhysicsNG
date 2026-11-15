#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace TestBench
{
	/** @brief Characters per row of the test crowd. */
	inline constexpr std::size_t kTestCrowdColumns = 5;
	/** @brief Distance, in game units, from the player to the first row. */
	inline constexpr float kTestCrowdFirstRow = 220.0f;
	/** @brief Distance between neighbouring characters, in game units. */
	inline constexpr float kTestCrowdSpacing = 80.0f;
	/** @brief Most characters one spawn may ask for. */
	inline constexpr std::size_t kMaxTestCrowd = 60;
	/** @brief How many characters a spawn asks for when the request gives no count. */
	inline constexpr std::size_t kDefaultTestCrowd = 10;

	/** @brief Where a test character stands relative to the player: ahead along the facing, and to the right of it. */
	struct TestCrowdSlot
	{
		float forward = 0.0f;
		float right = 0.0f;
	};

	/** @brief The slot of the a_index-th character, in rows of kTestCrowdColumns centred on the player's line of sight. */
	[[nodiscard]] TestCrowdSlot TestCrowdSlotFor(std::size_t a_index) noexcept;

	/**
	 * @brief The idle test characters play unless one is chosen: a Cicero dance, else any Cicero idle.
	 * @param a_editorIds Editor IDs of the idles on offer, matched ignoring case.
	 * @return The index of the first Cicero dance, else of the first Cicero idle, else none.
	 */
	[[nodiscard]] std::optional<std::size_t> PickDefaultIdle(const std::vector<std::string>& a_editorIds);

	/** @brief Whether an animation graph event tag means the idle a character was playing has finished. */
	[[nodiscard]] bool IsIdleEndEvent(std::string_view a_tag) noexcept;

	/** @return The base the a_index-th character is made from: bases cycle in order, and a_fallback stands in when none are given. */
	[[nodiscard]] std::uint32_t BaseForSlot(std::size_t a_index, const std::vector<std::uint32_t>& a_bases, std::uint32_t a_fallback) noexcept;
}
