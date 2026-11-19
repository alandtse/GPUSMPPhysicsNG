#pragma once

#include <algorithm>
#include <cctype>
#include <numeric>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace SettingsUi
{
	/** @brief Whether a list row shows under the filter text (already lower-case) and the Show mode: 0 all, 1 with physics, 2 needing attention. */
	[[nodiscard]] inline bool RowVisible(std::string_view a_lowerName, std::string_view a_lowerFilter, int a_showMode, int a_state)
	{
		if (!a_lowerFilter.empty() && a_lowerName.find(a_lowerFilter) == std::string_view::npos)
			return false;
		return !((a_showMode == 1 && a_state == 0) || (a_showMode == 2 && a_state < 2));
	}

	/**
	 * @brief The index a Previous or Next button moves to, walking only the rows RowVisible shows and wrapping at the ends.
	 * @param a_lowerNames The lower-case row names.
	 * @param a_states Each row's state, shorter than a_lowerNames for rows with none.
	 * @param a_step 1 for Next, -1 for Previous.
	 * @param a_order The row indexes in the order the list shows them, or empty for the natural order; the walk follows it.
	 * @return The new index, the first or last visible row when a_current is not visible, or a_current when no row is visible.
	 */
	[[nodiscard]] inline int StepVisible(std::span<const std::string> a_lowerNames, std::string_view a_lowerFilter, int a_showMode, std::span<const int> a_states, int a_current, int a_step,
		std::span<const int> a_order = {})
	{
		const int count = static_cast<int>(a_lowerNames.size());
		const auto visible = [&](int a_index) {
			const std::size_t slot = static_cast<std::size_t>(a_index);
			return RowVisible(a_lowerNames[slot], a_lowerFilter, a_showMode, slot < a_states.size() ? a_states[slot] : 0);
		};
		const bool ordered = static_cast<int>(a_order.size()) == count;
		const auto indexAt = [&](int a_position) { return ordered ? a_order[static_cast<std::size_t>(a_position)] : a_position; };
		const int direction = a_step < 0 ? -1 : 1;
		const bool onRow = a_current >= 0 && a_current < count;
		const int currentPosition = onRow && ordered ? static_cast<int>(std::ranges::find(a_order, a_current) - a_order.begin()) : a_current;
		const int start = onRow ? currentPosition : (direction > 0 ? -1 : count);
		for (int walked = 1; walked <= count; ++walked) {
			const int index = indexAt(((start + direction * walked) % count + count) % count);
			if (visible(index))
				return index;
		}
		return a_current;
	}

	/** @brief The orders every list can show its rows in besides the ones a row adds. */
	enum class Sort
	{
		Natural,
		NameAscending,
		NameDescending,
		WorstFirst,       ///< Rows with a broken physics file first, then repaired, then working, then those with none.
		HasPhysicsFirst,  ///< Rows with physics before those without.
		MostBones,        ///< Rows with the most physics bones first, those with none last.
		FewestBones       ///< Rows with the fewest physics bones first, those with none last.
	};

	/**
	 * @brief The row indexes in the order a_sort shows them, empty for the natural order; ties keep the natural order.
	 * @param a_states Each row's state as RowVisible takes it, shorter than a_names for rows with none.
	 * @param a_bones Each row's physics bone count, shorter than a_names for rows with none; what the bone sorts order by.
	 */
	[[nodiscard]] inline std::vector<int> SortedOrder(Sort a_sort, std::span<const std::string> a_names, std::span<const int> a_states, std::span<const int> a_bones = {})
	{
		if (a_sort == Sort::Natural)
			return {};
		const auto state = [&](int a_index) { return static_cast<std::size_t>(a_index) < a_states.size() ? a_states[static_cast<std::size_t>(a_index)] : 0; };
		const auto bones = [&](int a_index) { return static_cast<std::size_t>(a_index) < a_bones.size() ? a_bones[static_cast<std::size_t>(a_index)] : 0; };
		const auto fewer = [&](int a_left, int a_right) {
			const int left = bones(a_left);
			const int right = bones(a_right);
			return left != 0 && (right == 0 || left < right);
		};
		const auto lessName = [&](int a_left, int a_right) {
			return std::ranges::lexicographical_compare(a_names[static_cast<std::size_t>(a_left)], a_names[static_cast<std::size_t>(a_right)],
				[](unsigned char a_l, unsigned char a_r) { return std::tolower(a_l) < std::tolower(a_r); });
		};
		std::vector<int> order(a_names.size());
		std::iota(order.begin(), order.end(), 0);
		std::ranges::stable_sort(order, [&](int a_left, int a_right) {
			switch (a_sort) {
			case Sort::NameAscending:
				return lessName(a_left, a_right);
			case Sort::NameDescending:
				return lessName(a_right, a_left);
			case Sort::WorstFirst:
				return state(a_left) > state(a_right);
			case Sort::MostBones:
				return bones(a_left) > bones(a_right);
			case Sort::FewestBones:
				return fewer(a_left, a_right);
			default:
				return (state(a_left) != 0) > (state(a_right) != 0);
			}
		});
		return order;
	}

	/**
	 * @brief Whether a list should scroll to its selected row this frame.
	 * @param a_picked The row the user just clicked, or -1; the list already shows it under the cursor, so it must not move.
	 */
	[[nodiscard]] constexpr bool ShouldScroll(int a_selected, int a_lastSelected, int a_picked) noexcept
	{
		return a_selected != a_lastSelected && a_selected != a_picked;
	}
}
