#pragma once

#include <algorithm>
#include <cstdint>
#include <unordered_map>
#include <vector>

namespace TestBench
{
	/**
	 * @brief Which items the test tool put into an owner's inventory, so it removes those and never one the owner already had.
	 * @details An item is noted only when it was not already there, so taking it back leaves the owner's own copies alone.
	 */
	class AddedItems
	{
	public:
		void Note(std::uint32_t a_owner, std::uint32_t a_item)
		{
			std::vector<std::uint32_t>& items = added[a_owner];
			if (std::ranges::find(items, a_item) == items.end())
				items.push_back(a_item);
		}

		/** @return Whether a_item was noted for a_owner; it no longer is. */
		[[nodiscard]] bool Take(std::uint32_t a_owner, std::uint32_t a_item)
		{
			const auto owner = added.find(a_owner);
			if (owner == added.end())
				return false;
			const auto found = std::ranges::find(owner->second, a_item);
			if (found == owner->second.end())
				return false;
			owner->second.erase(found);
			return true;
		}

		/** @return Every item noted for a_owner; none are noted afterwards. */
		[[nodiscard]] std::vector<std::uint32_t> TakeAll(std::uint32_t a_owner)
		{
			const auto owner = added.find(a_owner);
			if (owner == added.end())
				return {};
			std::vector<std::uint32_t> items = std::move(owner->second);
			added.erase(owner);
			return items;
		}

	private:
		std::unordered_map<std::uint32_t, std::vector<std::uint32_t>> added;
	};
}
