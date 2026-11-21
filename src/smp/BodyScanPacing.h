#pragma once

#include <algorithm>
#include <cstdint>
#include <mutex>
#include <unordered_map>

namespace SmpPhysics
{
	inline constexpr double kBodyScanActiveSeconds = 1.0;  ///< Between scans while a claim settles or an event says the actor's meshes are changing.
	inline constexpr double kBodyScanIdleSeconds = 15.0;   ///< Between scans of an actor nothing has touched: the net for changes that raise no event.
	inline constexpr double kBodyScanWindowSeconds = 6.0;  ///< How long an event keeps an actor on the active interval, since its meshes attach after the event.

	/** @brief Seconds to wait between body scans of one claim. */
	[[nodiscard]] constexpr double BodyScanInterval(bool a_settled, bool a_eventWindowOpen) noexcept
	{
		return !a_settled || a_eventWindowOpen ? kBodyScanActiveSeconds : kBodyScanIdleSeconds;
	}

	/**
	 * @brief Which actors a game event has just touched, so their body scans run soon instead of waiting out the idle interval.
	 * @details Events arrive on whatever thread the game raises them from, so the table is locked.
	 */
	class BodyScanWindows
	{
	public:
		void Note(std::uint32_t a_actor)
		{
			const std::scoped_lock lock(mutex);
			windows[a_actor] = kBodyScanWindowSeconds;
		}

		/** @brief Whether a_actor's window is open; each call spends a_elapsed seconds of it. */
		[[nodiscard]] bool Open(std::uint32_t a_actor, double a_elapsed)
		{
			const std::scoped_lock lock(mutex);
			const auto found = windows.find(a_actor);
			if (found == windows.end())
				return false;
			found->second -= a_elapsed;
			if (found->second <= 0.0) {
				windows.erase(found);
				return false;
			}
			return true;
		}

		void Clear()
		{
			const std::scoped_lock lock(mutex);
			windows.clear();
		}

	private:
		std::mutex mutex;
		std::unordered_map<std::uint32_t, double> windows;
	};
}
