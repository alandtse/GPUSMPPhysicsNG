#pragma once

#include <algorithm>
#include <cstdint>
#include <unordered_map>

namespace SmpPhysics
{
	inline constexpr double kRefitBackoffBaseSeconds = 2.0;
	inline constexpr double kRefitBackoffMaxSeconds = 30.0;
	inline constexpr double kRefitStreakWindowSeconds = 60.0;
	inline constexpr int kRefitWarnStreak = 2;
	inline constexpr int kMaxSettleChecks = 5;

	/**
	 * @brief Spaces out the soft reloads one actor asks hdtSMP64 for, so a refit that keeps being asked for cannot stall every second.
	 * @details A reload replaces the actor's claims, so this lives in the feature, keyed by actor. The first refit is always
	 *          admitted; each further one inside the streak window waits twice as long as the last, up to a cap.
	 */
	class RefitGuard
	{
	public:
		struct Decision
		{
			bool allowed{};
			double retryIn{};  ///< Seconds until the next refit is admitted; zero when this one was.
			int streak{};      ///< Refits admitted for the actor in a row, counting this one from zero.
		};

		[[nodiscard]] Decision Admit(std::uint32_t a_actor, double a_now)
		{
			Entry& entry = entries[a_actor];
			if (entry.any && a_now < entry.nextAllowed)
				return { false, entry.nextAllowed - a_now, entry.streak };
			entry.streak = entry.any && a_now - entry.lastAdmitted <= kRefitStreakWindowSeconds ? entry.streak + 1 : 0;
			entry.any = true;
			entry.lastAdmitted = a_now;
			double delay = kRefitBackoffBaseSeconds;
			for (int step = 0; step < entry.streak && delay < kRefitBackoffMaxSeconds; ++step)
				delay *= 2.0;
			entry.nextAllowed = a_now + std::min(delay, kRefitBackoffMaxSeconds);
			return { true, 0.0, entry.streak };
		}

		void Clear() { entries.clear(); }

	private:
		struct Entry
		{
			double lastAdmitted{};
			double nextAllowed{};
			int streak{};
			bool any{};
		};

		std::unordered_map<std::uint32_t, Entry> entries;
	};

	enum class BaselineStep
	{
		Compare,  ///< The baseline is settled: a different signature is a change.
		Wait,     ///< The claimed model is not under the actor yet, so a scan cannot see what the claim will wear.
		Adopt     ///< Take this scan as the baseline without asking for a refit.
	};

	/** @brief What a claim's periodic scan does with its signature: it is compared only once the claim's own model is part of the scanned graph. */
	[[nodiscard]] inline BaselineStep NextBaselineStep(bool a_settled, bool a_modelAttached, int a_checksWaited) noexcept
	{
		if (a_settled)
			return BaselineStep::Compare;
		return a_modelAttached || a_checksWaited >= kMaxSettleChecks ? BaselineStep::Adopt : BaselineStep::Wait;
	}
}
