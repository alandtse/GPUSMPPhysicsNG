#pragma once

#include "smp/DebugShapes.h"

#include <cstdint>
#include <vector>

namespace SmpPhysics
{
	/**
	 * @brief Receives the developer overlay's lines. The standalone plugin has no deferred G-buffer to draw them into,
	 *        so the lines are counted and dropped; the interface stays so the feature code that builds them is unchanged.
	 */
	class DebugLines
	{
	public:
		void SetSegments(std::vector<DebugSegment> a_segments) { stats.requested = static_cast<std::uint32_t>(a_segments.size()); }
		void Render() {}
		void InvalidateShaders() {}

		struct Stats
		{
			std::uint32_t requested{};
			std::uint32_t drawn{};
			std::uint64_t stateFailures{};
			bool failed{};
		};
		[[nodiscard]] Stats GetStats() const { return stats; }

	private:
		Stats stats;
	};
}
