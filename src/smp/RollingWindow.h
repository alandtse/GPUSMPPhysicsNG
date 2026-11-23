#pragma once

#include <algorithm>
#include <array>
#include <cstddef>

namespace SmpPhysics
{
	/** @brief The mean and the largest of the last N samples added; both are zero until one is. */
	template <std::size_t N>
	class RollingWindow
	{
	public:
		void Add(float a_value) noexcept
		{
			samples[next] = a_value;
			next = (next + 1u) % N;
			count = std::min(count + 1u, N);
		}

		[[nodiscard]] std::size_t Count() const noexcept { return count; }

		[[nodiscard]] float Average() const noexcept
		{
			float sum = 0.0f;
			for (std::size_t index = 0; index < count; ++index)
				sum += samples[index];
			return count ? sum / static_cast<float>(count) : 0.0f;
		}

		[[nodiscard]] float Max() const noexcept
		{
			return count ? *std::max_element(samples.begin(), samples.begin() + static_cast<std::ptrdiff_t>(count)) : 0.0f;
		}

	private:
		std::array<float, N> samples{};
		std::size_t next{};
		std::size_t count{};
	};
}
