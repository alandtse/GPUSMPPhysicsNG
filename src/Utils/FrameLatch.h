#pragma once

#include <cstdint>
#include <optional>

namespace Util
{
	/** @brief Lets per-frame state advance at most once per rendered frame, however often the update is requested. */
	class FrameLatch
	{
	public:
		/** @brief True when the caller should advance for a_frame. */
		bool TryAdvance(std::uint32_t a_frame)
		{
			if (lastFrame == a_frame)
				return false;
			lastFrame = a_frame;
			return true;
		}

		/** @brief Forgets the latched frame so the next call advances. */
		void Reset() { lastFrame.reset(); }

	private:
		std::optional<std::uint32_t> lastFrame;
	};
}
