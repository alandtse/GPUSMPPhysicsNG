#pragma once

#include <memory>
#include <string_view>

#ifdef TRACY_ENABLE
#	include <tracy/Tracy.hpp>
#endif

/**
 * @brief Tracy CPU and GPU zones for a render pass; compiles to an empty type unless TRACY_ENABLE is defined.
 *
 * The GPU zone comes from D3D11 timestamp queries on the game's immediate context, so it shows each pass's GPU
 * duration but not queue waits; those need an ETW trace.
 */
namespace TracyGpu
{
#ifdef TRACY_ENABLE
	class Zone
	{
	public:
		explicit Zone(std::string_view a_name);
		~Zone();
		Zone(const Zone&) = delete;
		Zone& operator=(const Zone&) = delete;

	private:
		struct Impl;
		std::unique_ptr<Impl> impl;
	};

	/** @brief Marks the frame boundary and collects finished GPU timestamps; call once per frame on the render thread. */
	void NextFrame();
#else
	class Zone
	{
	public:
		explicit Zone(std::string_view) {}
	};

	inline void NextFrame() {}
#endif
}
