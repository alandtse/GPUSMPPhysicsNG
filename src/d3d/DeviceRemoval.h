#pragma once

#include <cstdint>
#include <string_view>

namespace SmpPhysics
{
	/** @brief The name of a GetDeviceRemovedReason result, so the log says why the device went away. */
	[[nodiscard]] constexpr std::string_view DeviceRemovedName(std::int32_t a_reason) noexcept
	{
		switch (static_cast<std::uint32_t>(a_reason)) {
		case 0x887A0005u:
			return "DXGI_ERROR_DEVICE_REMOVED";
		case 0x887A0006u:
			return "DXGI_ERROR_DEVICE_HUNG";
		case 0x887A0007u:
			return "DXGI_ERROR_DEVICE_RESET";
		case 0x887A0020u:
			return "DXGI_ERROR_DRIVER_INTERNAL_ERROR";
		case 0x887A0001u:
			return "DXGI_ERROR_INVALID_CALL";
		default:
			return "an unknown reason";
		}
	}
}
