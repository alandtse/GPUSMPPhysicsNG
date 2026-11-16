// Unit tests for the device-removed reason names the GPU solver logs.

#include "d3d/DeviceRemoval.h"

#include <catch2/catch_test_macros.hpp>

using SmpPhysics::DeviceRemovedName;

TEST_CASE("Device-removed reasons are named", "[smpphysics][device]")
{
	REQUIRE(DeviceRemovedName(static_cast<std::int32_t>(0x887A0005u)) == "DXGI_ERROR_DEVICE_REMOVED");
	REQUIRE(DeviceRemovedName(static_cast<std::int32_t>(0x887A0006u)) == "DXGI_ERROR_DEVICE_HUNG");
	REQUIRE(DeviceRemovedName(static_cast<std::int32_t>(0x887A0007u)) == "DXGI_ERROR_DEVICE_RESET");
	REQUIRE(DeviceRemovedName(static_cast<std::int32_t>(0x887A0020u)) == "DXGI_ERROR_DRIVER_INTERNAL_ERROR");
	REQUIRE(DeviceRemovedName(static_cast<std::int32_t>(0x887A0001u)) == "DXGI_ERROR_INVALID_CALL");
	REQUIRE(DeviceRemovedName(1) == "an unknown reason");
}
