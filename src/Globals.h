#pragma once

#ifndef NOMINMAX
#	define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#	define WIN32_LEAN_AND_MEAN
#endif

#include <d3d11.h>

/** @brief The D3D11 device and immediate context the solver runs on; the plugin sets them from the game's renderer, tests from their own device. */
namespace globals::d3d
{
	inline ID3D11Device* device = nullptr;
	inline ID3D11DeviceContext* context = nullptr;
}
