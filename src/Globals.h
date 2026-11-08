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

#ifdef SMP_USE_SKSE_LOG
#	include "pch.h"
#	include <REX/W32/Bridge.h>

#	include <atomic>

/** @brief Plugin-wide state the ported feature reads; DrawSettings-style developer switches are off in this plugin. */
struct State
{
	std::atomic<std::uint32_t> frameCountAtomic{ 0 };
	[[nodiscard]] bool IsDeveloperMode() const noexcept { return developerMode; }
	bool developerMode{ false };
};

namespace globals
{
	inline State defaultState;
	inline State* state = &defaultState;

	namespace game
	{
		inline bool isVR = false;
		inline float* deltaTime = nullptr;
		inline RE::PlayerCharacter* player = nullptr;
		inline RE::PlayerCamera* playerCamera = nullptr;
		inline RE::BSGraphics::RendererShadowState* shadowState = nullptr;
		inline RE::BSGraphics::Renderer* renderer = nullptr;
		inline REL::Relocation<ID3D11Buffer**> perFrame;  ///< The engine's per-frame constant buffer slot.
	}
}

namespace Util
{
	using REX::W32::AsReal;
	using REX::W32::AsW32;
	using REX::W32::CastTo;
}
#endif
