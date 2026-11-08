// SPDX-License-Identifier: GPL-3.0-or-later WITH LicenseRef-Modding-Exception

#pragma once

#define WIN32_LEAN_AND_MEAN
#define NOMINMAX

#include <RE/Skyrim.h>
#include <REX/REX.h>
#include <SKSE/SKSE.h>

#include <Windows.h>
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <d3d11.h>
#include <filesystem>
#include <format>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>
#include <winrt/base.h>
#include <wrl/client.h>

#include <SimpleMath.h>
#include <Tracy/Tracy.hpp>
#include <nlohmann/json.hpp>

namespace stl
{
	using namespace SKSE::stl;

	template <class T, std::size_t Size = 5>
	void write_thunk_call(std::uintptr_t a_src)
	{
		auto& trampoline = SKSE::GetTrampoline();
		if constexpr (Size == 6) {
			T::func = *(std::uintptr_t*)trampoline.write_call<6>(a_src, T::thunk);
		} else {
			T::func = trampoline.write_call<Size>(a_src, T::thunk);
		}
	}

	template <class F, std::size_t index, class T>
	void write_vfunc()
	{
		REL::Relocation<std::uintptr_t> vtbl{ F::VTABLE[index] };
		T::func = vtbl.write_vfunc(T::size, T::thunk);
	}

	template <std::size_t idx, class T>
	void write_vfunc(REL::VariantID id)
	{
		REL::Relocation<std::uintptr_t> vtbl{ id };
		T::func = vtbl.write_vfunc(idx, T::thunk);
	}

	template <class F, class T>
	void write_vfunc()
	{
		write_vfunc<F, 0, T>();
	}
}

namespace logger = SKSE::log;
using namespace std::literals;

using json = nlohmann::json;
using float2 = DirectX::SimpleMath::Vector2;
using float3 = DirectX::SimpleMath::Vector3;
using float4 = DirectX::SimpleMath::Vector4;
using float4x4 = DirectX::SimpleMath::Matrix;
using uint = std::uint32_t;
