#pragma once

#include "Globals.h"
#include "Logger.h"

#include <cstdint>
#include <d3d11.h>
#include <d3dcompiler.h>
#include <format>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>
#include <winrt/base.h>

namespace DX
{
	/** @brief Throws std::runtime_error with the HRESULT when a D3D call fails; the solvers catch it and fall back. */
	inline void ThrowIfFailed(HRESULT a_result)
	{
		if (FAILED(a_result))
			throw std::runtime_error(std::format("D3D call failed with HRESULT 0x{:08X}", static_cast<std::uint32_t>(a_result)));
	}
}

namespace Util
{
	/** @brief Names a D3D11 resource for RenderDoc/PIX with a printf-style format. */
	void SetResourceName(ID3D11DeviceChild* Resource, const char* Format, ...);

	std::string WStringToString(const wchar_t* a_text);

	/** @brief Directory that holds SmpPhysics/*.hlsl and Common/*.hlsli; the plugin sets it from its Data folder, tests from the repo. */
	void SetShaderRoot(const std::wstring& a_root);
	const std::wstring& GetShaderRoot();

	/** @brief Compiles FilePath (relative to the shader root) and returns the blob, or null after logging the error. */
	winrt::com_ptr<ID3DBlob> CompileShaderBlob(const wchar_t* FilePath, const std::vector<std::pair<const char*, const char*>>& Defines, const char* ProgramType, const char* Program);

	/** @brief Compiles FilePath into a D3D11 shader object of the kind the profile names (cs_5_0, vs_5_0, ps_5_0), or null. */
	ID3D11DeviceChild* CompileShader(const wchar_t* FilePath, const std::vector<std::pair<const char*, const char*>>& Defines, const char* ProgramType, const char* Program);

	/** @brief Writes a_records into a default-usage structured buffer's slots from a_first and leaves every other slot as it is. */
	template <typename T>
	inline void UpdateBufferSlots(ID3D11Buffer* a_buffer, std::uint32_t a_first, const std::vector<T>& a_records)
	{
		if (!a_buffer || a_records.empty())
			return;
		const UINT begin = a_first * static_cast<UINT>(sizeof(T));
		const D3D11_BOX box{ begin, 0, 0, begin + static_cast<UINT>(a_records.size() * sizeof(T)), 1, 1 };
		globals::d3d::context->UpdateSubresource(a_buffer, 0, &box, a_records.data(), 0, 0);
	}
}
