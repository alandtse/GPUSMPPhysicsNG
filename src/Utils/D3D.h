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
	/** @brief Folder where compiled shader blobs are kept between runs; empty (the default) compiles every time. */
	void SetShaderCacheDir(const std::wstring& a_dir);
	/** @brief Forgets which shaders failed to compile, so the next request compiles them again. */
	void ClearShaderFailures();

	/** @brief Compiles FilePath (relative to the shader root) and returns the blob, or null after logging the error. */
	winrt::com_ptr<ID3DBlob> CompileShaderBlob(const wchar_t* FilePath, const std::vector<std::pair<const char*, const char*>>& Defines, const char* ProgramType, const char* Program);

	/** @brief One compute shader the plugin will need, named as its call site names it so the cache key matches. */
	struct ShaderJob
	{
		const wchar_t* path;
		const char* target;
		const char* entry;
	};

	struct ShaderPrewarmStatus
	{
		std::size_t done = 0;
		std::size_t total = 0;
		bool running = false;
	};

	/** @brief Compiles a_jobs on background threads into the shader cache so the first claim finds them ready; does nothing without a cache folder. */
	void StartShaderPrewarm(std::vector<ShaderJob> a_jobs);
	[[nodiscard]] ShaderPrewarmStatus GetShaderPrewarmStatus();
	/** @brief Blocks until the prewarm ends; returns at once when none is running. */
	void WaitForShaderPrewarm();

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

	/** @brief Release a raw D3D11 COM pointer if non-null, then null it. */
	template <typename T>
	inline void SafeRelease(T*& ptr)
	{
		if (ptr) {
			ptr->Release();
			ptr = nullptr;
		}
	}

	/** @brief SafeRelease every element of a fixed-size array of raw D3D11 COM pointers. */
	template <typename T, size_t N>
	inline void SafeReleaseArray(T* (&arr)[N])
	{
		for (size_t i = 0; i < N; ++i)
			SafeRelease(arr[i]);
	}

	/**
	 * @brief RAII guard that snapshots the D3D11 pipeline state a fullscreen
	 * draw pass clobbers and restores+Releases it on scope exit.
	 *
	 * Covers OM (RTV/DSV, blend, depth-stencil), RS (state, viewports),
	 * the VS/PS/GS/HS/DS shaders, IA (input layout, vertex/index buffers,
	 * topology), and the requested PS sampler/resource/constant-buffer ranges.
	 * The destructor nulls the saved PS SRV range before restoring to break any SRV-vs-RTV
	 * hazard left by the wrapped pass. Construct it, set up + issue the pass,
	 * then let it go out of scope.
	 */
	struct FullscreenPassScope
	{
		/** @brief Saves PS SRVs starting at zero and the specified CB range; defaults preserve SRV0 and CB1. */
		explicit FullscreenPassScope(ID3D11DeviceContext* a_context, UINT a_psSRVCount = 1, UINT a_psCBStart = 1, UINT a_psCBCount = 1, UINT a_psSamplerCount = 1);
		~FullscreenPassScope();
		FullscreenPassScope(const FullscreenPassScope&) = delete;
		FullscreenPassScope& operator=(const FullscreenPassScope&) = delete;

	private:
		ID3D11DeviceContext* ctx = nullptr;
		ID3D11RenderTargetView* savedRTV[D3D11_SIMULTANEOUS_RENDER_TARGET_COUNT] = {};
		ID3D11DepthStencilView* savedDSV = nullptr;
		UINT numVP = D3D11_VIEWPORT_AND_SCISSORRECT_OBJECT_COUNT_PER_PIPELINE;
		D3D11_VIEWPORT savedVP[D3D11_VIEWPORT_AND_SCISSORRECT_OBJECT_COUNT_PER_PIPELINE] = {};
		ID3D11BlendState* savedBlend = nullptr;
		FLOAT savedBlendFactor[4] = {};
		UINT savedSampleMask = 0;
		ID3D11DepthStencilState* savedDSState = nullptr;
		UINT savedStencilRef = 0;
		ID3D11VertexShader* savedVS = nullptr;
		ID3D11PixelShader* savedPS = nullptr;
		ID3D11GeometryShader* savedGS = nullptr;
		ID3D11HullShader* savedHS = nullptr;
		ID3D11DomainShader* savedDS = nullptr;
		ID3D11RasterizerState* savedRS = nullptr;
		ID3D11SamplerState* savedPSSamplers[D3D11_COMMONSHADER_SAMPLER_SLOT_COUNT] = {};
		UINT psSRVCount = 1;
		UINT psCBStart = 1;
		UINT psCBCount = 1;
		UINT psSamplerCount = 1;
		ID3D11ShaderResourceView* savedPSSRVs[D3D11_COMMONSHADER_INPUT_RESOURCE_SLOT_COUNT] = {};
		ID3D11Buffer* savedPSCBs[D3D11_COMMONSHADER_CONSTANT_BUFFER_API_SLOT_COUNT] = {};
		ID3D11InputLayout* savedIL = nullptr;
		ID3D11Buffer* savedVB[D3D11_IA_VERTEX_INPUT_RESOURCE_SLOT_COUNT] = {};
		UINT savedVBStride[D3D11_IA_VERTEX_INPUT_RESOURCE_SLOT_COUNT] = {};
		UINT savedVBOffset[D3D11_IA_VERTEX_INPUT_RESOURCE_SLOT_COUNT] = {};
		ID3D11Buffer* savedIB = nullptr;
		DXGI_FORMAT savedIBFormat = DXGI_FORMAT_UNKNOWN;
		UINT savedIBOffset = 0;
		D3D11_PRIMITIVE_TOPOLOGY savedTopology = D3D11_PRIMITIVE_TOPOLOGY_UNDEFINED;
	};
}
