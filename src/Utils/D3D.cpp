#include "Utils/D3D.h"

#include <chrono>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <format>
#include <fstream>
#include <iterator>
#include <mutex>
#include <string_view>
#include <unordered_set>
#include <vector>

namespace Util
{
	namespace
	{
		constexpr GUID kDebugObjectNameT = { 0x4cca5fd8, 0x921f, 0x42c8, { 0x85, 0x66, 0x70, 0xca, 0xf2, 0xa9, 0xb7, 0x41 } };

		std::wstring shaderRoot;
		std::filesystem::path shaderCacheDir;
		std::mutex failureMutex;
		std::unordered_set<std::string> compileFailures;

		/** @brief Resolves #include "X" against the including file's folder, then the shader root. */
		class RootInclude : public ID3DInclude
		{
		public:
			RootInclude(std::filesystem::path a_source, std::filesystem::path a_root) :
				source(std::move(a_source)), root(std::move(a_root)) {}

			std::vector<std::filesystem::path> opened;

			HRESULT Open(D3D_INCLUDE_TYPE, LPCSTR a_file, LPCVOID, LPCVOID* a_data, UINT* a_size) override
			{
				for (const auto& base : { source.parent_path(), root }) {
					std::ifstream stream(base / a_file, std::ios::binary);
					if (!stream)
						continue;
					std::string bytes((std::istreambuf_iterator<char>(stream)), std::istreambuf_iterator<char>());
					opened.push_back(base / a_file);
					auto* copy = new char[bytes.size()];
					std::memcpy(copy, bytes.data(), bytes.size());
					*a_data = copy;
					*a_size = static_cast<UINT>(bytes.size());
					return S_OK;
				}
				return HRESULT_FROM_WIN32(ERROR_FILE_NOT_FOUND);
			}

			HRESULT Close(LPCVOID a_data) override
			{
				delete[] static_cast<const char*>(a_data);
				return S_OK;
			}

		private:
			std::filesystem::path source;
			std::filesystem::path root;
		};

		constexpr UINT kCompileFlags = D3DCOMPILE_ENABLE_STRICTNESS | D3DCOMPILE_OPTIMIZATION_LEVEL3;
		constexpr std::string_view kCacheTag = "v2";

		/** @brief A dependency as "path-under-shader-root<TAB>content hash", so a cache built elsewhere stays valid. */
		std::string StampOf(const std::filesystem::path& a_path)
		{
			std::ifstream stream(a_path, std::ios::binary);
			std::uint64_t hash = 1469598103934665603ull;
			char buffer[4096];
			while (stream.read(buffer, sizeof(buffer)) || stream.gcount() > 0) {
				for (std::streamsize i = 0; i < stream.gcount(); ++i) {
					hash ^= static_cast<unsigned char>(buffer[i]);
					hash *= 1099511628211ull;
				}
			}
			std::error_code error;
			const auto relative = std::filesystem::relative(a_path, std::filesystem::path(shaderRoot), error);
			return std::format("{}\t{:016x}", error || relative.empty() ? a_path.generic_string() : relative.generic_string(), hash);
		}

		std::string CacheName(std::string_view a_key)
		{
			std::uint64_t hash = 1469598103934665603ull;
			for (const char character : a_key) {
				hash ^= static_cast<unsigned char>(character);
				hash *= 1099511628211ull;
			}
			return std::format("{:016x}", hash);
		}

		/** @brief A cached blob is good only while the source and every include it was built from keep their size and time. */
		winrt::com_ptr<ID3DBlob> ReadCache(const std::string& a_name)
		{
			winrt::com_ptr<ID3DBlob> blob;
			if (shaderCacheDir.empty())
				return blob;
			std::ifstream deps(shaderCacheDir / (a_name + ".dep"));
			std::string line;
			if (!deps || !std::getline(deps, line) || line != kCacheTag)
				return blob;
			while (std::getline(deps, line)) {
				const std::filesystem::path stored = line.substr(0, line.find('\t'));
				const std::filesystem::path file = stored.is_absolute() ? stored : std::filesystem::path(shaderRoot) / stored;
				if (StampOf(file) != line)
					return blob;
			}
			std::ifstream data(shaderCacheDir / (a_name + ".bin"), std::ios::binary);
			const std::string bytes((std::istreambuf_iterator<char>(data)), std::istreambuf_iterator<char>());
			if (bytes.empty() || FAILED(D3DCreateBlob(bytes.size(), blob.put())))
				return nullptr;
			std::memcpy(blob->GetBufferPointer(), bytes.data(), bytes.size());
			return blob;
		}

		void WriteCache(const std::string& a_name, ID3DBlob* a_blob, const std::filesystem::path& a_source, const std::vector<std::filesystem::path>& a_includes)
		{
			if (shaderCacheDir.empty())
				return;
			std::error_code error;
			std::filesystem::create_directories(shaderCacheDir, error);
			const auto stem = shaderCacheDir / a_name;
			const auto temporary = std::filesystem::path(stem.string() + std::format(".{}.tmp", std::chrono::steady_clock::now().time_since_epoch().count()));
			{
				std::ofstream data(temporary, std::ios::binary);
				data.write(static_cast<const char*>(a_blob->GetBufferPointer()), static_cast<std::streamsize>(a_blob->GetBufferSize()));
			}
			std::filesystem::rename(temporary, stem.string() + ".bin", error);
			if (error) {
				std::filesystem::remove(temporary, error);
				return;
			}
			std::ofstream deps(stem.string() + ".dep", std::ios::trunc);
			deps << kCacheTag << '\n'
			     << StampOf(a_source) << '\n';
			for (const auto& include : a_includes)
				deps << StampOf(include) << '\n';
		}

		std::filesystem::path Resolve(const wchar_t* a_path)
		{
			const std::filesystem::path path{ a_path };
			return path.is_absolute() || shaderRoot.empty() ? path : std::filesystem::path(shaderRoot) / path;
		}
	}

	void SetResourceName(ID3D11DeviceChild* Resource, const char* Format, ...)
	{
		if (!Resource)
			return;
		char buffer[1024];
		va_list va;
		va_start(va, Format);
		int len = _vsnprintf_s(buffer, _TRUNCATE, Format, va);
		va_end(va);
		if (len < 0)
			len = static_cast<int>(strnlen(buffer, sizeof(buffer)));
		Resource->SetPrivateData(kDebugObjectNameT, static_cast<UINT>(len), buffer);
	}

	std::string WStringToString(const wchar_t* a_text)
	{
		return std::filesystem::path(a_text).string();
	}

	void SetShaderRoot(const std::wstring& a_root)
	{
		shaderRoot = a_root;
		std::lock_guard lock(failureMutex);
		compileFailures.clear();
	}

	void SetShaderCacheDir(const std::wstring& a_dir)
	{
		shaderCacheDir = a_dir;
	}

	void ClearShaderFailures()
	{
		std::lock_guard lock(failureMutex);
		compileFailures.clear();
	}

	const std::wstring& GetShaderRoot()
	{
		return shaderRoot;
	}

	winrt::com_ptr<ID3DBlob> CompileShaderBlob(const wchar_t* FilePath, const std::vector<std::pair<const char*, const char*>>& Defines, const char* ProgramType, const char* Program)
	{
		const std::filesystem::path source = Resolve(FilePath);
		std::string failureKey = std::format("{}|{}|{}", source.string(), ProgramType, Program);
		std::vector<D3D_SHADER_MACRO> macros;
		for (const auto& [name, value] : Defines) {
			if (!name || !*name)
				continue;
			macros.push_back({ name, value ? value : "" });
			failureKey += std::format("|{}={}", name, value ? value : "");
		}
		macros.push_back({ nullptr, nullptr });
		{
			std::lock_guard lock(failureMutex);
			if (compileFailures.contains(failureKey))
				return nullptr;
		}

		std::error_code relativeError;
		const auto relativeSource = std::filesystem::relative(source, std::filesystem::path(shaderRoot), relativeError);
		std::string cacheKey = failureKey;
		if (!relativeError && !relativeSource.empty())
			cacheKey.replace(0, source.string().size(), relativeSource.generic_string());
		const std::string cacheName = CacheName(cacheKey + '|' + std::to_string(kCompileFlags));
		if (auto cached = ReadCache(cacheName))
			return cached;

		std::ifstream stream(source, std::ios::binary);
		winrt::com_ptr<ID3DBlob> blob;
		winrt::com_ptr<ID3DBlob> errors;
		HRESULT result = E_FAIL;
		if (stream) {
			const std::string text((std::istreambuf_iterator<char>(stream)), std::istreambuf_iterator<char>());
			RootInclude include(source, shaderRoot);
			result = D3DCompile(text.data(), text.size(), source.string().c_str(), macros.data(), &include, Program, ProgramType,
				kCompileFlags, 0, blob.put(), errors.put());
			if (SUCCEEDED(result))
				WriteCache(cacheName, blob.get(), source, include.opened);
		} else {
			logger::error("Failed to compile shader; {} does not exist", source.string());
		}
		if (FAILED(result)) {
			if (errors)
				logger::warn("Shader compilation failed:\n\n{}", std::string_view(static_cast<const char*>(errors->GetBufferPointer()), errors->GetBufferSize()));
			std::lock_guard lock(failureMutex);
			compileFailures.insert(failureKey);
			return nullptr;
		}
		return blob;
	}

	ID3D11DeviceChild* CompileShader(const wchar_t* FilePath, const std::vector<std::pair<const char*, const char*>>& Defines, const char* ProgramType, const char* Program)
	{
		auto* device = globals::d3d::device;
		auto blob = CompileShaderBlob(FilePath, Defines, ProgramType, Program);
		if (!device || !blob)
			return nullptr;

		ID3D11DeviceChild* shader = nullptr;
		HRESULT result = E_FAIL;
		const std::string_view profile(ProgramType);
		if (profile.starts_with("cs_")) {
			ID3D11ComputeShader* created = nullptr;
			result = device->CreateComputeShader(blob->GetBufferPointer(), blob->GetBufferSize(), nullptr, &created);
			shader = created;
		} else if (profile.starts_with("vs_")) {
			ID3D11VertexShader* created = nullptr;
			result = device->CreateVertexShader(blob->GetBufferPointer(), blob->GetBufferSize(), nullptr, &created);
			shader = created;
		} else if (profile.starts_with("ps_")) {
			ID3D11PixelShader* created = nullptr;
			result = device->CreatePixelShader(blob->GetBufferPointer(), blob->GetBufferSize(), nullptr, &created);
			shader = created;
		}
		if (FAILED(result)) {
			logger::error("Failed to create shader object for {}:{}", WStringToString(FilePath), Program);
			return nullptr;
		}
		SetResourceName(shader, "%s:%s", WStringToString(FilePath).c_str(), Program);
		return shader;
	}

	FullscreenPassScope::FullscreenPassScope(ID3D11DeviceContext* a_context, UINT a_psSRVCount, UINT a_psCBStart, UINT a_psCBCount, UINT a_psSamplerCount) :
		ctx(a_context),
		psSRVCount(a_psSRVCount),
		psCBStart(a_psCBStart),
		psCBCount(a_psCBCount),
		psSamplerCount(a_psSamplerCount)
	{
		assert(ctx && psSRVCount <= D3D11_COMMONSHADER_INPUT_RESOURCE_SLOT_COUNT &&
			   psCBStart <= D3D11_COMMONSHADER_CONSTANT_BUFFER_API_SLOT_COUNT &&
			   psCBCount <= D3D11_COMMONSHADER_CONSTANT_BUFFER_API_SLOT_COUNT - psCBStart &&
			   psSamplerCount <= D3D11_COMMONSHADER_SAMPLER_SLOT_COUNT);
		// Save all MRT slots, not just slot 0 — a caller may enter with multiple RTVs bound.
		ctx->OMGetRenderTargets(D3D11_SIMULTANEOUS_RENDER_TARGET_COUNT, savedRTV, &savedDSV);
		ctx->RSGetViewports(&numVP, savedVP);
		ctx->OMGetBlendState(&savedBlend, savedBlendFactor, &savedSampleMask);
		ctx->OMGetDepthStencilState(&savedDSState, &savedStencilRef);
		ctx->VSGetShader(&savedVS, nullptr, nullptr);
		ctx->PSGetShader(&savedPS, nullptr, nullptr);
		ctx->GSGetShader(&savedGS, nullptr, nullptr);
		ctx->HSGetShader(&savedHS, nullptr, nullptr);
		ctx->DSGetShader(&savedDS, nullptr, nullptr);
		ctx->RSGetState(&savedRS);
		ctx->PSGetSamplers(0, psSamplerCount, savedPSSamplers);
		ctx->PSGetShaderResources(0, psSRVCount, savedPSSRVs);
		ctx->PSGetConstantBuffers(psCBStart, psCBCount, savedPSCBs);
		ctx->IAGetInputLayout(&savedIL);
		ctx->IAGetVertexBuffers(0, D3D11_IA_VERTEX_INPUT_RESOURCE_SLOT_COUNT, savedVB, savedVBStride, savedVBOffset);
		ctx->IAGetIndexBuffer(&savedIB, &savedIBFormat, &savedIBOffset);
		ctx->IAGetPrimitiveTopology(&savedTopology);
	}

	FullscreenPassScope::~FullscreenPassScope()
	{
		// Restore targets before SRVs so an outgoing target cannot suppress a saved input.
		ID3D11ShaderResourceView* nullSRVs[D3D11_COMMONSHADER_INPUT_RESOURCE_SLOT_COUNT] = {};
		ctx->PSSetShaderResources(0, psSRVCount, nullSRVs);

		ctx->OMSetRenderTargets(D3D11_SIMULTANEOUS_RENDER_TARGET_COUNT, savedRTV, savedDSV);
		ctx->PSSetShaderResources(0, psSRVCount, savedPSSRVs);
		ctx->RSSetViewports(numVP, savedVP);
		ctx->OMSetBlendState(savedBlend, savedBlendFactor, savedSampleMask);
		ctx->OMSetDepthStencilState(savedDSState, savedStencilRef);
		ctx->VSSetShader(savedVS, nullptr, 0);
		ctx->PSSetShader(savedPS, nullptr, 0);
		ctx->GSSetShader(savedGS, nullptr, 0);
		ctx->HSSetShader(savedHS, nullptr, 0);
		ctx->DSSetShader(savedDS, nullptr, 0);
		ctx->RSSetState(savedRS);
		ctx->PSSetSamplers(0, psSamplerCount, savedPSSamplers);
		ctx->PSSetConstantBuffers(psCBStart, psCBCount, savedPSCBs);
		ctx->IASetInputLayout(savedIL);
		ctx->IASetVertexBuffers(0, D3D11_IA_VERTEX_INPUT_RESOURCE_SLOT_COUNT, savedVB, savedVBStride, savedVBOffset);
		ctx->IASetIndexBuffer(savedIB, savedIBFormat, savedIBOffset);
		ctx->IASetPrimitiveTopology(savedTopology);

		SafeReleaseArray(savedRTV);
		SafeRelease(savedDSV);
		SafeRelease(savedBlend);
		SafeRelease(savedDSState);
		SafeRelease(savedVS);
		SafeRelease(savedPS);
		SafeRelease(savedGS);
		SafeRelease(savedHS);
		SafeRelease(savedDS);
		SafeRelease(savedRS);
		for (UINT i = 0; i < psSamplerCount; ++i)
			SafeRelease(savedPSSamplers[i]);
		for (UINT i = 0; i < psSRVCount; ++i)
			SafeRelease(savedPSSRVs[i]);
		for (UINT i = 0; i < psCBCount; ++i)
			SafeRelease(savedPSCBs[i]);
		SafeRelease(savedIL);
		SafeReleaseArray(savedVB);
		SafeRelease(savedIB);
	}
}
