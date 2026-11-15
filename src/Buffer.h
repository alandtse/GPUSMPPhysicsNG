#pragma once

#include "Globals.h"
#include "Utils/D3D.h"

#include <d3d11.h>
#include <stdexcept>
#include <string>

#include <Windows.Foundation.h>
#include <stdio.h>
#include <winrt/base.h>
#include <wrl\client.h>
#include <wrl\wrappers\corewrappers.h>

// Forward declaration — keeps Buffer.h free of Utils/D3D.h's game-type dependencies
namespace Util
{
	void SetResourceName(ID3D11DeviceChild* Resource, const char* Format, ...);
}

namespace detail
{
	inline void SetD3DName(ID3D11DeviceChild* resource, const std::string& name, const char* suffix = nullptr)
	{
		if (!resource || name.empty())
			return;
		if (suffix)
			Util::SetResourceName(resource, "%s%s", name.c_str(), suffix);
		else
			Util::SetResourceName(resource, "%s", name.c_str());
	}
}

/** @brief Creates a D3D11 buffer descriptor for a structured buffer with the given element count. */
template <typename T>
D3D11_BUFFER_DESC StructuredBufferDesc(uint64_t count, bool uav = true, bool dynamic = false)
{
	D3D11_BUFFER_DESC desc{};
	desc.Usage = (uav || !dynamic) ? D3D11_USAGE_DEFAULT : D3D11_USAGE_DYNAMIC;
	desc.BindFlags = D3D11_BIND_SHADER_RESOURCE;
	if (uav)
		desc.BindFlags |= D3D11_BIND_UNORDERED_ACCESS;
	desc.MiscFlags = D3D11_RESOURCE_MISC_BUFFER_STRUCTURED;
	desc.CPUAccessFlags = !dynamic ? 0 : D3D11_CPU_ACCESS_WRITE;
	desc.StructureByteStride = sizeof(T);
	desc.ByteWidth = (UINT)(sizeof(T) * count);
	return desc;
}

/** @brief Rounds a buffer size up to the next 64-byte boundary for constant buffer alignment. */
static constexpr std::uint32_t GetCBufferSize(std::uint32_t buffer_size)
{
	return (buffer_size + (64 - 1)) & ~(64 - 1);
}

/** @brief Creates a D3D11 buffer descriptor for a constant buffer of the given byte size. */
inline D3D11_BUFFER_DESC ConstantBufferDesc(uint32_t size, bool dynamic = true)
{
	D3D11_BUFFER_DESC desc{};
	ZeroMemory(&desc, sizeof(desc));
	desc.Usage = dynamic ? D3D11_USAGE_DYNAMIC : D3D11_USAGE_DEFAULT;
	desc.BindFlags = D3D11_BIND_CONSTANT_BUFFER;
	desc.CPUAccessFlags = dynamic ? D3D11_CPU_ACCESS_WRITE : 0;
	desc.ByteWidth = GetCBufferSize(size);
	return desc;
}

/** @brief Creates a D3D11 constant buffer descriptor sized for type T. */
template <typename T>
D3D11_BUFFER_DESC ConstantBufferDesc(bool dynamic = true)
{
	return ConstantBufferDesc(sizeof(T), dynamic);
}

/** @brief RAII wrapper around a D3D11 constant buffer with map/update support. */
class ConstantBuffer
{
public:
	explicit ConstantBuffer(D3D11_BUFFER_DESC const& a_desc, const char* name = nullptr) :
		desc(a_desc)
	{
		auto device = globals::d3d::device;
		if (!device)
			throw std::runtime_error("no D3D device");
		DX::ThrowIfFailed(device->CreateBuffer(&desc, nullptr, resource.put()));
		if (name)
			detail::SetD3DName(resource.get(), name);
	}

	/** @brief Gets the underlying ID3D11Buffer pointer. */
	ID3D11Buffer* CB() const { return resource.get(); }

	/**
	 * @brief Uploads data to the constant buffer.
	 * @param src_data Pointer to source data.
	 * @param data_size Size in bytes to copy.
	 */
	void Update(void const* src_data, size_t data_size)
	{
		auto ctx = globals::d3d::context;
		if (desc.Usage == D3D11_USAGE_DYNAMIC) {
			D3D11_MAPPED_SUBRESOURCE mapped_buffer{};
			ZeroMemory(&mapped_buffer, sizeof(D3D11_MAPPED_SUBRESOURCE));
			DX::ThrowIfFailed(ctx->Map(resource.get(), 0u, D3D11_MAP_WRITE_DISCARD, 0u, &mapped_buffer));
			memcpy(mapped_buffer.pData, src_data, data_size);
			ctx->Unmap(resource.get(), 0);
		} else
			ctx->UpdateSubresource(resource.get(), 0, nullptr, src_data, 0, 0);
	}

	/** @brief Uploads a typed value to the constant buffer. */
	template <typename T>
	void Update(T const& src_data)
	{
		Update(&src_data, sizeof(T));
	}

private:
	winrt::com_ptr<ID3D11Buffer> resource;
	D3D11_BUFFER_DESC desc;
};

/** @brief RAII wrapper around a generic D3D11 buffer with SRV and UAV support. */
class Buffer
{
public:
	explicit Buffer(D3D11_BUFFER_DESC const& a_desc, D3D11_SUBRESOURCE_DATA* a_init = nullptr, const char* name = nullptr) :
		desc(a_desc)
	{
		auto device = globals::d3d::device;
		if (!device)
			throw std::runtime_error("no D3D device");
		DX::ThrowIfFailed(device->CreateBuffer(&desc, a_init, resource.put()));
		if (name) {
			name_ = name;
			detail::SetD3DName(resource.get(), name_);
		}
	}

	/** @brief Creates a shader resource view from the given descriptor. */
	void CreateSRV(D3D11_SHADER_RESOURCE_VIEW_DESC const& a_desc)
	{
		auto device = globals::d3d::device;
		DX::ThrowIfFailed(device->CreateShaderResourceView(resource.get(), &a_desc, srv.put()));
		detail::SetD3DName(srv.get(), name_, " SRV");
	}

	/** @brief Creates an unordered access view from the given descriptor. */
	void CreateUAV(D3D11_UNORDERED_ACCESS_VIEW_DESC const& a_desc)
	{
		auto device = globals::d3d::device;
		DX::ThrowIfFailed(device->CreateUnorderedAccessView(resource.get(), &a_desc, uav.put()));
		detail::SetD3DName(uav.get(), name_, " UAV");
	}

	D3D11_BUFFER_DESC desc;
	winrt::com_ptr<ID3D11Buffer> resource;
	winrt::com_ptr<ID3D11ShaderResourceView> srv;
	winrt::com_ptr<ID3D11UnorderedAccessView> uav;

private:
	std::string name_;
};
