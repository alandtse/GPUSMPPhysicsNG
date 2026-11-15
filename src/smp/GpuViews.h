#pragma once

#include <d3d11.h>

#include <cstdint>

namespace SmpPhysics
{
	inline constexpr const char* kLogPrefix = "[GPU SMP Physics]";
	inline constexpr const char* kShaderTarget = "cs_5_0";
	inline constexpr std::uint32_t kThreadsPerGroup = 64;

	/** @brief A CPU-readable staging buffer of a_bytes, which a GPU buffer is copied into before a Map. */
	inline D3D11_BUFFER_DESC StagingBufferDesc(std::uint32_t a_bytes) noexcept
	{
		D3D11_BUFFER_DESC desc{};
		desc.Usage = D3D11_USAGE_STAGING;
		desc.BindFlags = 0;
		desc.CPUAccessFlags = D3D11_CPU_ACCESS_READ;
		desc.ByteWidth = a_bytes;
		return desc;
	}

	/** @brief A view over every element of a structured buffer, as a shader resource. */
	inline D3D11_SHADER_RESOURCE_VIEW_DESC StructuredSrvDesc(std::uint32_t a_count) noexcept
	{
		D3D11_SHADER_RESOURCE_VIEW_DESC desc{};
		desc.Format = DXGI_FORMAT_UNKNOWN;
		desc.ViewDimension = D3D11_SRV_DIMENSION_BUFFEREX;
		desc.Buffer.FirstElement = 0;
		desc.Buffer.NumElements = a_count;
		return desc;
	}

	/** @brief A view over every element of a structured buffer, as an unordered access target. */
	inline D3D11_UNORDERED_ACCESS_VIEW_DESC StructuredUavDesc(std::uint32_t a_count) noexcept
	{
		D3D11_UNORDERED_ACCESS_VIEW_DESC desc{};
		desc.Format = DXGI_FORMAT_UNKNOWN;
		desc.ViewDimension = D3D11_UAV_DIMENSION_BUFFER;
		desc.Buffer.FirstElement = 0;
		desc.Buffer.NumElements = a_count;
		desc.Buffer.Flags = 0;
		return desc;
	}
}
