#include "engine/DebugLines.h"

#include "Buffer.h"
#include "Globals.h"
#include "GpuPass.h"
#include "Utils/D3D.h"
#include "smp/GpuViews.h"

#include <algorithm>
#include <array>
#include <cstring>
#include <exception>
#include <utility>

namespace SmpPhysics
{
	namespace
	{
		constexpr const wchar_t* kDrawPath = L"SmpPhysics\\DebugLines.hlsl";
		constexpr std::uint32_t kDrawConstantsRegister = 1;
		constexpr std::uint32_t kPerFrameRegister = 12;
		constexpr std::uint32_t kStereoRegister = 13;
		constexpr std::uint32_t kVerticesPerLine = 6;  ///< Two triangles.
		constexpr std::uint32_t kMaximumLines = 65536;
		constexpr std::uint32_t kMinimumCapacity = 256;
		constexpr float kLineWidth = 0.25f;         ///< World width of a line, in game units.
		constexpr float kMinimumPixelWidth = 2.0f;  ///< Narrowest a line draws, so it stays visible at a distance.
		constexpr UINT kSavedPixelConstantStart = 1;
		constexpr UINT kSavedPixelConstantCount = 13;
		constexpr UINT kVertexConstantSlots = 14;
		constexpr std::uintptr_t kVrStereoBufferOffset = 0x3180688;
		/** @brief Colours of DebugColor, in order. */
		constexpr float kPalette[static_cast<std::size_t>(DebugColor::Count)][3] = { { 1.0f, 0.9f, 0.1f }, { 0.1f, 1.0f, 0.2f }, { 1.0f, 0.15f, 0.1f }, { 0.1f, 0.9f, 1.0f }, { 1.0f, 1.0f, 1.0f }, { 1.0f, 0.2f, 1.0f }, { 0.45f, 0.55f, 1.0f }, { 1.0f, 0.5f, 0.0f }, { 0.7f, 0.7f, 0.7f }, { 1.0f, 0.8f, 0.5f }, { 1.0f, 0.55f, 0.75f }, { 0.0f, 0.7f, 0.6f }, { 0.55f, 1.0f, 0.1f }, { 0.8f, 0.6f, 0.15f }, { 0.5f, 0.5f, 0.5f }, { 0.7f, 0.3f, 1.0f } };

		/** @brief One end of a line, as the vertex shader reads it. */
		struct DebugLineVertex
		{
			float position[3]{};
			float width{};
		};
		static_assert(sizeof(DebugLineVertex) == 16);

		/** @brief The line shaders' constant buffer, mirrored by DebugLines.hlsl. */
		struct DebugLineConstants
		{
			std::uint32_t lineCount{};
			float minimumPixelWidth{};
			float viewport[2]{};
			float color[4]{};
		};
		static_assert(sizeof(DebugLineConstants) == 32);

		/** @brief The scene's main render target and the viewport the world was drawn with; false when either is missing. */
		bool QueryRenderState(ID3D11RenderTargetView*& a_target, D3D11_VIEWPORT& a_viewport)
		{
			auto* renderer = globals::game::renderer;
			if (!renderer || !globals::game::perFrame.address())
				return false;
			a_target = Util::AsReal(renderer->GetRuntimeData().renderTargets[static_cast<int>(RE::RENDER_TARGET::kMAIN)].RTV);
			if (!a_target)
				return false;
			UINT viewportCount = 1;
			globals::d3d::context->RSGetViewports(&viewportCount, &a_viewport);
			return viewportCount != 0 && a_viewport.Width > 0.0f && a_viewport.Height > 0.0f;
		}
	}

	struct DebugLines::Resources
	{
		winrt::com_ptr<ID3D11BlendState> blend;
		winrt::com_ptr<ID3D11DepthStencilState> depth;  ///< Always passes and writes nothing, so the overlay shows through hair and heads.
		winrt::com_ptr<ID3D11RasterizerState> raster;
	};

	DebugLines::DebugLines() = default;
	DebugLines::~DebugLines() = default;

	void DebugLines::SetSegments(std::vector<DebugSegment> a_segments)
	{
		const std::scoped_lock lock(mutex);
		segments = std::move(a_segments);
	}

	DebugLines::Stats DebugLines::GetStats() const
	{
		const std::scoped_lock lock(mutex);
		return stats;
	}

	void DebugLines::InvalidateShaders()
	{
		const std::scoped_lock lock(mutex);
		if (vertexShader)
			vertexShader->Reset();
		if (pixelShader)
			pixelShader->Reset();
		stats.failed = false;
	}

	bool DebugLines::EnsureShaders()
	{
		if (!vertexShader)
			vertexShader = std::make_unique<Util::LazyShader<ID3D11VertexShader>>();
		if (!pixelShader)
			pixelShader = std::make_unique<Util::LazyShader<ID3D11PixelShader>>();
		std::vector<std::pair<const char*, const char*>> defines;
		if (globals::game::isVR)
			defines.emplace_back("VR", "");
		auto vertexDefines = defines;
		vertexDefines.emplace_back("VSHADER", "");
		auto pixelDefines = defines;
		pixelDefines.emplace_back("PSHADER", "");
		try {
			if (!constants)
				constants = std::make_unique<ConstantBuffer>(ConstantBufferDesc<DebugLineConstants>(true), "SmpPhysics::DebugLineConstants");
			return vertexShader->Get(kDrawPath, vertexDefines, "vs_5_0", "VSMain", "SmpPhysics::DebugLines VS") &&
			       pixelShader->Get(kDrawPath, pixelDefines, "ps_5_0", "PSMain", "SmpPhysics::DebugLines PS");
		} catch (const std::exception& error) {
			logger::warn("{} the debug line shaders did not compile: {}", kLogPrefix, error.what());
			return false;
		}
	}

	bool DebugLines::EnsureStates()
	{
		if (!resources)
			resources = std::make_unique<Resources>();
		auto* device = globals::d3d::device;
		if (!device)
			return false;
		if (!resources->blend) {
			D3D11_BLEND_DESC desc{};
			desc.IndependentBlendEnable = FALSE;
			desc.RenderTarget[0].BlendEnable = FALSE;
			desc.RenderTarget[0].RenderTargetWriteMask = D3D11_COLOR_WRITE_ENABLE_ALL;
			if (FAILED(device->CreateBlendState(&desc, resources->blend.put())))
				return false;
			Util::SetResourceName(resources->blend.get(), "SmpPhysics::DebugLinesBlend");
		}
		if (!resources->depth) {
			D3D11_DEPTH_STENCIL_DESC desc{};
			desc.DepthEnable = FALSE;
			desc.DepthWriteMask = D3D11_DEPTH_WRITE_MASK_ZERO;
			desc.StencilEnable = FALSE;
			if (FAILED(device->CreateDepthStencilState(&desc, resources->depth.put())))
				return false;
			Util::SetResourceName(resources->depth.get(), "SmpPhysics::DebugLinesDepth");
		}
		if (!resources->raster) {
			D3D11_RASTERIZER_DESC desc{};
			desc.FillMode = D3D11_FILL_SOLID;
			desc.CullMode = D3D11_CULL_NONE;
			desc.DepthClipEnable = TRUE;
			if (FAILED(device->CreateRasterizerState(&desc, resources->raster.put())))
				return false;
			Util::SetResourceName(resources->raster.get(), "SmpPhysics::DebugLinesRaster");
		}
		return true;
	}

	void DebugLines::Render()
	{
		const std::scoped_lock lock(mutex);
		auto* context = globals::d3d::context;
		if (segments.empty() || !context || stats.failed) {
			stats.requested = 0;
			stats.drawn = 0;
			return;
		}
		if (!EnsureShaders() || !EnsureStates()) {
			stats.failed = true;
			return;
		}

		ID3D11RenderTargetView* target = nullptr;
		D3D11_VIEWPORT viewport{};
		stats.requested = static_cast<std::uint32_t>(segments.size());
		if (!QueryRenderState(target, viewport)) {
			++stats.stateFailures;
			return;
		}

		const auto lineCount = static_cast<std::uint32_t>(std::min<std::size_t>(segments.size(), kMaximumLines));
		stats.drawn = lineCount;
		std::stable_sort(segments.begin(), segments.end(), [](const DebugSegment& a_left, const DebugSegment& a_right) { return a_left.color < a_right.color; });

		std::vector<DebugLineVertex> lineVertices(static_cast<std::size_t>(lineCount) * 2);
		std::array<std::pair<std::uint32_t, std::uint32_t>, static_cast<std::size_t>(DebugColor::Count)> ranges{};
		for (std::uint32_t line = 0; line < lineCount; ++line) {
			const DebugSegment& segment = segments[line];
			const Vec3 ends[2] = { segment.start, segment.end };
			for (std::uint32_t end = 0; end < 2; ++end) {
				DebugLineVertex& vertex = lineVertices[static_cast<std::size_t>(line) * 2 + end];
				vertex.position[0] = ends[end].x;
				vertex.position[1] = ends[end].y;
				vertex.position[2] = ends[end].z;
				vertex.width = kLineWidth;
			}
			auto& range = ranges[static_cast<std::size_t>(segment.color)];
			if (range.second == 0)
				range.first = line;
			++range.second;
		}

		try {
			if (!vertices || capacity < lineCount) {
				capacity = std::max(lineCount, kMinimumCapacity);
				vertices = std::make_unique<Buffer>(StructuredBufferDesc<DebugLineVertex>(capacity * 2, false, true), nullptr, "SmpPhysics::DebugLineVertices");
				vertices->CreateSRV(StructuredSrvDesc(capacity * 2));
			}
		} catch (const std::exception& error) {
			logger::warn("{} the debug overlay could not allocate its lines: {}", kLogPrefix, error.what());
			vertices.reset();
			capacity = 0;
			return;
		}
		D3D11_MAPPED_SUBRESOURCE mapped{};
		if (FAILED(context->Map(vertices->resource.get(), 0, D3D11_MAP_WRITE_DISCARD, 0, &mapped)))
			return;
		std::memcpy(mapped.pData, lineVertices.data(), lineVertices.size() * sizeof(DebugLineVertex));
		context->Unmap(vertices->resource.get(), 0);

		CS_GPU_PASS("SmpPhysics::DebugLines");
		Util::FullscreenPassScope scope(context, 1, kSavedPixelConstantStart, kSavedPixelConstantCount);
		ID3D11Buffer* savedVertexConstants[kVertexConstantSlots]{};
		ID3D11ShaderResourceView* savedVertexView = nullptr;
		context->VSGetConstantBuffers(0, kVertexConstantSlots, savedVertexConstants);
		context->VSGetShaderResources(0, 1, &savedVertexView);

		ID3D11Buffer* perFrame = *globals::game::perFrame.get();
		ID3D11Buffer* drawConstants = constants->CB();
		ID3D11ShaderResourceView* vertexView = vertices->srv.get();

		context->OMSetRenderTargets(1, &target, nullptr);
		context->RSSetViewports(1, &viewport);
		context->OMSetBlendState(resources->blend.get(), nullptr, 0xFFFFFFFFu);
		context->OMSetDepthStencilState(resources->depth.get(), 0);
		context->RSSetState(resources->raster.get());
		context->IASetInputLayout(nullptr);
		context->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
		context->VSSetShader(vertexShader->get(), nullptr, 0);
		context->PSSetShader(pixelShader->get(), nullptr, 0);
		context->GSSetShader(nullptr, nullptr, 0);
		context->HSSetShader(nullptr, nullptr, 0);
		context->DSSetShader(nullptr, nullptr, 0);
		context->VSSetConstantBuffers(kDrawConstantsRegister, 1, &drawConstants);
		context->VSSetConstantBuffers(kPerFrameRegister, 1, &perFrame);
		context->PSSetConstantBuffers(kDrawConstantsRegister, 1, &drawConstants);
		if (globals::game::isVR) {
			static REL::Relocation<ID3D11Buffer**> stereoValues{ REL::Offset(kVrStereoBufferOffset) };
			ID3D11Buffer* stereoBuffer = *stereoValues.get();
			context->VSSetConstantBuffers(kStereoRegister, 1, &stereoBuffer);
		}
		context->VSSetShaderResources(0, 1, &vertexView);

		DebugLineConstants values;
		values.minimumPixelWidth = kMinimumPixelWidth;
		values.viewport[0] = viewport.Width;
		values.viewport[1] = viewport.Height;
		for (std::size_t color = 0; color < ranges.size(); ++color) {
			if (ranges[color].second == 0)
				continue;
			values.lineCount = lineCount;
			for (int channel = 0; channel < 3; ++channel)
				values.color[channel] = kPalette[color][channel];
			try {
				constants->Update(values);
			} catch (const std::exception& error) {
				stats.failed = true;
				logger::warn("{} the debug overlay could not update its constants: {}", kLogPrefix, error.what());
				break;
			}
			context->DrawInstanced(ranges[color].second * kVerticesPerLine, globals::game::isVR ? 2u : 1u, ranges[color].first * kVerticesPerLine, 0);
		}

		context->VSSetShaderResources(0, 1, &savedVertexView);
		context->VSSetConstantBuffers(0, kVertexConstantSlots, savedVertexConstants);
		if (savedVertexView)
			savedVertexView->Release();
		for (auto* buffer : savedVertexConstants) {
			if (buffer)
				buffer->Release();
		}
	}
}
