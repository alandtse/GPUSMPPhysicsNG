#pragma once

#include "smp/DebugShapes.h"
#include "Utils/LazyShader.h"

#include <cstdint>
#include <memory>
#include <mutex>
#include <vector>

class Buffer;
class ConstantBuffer;

namespace SmpPhysics
{
	/**
	 * @brief Draws the debug overlay's lines as flat, unlit ribbons into the deferred G-buffer.
	 * @details The overlay's lines are collected on the game thread and drawn from the deferred geometry end hook, on top
	 *          of hair and heads, with the engine's target layout so the upscalers and temporal anti-aliasing treat them
	 *          like the scene.
	 */
	class DebugLines
	{
	public:
		DebugLines();
		~DebugLines();
		DebugLines(const DebugLines&) = delete;
		DebugLines& operator=(const DebugLines&) = delete;

		/** @brief Replaces the lines drawn next. Call from the game thread; the lines draw until they are replaced or cleared. */
		void SetSegments(std::vector<DebugSegment> a_segments);
		/** @brief Draws the lines as bright ribbons on top of the scene. Run from the deferred geometry end hook. */
		void Render();
		/** @brief Drops the compiled shaders so the next draw recompiles them. */
		void InvalidateShaders();

		/** @brief Counts from the last Render, for the devbench status. */
		struct Stats
		{
			std::uint32_t requested{};      ///< Lines handed over for the last frame.
			std::uint32_t drawn{};          ///< Of those, the lines that fit the line budget and were drawn.
			std::uint64_t stateFailures{};  ///< Frames the lines were skipped because the render targets were not the G-buffer.
			bool failed{};
		};
		[[nodiscard]] Stats GetStats() const;

	private:
		struct Resources;
		[[nodiscard]] bool EnsureShaders();
		[[nodiscard]] bool EnsureStates();

		mutable std::mutex mutex;
		std::vector<DebugSegment> segments;
		std::unique_ptr<Resources> resources;
		std::unique_ptr<Util::LazyShader<ID3D11VertexShader>> vertexShader;
		std::unique_ptr<Util::LazyShader<ID3D11PixelShader>> pixelShader;
		std::unique_ptr<ConstantBuffer> constants;
		std::unique_ptr<Buffer> vertices;
		std::uint32_t capacity{};  ///< Lines the vertex buffer holds.
		Stats stats;
	};
}
