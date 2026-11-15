#pragma once

#include "Globals.h"

#include <array>
#include <atomic>
#include <cstdint>
#include <d3d11.h>
#include <map>
#include <mutex>
#include <string>
#include <string_view>
#include <vector>
#include <winrt/base.h>

/**
 * @brief Measures the GPU time of every CS_GPU_PASS with timestamp queries, averaged per frame.
 *
 * Off by default, so a normal frame pays one atomic load per pass. NextFrame() closes the frame that
 * was open, opens a new one, and folds in frames whose queries have finished; a frame whose slot is
 * still unresolved is skipped rather than waited on.
 */
namespace GpuPassTimer
{
	struct PassStat
	{
		std::string name;
		double millisecondsPerFrame{ 0.0 };
		double callsPerFrame{ 0.0 };
	};

	struct Stats
	{
		bool enabled{ false };
		std::uint64_t frames{ 0 };
		std::vector<PassStat> passes;
	};

	namespace detail
	{
		constexpr std::size_t kSlots = 4;
		constexpr std::uint64_t kNoToken = ~0ull;

		struct Sample
		{
			std::string name;
			winrt::com_ptr<ID3D11Query> begin;
			winrt::com_ptr<ID3D11Query> end;
			bool ended{ false };
		};

		struct Slot
		{
			winrt::com_ptr<ID3D11Query> disjoint;
			std::vector<Sample> samples;
			std::size_t used{ 0 };
			bool open{ false };
			bool pending{ false };
		};

		struct Accumulator
		{
			double milliseconds{ 0.0 };
			std::uint64_t calls{ 0 };
		};

		struct State
		{
			std::atomic<bool> enabled{ false };
			std::atomic<bool> hasOpenFrame{ false };
			std::mutex mutex;
			std::array<Slot, kSlots> slots;
			std::size_t current{ 0 };
			std::uint64_t frames{ 0 };
			std::map<std::string, Accumulator> totals;
		};

		inline State& GetState()
		{
			static State state;
			return state;
		}

		inline winrt::com_ptr<ID3D11Query> MakeQuery(D3D11_QUERY a_type)
		{
			winrt::com_ptr<ID3D11Query> query;
			D3D11_QUERY_DESC desc{ a_type, 0 };
			globals::d3d::device->CreateQuery(&desc, query.put());
			return query;
		}

		/** @brief Adds a finished frame's passes to the totals; returns false while the GPU has not finished it. */
		inline bool TryResolve(State& a_state, Slot& a_slot)
		{
			D3D11_QUERY_DATA_TIMESTAMP_DISJOINT disjoint{};
			if (globals::d3d::context->GetData(a_slot.disjoint.get(), &disjoint, sizeof(disjoint), D3D11_ASYNC_GETDATA_DONOTFLUSH) != S_OK)
				return false;
			if (!disjoint.Disjoint && disjoint.Frequency != 0) {
				for (std::size_t i = 0; i < a_slot.used; ++i) {
					Sample& sample = a_slot.samples[i];
					std::uint64_t begin = 0, end = 0;
					if (!sample.ended ||
						globals::d3d::context->GetData(sample.begin.get(), &begin, sizeof(begin), 0) != S_OK ||
						globals::d3d::context->GetData(sample.end.get(), &end, sizeof(end), 0) != S_OK)
						continue;
					Accumulator& total = a_state.totals[sample.name];
					total.milliseconds += static_cast<double>(end - begin) * 1000.0 / static_cast<double>(disjoint.Frequency);
					++total.calls;
				}
				++a_state.frames;
			}
			return true;
		}
	}

	/** @brief Starts or stops measuring; stopping only raises a flag, because the context belongs to the thread that issues passes and NextFrame() closes the frames there. */
	inline void SetEnabled(bool a_enabled)
	{
		detail::GetState().enabled.store(a_enabled, std::memory_order_relaxed);
	}

	inline void Reset()
	{
		auto& state = detail::GetState();
		const std::scoped_lock lock(state.mutex);
		state.totals.clear();
		state.frames = 0;
	}

	/** @brief Call once per frame, on the thread that issues the passes; closes the open frame, resolves finished ones and opens the next. */
	inline void NextFrame()
	{
		auto& state = detail::GetState();
		if (!globals::d3d::device || !globals::d3d::context)
			return;
		if (!state.enabled.load(std::memory_order_relaxed)) {
			if (!state.hasOpenFrame.exchange(false))
				return;
			const std::scoped_lock lock(state.mutex);
			for (auto& slot : state.slots) {
				if (slot.open)
					globals::d3d::context->End(slot.disjoint.get());
				slot.open = false;
				slot.pending = false;
				slot.used = 0;
			}
			return;
		}
		const std::scoped_lock lock(state.mutex);
		detail::Slot& finished = state.slots[state.current];
		if (finished.open) {
			globals::d3d::context->End(finished.disjoint.get());
			finished.open = false;
			finished.pending = true;
		}
		for (auto& slot : state.slots) {
			if (slot.pending && detail::TryResolve(state, slot)) {
				slot.pending = false;
				slot.used = 0;
			}
		}
		state.current = (state.current + 1) % detail::kSlots;
		detail::Slot& next = state.slots[state.current];
		if (next.pending)
			return;
		if (!next.disjoint)
			next.disjoint = detail::MakeQuery(D3D11_QUERY_TIMESTAMP_DISJOINT);
		if (!next.disjoint)
			return;
		globals::d3d::context->Begin(next.disjoint.get());
		state.hasOpenFrame.store(true, std::memory_order_relaxed);
		next.open = true;
		next.used = 0;
	}

	/** @brief Marks the start of a pass; returns a token for End(), or a value End() ignores when nothing is measured. */
	[[nodiscard]] inline std::uint64_t Begin(std::string_view a_name)
	{
		auto& state = detail::GetState();
		if (!state.enabled.load(std::memory_order_relaxed))
			return detail::kNoToken;
		const std::scoped_lock lock(state.mutex);
		detail::Slot& slot = state.slots[state.current];
		if (!slot.open)
			return detail::kNoToken;
		if (slot.used == slot.samples.size())
			slot.samples.emplace_back();
		detail::Sample& sample = slot.samples[slot.used];
		if (!sample.begin)
			sample.begin = detail::MakeQuery(D3D11_QUERY_TIMESTAMP);
		if (!sample.end)
			sample.end = detail::MakeQuery(D3D11_QUERY_TIMESTAMP);
		if (!sample.begin || !sample.end)
			return detail::kNoToken;
		sample.name.assign(a_name);
		sample.ended = false;
		globals::d3d::context->End(sample.begin.get());
		return (static_cast<std::uint64_t>(state.current) << 32) | slot.used++;
	}

	inline void End(std::uint64_t a_token)
	{
		if (a_token == detail::kNoToken)
			return;
		auto& state = detail::GetState();
		const std::scoped_lock lock(state.mutex);
		detail::Slot& slot = state.slots[a_token >> 32];
		const std::size_t index = a_token & 0xFFFFFFFFull;
		if (!slot.open || index >= slot.used)
			return;
		globals::d3d::context->End(slot.samples[index].end.get());
		slot.samples[index].ended = true;
	}

	[[nodiscard]] inline Stats GetStats()
	{
		auto& state = detail::GetState();
		const std::scoped_lock lock(state.mutex);
		Stats stats;
		stats.enabled = state.enabled.load(std::memory_order_relaxed);
		stats.frames = state.frames;
		if (state.frames == 0)
			return stats;
		for (const auto& [name, total] : state.totals)
			stats.passes.push_back({ name, total.milliseconds / static_cast<double>(state.frames), static_cast<double>(total.calls) / static_cast<double>(state.frames) });
		return stats;
	}
}
