#include "d3d/ContextLock.h"

#include <atomic>

namespace SmpPhysics::ContextLock
{
	namespace
	{
		std::atomic<void*> g_lock{ nullptr };
		std::atomic<TryEnterProc> g_tryEnter{ nullptr };
		std::atomic<LeaveProc> g_leave{ nullptr };

		std::atomic<std::uint64_t> g_acquired{ 0 };
		std::atomic<std::uint64_t> g_busy{ 0 };
		std::atomic<std::uint64_t> g_unlocked{ 0 };

		// The guard the calling thread holds, so work nested inside a guarded region can see that it may record.
		thread_local const void* g_held{ nullptr };
	}

	void SetLock(void* a_lock, TryEnterProc a_tryEnter, LeaveProc a_leave)
	{
		g_lock.store(a_lock, std::memory_order_release);
		g_tryEnter.store(a_tryEnter, std::memory_order_release);
		g_leave.store(a_leave, std::memory_order_release);
		if (!a_lock || !a_tryEnter || !a_leave) {
			g_lock.store(nullptr, std::memory_order_release);
			g_tryEnter.store(nullptr, std::memory_order_release);
			g_leave.store(nullptr, std::memory_order_release);
		}
	}

	void ClearLock()
	{
		SetLock(nullptr, nullptr, nullptr);
	}

	bool Configured()
	{
		return g_lock.load(std::memory_order_acquire) != nullptr;
	}

	bool HeldByCurrentThread() noexcept
	{
		return !Configured() || g_held != nullptr;
	}

	void ResetStats()
	{
		g_acquired.store(0, std::memory_order_relaxed);
		g_busy.store(0, std::memory_order_relaxed);
		g_unlocked.store(0, std::memory_order_relaxed);
	}

	Stats GetStats()
	{
		return Stats{
			g_acquired.load(std::memory_order_relaxed),
			g_busy.load(std::memory_order_relaxed),
			g_unlocked.load(std::memory_order_relaxed),
		};
	}

	Guard::Guard()
	{
		void* const installed = g_lock.load(std::memory_order_acquire);
		if (!installed) {
			admitted = true;
			g_unlocked.fetch_add(1, std::memory_order_relaxed);
			return;
		}
		const TryEnterProc enter = g_tryEnter.load(std::memory_order_acquire);
		const LeaveProc exit = g_leave.load(std::memory_order_acquire);
		if (!enter || !exit) {
			admitted = true;
			g_unlocked.fetch_add(1, std::memory_order_relaxed);
			return;
		}
		if (!enter(installed)) {
			g_busy.fetch_add(1, std::memory_order_relaxed);
			return;
		}
		this->lock = installed;
		this->leave = exit;
		previousHeld = g_held;
		admitted = true;
		g_held = installed;
		g_acquired.fetch_add(1, std::memory_order_relaxed);
	}

	Guard::~Guard()
	{
		if (lock && leave) {
			g_held = previousHeld;
			leave(lock);
		}
	}
}
