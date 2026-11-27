// Unit tests for the renderer try-lock that keeps the plugin's context submissions from racing the mods that submit from
// worker threads (MuDynamicNormalMap and RaceMenu take the engine's own renderer lock).

#include "d3d/ContextLock.h"

#include <catch2/catch_test_macros.hpp>

#include <atomic>
#include <thread>

namespace
{
	/** @brief A stand-in for the engine's critical section: held by one owner, recursive per thread like the real one. */
	struct FakeLock
	{
		std::atomic<std::thread::id> owner{};
		std::atomic<int> recursion{ 0 };
		std::atomic<std::thread::id> lastLeave{};
	};

	bool TryEnter(void* a_lock) noexcept
	{
		auto& lock = *static_cast<FakeLock*>(a_lock);
		const std::thread::id self = std::this_thread::get_id();
		if (lock.owner.load() == std::thread::id{}) {
			lock.owner.store(self);
			lock.recursion.store(1);
			return true;
		}
		if (lock.owner.load() == self) {
			lock.recursion.fetch_add(1);
			return true;
		}
		return false;
	}

	void Leave(void* a_lock) noexcept
	{
		auto& lock = *static_cast<FakeLock*>(a_lock);
		lock.lastLeave.store(std::this_thread::get_id());
		lock.recursion.fetch_sub(1);
		if (lock.recursion.load() == 0)
			lock.owner.store(std::thread::id{});
	}

	/** @brief Installs a fresh stand-in lock for one case and uninstalls it on scope exit. */
	struct ScopedLock
	{
		FakeLock lock;
		ScopedLock() { SmpPhysics::ContextLock::SetLock(&lock, TryEnter, Leave); }
		~ScopedLock() { SmpPhysics::ContextLock::ClearLock(); }
		ScopedLock(const ScopedLock&) = delete;
		ScopedLock& operator=(const ScopedLock&) = delete;
	};
}

TEST_CASE("With no lock installed every attempt is admitted", "[contextlock]")
{
	SmpPhysics::ContextLock::ClearLock();
	SmpPhysics::ContextLock::ResetStats();
	REQUIRE_FALSE(SmpPhysics::ContextLock::Configured());
	{
		const SmpPhysics::ContextLock::Guard guard;
		REQUIRE(guard.Admitted());
	}
	const auto stats = SmpPhysics::ContextLock::GetStats();
	REQUIRE(stats.unlocked == 1);
	REQUIRE(stats.acquired == 0);
	REQUIRE(stats.busy == 0);
}

TEST_CASE("An installed lock is taken and released", "[contextlock]")
{
	ScopedLock scoped;
	SmpPhysics::ContextLock::ResetStats();
	REQUIRE(SmpPhysics::ContextLock::Configured());
	{
		const SmpPhysics::ContextLock::Guard guard;
		REQUIRE(guard.Admitted());
		REQUIRE(scoped.lock.recursion.load() == 1);
	}
	REQUIRE(scoped.lock.recursion.load() == 0);
	REQUIRE(scoped.lock.owner.load() == std::thread::id{});
	REQUIRE(SmpPhysics::ContextLock::GetStats().acquired == 1);
}

TEST_CASE("A lock another thread holds is refused rather than waited on", "[contextlock]")
{
	ScopedLock scoped;
	SmpPhysics::ContextLock::ResetStats();

	// Stand in for MuDynamicNormalMap's worker: hold the engine's renderer lock on its own thread.
	std::atomic<bool> held{ false };
	std::atomic<bool> release{ false };
	std::thread worker([&] {
		REQUIRE(TryEnter(&scoped.lock));
		held.store(true);
		while (!release.load())
			std::this_thread::yield();
		Leave(&scoped.lock);
	});
	while (!held.load())
		std::this_thread::yield();

	bool admitted = true;
	{
		const SmpPhysics::ContextLock::Guard guard;
		admitted = guard.Admitted();
	}
	REQUIRE_FALSE(admitted);
	REQUIRE(SmpPhysics::ContextLock::GetStats().busy == 1);
	REQUIRE(SmpPhysics::ContextLock::GetStats().acquired == 0);
	REQUIRE(scoped.lock.recursion.load() == 1);

	release.store(true);
	worker.join();
	REQUIRE(scoped.lock.recursion.load() == 0);

	// Once the other thread is done the next frame is admitted again.
	{
		const SmpPhysics::ContextLock::Guard guard;
		REQUIRE(guard.Admitted());
	}
	REQUIRE(SmpPhysics::ContextLock::GetStats().acquired == 1);
}

TEST_CASE("Nesting on the owning thread is admitted, as the engine's recursive section allows", "[contextlock]")
{
	ScopedLock scoped;
	SmpPhysics::ContextLock::ResetStats();
	// The render thread already holds the lock while it draws the world, so DebugLines and the bone hook nest inside it.
	{
		const SmpPhysics::ContextLock::Guard outer;
		REQUIRE(outer.Admitted());
		{
			const SmpPhysics::ContextLock::Guard inner;
			REQUIRE(inner.Admitted());
			REQUIRE(scoped.lock.recursion.load() == 2);
		}
		REQUIRE(scoped.lock.recursion.load() == 1);
	}
	REQUIRE(scoped.lock.recursion.load() == 0);
	REQUIRE(SmpPhysics::ContextLock::GetStats().acquired == 2);
}

TEST_CASE("An incomplete lock triple leaves everything unlocked", "[contextlock]")
{
	FakeLock lock;
	SmpPhysics::ContextLock::ResetStats();
	SmpPhysics::ContextLock::SetLock(&lock, TryEnter, nullptr);
	REQUIRE_FALSE(SmpPhysics::ContextLock::Configured());
	{
		const SmpPhysics::ContextLock::Guard guard;
		REQUIRE(guard.Admitted());
	}
	REQUIRE(SmpPhysics::ContextLock::GetStats().unlocked == 1);
	SmpPhysics::ContextLock::ClearLock();
}
