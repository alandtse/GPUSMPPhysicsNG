#pragma once

#include <cstdint>

/**
 * @brief The engine's renderer lock, which every mod that drives the game's immediate context from its own thread also
 *        takes.
 *
 * The plugin installs the real critical section; a test installs a stand-in. Until one is installed every attempt is
 * admitted, so a plugin that has not reached kDataLoaded records exactly as before.
 */
namespace SmpPhysics::ContextLock
{
	using TryEnterProc = bool (*)(void* a_lock) noexcept;
	using LeaveProc = void (*)(void* a_lock) noexcept;

	struct Stats
	{
		std::uint64_t acquired{ 0 };  ///< Attempts that took the lock and recorded.
		std::uint64_t busy{ 0 };      ///< Attempts that found another thread submitting, so their work was skipped.
		std::uint64_t unlocked{ 0 };  ///< Attempts that ran with no lock installed.
	};

	/** @brief Installs a_lock and the pair that enters and leaves it; an incomplete triple leaves everything unlocked. */
	void SetLock(void* a_lock, TryEnterProc a_tryEnter, LeaveProc a_leave);
	/** @brief Uninstalls the lock, so the next attempts are admitted without one. */
	void ClearLock();
	[[nodiscard]] bool Configured();

	void ResetStats();
	[[nodiscard]] Stats GetStats();

	/**
	 * @brief Whether the calling thread is inside a Guard that took the lock, or no lock is installed.
	 *
	 * For context work that has no Guard of its own and must know whether the caller already holds the lock, such as a
	 * check that copies and maps on the immediate context.
	 */
	[[nodiscard]] bool HeldByCurrentThread() noexcept;

	/**
	 * @brief Takes the lock without waiting for it, so a frame can never hold up the game.
	 *
	 * While the guard lives the caller owns the context; not admitted means another thread is submitting and the
	 * caller must skip this frame's GPU work. Recursive per thread, so nesting on the render thread is safe.
	 */
	class Guard
	{
	public:
		Guard();
		~Guard();
		Guard(const Guard&) = delete;
		Guard& operator=(const Guard&) = delete;
		Guard(Guard&&) = delete;
		Guard& operator=(Guard&&) = delete;

		/** @return Whether the caller may record; false means it must skip this frame's GPU work. */
		[[nodiscard]] bool Admitted() const noexcept { return admitted; }

	private:
		void* lock{ nullptr };
		LeaveProc leave{ nullptr };
		const void* previousHeld{ nullptr };
		bool admitted{ false };
	};
}
