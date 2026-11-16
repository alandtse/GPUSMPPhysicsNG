#pragma once

#include <Windows.h>

namespace GuardedCall
{
	using Function = bool (*)(void*);

	/** @brief Calls a_function and returns false with a_faulted set when it raises a hardware exception; kept free of C++ objects so __try compiles. */
	[[nodiscard]] inline bool Invoke(Function a_function, void* a_context, bool& a_faulted) noexcept
	{
		__try {
			return a_function(a_context);
		} __except (EXCEPTION_EXECUTE_HANDLER) {
			a_faulted = true;
			return false;
		}
	}
}
