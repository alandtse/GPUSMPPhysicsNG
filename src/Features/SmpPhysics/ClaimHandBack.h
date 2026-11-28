#pragma once

namespace SmpPhysics
{
	/** @brief Whether a claim the GPU cannot simulate goes back to hdtSMP64: its rig did not fit the solver, or the solver is gone. Culled claims are not failures. */
	[[nodiscard]] constexpr bool ShouldHandBack(bool a_gpuAvailable, bool a_registrationFailed) noexcept
	{
		return !a_gpuAvailable || a_registrationFailed;
	}

	/** @brief Whether a refused registration is the solver failing to take the rig; only a retryable refusal is not. */
	[[nodiscard]] constexpr bool RegistrationFailed(bool a_registered, bool a_retryable) noexcept
	{
		return !a_registered && !a_retryable;
	}
}
