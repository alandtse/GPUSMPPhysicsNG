#pragma once

#include <string_view>

namespace SmpPhysics
{
	/** @brief The solver settings of hdtSMP64 this solver follows, with hdtSMP64's own defaults. */
	struct HdtSolverConfig
	{
		int iterations{ 10 };
		float erp{ 0.2f };
		int minFps{ 60 };  ///< The fixed step is one over this.
		int maxSubSteps{ 4 };
	};

	/**
	 * @brief Reads the "solver" section of hdtSMP64's configs.json, then lets userConfigs.json override it, as hdtSMP64 loads them.
	 * @details Values are clamped to the ranges hdtSMP64 accepts. Text that does not parse leaves the values it would have changed.
	 */
	[[nodiscard]] HdtSolverConfig ParseHdtSolverConfig(std::string_view a_defaults, std::string_view a_user);
}
