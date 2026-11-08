#pragma once

#include <string_view>

namespace TestBench
{
	/** @brief The devbench owner name of the test crowd's commands and queries; it names no feature. */
	inline constexpr std::string_view kOwnerName = "TestCrowd";

	/** @brief Registers the test crowd commands and queries with devbench. */
	void RegisterUxActions();
}
