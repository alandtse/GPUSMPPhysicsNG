// Unit tests for reading hdtSMP64's solver settings from its configuration files.

#include "Features/SmpPhysics/HdtSolverConfig.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

TEST_CASE("hdtSMP64's solver settings default as hdtSMP64 defaults them", "[solver][config]")
{
	const SmpPhysics::HdtSolverConfig config = SmpPhysics::ParseHdtSolverConfig("", "not json");
	CHECK(config.iterations == 10);
	CHECK(config.minFps == 60);
	CHECK(config.maxSubSteps == 4);
	CHECK(config.erp == Catch::Approx(0.2f));
}

TEST_CASE("userConfigs.json overrides configs.json one value at a time", "[solver][config]")
{
	const char* defaults = R"({ "smp": {}, "solver": { "numIterations": 16, "erp": 0.25, "min-fps": 60, "maxSubSteps": 4 } })";
	const char* user =
		"\xEF\xBB\xBF"
		R"({ "solver": { "numIterations": 32, "min-fps": 90 } })";
	const SmpPhysics::HdtSolverConfig config = SmpPhysics::ParseHdtSolverConfig(defaults, user);
	CHECK(config.iterations == 32);
	CHECK(config.minFps == 90);
	CHECK(config.maxSubSteps == 4);
	CHECK(config.erp == Catch::Approx(0.25f));
}

TEST_CASE("hdtSMP64's solver settings are clamped to the ranges hdtSMP64 accepts", "[solver][config]")
{
	const SmpPhysics::HdtSolverConfig config =
		SmpPhysics::ParseHdtSolverConfig(R"({ "solver": { "numIterations": 1000, "erp": 0, "min-fps": 0, "maxSubSteps": "four" } })", "");
	CHECK(config.iterations == 128);
	CHECK(config.minFps == 1);
	CHECK(config.maxSubSteps == 4);
	CHECK(config.erp == Catch::Approx(0.01f));
}
