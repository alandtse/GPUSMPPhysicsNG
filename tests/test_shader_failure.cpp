// Unit tests for the shader failure text the menu shows (src/Utils/ShaderFailure.h).

#include "Utils/D3D.h"
#include "Utils/ShaderFailure.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <filesystem>
#include <fstream>

TEST_CASE("The first compiler error is picked out and its folder dropped", "[shader_failure]")
{
	const std::string message =
		"Shader compilation failed:\n\n"
		"F:\\repos\\shaders\\SmpPhysics\\DebugLines.hlsl(116,21-31): error X3004: undeclared identifier 'kNoiseScale'\n"
		"F:\\repos\\shaders\\SmpPhysics\\DebugLines.hlsl(120,5-9): error X3000: syntax error\n";
	CHECK(Util::FirstShaderErrorLine(message) == "DebugLines.hlsl(116,21-31): error X3004: undeclared identifier 'kNoiseScale'");
}

TEST_CASE("A message with no error marker falls back to its first line", "[shader_failure]")
{
	CHECK(Util::FirstShaderErrorLine("\n  \nsomething went wrong  \r\nsecond line\n") == "something went wrong");
	CHECK(Util::FirstShaderErrorLine("").empty());
}

TEST_CASE("A slash separated path loses its folder too", "[shader_failure]")
{
	CHECK(Util::FirstShaderErrorLine("a/b/X.hlsl(1,1): error X1: bad") == "X.hlsl(1,1): error X1: bad");
}

TEST_CASE("A failed compile is described by file, entry and detail", "[shader_failure]")
{
	CHECK(Util::DescribeShaderFailure("SmpPhysics/DebugLines.hlsl", "PSMain", "error X3004: undeclared identifier") ==
		  "SmpPhysics/DebugLines.hlsl (PSMain): error X3004: undeclared identifier");
	CHECK(Util::DescribeShaderFailure("A.hlsl", "main", "") == "A.hlsl (main): compile failed");
}

TEST_CASE("A shader that fails to compile is listed for the menu until the failures are cleared", "[shader_failure][d3d]")
{
	const auto root = std::filesystem::temp_directory_path() / "GPUSMPPhysicsNG-shader-failure-test";
	std::filesystem::remove_all(root);
	std::filesystem::create_directories(root);
	std::ofstream(root / "Broken.hlsl") << "[numthreads(1, 1, 1)] void Main() { missingName = 1; }\n";
	Util::SetShaderRoot(root.wstring());
	Util::SetShaderCacheDir(L"");

	REQUIRE(Util::GetShaderFailures().empty());
	CHECK_FALSE(Util::CompileShaderBlob(L"Broken.hlsl", {}, "cs_5_0", "Main"));
	CHECK_FALSE(Util::CompileShaderBlob(L"Absent.hlsl", {}, "cs_5_0", "Main"));

	const std::vector<std::string> failures = Util::GetShaderFailures();
	REQUIRE(failures.size() == 2);
	CHECK(std::ranges::any_of(failures, [](const std::string& a_line) { return a_line.starts_with("Absent.hlsl (Main): the shader file does not exist"); }));
	CHECK(std::ranges::any_of(failures, [](const std::string& a_line) { return a_line.starts_with("Broken.hlsl (Main): Broken.hlsl(") && a_line.find("missingName") != std::string::npos; }));

	Util::ClearShaderFailures();
	CHECK(Util::GetShaderFailures().empty());
	std::filesystem::remove_all(root);
}
