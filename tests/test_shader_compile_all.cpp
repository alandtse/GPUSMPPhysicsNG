// Compiles the graphics shaders no GPU test reaches, in the flat and VR variants the plugin builds them in.

#include "Utils/D3D.h"
#include "shader_cache_dir.h"

#include <catch2/catch_test_macros.hpp>

#include <filesystem>
#include <iostream>
#include <utility>
#include <vector>

namespace
{
	using Defines = std::vector<std::pair<const char*, const char*>>;

	bool Compiles(const wchar_t* a_path, Defines a_defines, const char* a_target, const char* a_entry, bool a_vr)
	{
		if (a_vr)
			a_defines.emplace_back("VR", "");
		return static_cast<bool>(Util::CompileShaderBlob(a_path, a_defines, a_target, a_entry));
	}
}

TEST_CASE("The skin palette and surface prep compute shaders compile", "[d3d][shader_compile]")
{
	logger::GetSink() = [](logger::Level, const std::string& a_message) { std::cerr << a_message << std::endl; };
	Util::SetShaderRoot(std::filesystem::path(SMPPHYSICS_SHADER_DIR).wstring());
	Util::SetShaderCacheDir(TestShaderCacheDir());
	CHECK(Compiles(L"SmpPhysics/SkinPaletteCS.hlsl", {}, "cs_5_0", "main", false));
	CHECK(Compiles(L"SmpPhysics/SkinPaletteBatchCS.hlsl", {}, "cs_5_0", "main", false));
	for (const char* entry : { "SurfaceBones", "SurfacePose", "SurfaceTriangles", "SurfaceLeaves", "SurfaceRefit" })
		CHECK(Compiles(L"SmpPhysics/SurfacePrepCS.hlsl", {}, "cs_5_0", entry, false));
}

TEST_CASE("The cloth triangle contact shaders compile", "[d3d][shader_compile]")
{
	logger::GetSink() = [](logger::Level, const std::string& a_message) { std::cerr << a_message << std::endl; };
	Util::SetShaderRoot(std::filesystem::path(SMPPHYSICS_SHADER_DIR).wstring());
	Util::SetShaderCacheDir(TestShaderCacheDir());
	for (const char* entry : { "ClothVertices", "ClothTriangles" })
		CHECK(Compiles(L"SmpPhysics/ContactCS.hlsl", {}, "cs_5_0", entry, false));
}

TEST_CASE("The debug line shader compiles both stages in the flat and VR variants", "[d3d][shader_compile]")
{
	logger::GetSink() = [](logger::Level, const std::string& a_message) { std::cerr << a_message << std::endl; };
	Util::SetShaderRoot(std::filesystem::path(SMPPHYSICS_SHADER_DIR).wstring());
	Util::SetShaderCacheDir(TestShaderCacheDir());
	for (const bool vr : { false, true }) {
		CHECK(Compiles(L"SmpPhysics/DebugLines.hlsl", { { "VSHADER", "" } }, "vs_5_0", "VSMain", vr));
		CHECK(Compiles(L"SmpPhysics/DebugLines.hlsl", { { "PSHADER", "" } }, "ps_5_0", "PSMain", vr));
	}
}
