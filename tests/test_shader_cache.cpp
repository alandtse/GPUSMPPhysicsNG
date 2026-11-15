#include "Utils/D3D.h"

#include <catch2/catch_test_macros.hpp>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <string>
#include <thread>

namespace
{
	void Write(const std::filesystem::path& a_path, const std::string& a_text)
	{
		std::ofstream(a_path, std::ios::trunc) << a_text;
	}

	std::size_t CacheFiles(const std::filesystem::path& a_dir, const char* a_extension)
	{
		std::size_t count = 0;
		for (const auto& entry : std::filesystem::directory_iterator(a_dir))
			count += entry.path().extension() == a_extension ? 1 : 0;
		return count;
	}

	std::string Bytes(const winrt::com_ptr<ID3DBlob>& a_blob)
	{
		return std::string(static_cast<const char*>(a_blob->GetBufferPointer()), a_blob->GetBufferSize());
	}

	std::string Kernel(const char* a_value)
	{
		return std::string("#include \"Value.hlsli\"\nRWStructuredBuffer<uint> Out : register(u0);\n[numthreads(1, 1, 1)] void Main() { Out[0] = ") + a_value + "; }\n";
	}
}

TEST_CASE("A compiled shader is read back from the cache until its source or an include changes", "[d3d][shader_cache]")
{
	const auto root = std::filesystem::temp_directory_path() / "GPUSMPPhysicsNG-shader-cache-test";
	std::filesystem::remove_all(root);
	std::filesystem::create_directories(root / "shaders");
	Util::SetShaderRoot((root / "shaders").wstring());
	Util::SetShaderCacheDir((root / "cache").wstring());

	Write(root / "shaders" / "Value.hlsli", "static const uint kValue = 7;\n");
	Write(root / "shaders" / "Test.hlsl", Kernel("kValue"));

	const auto first = Util::CompileShaderBlob(L"Test.hlsl", {}, "cs_5_0", "Main");
	REQUIRE(first);
	REQUIRE(CacheFiles(root / "cache", ".bin") == 1);
	REQUIRE(CacheFiles(root / "cache", ".dep") == 1);

	const auto cached = Util::CompileShaderBlob(L"Test.hlsl", {}, "cs_5_0", "Main");
	REQUIRE(cached);
	CHECK(Bytes(cached) == Bytes(first));

	std::this_thread::sleep_for(std::chrono::milliseconds(50));
	Write(root / "shaders" / "Value.hlsli", "static const uint kValue = 9;\n");
	const auto afterInclude = Util::CompileShaderBlob(L"Test.hlsl", {}, "cs_5_0", "Main");
	REQUIRE(afterInclude);
	CHECK(Bytes(afterInclude) != Bytes(first));

	std::this_thread::sleep_for(std::chrono::milliseconds(50));
	Write(root / "shaders" / "Test.hlsl", Kernel("kValue + 1"));
	const auto afterSource = Util::CompileShaderBlob(L"Test.hlsl", {}, "cs_5_0", "Main");
	REQUIRE(afterSource);
	CHECK(Bytes(afterSource) != Bytes(afterInclude));

	// A cache built under one folder serves the same shaders copied to another, as a shipped cache must.
	const std::size_t before = CacheFiles(root / "cache", ".bin");
	std::filesystem::copy(root / "shaders", root / "moved", std::filesystem::copy_options::recursive);
	Util::SetShaderRoot((root / "moved").wstring());
	const auto moved = Util::CompileShaderBlob(L"Test.hlsl", {}, "cs_5_0", "Main");
	REQUIRE(moved);
	CHECK(Bytes(moved) == Bytes(afterSource));
	CHECK(CacheFiles(root / "cache", ".bin") == before);

	Util::SetShaderCacheDir(L"");
	std::filesystem::remove_all(root);
}

TEST_CASE("A corrupted cached blob is recompiled rather than served", "[d3d][shader_cache]")
{
	const auto root = std::filesystem::temp_directory_path() / "GPUSMPPhysicsNG-shader-cache-corrupt-test";
	std::filesystem::remove_all(root);
	std::filesystem::create_directories(root / "shaders");
	Util::SetShaderRoot((root / "shaders").wstring());
	Util::SetShaderCacheDir((root / "cache").wstring());
	Write(root / "shaders" / "Value.hlsli", "static const uint kValue = 7;\n");
	Write(root / "shaders" / "Test.hlsl", Kernel("kValue"));

	const auto first = Util::CompileShaderBlob(L"Test.hlsl", {}, "cs_5_0", "Main");
	REQUIRE(first);
	for (const auto& entry : std::filesystem::directory_iterator(root / "cache")) {
		if (entry.path().extension() == ".bin") {
			std::ofstream damaged(entry.path(), std::ios::binary | std::ios::trunc);
			damaged << "not bytecode";
		}
	}

	const auto recompiled = Util::CompileShaderBlob(L"Test.hlsl", {}, "cs_5_0", "Main");
	REQUIRE(recompiled);
	CHECK(Bytes(recompiled) == Bytes(first));

	Util::SetShaderCacheDir(L"");
	std::filesystem::remove_all(root);
}

TEST_CASE("A prewarm fills the cache on background threads and finishes even when a shader is missing", "[d3d][shader_cache]")
{
	const auto root = std::filesystem::temp_directory_path() / "GPUSMPPhysicsNG-shader-prewarm-test";
	std::filesystem::remove_all(root);
	std::filesystem::create_directories(root / "shaders");
	Util::SetShaderRoot((root / "shaders").wstring());
	Util::SetShaderCacheDir((root / "cache").wstring());
	Write(root / "shaders" / "Pair.hlsl",
		"RWStructuredBuffer<uint> Out : register(u0);\n"
		"[numthreads(1, 1, 1)] void First() { Out[0] = 1; }\n"
		"[numthreads(1, 1, 1)] void Second() { Out[0] = 2; }\n");

	Util::StartShaderPrewarm({ { L"Pair.hlsl", "cs_5_0", "First" }, { L"Pair.hlsl", "cs_5_0", "Second" }, { L"Absent.hlsl", "cs_5_0", "Main" } });
	Util::WaitForShaderPrewarm();

	const auto status = Util::GetShaderPrewarmStatus();
	CHECK(status.total == 3);
	CHECK(status.done == 3);
	CHECK_FALSE(status.running);
	CHECK(CacheFiles(root / "cache", ".bin") == 2);

	Util::SetShaderCacheDir(L"");
	std::filesystem::remove_all(root);
}
