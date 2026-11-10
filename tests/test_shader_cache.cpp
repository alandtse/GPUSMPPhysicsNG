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

	Util::SetShaderCacheDir(L"");
	std::filesystem::remove_all(root);
}
