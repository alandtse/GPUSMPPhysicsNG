#pragma once

#include <cstdlib>
#include <filesystem>
#include <string>

/** @brief Where the tests keep compiled shaders: GPUSMP_SHADER_CACHE_DIR when set (to build a cache to ship), else a temp folder. */
inline std::wstring TestShaderCacheDir()
{
	char* value = nullptr;
	std::size_t length = 0;
	std::string chosen;
	if (_dupenv_s(&value, &length, "GPUSMP_SHADER_CACHE_DIR") == 0 && value) {
		chosen = value;
		std::free(value);
	}
	return chosen.empty() ? (std::filesystem::temp_directory_path() / "GPUSMPPhysicsNG-test-shaders").wstring() : std::filesystem::path(chosen).wstring();
}
