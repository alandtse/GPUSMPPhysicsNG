#pragma once

#include "Features/SmpPhysics.h"

#include <filesystem>

namespace PluginSettings
{
	/** @brief Data/SKSE/Plugins/GPUSMPPhysicsNG/Settings.json; written with defaults the first time the game runs without it. */
	std::filesystem::path File();
	/** @brief Data/SKSE/Plugins/GPUSMPPhysicsNG/Shaders, the HLSL sources the solver compiles at start. */
	std::filesystem::path ShaderRoot();
	/** @brief Data/SKSE/Plugins/GPUSMPPhysicsNG/ShaderCache, compiled shader blobs kept between runs so a launch skips the compile. */
	std::filesystem::path ShaderCacheRoot();
	/** @brief Data/SKSE/Plugins/GPUSMPPhysicsNG/Overrides.json, the rules that keep physics files with hdtSMP64; written when a rule changes. */
	std::filesystem::path OverridesFile();
	void SaveOverrides(SmpPhysicsFeature& a_feature);
	void Load(SmpPhysicsFeature& a_feature);
	void Save(SmpPhysicsFeature& a_feature);
}
