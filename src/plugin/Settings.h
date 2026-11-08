#pragma once

#include "Features/SmpPhysics.h"

#include <filesystem>

namespace PluginSettings
{
	/** @brief Data/SKSE/Plugins/GPUSMPPhysicsNG/Settings.json; written with defaults the first time the game runs without it. */
	std::filesystem::path File();
	/** @brief Data/SKSE/Plugins/GPUSMPPhysicsNG/Shaders, the HLSL sources the solver compiles at start. */
	std::filesystem::path ShaderRoot();
	void Load(SmpPhysicsFeature& a_feature);
	void Save(SmpPhysicsFeature& a_feature);
}
