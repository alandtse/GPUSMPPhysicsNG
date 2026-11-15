#include "plugin/Settings.h"

#include <fstream>

namespace
{
	std::filesystem::path PluginFolder()
	{
		return std::filesystem::path("Data") / "SKSE" / "Plugins" / "GPUSMPPhysicsNG";
	}
}

std::filesystem::path PluginSettings::File()
{
	return PluginFolder() / "Settings.json";
}

std::filesystem::path PluginSettings::ShaderRoot()
{
	return PluginFolder() / "Shaders";
}

std::filesystem::path PluginSettings::ShaderCacheRoot()
{
	return PluginFolder() / "ShaderCache";
}

void PluginSettings::Load(SmpPhysicsFeature& a_feature)
{
	json document = json::object();
	if (std::ifstream stream(File()); stream) {
		try {
			stream >> document;
		} catch (const std::exception& error) {
			logger::warn("Settings.json could not be read ({}); using defaults", error.what());
			document = json::object();
		}
	}
	try {
		a_feature.LoadSettings(document);
	} catch (const std::exception& error) {
		logger::warn("Settings.json has a value of the wrong type ({}); using defaults", error.what());
		json defaults = json::object();
		a_feature.LoadSettings(defaults);
	}
	if (!std::filesystem::exists(File()))
		Save(a_feature);
}

void PluginSettings::Save(SmpPhysicsFeature& a_feature)
{
	json document = json::object();
	a_feature.SaveSettings(document);
	std::error_code error;
	std::filesystem::create_directories(PluginFolder(), error);
	if (error)
		logger::warn("Could not create {} to save Settings.json: {}", PluginFolder().string(), error.message());
	std::ofstream stream(File());
	if (!stream) {
		logger::warn("Could not open {} to save the settings", File().string());
		return;
	}
	stream << document.dump(2);
	stream.flush();
	if (!stream)
		logger::warn("Writing {} failed", File().string());
}
