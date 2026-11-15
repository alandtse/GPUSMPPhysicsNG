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
	if (std::ofstream stream(File()); stream)
		stream << document.dump(2);
}
