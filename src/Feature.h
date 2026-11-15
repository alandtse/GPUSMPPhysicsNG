#pragma once

#include "pch.h"

namespace FeatureCategories
{
	inline constexpr std::string_view kCharacters = "Characters";
}

/** @brief Translation lookup collapses to the English default in the standalone plugin. */
inline std::string T(const char*, const char* a_default) { return a_default; }
#define TKEY(key) key

struct Feature
{
	virtual ~Feature() = default;
	bool loaded{ true };
	virtual std::string GetName() = 0;
	virtual std::string GetDisplayName() { return GetName(); }
	virtual std::string GetShortName() = 0;
	virtual std::string_view GetCategory() const { return {}; }
	virtual bool SupportsVR() { return true; }
	virtual std::pair<std::string, std::vector<std::string>> GetFeatureSummary() { return {}; }
	virtual void OnPluginMessage(const SKSE::MessagingInterface::Message*) {}
	virtual void DataLoaded() {}
	virtual void PostPostLoad() {}
	virtual void DrawSettings() {}
	virtual void ClearShaderCache() {}
	virtual bool WantsBoneUploadHook() const { return false; }
	virtual void OnDeferredGeometryEnd() {}
	virtual void OnBoneMatricesUploaded(RE::NiSkinInstance*) {}
	virtual void LoadSettings(json&) {}
	virtual void SaveSettings(json&) {}
	virtual void RestoreDefaultSettings() {}
	virtual void RegisterUxActions() {}
};
