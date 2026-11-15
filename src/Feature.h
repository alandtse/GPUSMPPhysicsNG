#pragma once

#include "pch.h"

struct Feature
{
	virtual ~Feature() = default;
	bool loaded{ true };
	virtual std::string GetShortName() = 0;
	virtual void OnPluginMessage(const SKSE::MessagingInterface::Message*) {}
	virtual void DataLoaded() {}
	virtual void OnDeferredGeometryEnd() {}
	virtual void OnBoneMatricesUploaded(RE::NiSkinInstance*) {}
	virtual void LoadSettings(json&) {}
	virtual void SaveSettings(json&) {}
	virtual void RegisterUxActions() {}
};
