// SPDX-License-Identifier: GPL-3.0-or-later WITH LicenseRef-Modding-Exception

#include "pch.h"

#include "Features/Wind/Wind.h"
#include "Globals.h"
#include "Utils/D3D.h"
#include "plugin/BoneUploadHook.h"
#include "plugin/Instance.h"
#include "plugin/Settings.h"
#include "plugin/UiModel.h"

#include <fstream>

SmpPhysicsFeature& Instance()
{
	static SmpPhysicsFeature instance;
	return instance;
}

namespace
{
	void BindGameGlobals()
	{
		using namespace globals::game;
		isVR = REL::Module::IsVR();
		player = RE::PlayerCharacter::GetSingleton();
		playerCamera = RE::PlayerCamera::GetSingleton();
		shadowState = RE::BSGraphics::RendererShadowState::GetSingleton();
		renderer = RE::BSGraphics::Renderer::GetSingleton();
		deltaTime = reinterpret_cast<float*>(REL::RelocationID(523660, 410199).address());
		globals::d3d::device = Util::AsReal(renderer->GetRuntimeData().forwarder);
		globals::d3d::context = Util::AsReal(renderer->GetRuntimeData().context);
	}

	void OnMessage(SKSE::MessagingInterface::Message* a_message)
	{
		if (!a_message)
			return;
		if (a_message->sender && _stricmp(a_message->sender, "SKSE") != 0) {
			Instance().OnPluginMessage(a_message);
			return;
		}
		switch (a_message->type) {
		case SKSE::MessagingInterface::kPostLoad:
			PluginSettings::Load(Instance());
			if (!SKSE::GetMessagingInterface()->RegisterListener("hdtsmp64", OnMessage))
				logger::warn("hdtSMP64's startup message could not be subscribed to; nothing will be claimed");
			break;
		case SKSE::MessagingInterface::kPostPostLoad:
			Util::SetShaderRoot(PluginSettings::ShaderRoot().wstring());
			BoneUploadHook::Install();
			break;
		case SKSE::MessagingInterface::kDataLoaded:
			BindGameGlobals();
			globals::features::wind.ConnectCommunityShaders();
			Instance().DataLoaded();
			PluginUi::Register();
			break;
		default:
			break;
		}
	}
}

SKSEPluginLoad(const SKSE::LoadInterface* a_skse)
{
	SKSE::Init(a_skse);
	smplog::GetSink() = [](smplog::Level a_level, const std::string& a_message) {
		switch (a_level) {
		case smplog::Level::Debug:
			logger::debug("{}", a_message);
			break;
		case smplog::Level::Info:
			logger::info("{}", a_message);
			break;
		case smplog::Level::Warn:
			logger::warn("{}", a_message);
			break;
		case smplog::Level::Error:
			logger::error("{}", a_message);
			break;
		}
	};
	SKSE::AllocTrampoline(64);

	auto* messaging = SKSE::GetMessagingInterface();
	if (!messaging || !messaging->RegisterListener(OnMessage)) {
		SKSE::stl::report_and_fail("failed to register the SKSE message listener"sv);
	}
	logger::info("GPUSMPPhysicsNG loaded");
	return true;
}
