// SPDX-License-Identifier: GPL-3.0-or-later WITH LicenseRef-Modding-Exception

#include "pch.h"

#include "Features/Wind/Wind.h"
#include "Globals.h"
#include "Utils/D3D.h"
#include "Utils/TestBench/IdleCache.h"
#include "d3d/ContextLock.h"
#include "d3d/GpuSolver.h"
#include "engine/GpuSkinning.h"
#include "plugin/BoneUploadHook.h"
#include "plugin/DevBench.h"
#include "plugin/Instance.h"
#include "plugin/OverlayHook.h"
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
	/**
	 * @brief Points the solver at the engine's own renderer lock, which mods submitting from worker threads also take.
	 */
	void BindContextLock()
	{
		static_assert(sizeof(REX::W32::CRITICAL_SECTION) == sizeof(::CRITICAL_SECTION));
		const auto tryEnter = [](void* a_lock) noexcept {
			return ::TryEnterCriticalSection(reinterpret_cast<::CRITICAL_SECTION*>(a_lock)) != FALSE;
		};
		const auto leave = [](void* a_lock) noexcept {
			::LeaveCriticalSection(reinterpret_cast<::CRITICAL_SECTION*>(a_lock));
		};
		SmpPhysics::ContextLock::SetLock(&globals::game::renderer->GetLock(), tryEnter, leave);
	}

	void BindGameGlobals()
	{
		using namespace globals::game;
		isVR = REL::Module::IsVR();
		player = RE::PlayerCharacter::GetSingleton();
		playerCamera = RE::PlayerCamera::GetSingleton();
		shadowState = RE::BSGraphics::RendererShadowState::GetSingleton();
		renderer = RE::BSGraphics::Renderer::GetSingleton();
		perFrame = REL::Relocation<ID3D11Buffer**>{ REL::RelocationID(524768, 411384) };
		deltaTime = reinterpret_cast<float*>(REL::RelocationID(523660, 410199).address());
		globals::d3d::device = Util::AsReal(renderer->GetRuntimeData().forwarder);
		globals::d3d::context = Util::AsReal(renderer->GetRuntimeData().context);
		BindContextLock();
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
			Util::SetShaderCacheDir(PluginSettings::ShaderCacheRoot().wstring());
			{
				auto jobs = SmpPhysics::GpuSolver::ShaderJobs();
				const auto skinning = SmpPhysics::GpuSkinning::ShaderJobs();
				jobs.insert(jobs.end(), skinning.begin(), skinning.end());
				Util::StartShaderPrewarm(std::move(jobs));
			}
			BoneUploadHook::Install();
			OverlayHook::Install();
			break;
		case SKSE::MessagingInterface::kDataLoaded:
			BindGameGlobals();
			globals::features::wind.ConnectCommunityShaders();
			Instance().DataLoaded();
			PluginUi::Register();
			DevBench::Install();
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
	constexpr std::size_t kTrampolineBytes = 128;  ///< Room for the hook branch stubs this plugin writes.
	SKSE::AllocTrampoline(kTrampolineBytes);
	TestBench::IdleCache::Install();

	auto* messaging = SKSE::GetMessagingInterface();
	if (!messaging || !messaging->RegisterListener(OnMessage)) {
		SKSE::stl::report_and_fail("failed to register the SKSE message listener"sv);
	}
	logger::info("GPUSMPPhysicsNG loaded");
	return true;
}
