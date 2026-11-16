#include "Features/Wind/Wind.h"

#include "Features/Wind/CsBuild.h"
#include "Features/Wind/GuardedCall.h"
#include "VRAPI/CSinterface001.h"

#include <atomic>
#include <cmath>
#include <numbers>
#include <string>
#include <vector>

CSPluginAPI::ICSInterface001* g_CSInterface = nullptr;

namespace
{
	std::atomic_bool g_publishedWindFaulted{ false };

	std::string CommunityShadersFileVersion()
	{
		char path[MAX_PATH]{};
		const auto module = GetModuleHandleA("CommunityShaders.dll");
		if (!module || !GetModuleFileNameA(module, path, MAX_PATH))
			return {};
		const DWORD size = GetFileVersionInfoSizeA(path, nullptr);
		if (!size)
			return {};
		std::vector<char> data(size);
		void* value = nullptr;
		UINT length = 0;
		if (!GetFileVersionInfoA(path, 0, size, data.data()) || !VerQueryValueA(data.data(), "\\StringFileInfo\\040904b0\\FileVersion", &value, &length) || !value)
			return {};
		return static_cast<const char*>(value);
	}
}

CSPluginAPI::ICSInterface001* CSPluginAPI::GetCSInterface001()
{
	if (g_CSInterface)
		return g_CSInterface;
	if (CsBuild::IsExpanded(CommunityShadersFileVersion()))
		return nullptr;
	const auto* messaging = SKSE::GetMessagingInterface();
	CSMessage message;
	if (!messaging || !messaging->Dispatch(CSMessage::kMessage_GetInterface, &message, sizeof(CSMessage), CSPluginName) || !message.GetApiFunction)
		return nullptr;
	g_CSInterface = static_cast<ICSInterface001*>(message.GetApiFunction(CSInterfaceRevision));
	return g_CSInterface;
}

void Wind::ConnectCommunityShaders()
{
	if (CSPluginAPI::GetCSInterface001())
		logger::info("[GPU SMP Physics] Wind comes from Community Shaders' published wind.");
	else
		logger::info("[GPU SMP Physics] Community Shaders' wind is not available (not installed, or Community Shaders Expanded); wind comes from the weather.");
}

bool Wind::SamplePublishedWind(std::span<const float3> a_worldPositions, std::span<PublishedWindSample> a_samples) const noexcept
{
	if (auto* api = g_CSInterface; api && !g_publishedWindFaulted && a_samples.size() >= a_worldPositions.size() && a_worldPositions.size() <= CSPluginAPI::CSWindMaximumBatchSize) {
		thread_local std::vector<CSPluginAPI::WindSampleWithHavokExclusion> published;
		published.resize(a_worldPositions.size());
		static_assert(sizeof(float3) == sizeof(CSPluginAPI::WindVector));
		struct Call
		{
			CSPluginAPI::ICSInterface001* api;
			const CSPluginAPI::WindVector* positions;
			CSPluginAPI::WindSampleWithHavokExclusion* samples;
			std::uint32_t count;
		} call{ api, reinterpret_cast<const CSPluginAPI::WindVector*>(a_worldPositions.data()), published.data(), static_cast<std::uint32_t>(published.size()) };
		bool faulted = false;
		const bool sampled = GuardedCall::Invoke([](void* a_context) {
			const auto& c = *static_cast<Call*>(a_context);
			return c.api->SampleWindExcludingHavokImpulses(c.positions, c.samples, c.count);
		},
			&call, faulted);
		if (faulted) {
			g_publishedWindFaulted = true;
			logger::warn("[GPU SMP Physics] Community Shaders' wind function faulted; wind comes from the weather.");
		}
		if (sampled) {
			for (std::size_t index = 0; index < published.size(); ++index) {
				const auto& wind = published[index].windExcludingHavokImpulses;
				a_samples[index].windExcludingHavokImpulses = { wind.x, wind.y, wind.z };
			}
			return true;
		}
	}
	const auto* sky = RE::Sky::GetSingleton();
	const auto* player = RE::PlayerCharacter::GetSingleton();
	if (!sky || !player || !player->loadedData || a_samples.size() < a_worldPositions.size())
		return false;
	const auto* cell = player->parentCell;
	if (!cell || cell->IsInteriorCell())
		return false;

	constexpr float kWindAngleUnitsPerHalfTurn = 256.0f;  ///< The game stores a wind angle as 0..256 over a half turn.
	constexpr float kWindHeadingOffsetDegrees = 90.0f;
	const float degrees = (sky->currentWeather ? sky->currentWeather->data.windDirection : sky->windAngle) * 180.0f / kWindAngleUnitsPerHalfTurn;
	const float heading = (degrees - kWindHeadingOffsetDegrees) * std::numbers::pi_v<float> / 180.0f;
	const float3 wind{ -std::sin(heading) * sky->windSpeed, std::cos(heading) * sky->windSpeed, 0.0f };
	for (std::size_t index = 0; index < a_worldPositions.size(); ++index)
		a_samples[index].windExcludingHavokImpulses = wind;
	return true;
}
