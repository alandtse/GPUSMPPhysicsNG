#include "Features/Wind/Wind.h"

#include "VRAPI/CSinterface001.h"

#include <cmath>
#include <vector>
#include <numbers>

CSPluginAPI::ICSInterface001* g_CSInterface = nullptr;

CSPluginAPI::ICSInterface001* CSPluginAPI::GetCSInterface001()
{
	if (g_CSInterface)
		return g_CSInterface;
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
		logger::info("[GPU SMP Physics] Community Shaders' wind is not available; wind comes from the weather.");
}

bool Wind::SamplePublishedWind(std::span<const float3> a_worldPositions, std::span<PublishedWindSample> a_samples) const noexcept
{
	if (auto* api = g_CSInterface; api && a_samples.size() >= a_worldPositions.size() && a_worldPositions.size() <= CSPluginAPI::CSWindMaximumBatchSize) {
		thread_local std::vector<CSPluginAPI::WindSampleWithHavokExclusion> published;
		published.resize(a_worldPositions.size());
		static_assert(sizeof(float3) == sizeof(CSPluginAPI::WindVector));
		if (api->SampleWindExcludingHavokImpulses(reinterpret_cast<const CSPluginAPI::WindVector*>(a_worldPositions.data()), published.data(), static_cast<std::uint32_t>(published.size()))) {
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

	const float degrees = sky->currentWeather ? sky->currentWeather->data.windDirection * 180.0f / 256.0f : sky->windAngle * 180.0f / 256.0f;
	const float heading = (degrees - 90.0f) * std::numbers::pi_v<float> / 180.0f;
	const float3 wind{ -std::sin(heading) * sky->windSpeed, std::cos(heading) * sky->windSpeed, 0.0f };
	for (std::size_t index = 0; index < a_worldPositions.size(); ++index)
		a_samples[index].windExcludingHavokImpulses = wind;
	return true;
}
