#include "Features/Wind/Wind.h"

#include <cmath>
#include <numbers>

bool Wind::SamplePublishedWind(std::span<const float3> a_worldPositions, std::span<PublishedWindSample> a_samples) const noexcept
{
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
