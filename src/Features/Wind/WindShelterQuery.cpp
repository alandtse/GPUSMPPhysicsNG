#include "Features/Wind/WindShelterQuery.h"

#include "Utils/ActorUtils.h"

#include <cmath>
#include <numbers>

namespace WindShelter
{
	Estimate EstimateShelter(RE::Actor& a_ignored, const RE::NiPoint3& a_origin, const RE::NiPoint3& a_windVelocity)
	{
		Estimate estimate;
		if (!(std::hypot(a_windVelocity.x, a_windVelocity.y) > 1e-4f))
			return estimate;
		constexpr float kRadiansPerDegree = std::numbers::pi_v<float> / 180.0f;
		const float heading = std::atan2(-a_windVelocity.y, -a_windVelocity.x);
		for (std::size_t ray = 0; ray < kRays.size(); ++ray) {
			const float rayHeading = heading + kRays[ray].azimuthDegrees * kRadiansPerDegree;
			const float elevation = kRays[ray].elevationDegrees * kRadiansPerDegree;
			const RE::NiPoint3 direction{ std::cos(elevation) * std::cos(rayHeading), std::cos(elevation) * std::sin(rayHeading), std::sin(elevation) };
			const auto hit = Util::FirstOccluder(a_ignored, a_origin, a_origin + direction * kReach);
			estimate.hitDistance[ray] = hit ? hit->distance : -1.0f;
			if (hit && (estimate.blockerDistance < 0.0f || ray == 0)) {
				estimate.blockerDistance = hit->distance;
				estimate.blockerReference = hit->reference;
			}
		}
		estimate.exposure = Exposure(estimate.hitDistance);
		return estimate;
	}
}
