#include "WindShelter.h"

#include <algorithm>

namespace WindShelter
{
	float Exposure(const std::array<float, 3>& a_hitDistance)
	{
		if (a_hitDistance[0] >= 0.0f && a_hitDistance[0] < kNearWall)
			return 0.0f;
		float blockedWeight = 0.0f;
		for (std::size_t ray = 0; ray < kRays.size(); ++ray) {
			if (a_hitDistance[ray] >= 0.0f)
				blockedWeight += kRays[ray].weight;
		}
		return std::clamp(1.0f - blockedWeight, 0.0f, 1.0f);
	}
}
