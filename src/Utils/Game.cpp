#include "Utils/Game.h"

#include <algorithm>
#include <cmath>

namespace Util
{
	RE::NiPoint3 GetAverageEyePosition()
	{
		auto shadowState = globals::game::shadowState;
		if (!globals::game::isVR)
			return shadowState->GetRuntimeData().posAdjust.getEye();
		return (shadowState->GetVRRuntimeData().posAdjust.getEye(0) + shadowState->GetVRRuntimeData().posAdjust.getEye(1)) * 0.5f;
	}

	RE::NiPoint3 GetEyePosition(int eyeIndex)
	{
		auto shadowState = globals::game::shadowState;
		if (!globals::game::isVR)
			return shadowState->GetRuntimeData().posAdjust.getEye();
		return shadowState->GetVRRuntimeData().posAdjust.getEye(eyeIndex);
	}

	RE::BSGraphics::ViewData GetCameraData(int eyeIndex)
	{
		auto shadowState = globals::game::shadowState;
		if (!globals::game::isVR) {
			return shadowState->GetRuntimeData().cameraData.getEye();
		}
		return shadowState->GetVRRuntimeData().cameraData.getEye(eyeIndex);
	}

	bool SphereInViewFrustum(const RE::NiPoint3& a_centre, float a_radius)
	{
		const int eyes = globals::game::isVR ? 2 : 1;
		for (int eye = 0; eye < eyes; ++eye) {
			const auto view = GetCameraData(eye);
			const auto relative = a_centre - GetEyePosition(eye);
			const auto& m = view.viewProjMat;
			bool inside = true;
			for (int side = 0; side < 4 && inside; ++side) {
				const int axis = side >> 1;
				const float sign = (side & 1) ? -1.0f : 1.0f;
				const float px = m.m[0][3] + sign * m.m[0][axis];
				const float py = m.m[1][3] + sign * m.m[1][axis];
				const float pz = m.m[2][3] + sign * m.m[2][axis];
				const float pw = m.m[3][3] + sign * m.m[3][axis];
				const float length = std::sqrt(px * px + py * py + pz * pz);
				if (length > 0.0f && (px * relative.x + py * relative.y + pz * relative.z + pw) / length < -a_radius)
					inside = false;
			}
			if (inside)
				return true;
		}
		return false;
	}

	float SphereScreenHeightPercent(const RE::NiPoint3& a_centre, float a_radius)
	{
		const int eyes = globals::game::isVR ? 2 : 1;
		float largest = 0.0f;
		for (int eye = 0; eye < eyes; ++eye) {
			const auto view = GetCameraData(eye);
			const auto relative = a_centre - GetEyePosition(eye);
			const auto& m = view.viewProjMat;
			const float depth = m.m[0][3] * relative.x + m.m[1][3] * relative.y + m.m[2][3] * relative.z + m.m[3][3];
			if (depth <= 0.0f)
				continue;
			const float scale = std::sqrt(m.m[0][1] * m.m[0][1] + m.m[1][1] * m.m[1][1] + m.m[2][1] * m.m[2][1]);
			largest = std::max(largest, 100.0f * a_radius * scale / depth);
		}
		return largest;
	}

}
