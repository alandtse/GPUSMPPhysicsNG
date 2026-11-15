#include "Utils/Game.h"

#include "smp/ViewMath.h"

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
			if (SmpPhysics::SphereInFrustum(view.viewProjMat.m, relative.x, relative.y, relative.z, a_radius))
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
			largest = std::max(largest, SmpPhysics::SphereScreenHeightPercent(view.viewProjMat.m, relative.x, relative.y, relative.z, a_radius));
		}
		return largest;
	}
}
