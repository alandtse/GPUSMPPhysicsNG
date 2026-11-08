#pragma once

#include "Globals.h"

namespace Util
{
	/** @brief The render camera's world position for one eye; the single eye in flat. */
	RE::NiPoint3 GetEyePosition(int eyeIndex);
	/** @brief The render camera position averaged over both eyes in VR. */
	RE::NiPoint3 GetAverageEyePosition();
	RE::BSGraphics::ViewData GetCameraData(int eyeIndex);
	/** @brief True when a sphere reaches into either eye's view frustum. */
	bool SphereInViewFrustum(const RE::NiPoint3& a_centre, float a_radius);
	/** @brief The sphere's largest height across the eyes as a percentage of the screen height, or 0 when it is behind them. */
	float SphereScreenHeightPercent(const RE::NiPoint3& a_centre, float a_radius);
}
