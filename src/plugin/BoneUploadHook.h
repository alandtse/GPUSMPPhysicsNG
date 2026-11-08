#pragma once

namespace BoneUploadHook
{
	/** @brief Detours the engine's skin bone-matrix binding so a claimed mesh can be rebound to the GPU solver's pose right after the engine uploads its own. */
	void Install();
}
