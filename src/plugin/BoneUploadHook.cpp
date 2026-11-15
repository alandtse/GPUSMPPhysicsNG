#include "plugin/BoneUploadHook.h"

#include "Features/SmpPhysics.h"
#include "plugin/Instance.h"

#include <detours.h>

namespace
{
	struct NiSkinInstance_BindBoneMatrices
	{
		static void thunk(void* a_this, RE::NiSkinInstance* a_skin, void* a_context, RE::NiTransform* a_transform)
		{
			func(a_this, a_skin, a_context, a_transform);
			if (!Instance().loaded)
				return;
			try {
				Instance().OnBoneMatricesUploaded(a_skin);
			} catch (const std::exception& error) {
				logger::error("The bone upload hook failed: {}", error.what());
			}
		}

		static inline REL::Relocation<decltype(thunk)> func;
	};
}

void BoneUploadHook::Install()
{
	// Needs a VR address library that carries id 101335; without it the lookup aborts plugin load.
	NiSkinInstance_BindBoneMatrices::func = REL::RelocationID(101335, 108322).address();
	DetourTransactionBegin();
	DetourUpdateThread(GetCurrentThread());
	DetourAttach(reinterpret_cast<PVOID*>(&NiSkinInstance_BindBoneMatrices::func), reinterpret_cast<PVOID>(NiSkinInstance_BindBoneMatrices::thunk));
	if (const LONG result = DetourTransactionCommit(); result != NO_ERROR) {
		logger::error("Failed to hook the skin bone-matrix binding (error {}); GPU skinning stays off", result);
		NiSkinInstance_BindBoneMatrices::func = std::uintptr_t{ 0 };
		return;
	}
	logger::info("Hooked the skin bone-matrix binding");
}
