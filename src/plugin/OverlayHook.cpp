#include "plugin/OverlayHook.h"

#include "Features/SmpPhysics.h"
#include "plugin/Instance.h"

namespace
{
	/** @brief The call that renders the world; the overlay draws once it returns, with the scene's target still current. */
	struct Main_RenderWorld
	{
		static void thunk(bool a_arg)
		{
			func(a_arg);
			if (Instance().loaded)
				Instance().OnDeferredGeometryEnd();
		}

		static inline REL::Relocation<decltype(thunk)> func;
	};
}

void OverlayHook::Install()
{
	const bool isAE1799 = REL::Module::IsAtLeast(REL::Version(1, 7, 99, 0));
	const std::uintptr_t callSite = REL::RelocationID(35560, 36559).address() + REL::Relocate<std::uintptr_t>(0x831, isAE1799 ? 0x85E : 0x841, 0x791);
	auto& trampoline = SKSE::GetTrampoline();
	Main_RenderWorld::func = trampoline.write_call<5>(callSite, Main_RenderWorld::thunk);
	logger::info("Hooked the world render for the debug overlay");
}
