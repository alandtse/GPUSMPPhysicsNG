#include "IdleCache.h"

#include <atomic>
#include <map>
#include <mutex>

namespace TestBench::IdleCache
{
	namespace
	{
		constexpr std::string_view kPairedPrefix = "pa_";

		std::mutex cacheMutex;
		std::map<std::string, RE::TESIdleForm*> cache;  ///< By editor ID. Guarded by cacheMutex.
		std::atomic<bool> installed{ false };

		struct SetFormEditorIdHook
		{
			static bool thunk(RE::TESIdleForm* a_this, const char* a_editorId)
			{
				if (a_this && a_editorId && *a_editorId && !std::string_view(a_editorId).starts_with(kPairedPrefix)) {
					const std::scoped_lock lock(cacheMutex);
					cache.emplace(a_editorId, a_this);
				}
				return func(a_this, a_editorId);
			}
			static inline REL::Relocation<decltype(thunk)> func;
		};
	}

	void Install()
	{
		if (installed.exchange(true))
			return;
		stl::write_vfunc<0x33, SetFormEditorIdHook>(RE::VTABLE_TESIdleForm[0]);
	}

	std::vector<IdleChoice> Snapshot()
	{
		std::vector<IdleChoice> idles;
		const std::scoped_lock lock(cacheMutex);
		idles.reserve(cache.size());
		for (const auto& [editorId, form] : cache) {
			if (form && !form->animEventName.empty())
				idles.push_back({ editorId, form->animEventName.c_str(), form });
		}
		return idles;
	}

	RE::TESIdleForm* Find(std::string_view a_editorId)
	{
		const std::scoped_lock lock(cacheMutex);
		const auto found = cache.find(std::string(a_editorId));
		return found != cache.end() ? found->second : nullptr;
	}
}
