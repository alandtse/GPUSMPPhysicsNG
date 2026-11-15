#ifdef TRACY_ENABLE

#	include "TracyGpu.h"

#	include "Globals.h"

#	include <cstring>
#	include <d3d11.h>
#	include <optional>
#	include <string>
#	include <tracy/Tracy.hpp>
#	include <tracy/TracyD3D11.hpp>

namespace TracyGpu
{
	namespace
	{
		TracyD3D11Ctx context = nullptr;

		TracyD3D11Ctx Context()
		{
			if (!context && globals::d3d::device && globals::d3d::context) {
				context = TracyD3D11Context(globals::d3d::device, globals::d3d::context);
				constexpr char kContextName[] = "Game immediate context";
				TracyD3D11ContextName(context, kContextName, sizeof(kContextName) - 1);
			}
			return context;
		}
	}

	struct Zone::Impl
	{
		explicit Impl(std::string_view a_name) :
			name(a_name),
			cpu(0, __FILE__, std::strlen(__FILE__), __func__, std::strlen(__func__), name.c_str(), name.size(), true)
		{
			if (const auto ctx = Context())
				gpu.emplace(ctx, 0u, __FILE__, std::strlen(__FILE__), __func__, std::strlen(__func__), name.c_str(), name.size(), true);
		}

		std::string name;
		tracy::ScopedZone cpu;
		std::optional<tracy::D3D11ZoneScope> gpu;
	};

	Zone::Zone(std::string_view a_name) :
		impl(std::make_unique<Impl>(a_name))
	{}

	Zone::~Zone() = default;

	void NextFrame()
	{
		if (const auto ctx = Context())
			TracyD3D11Collect(ctx);
		FrameMark;
	}
}

#endif
