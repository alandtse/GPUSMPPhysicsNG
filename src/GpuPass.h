#pragma once

#include "Globals.h"

#include <d3d11_1.h>
#include <string>
#include <string_view>
#include <winrt/base.h>

/** @brief RAII PIX/RenderDoc event bracket around a render pass entry point; a no-op when the context has no annotation interface. */
struct ScopedGpuPass
{
	explicit ScopedGpuPass(std::string_view a_name)
	{
		if (globals::d3d::context && SUCCEEDED(globals::d3d::context->QueryInterface(IID_PPV_ARGS(annotation.put())))) {
			const std::wstring wide(a_name.begin(), a_name.end());
			annotation->BeginEvent(wide.c_str());
		}
	}

	~ScopedGpuPass()
	{
		if (annotation)
			annotation->EndEvent();
	}

	ScopedGpuPass(const ScopedGpuPass&) = delete;
	ScopedGpuPass& operator=(const ScopedGpuPass&) = delete;

private:
	winrt::com_ptr<ID3DUserDefinedAnnotation> annotation;
};

#define CS_GPU_PASS_CONCAT_INNER(a, b) a##b
#define CS_GPU_PASS_CONCAT(a, b) CS_GPU_PASS_CONCAT_INNER(a, b)
#define CS_GPU_PASS(name) ScopedGpuPass CS_GPU_PASS_CONCAT(cs_gpu_pass_, __LINE__){ name }

#ifndef ZoneScopedN
#	define ZoneScopedN(name)
#endif

#ifndef ZoneNamedN
#	define ZoneNamedN(variable, name, active)
#endif
