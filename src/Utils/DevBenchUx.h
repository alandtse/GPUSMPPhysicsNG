#pragma once

// The standalone plugin has no devbench host, so feature commands and queries are compiled but not registered.

#define DEVBENCH_UX_CONCAT_INNER(a, b) a##b
#define DEVBENCH_UX_CONCAT(a, b) DEVBENCH_UX_CONCAT_INNER(a, b)
#define FEATURE_COMMAND(Name, Desc, Fn) [[maybe_unused]] const auto DEVBENCH_UX_CONCAT(devbenchUnused_, __LINE__) = Fn;
#define FEATURE_QUERY(Name, Desc, Fn) [[maybe_unused]] const auto DEVBENCH_UX_CONCAT(devbenchUnused_, __LINE__) = Fn;
#define FEATURE_RENDER_COMMAND(Name, Desc, Fn) FEATURE_COMMAND(Name, Desc, Fn)
#define FEATURE_RENDER_QUERY(Name, Desc, Fn) FEATURE_QUERY(Name, Desc, Fn)
