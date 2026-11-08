#pragma once

// Profiling zones compile away; this plugin ships without the Tracy client.

#define ZoneScoped
#define ZoneScopedN(name)
#define ZoneNamedN(variable, name, active)
#define FrameMark
