#pragma once

#include "Features/SmpPhysics.h"

/** @brief The one feature instance; the hdtSMP64 callbacks and the engine hooks reach it from here. */
SmpPhysicsFeature& Instance();
