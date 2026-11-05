#pragma once

#include "core/Math.hpp"

namespace golf {

// Physical impulses about the centre of mass; no device or swing information.
struct ShotCommand {
    Vec3 impulse;        // N*s
    Vec3 angularImpulse; // N*m*s
};

} // namespace golf
