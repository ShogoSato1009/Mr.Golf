#pragma once

#include "core/Math.hpp"

namespace golf {

struct Environment {
    Vec3 gravity{0.0, -9.80665, 0.0}; // m/s^2
    Vec3 windVelocity;              // Air velocity in world coordinates, m/s.
    double airDensity = 1.225;      // kg/m^3; zero gives a vacuum.
};

} // namespace golf
