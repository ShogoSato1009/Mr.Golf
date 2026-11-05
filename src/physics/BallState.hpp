#pragma once

#include "core/Math.hpp"

namespace golf {

// SI units, right-handed coordinates, +Y up. Angular velocity is rad/s.
enum class MotionPhase { Airborne, Sliding, Rolling, Resting };

struct BallState {
    Vec3 position{0.0, 0.02135, 0.0}; // Centre, not the point of contact.
    Vec3 velocity;
    Vec3 angularVelocity;
    MotionPhase phase = MotionPhase::Airborne;
};

struct BallParameters {
    double mass = 0.04593;     // kg; tunable representative ball, not certification data.
    double radius = 0.02135;  // m
    double inertiaFactor = 0.4; // I = k*m*r^2; homogeneous solid sphere.

    double inertia() const { return inertiaFactor * mass * radius * radius; }
};

struct AerodynamicParameters {
    double dragCoefficient = 0.25;
    double liftSlope = 0.6;            // CL = min(liftSlope*S, maxLiftCoefficient).
    double maxLiftCoefficient = 0.35;
    double spinDecayPerDensity = 0.04; // d(omega)/dt = -rate*rho*omega, m^3/(kg*s).
};

} // namespace golf
