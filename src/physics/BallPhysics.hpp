#pragma once

#include "physics/BallState.hpp"
#include "shot/ShotCommand.hpp"
#include "world/Environment.hpp"
#include "world/Terrain.hpp"

#include <cstddef>

namespace golf {

struct PhysicsSettings {
    double fixedTimeStep = 1.0 / 240.0;
    double maxSubstep = 1.0 / 960.0;
    double maxTravelPerSubstep = 0.05; // m; terrain must be smooth at this scale.
    double contactTolerance = 1e-7;   // m
    double bounceThreshold = 0.20;    // incoming normal speed, m/s
    double stopSpeed = 0.005;         // m/s
    double angularStopSpeed = 0.05;   // rad/s
    double slipTolerance = 1e-8;      // m/s
    std::size_t collisionIterations = 32;
    std::size_t maxSubsteps = 8192;
};

class BallPhysics {
public:
    explicit BallPhysics(BallParameters ball = {}, AerodynamicParameters aero = {},
                         PhysicsSettings settings = {});

    // Adds delta-v = J/m and delta-omega = L/I. Does not reset position or velocity.
    void applyShot(BallState& state, const ShotCommand& command) const;

    // Exactly one fixed tick, independent of wall time and rendering. Caller owns state.
    // Invalid input, invalid terrain, or exhausted work budget throws without changing state.
    void step(BallState& state, const Terrain& terrain, const Environment& environment = {}) const;

    const BallParameters& ball() const { return ball_; }
    const PhysicsSettings& settings() const { return settings_; }

private:
    BallParameters ball_;
    AerodynamicParameters aero_;
    PhysicsSettings settings_;

    Vec3 acceleration(Vec3 velocity, Vec3 spin, const Environment& environment) const;
    BallState fly(const BallState& state, double dt, const Environment& environment) const;
    void moveOnGround(BallState& state, double dt, const Terrain& terrain,
                      const TerrainSample& surface, const Environment& environment) const;
    double separation(const BallState& state, const TerrainSample& surface) const;
    void placeOnSurface(BallState& state, const TerrainSample& surface) const;
};

} // namespace golf
