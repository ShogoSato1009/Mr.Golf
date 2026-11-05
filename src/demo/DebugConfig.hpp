#pragma once

#include "physics/BallPhysics.hpp"
#include "rendering/Camera.hpp"

#include <filesystem>

namespace golf::demo {

struct DebugConfig {
    BallParameters ball;
    AerodynamicParameters aero;
    PhysicsSettings physics;
    Environment environment{{0, -9.80665, 0}, {1, 0, 2}, 1.225};
    SurfaceMaterial material{0.35, 0.45, 0.30, 0.145, 0.015};
    double terrainHeight = 0;
    double slopeX = 0;
    double slopeZ = 0;
    double hillAmplitude = 0.4;
    double hillLength = 80;
    Vec3 start{0, 0, 0}; // X/Z start and height above the local contact surface.
    ShotCommand shot{{1.5, 0.87267, 0.04593}, {0, 0.00008375, 0.002093}};
    CameraSettings camera;
    int width = 1280;
    int height = 720;
    int fps = 30;
    double maxDuration = 60;
    double endHold = 2;

    Terrain makeTerrain() const;
    void validate() const;
};

DebugConfig loadConfig(const std::filesystem::path& path);

} // namespace golf::demo
