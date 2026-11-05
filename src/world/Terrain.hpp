#pragma once

#include "core/Math.hpp"

#include <functional>

namespace golf {

enum class SurfaceType { Green, Fairway, Rough, Sand };

struct SurfaceMaterial {
    double restitution = 0.45;
    double staticFriction = 0.45;
    double dynamicFriction = 0.30;
    double rollingResistance = 0.025; // Resisting torque = coefficient * N * radius.
    double spinningFriction = 0.015;  // Torque about contact normal / (N*radius).
    double impactBraking = 0.0;      // Tangential speed lost per unit incoming normal speed.
};

// Gameplay starting points; callers may replace any coefficient.
SurfaceMaterial materialFor(SurfaceType type);
void validateMaterial(const SurfaceMaterial& material);

struct TerrainSample {
    double height = 0.0;
    Vec3 normal{0.0, 1.0, 0.0};
    SurfaceMaterial material;
};

// A smooth, single-valued static heightfield. Queries must be deterministic,
// cover the entire simulation area, and return a normal matching the height.
// No mesh, rendering, collision response, overhangs, or vertical walls here.
class Terrain {
public:
    using Sampler = std::function<TerrainSample(double x, double z)>;

    explicit Terrain(Sampler sampler);
    TerrainSample sample(double x, double z) const;
    static Terrain plane(double height = 0.0, double slopeX = 0.0, double slopeZ = 0.0,
                         SurfaceMaterial material = {});

private:
    Sampler sampler_;
};

} // namespace golf
