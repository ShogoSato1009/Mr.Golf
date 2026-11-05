#include "world/Terrain.hpp"

#include <stdexcept>
#include <utility>

namespace golf {

SurfaceMaterial materialFor(SurfaceType type) {
    switch (type) {
    case SurfaceType::Green:   return {0.50, 0.40, 0.25, 0.010, 0.010};
    case SurfaceType::Fairway: return {0.45, 0.45, 0.30, 0.025, 0.015};
    case SurfaceType::Rough:   return {0.25, 0.65, 0.50, 0.080, 0.040};
    case SurfaceType::Sand:    return {0.10, 0.80, 0.65, 0.180, 0.080};
    }
    throw std::invalid_argument("Unknown surface type");
}

void validateMaterial(const SurfaceMaterial& m) {
    const auto nonnegative = [](double x) { return std::isfinite(x) && x >= 0.0; };
    if (!nonnegative(m.restitution) || m.restitution > 1.0 ||
        !nonnegative(m.staticFriction) || !nonnegative(m.dynamicFriction) ||
        m.dynamicFriction > m.staticFriction || !nonnegative(m.rollingResistance) ||
        !nonnegative(m.spinningFriction)) {
        throw std::invalid_argument("Invalid surface coefficients");
    }
}

Terrain::Terrain(Sampler sampler) : sampler_(std::move(sampler)) {
    if (!sampler_) throw std::invalid_argument("Terrain sampler is empty");
}

TerrainSample Terrain::sample(double x, double z) const {
    if (!std::isfinite(x) || !std::isfinite(z))
        throw std::invalid_argument("Non-finite terrain coordinates");
    auto s = sampler_(x, z);
    const double normalLength = length(s.normal);
    if (!std::isfinite(s.height) || !isFinite(s.normal) ||
        !std::isfinite(normalLength) || normalLength == 0.0)
        throw std::invalid_argument("Invalid terrain geometry");
    s.normal = s.normal / normalLength;
    if (s.normal.y < 0.1)
        throw std::invalid_argument("Heightfield requires an upward normal with y >= 0.1");
    validateMaterial(s.material);
    return s;
}

Terrain Terrain::plane(double height, double slopeX, double slopeZ, SurfaceMaterial material) {
    validateMaterial(material);
    if (!std::isfinite(height) || !std::isfinite(slopeX) || !std::isfinite(slopeZ))
        throw std::invalid_argument("Invalid plane");
    const Vec3 n = Vec3{-slopeX, 1.0, -slopeZ} / length({-slopeX, 1.0, -slopeZ});
    Terrain result([=](double x, double z) {
        return TerrainSample{height + slopeX*x + slopeZ*z, n, material};
    });
    (void)result.sample(0.0, 0.0);
    return result;
}

} // namespace golf
