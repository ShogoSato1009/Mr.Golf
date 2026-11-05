#pragma once

#include "physics/BallState.hpp"
#include "world/Terrain.hpp"

namespace golf::collision {

// Internal solver primitives: inputs are prevalidated and normals are unit length.
Vec3 slipVelocity(const BallState& state, Vec3 normal, double radius);
void applyTangentialImpulse(BallState& state, Vec3 impulse, Vec3 normal,
                            const BallParameters& ball);
bool applyFriction(BallState& state, Vec3 normal, double normalImpulse,
                   const BallParameters& ball, const SurfaceMaterial& material);
void resolveImpact(BallState& state, Vec3 normal, const BallParameters& ball,
                   const SurfaceMaterial& material, double bounceThreshold);

} // namespace golf::collision
