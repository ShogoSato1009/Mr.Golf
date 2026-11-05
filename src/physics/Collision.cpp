#include "physics/Collision.hpp"

#include <algorithm>

namespace golf::collision {

Vec3 slipVelocity(const BallState& s, Vec3 n, double radius) {
    return tangent(s.velocity, n) + cross(s.angularVelocity, -radius*n);
}

void applyTangentialImpulse(BallState& s, Vec3 impulse, Vec3 n, const BallParameters& b) {
    s.velocity += impulse / b.mass;
    s.angularVelocity += cross(-b.radius*n, impulse) / b.inertia();
}

bool applyFriction(BallState& s, Vec3 n, double normalImpulse,
                   const BallParameters& b, const SurfaceMaterial& m) {
    const Vec3 slip = slipVelocity(s, n, b.radius);
    const double slipSpeed = length(slip);
    if (slipSpeed == 0.0) return true;
    const double effectiveInverseMass = 1.0/b.mass + b.radius*b.radius/b.inertia();
    const double stoppingImpulse = slipSpeed / effectiveInverseMass;
    const bool sticks = stoppingImpulse <= m.staticFriction * normalImpulse;
    const double magnitude = sticks ? stoppingImpulse :
        std::min(stoppingImpulse, m.dynamicFriction * normalImpulse);
    applyTangentialImpulse(s, -(magnitude/slipSpeed)*slip, n, b);
    return sticks;
}

void resolveImpact(BallState& s, Vec3 n, const BallParameters& b,
                   const SurfaceMaterial& m, double bounceThreshold) {
    const double vn = dot(s.velocity, n);
    if (vn >= 0.0) return;
    const double restitution = -vn > bounceThreshold ? m.restitution : 0.0;
    const double normalImpulse = -(1.0 + restitution)*b.mass*vn;
    s.velocity += (normalImpulse/b.mass)*n;
    if (m.impactBraking > 0.0) {
        // Approximate the translational energy absorbed by deforming turf during impact.
        // Limit the loss to the incoming tangential speed before solving contact friction.
        const Vec3 travel = tangent(s.velocity, n);
        const double speed = length(travel);
        if (speed > 0.0) {
            const double loss = std::min(speed, m.impactBraking*(-vn));
            s.velocity -= (loss/speed)*travel;
        }
    }
    applyFriction(s, n, normalImpulse, b, m);
}

} // namespace golf::collision
