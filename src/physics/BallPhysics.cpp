#include "physics/BallPhysics.hpp"
#include "physics/Collision.hpp"

#include <algorithm>
#include <stdexcept>

namespace golf {
namespace {

bool nonnegative(double x) { return std::isfinite(x) && x >= 0.0; }
bool positive(double x) { return std::isfinite(x) && x > 0.0; }
bool finiteState(const BallState& s) {
    return isFinite(s.position) && isFinite(s.velocity) && isFinite(s.angularVelocity);
}

Vec3 shorten(Vec3 value, double amount) {
    const double magnitude = length(value);
    return magnitude <= amount ? Vec3{} : value * (1.0 - amount/magnitude);
}

} // namespace

BallPhysics::BallPhysics(BallParameters ball, AerodynamicParameters aero, PhysicsSettings settings)
    : ball_(ball), aero_(aero), settings_(settings) {
    if (!positive(ball.mass) || !positive(ball.radius) || !positive(ball.inertiaFactor) ||
        !positive(ball.inertia()) || !positive(pi*ball.radius*ball.radius) ||
        !positive(1.0/ball.mass) || !positive(1.0/ball.inertia()) ||
        !nonnegative(aero.dragCoefficient) || !nonnegative(aero.liftSlope) ||
        !nonnegative(aero.maxLiftCoefficient) || !nonnegative(aero.spinDecayPerDensity) ||
        !positive(settings.fixedTimeStep) || !positive(settings.maxSubstep) ||
        !positive(settings.maxTravelPerSubstep) || !positive(settings.contactTolerance) ||
        !nonnegative(settings.bounceThreshold) || !nonnegative(settings.stopSpeed) ||
        !nonnegative(settings.angularStopSpeed) || !positive(settings.slipTolerance) ||
        settings.collisionIterations == 0 || settings.collisionIterations > 64 ||
        settings.maxSubsteps == 0) {
        throw std::invalid_argument("Invalid ball, aerodynamic, or integration parameters");
    }
}

void BallPhysics::applyShot(BallState& state, const ShotCommand& command) const {
    if (!finiteState(state) || !isFinite(command.impulse) || !isFinite(command.angularImpulse))
        throw std::invalid_argument("Non-finite ball state or impulse");
    BallState next = state;
    next.velocity += command.impulse / ball_.mass;
    next.angularVelocity += command.angularImpulse / ball_.inertia();
    if (!finiteState(next)) throw std::overflow_error("Impulse overflow");
    if (length(command.impulse) > 0.0 || length(command.angularImpulse) > 0.0)
        next.phase = MotionPhase::Airborne; // Contact is reevaluated on the next tick.
    state = next;
}

Vec3 BallPhysics::acceleration(Vec3 v, Vec3 spin, const Environment& env) const {
    const Vec3 relative = v - env.windVelocity;
    const double speed = length(relative);
    if (speed == 0.0 || env.airDensity == 0.0) return env.gravity;

    const double area = pi * ball_.radius * ball_.radius;
    const double forceScale = 0.5 * env.airDensity * area * speed * speed;
    Vec3 force = -(forceScale * aero_.dragCoefficient / speed) * relative;
    const Vec3 liftAxis = cross(spin, relative);
    const double liftAxisLength = length(liftAxis);
    if (liftAxisLength > 0.0) {
        // Only spin perpendicular to the airflow contributes to lift.
        const double spinRatio = ball_.radius * liftAxisLength / (speed*speed);
        const double liftCoefficient = std::min(aero_.liftSlope*spinRatio, aero_.maxLiftCoefficient);
        force += (forceScale * liftCoefficient / liftAxisLength) * liftAxis;
    }
    const Vec3 result = env.gravity + force / ball_.mass;
    if (!isFinite(result)) throw std::overflow_error("Aerodynamic force overflow");
    return result;
}

BallState BallPhysics::fly(const BallState& s, double dt, const Environment& env) const {
    // RK4 for translation; exponential integration for isotropic spin damping.
    const double halfDecay = std::exp(-aero_.spinDecayPerDensity * env.airDensity * dt * 0.5);
    const Vec3 halfSpin = s.angularVelocity * halfDecay;
    const Vec3 endSpin = halfSpin * halfDecay;
    const Vec3 a1 = acceleration(s.velocity, s.angularVelocity, env);
    const Vec3 v2 = s.velocity + (dt*0.5)*a1;
    const Vec3 a2 = acceleration(v2, halfSpin, env);
    const Vec3 v3 = s.velocity + (dt*0.5)*a2;
    const Vec3 a3 = acceleration(v3, halfSpin, env);
    const Vec3 v4 = s.velocity + dt*a3;
    const Vec3 a4 = acceleration(v4, endSpin, env);
    BallState result = s;
    result.position += (dt/6.0)*(s.velocity + 2.0*v2 + 2.0*v3 + v4);
    result.velocity += (dt/6.0)*(a1 + 2.0*a2 + 2.0*a3 + a4);
    result.angularVelocity = endSpin;
    result.phase = MotionPhase::Airborne;
    if (!finiteState(result)) throw std::overflow_error("Flight integration overflow");
    return result;
}

double BallPhysics::separation(const BallState& s, const TerrainSample& surface) const {
    // Exact for a plane. For curved heightfields this is the local tangent approximation.
    return (s.position.y - surface.height)*surface.normal.y - ball_.radius;
}

void BallPhysics::placeOnSurface(BallState& s, const TerrainSample& surface) const {
    s.position.y = surface.height + ball_.radius/surface.normal.y;
}

void BallPhysics::moveOnGround(BallState& s, double dt, const Terrain& terrain,
                               const TerrainSample& surface, const Environment& env) const {
    const Vec3 n = surface.normal;
    const auto& m = surface.material;
    const Vec3 external = acceleration(s.velocity, s.angularVelocity, env);
    const double loadAcceleration = std::max(0.0, -dot(external, n));
    const double normalImpulse = ball_.mass * loadAcceleration * dt;
    const Vec3 initialVelocity = tangent(s.velocity, n);
    s.velocity = initialVelocity + dt*tangent(external, n);
    s.angularVelocity *= std::exp(-aero_.spinDecayPerDensity * env.airDensity * dt);

    // Apply the resisting torque before Coulomb friction: on a frictionless
    // surface it slows spin only, rather than inventing a translational force.
    const double rollingSpinReduction = m.rollingResistance * normalImpulse * ball_.radius / ball_.inertia();
    s.angularVelocity = shorten(tangent(s.angularVelocity, n), rollingSpinReduction) +
                        dot(s.angularVelocity, n)*n;
    collision::applyFriction(s, n, normalImpulse, ball_, m);
    const bool rolling = length(collision::slipVelocity(s, n, ball_.radius)) <= settings_.slipTolerance;

    // Torsional friction removes spin about the contact normal without changing translation.
    const double normalSpin = dot(s.angularVelocity, n);
    const double spinReduction = m.spinningFriction * normalImpulse * ball_.radius / ball_.inertia();
    const double remainingSpin = std::max(0.0, std::abs(normalSpin) - spinReduction);
    s.angularVelocity += (std::copysign(remainingSpin, normalSpin) - normalSpin)*n;

    // Sleep only when rolling resistance can oppose the local driving force.
    // Reevaluate every tick, so changed wind/material/slope wakes a stopped ball.
    const bool canHold = length(tangent(external, n)) <=
                        std::min(m.rollingResistance, m.staticFriction)*loadAcceleration;
    if (rolling && canHold && length(s.velocity) <= settings_.stopSpeed &&
        length(s.angularVelocity)*ball_.radius <= settings_.stopSpeed &&
        std::abs(dot(s.angularVelocity, n)) <= settings_.angularStopSpeed) {
        s.velocity = {};
        s.angularVelocity = {};
        s.phase = MotionPhase::Resting;
    } else {
        s.phase = rolling ? MotionPhase::Rolling : MotionPhase::Sliding;
    }

    s.position += (dt*0.5)*(initialVelocity + s.velocity);
    const auto endSurface = terrain.sample(s.position.x, s.position.z);
    placeOnSurface(s, endSurface);
    // Transport contact velocities to the new local tangent; valid for smooth terrain.
    const double axialSpin = dot(s.angularVelocity, n);
    s.velocity = tangent(s.velocity, endSurface.normal);
    if (rolling)
        s.angularVelocity = cross(endSurface.normal, s.velocity)/ball_.radius +
                            axialSpin*endSurface.normal;
}

void BallPhysics::step(BallState& state, const Terrain& terrain, const Environment& env) const {
    if (!finiteState(state) || !isFinite(env.gravity) || !isFinite(env.windVelocity) ||
        !nonnegative(env.airDensity))
        throw std::invalid_argument("Invalid state or environment");

    BallState next = state;
    double remaining = settings_.fixedTimeStep;
    std::size_t work = 0;
    while (remaining > 0.0) {
        if (++work > settings_.maxSubsteps)
            throw std::runtime_error("Physics substep budget exhausted; reduce tick or input magnitudes");
        const auto surface = terrain.sample(next.position.x, next.position.z);
        const Vec3 n = surface.normal;
        const Vec3 a = acceleration(next.velocity, next.angularVelocity, env);
        const double estimatedSpeed = length(next.velocity) + length(a)*settings_.maxSubstep;
        const double travelTime = estimatedSpeed > 0.0 ?
            settings_.maxTravelPerSubstep / estimatedSpeed : settings_.maxSubstep;
        const double dt = std::min({remaining, settings_.maxSubstep, travelTime});
        if (!(dt > 0.0) || remaining - dt == remaining)
            throw std::runtime_error("Physics step is too small to advance time");

        const double gap = separation(next, surface);
        if (gap <= settings_.contactTolerance) {
            placeOnSurface(next, surface); // Also recovers initial penetration.
            collision::resolveImpact(next, n, ball_, surface.material, settings_.bounceThreshold);
            const Vec3 contactAcceleration = acceleration(next.velocity, next.angularVelocity, env);
            if (dot(next.velocity, n) <= settings_.slipTolerance && dot(contactAcceleration, n) <= 0.0) {
                moveOnGround(next, dt, terrain, surface, env);
                remaining = std::max(0.0, remaining - dt);
                if (!finiteState(next)) throw std::overflow_error("Contact integration overflow");
                continue;
            }
        }

        const BallState end = fly(next, dt, env);
        const auto endSurface = terrain.sample(end.position.x, end.position.z);
        if (separation(end, endSurface) > 0.0) {
            next = end;
            remaining = std::max(0.0, remaining - dt);
            continue;
        }

        // Bracket the time of impact along the integrated path, not a straight chord.
        double low = 0.0;
        double high = dt;
        for (std::size_t i = 0; i < settings_.collisionIterations; ++i) {
            const double middle = (low + high)*0.5;
            const auto candidate = fly(next, middle, env);
            const auto sample = terrain.sample(candidate.position.x, candidate.position.z);
            if (separation(candidate, sample) > 0.0) low = middle;
            else high = middle;
        }
        next = fly(next, high, env);
        const auto hitSurface = terrain.sample(next.position.x, next.position.z);
        placeOnSurface(next, hitSurface);
        collision::resolveImpact(next, hitSurface.normal, ball_, hitSurface.material,
                                 settings_.bounceThreshold);
        next.phase = dot(next.velocity, hitSurface.normal) > settings_.slipTolerance ?
            MotionPhase::Airborne : MotionPhase::Sliding;
        remaining = std::max(0.0, remaining - high);
    }
    if (!finiteState(next)) throw std::overflow_error("Physics integration overflow");
    state = next;
}

} // namespace golf
