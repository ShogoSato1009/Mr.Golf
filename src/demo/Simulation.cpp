#include "demo/Simulation.hpp"

#include <algorithm>
#include <stdexcept>

namespace golf::demo {

Simulation simulate(const DebugConfig& c) {
    c.validate();
    const BallPhysics physics(c.ball, c.aero, c.physics);
    const auto terrain = c.makeTerrain();
    Simulation result;
    result.dt = c.physics.fixedTimeStep;
    BallState ball;
    const auto surface = terrain.sample(c.start.x, c.start.z);
    ball.position = {c.start.x, surface.height+c.ball.radius/surface.normal.y+c.start.y, c.start.z};
    physics.applyShot(ball, c.shot);
    result.apex = ball.position.y-surface.height;
    result.ticks.push_back(ball);
    const auto maxTicks = static_cast<std::size_t>(std::ceil(c.maxDuration/result.dt));
    for (std::size_t tick = 1; tick <= maxTicks; ++tick) {
        const Vec3 previousVelocity = ball.velocity;
        physics.step(ball, terrain, c.environment);
        const auto ground = terrain.sample(ball.position.x, ball.position.z);
        const double gap = (ball.position.y-ground.height)*ground.normal.y-c.ball.radius;
        if (!isFinite(ball.position) || !isFinite(ball.velocity) || !isFinite(ball.angularVelocity) ||
            gap < -1e-6)
            throw std::runtime_error("Invalid simulated state or terrain penetration");
        result.ticks.push_back(ball);
        result.apex = std::max(result.apex, ball.position.y-ground.height);
        const double time = static_cast<double>(tick)*result.dt;
        const double incoming = dot(previousVelocity, ground.normal);
        if (result.firstContactTime < 0 && (ball.phase != MotionPhase::Airborne ||
            (incoming < 0 && dot(ball.velocity, ground.normal) > 0 && gap < -incoming*result.dt+1e-6)))
            result.firstContactTime = time;
        if (ball.phase == MotionPhase::Resting) {
            if (result.restTime < 0) result.restTime = time;
            if (time-result.restTime >= c.endHold) break;
        } else result.restTime = -1;
    }
    result.duration = std::min(c.maxDuration, static_cast<double>(result.ticks.size()-1)*result.dt);
    return result;
}

BallState Simulation::sample(double time) const {
    if (ticks.empty() || !std::isfinite(time) || dt <= 0)
        throw std::invalid_argument("Invalid presentation sample");
    const double index = std::clamp(time/dt, 0.0, static_cast<double>(ticks.size()-1));
    const auto lo = static_cast<std::size_t>(std::floor(index));
    const auto hi = std::min(lo+1, ticks.size()-1);
    const double alpha = index-static_cast<double>(lo);
    BallState result = ticks[lo];
    result.position += alpha*(ticks[hi].position-result.position);
    result.velocity += alpha*(ticks[hi].velocity-result.velocity);
    result.angularVelocity += alpha*(ticks[hi].angularVelocity-result.angularVelocity);
    return result; // Interpolation affects this copy only, never stored physics ticks.
}

const char* phaseName(MotionPhase phase) {
    switch (phase) {
    case MotionPhase::Airborne: return "AIRBORNE";
    case MotionPhase::Sliding: return "SLIDING";
    case MotionPhase::Rolling: return "ROLLING";
    case MotionPhase::Resting: return "RESTING";
    }
    return "UNKNOWN";
}

std::uint64_t frameCount(double duration, int fps) {
    if (!std::isfinite(duration) || duration < 0 || fps < 1)
        throw std::invalid_argument("Invalid frame timeline");
    return static_cast<std::uint64_t>(std::ceil(duration*fps))+1;
}

} // namespace golf::demo
