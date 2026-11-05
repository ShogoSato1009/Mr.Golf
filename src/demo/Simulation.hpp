#pragma once

#include "demo/DebugConfig.hpp"

#include <cstdint>
#include <vector>

namespace golf::demo {

struct Simulation {
    double dt = 0;
    double duration = 0;
    double firstContactTime = -1; // Estimated to the nearest recorded tick.
    double restTime = -1;
    double apex = 0;
    std::vector<BallState> ticks;
    BallState sample(double time) const;
};

Simulation simulate(const DebugConfig& config);
const char* phaseName(MotionPhase phase);
std::uint64_t frameCount(double duration, int fps);

} // namespace golf::demo
