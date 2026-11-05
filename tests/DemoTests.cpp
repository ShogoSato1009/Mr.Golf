#include "demo/Simulation.hpp"

#include <iostream>
#include <stdexcept>

namespace {

void check(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

} // namespace

int main(int argc, char** argv) {
    try {
        check(argc == 2, "Expected path to demo.cfg");
        auto config = golf::demo::loadConfig(argv[1]);
        const auto shot = golf::demo::simulate(config);
        using golf::MotionPhase;
        check(shot.ticks.front().phase == MotionPhase::Airborne && shot.apex > 10, "Shot must fly");
        double rollTime = -1;
        for (std::size_t i = 0; i < shot.ticks.size() && rollTime < 0; ++i)
            if (shot.ticks[i].phase == MotionPhase::Rolling) rollTime = static_cast<double>(i)*shot.dt;
        check(shot.firstContactTime > 0 && rollTime > shot.firstContactTime && shot.restTime > rollTime &&
              shot.ticks.back().phase == MotionPhase::Resting &&
              golf::length(shot.ticks.back().velocity) == 0 && golf::length(shot.ticks.back().angularVelocity) == 0,
              "Expected flight, ground contact, rolling and complete rest");
        const auto last = shot.ticks.back().position;
        auto previousConfig = config;
        previousConfig.material.restitution = 0.35;
        previousConfig.material.impactBraking = 0;
        const auto previous = golf::demo::simulate(previousConfig);
        check(shot.firstContactTime == previous.firstContactTime, "Impact tuning changed flight time");
        const auto landingTick = static_cast<std::size_t>(std::llround(shot.firstContactTime/shot.dt));
        for (std::size_t i = 0; i < landingTick; ++i) {
            check(golf::length(shot.ticks[i].position-previous.ticks[i].position) == 0 &&
                  golf::length(shot.ticks[i].velocity-previous.ticks[i].velocity) == 0,
                  "Impact tuning changed the airborne trajectory");
        }
        const auto run = last-shot.ticks[landingTick].position;
        const auto oldRun = previous.ticks.back().position-previous.ticks[landingTick].position;
        const double runDistance = std::hypot(run.x, run.z);
        check(runDistance >= 14 && runDistance <= 34 && runDistance < 0.5*std::hypot(oldRun.x, oldRun.z),
              "Default shot should have a fairway-length run and improve on the old model");
        config.fps = 29; // 29 does not divide the 240 Hz physics rate.
        const auto alternate = golf::demo::simulate(config);
        check(alternate.ticks.size() == shot.ticks.size() && alternate.restTime == shot.restTime &&
              golf::length(alternate.ticks.back().position-last) == 0, "Video FPS changed the physics");
        const double t = 1.0/config.fps;
        const auto next = static_cast<std::size_t>(std::ceil(t/shot.dt));
        const double alpha = t/shot.dt-static_cast<double>(next-1);
        const auto lo = shot.ticks[next-1].position, hi = shot.ticks[next].position;
        check(golf::length(shot.sample(t).position - (lo+alpha*(hi-lo))) < 1e-12 &&
              golf::length(shot.sample(shot.duration+1).position-last) == 0,
              "Presentation does not sample the fixed-tick trajectory correctly");
        config.environment.windVelocity.z += 6;
        config.slopeX = 0.01;
        config.aero.dragCoefficient = 0.35;
        const auto changed = golf::demo::simulate(config);
        check(golf::length(changed.ticks.back().position-last) > 1, "Debug configuration did not alter trajectory");
        golf::Camera camera(config.camera);
        camera.reset({});
        const golf::Vec3 target{20, 5, 3};
        for (int i = 0; i < 60; ++i) camera.follow(target, 1.0/30);
        check(golf::length(camera.target()-target) < 0.01 &&
              golf::length(camera.eye()-camera.target()-config.camera.offset) < 1e-12,
              "Camera does not settle on the target");
        for (int i = 0; i < 60; ++i) camera.follow(target, 1.0/30, 0.18);
        check(golf::length(camera.eye()-camera.target()-0.18*config.camera.offset) < 0.01,
              "Ground view did not move closer to the ball");
        std::cout << "PASS flight, landing, calibrated run, rest, FPS independence, debug overrides, camera\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "FAIL " << e.what() << '\n';
        return 1;
    }
}
