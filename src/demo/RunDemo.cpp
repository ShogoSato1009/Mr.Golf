#include "demo/RunDemo.hpp"

#include "demo/Simulation.hpp"
#include "rendering/Renderer.hpp"

#include <fcntl.h>
#include <io.h>

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <thread>

namespace golf::demo {
namespace {

void printSummary(const Simulation& s) {
    std::cerr << std::fixed << std::setprecision(3);
    auto previous = s.ticks.front().phase;
    std::cerr << "0.000 s  " << phaseName(previous) << '\n';
    const bool landed = s.firstContactTime >= 0 && s.firstContactTime <= s.duration;
    if (landed) std::cerr << s.firstContactTime << " s  FIRST CONTACT (tick estimate)\n";
    std::optional<Vec3> firstRoll;
    for (std::size_t i = 1; i < s.ticks.size(); ++i) {
        const auto phase = s.ticks[i].phase;
        const double time = static_cast<double>(i)*s.dt;
        if (time > s.duration) break;
        if (phase != previous) {
            std::cerr << time << " s  " << phaseName(phase) << '\n';
            if (!firstRoll && phase == MotionPhase::Rolling) firstRoll = s.ticks[i].position;
            previous = phase;
        }
    }
    const auto endPosition = s.sample(s.duration).position;
    const Vec3 displacement = endPosition - s.ticks.front().position;
    std::cerr << "Apex above terrain: " << s.apex << " m; horizontal distance: "
              << std::hypot(displacement.x, displacement.z) << " m; duration: " << s.duration << " s\n";
    if (landed) {
        const auto landing = s.sample(s.firstContactTime).position;
        const auto carry = landing - s.ticks.front().position;
        const auto run = endPosition - landing;
        const auto tick = static_cast<std::size_t>(std::llround(s.firstContactTime/s.dt));
        const auto incoming = s.ticks[tick-1].velocity;
        const double angle = std::atan2(-incoming.y, std::hypot(incoming.x, incoming.z))*180/pi;
        std::cerr << "Carry: " << std::hypot(carry.x, carry.z) << " m; run: "
                  << std::hypot(run.x, run.z) << " m; landing angle: " << angle << " deg\n";
    }
    if (firstRoll) {
        const auto roll = endPosition - *firstRoll;
        std::cerr << "Horizontal distance from first rolling tick: " << std::hypot(roll.x, roll.z) << " m\n";
    }
    if (s.restTime < 0) std::cerr << "Ball did not stop by max_duration\n";
}

// Returns true only when the preview configuration changes.
bool present(const DebugConfig& c, const Simulation& s, Mode mode,
             const std::filesystem::path& configPath, std::filesystem::file_time_type version) {
    const bool preview = mode == Mode::Preview;
    const auto terrain = c.makeTerrain();
    Camera camera(c.camera);
    Renderer renderer(c.width, c.height, preview);
    const double frameTime = 1.0/c.fps;
    const auto count = frameCount(s.duration, c.fps);
    if (!preview) {
        if (_setmode(_fileno(stdout), _O_BINARY) == -1 ||
            std::printf("%d %d %d\n", c.width, c.height, c.fps) < 0 || std::fflush(stdout) != 0)
            throw std::runtime_error("Cannot initialize frame output");
    }
    const auto start = std::chrono::steady_clock::now();
    camera.reset(s.ticks.front().position);
    const auto landing = s.sample(s.firstContactTime).position;
    std::vector<Vec3> trail;
    std::size_t nextTrailTick = 0;
    const auto trailStride = static_cast<std::size_t>(std::max(1.0, std::round(0.05/s.dt)));
    for (std::uint64_t frame = 0; ; ++frame) {
        if (preview) {
            if (!renderer.processEvents()) return false;
            std::error_code ec;
            const auto modified = std::filesystem::last_write_time(configPath, ec);
            if (!ec && modified != version) return true;
        } else if (frame >= count) break;
        const double time = std::min(static_cast<double>(frame)*frameTime, s.duration);
        const auto ball = s.sample(time);
        // Velocity lead compensates the chase filter's lag during fast flight.
        const double aboveGround = ball.position.y - terrain.sample(ball.position.x, ball.position.z).height;
        const double distanceScale = ball.phase == MotionPhase::Airborne ? 1.0 : 0.18;
        camera.follow(ball.position + ball.velocity*0.16 - Vec3{0, std::clamp(aboveGround*0.23, 0.0, 4.0), 0},
                      frameTime, distanceScale);
        while (nextTrailTick < s.ticks.size() && static_cast<double>(nextTrailTick)*s.dt < time) {
            trail.push_back(s.ticks[nextTrailTick].position);
            nextTrailTick += trailStride;
        }
        RenderInfo info{time, s.duration, s.dt, c.fps, phaseName(ball.phase), c.environment,
                        s.ticks.front().position, std::nullopt};
        if (s.firstContactTime >= 0 && time >= s.firstContactTime) info.landingPosition = landing;
        renderer.draw(ball, c.ball.radius, terrain, camera, trail, info);
        if (!preview) {
            if (std::fwrite(renderer.pixels(), 1, renderer.byteCount(), stdout) != renderer.byteCount())
                throw std::runtime_error("Failed to write video frame");
        } else if (frame+1 >= count) {
            // Keep the final image visible and wait for a config edit or window close.
            while (renderer.processEvents()) {
                std::error_code ec;
                const auto modified = std::filesystem::last_write_time(configPath, ec);
                if (!ec && modified != version) return true;
                std::this_thread::sleep_for(std::chrono::milliseconds(80));
            }
            return false;
        } else {
            std::this_thread::sleep_until(start + std::chrono::duration<double>(static_cast<double>(frame+1)*frameTime));
        }
    }
    if (std::fflush(stdout) != 0) throw std::runtime_error("Cannot flush video frames");
    return false;
}

} // namespace

int run(const std::filesystem::path& configPath, Mode mode) {
    do {
        const auto version = std::filesystem::last_write_time(configPath);
        const auto c = loadConfig(configPath);
        const auto s = simulate(c);
        printSummary(s);
        if (mode == Mode::Summary) break;
        if (!present(c, s, mode, configPath, version)) break;
    } while (mode == Mode::Preview);
    return 0;
}

} // namespace golf::demo
