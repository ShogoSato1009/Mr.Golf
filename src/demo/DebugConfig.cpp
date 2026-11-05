#include "demo/DebugConfig.hpp"

#include <fstream>
#include <map>
#include <sstream>
#include <stdexcept>

namespace golf::demo {

Terrain DebugConfig::makeTerrain() const {
    const auto c = *this;
    return Terrain([c](double x, double z) {
        const double k = 2*pi/c.hillLength;
        const double height = c.terrainHeight + c.slopeX*x + c.slopeZ*z +
                              c.hillAmplitude*std::sin(k*x)*std::cos(k*z);
        const double dx = c.slopeX + c.hillAmplitude*k*std::cos(k*x)*std::cos(k*z);
        const double dz = c.slopeZ - c.hillAmplitude*k*std::sin(k*x)*std::sin(k*z);
        return TerrainSample{height, {-dx, 1, -dz}, c.material};
    });
}

void DebugConfig::validate() const {
    const auto bounded = [](double x, double lo, double hi) {
        return std::isfinite(x) && x >= lo && x <= hi;
    };
    (void)BallPhysics(ball, aero, physics);
    (void)Camera(camera);
    validateMaterial(material);
    if (width < 320 || width > 3840 || height < 240 || height > 2160 || width%2 || height%2 ||
        fps < 1 || fps > 240 || !bounded(maxDuration, 0.1, 300) ||
        !bounded(endHold, 0.1, 10) || !bounded(physics.fixedTimeStep, 1.0/10000, 0.1) ||
        !isFinite(environment.windVelocity) || !isFinite(environment.gravity) ||
        !bounded(environment.airDensity, 0, 100) || !isFinite(start) || start.y < 0 ||
        !isFinite(shot.impulse) || !isFinite(shot.angularImpulse) ||
        !bounded(terrainHeight, -1000, 1000) || !bounded(slopeX, -1, 1) ||
        !bounded(slopeZ, -1, 1) || !bounded(hillAmplitude, -10, 10) ||
        !bounded(hillLength, 20, 1000))
        throw std::invalid_argument("Invalid debug configuration (see configs/demo.cfg)");
    (void)makeTerrain().sample(start.x, start.z);
}

DebugConfig loadConfig(const std::filesystem::path& path) {
    DebugConfig c;
    std::map<std::string, double*> values{
        {"mass", &c.ball.mass}, {"radius", &c.ball.radius}, {"inertia_factor", &c.ball.inertiaFactor},
        {"drag", &c.aero.dragCoefficient}, {"lift_slope", &c.aero.liftSlope},
        {"max_lift", &c.aero.maxLiftCoefficient}, {"spin_decay", &c.aero.spinDecayPerDensity},
        {"physics_dt", &c.physics.fixedTimeStep}, {"max_substep", &c.physics.maxSubstep},
        {"max_travel", &c.physics.maxTravelPerSubstep}, {"bounce_threshold", &c.physics.bounceThreshold},
        {"stop_speed", &c.physics.stopSpeed}, {"angular_stop_speed", &c.physics.angularStopSpeed},
        {"wind_x", &c.environment.windVelocity.x}, {"wind_y", &c.environment.windVelocity.y},
        {"wind_z", &c.environment.windVelocity.z}, {"gravity_x", &c.environment.gravity.x},
        {"gravity_y", &c.environment.gravity.y}, {"gravity_z", &c.environment.gravity.z},
        {"air_density", &c.environment.airDensity}, {"terrain_height", &c.terrainHeight},
        {"slope_x", &c.slopeX}, {"slope_z", &c.slopeZ}, {"hill_amplitude", &c.hillAmplitude},
        {"hill_length", &c.hillLength}, {"restitution", &c.material.restitution},
        {"static_friction", &c.material.staticFriction}, {"dynamic_friction", &c.material.dynamicFriction},
        {"rolling_resistance", &c.material.rollingResistance}, {"spin_friction", &c.material.spinningFriction},
        {"impact_braking", &c.material.impactBraking},
        {"start_x", &c.start.x}, {"start_height", &c.start.y}, {"start_z", &c.start.z},
        {"impulse_x", &c.shot.impulse.x}, {"impulse_y", &c.shot.impulse.y}, {"impulse_z", &c.shot.impulse.z},
        {"angular_impulse_x", &c.shot.angularImpulse.x},
        {"angular_impulse_y", &c.shot.angularImpulse.y}, {"angular_impulse_z", &c.shot.angularImpulse.z},
        {"camera_x", &c.camera.offset.x}, {"camera_y", &c.camera.offset.y}, {"camera_z", &c.camera.offset.z},
        {"camera_response", &c.camera.response}, {"camera_fov", &c.camera.verticalFov},
        {"max_duration", &c.maxDuration}, {"end_hold", &c.endHold}
    };
    std::map<std::string, int*> integers{{"width", &c.width}, {"height", &c.height}, {"fps", &c.fps}};
    std::ifstream file(path);
    if (!file) throw std::runtime_error("Cannot open config: " + path.u8string());
    std::string line;
    std::size_t number = 0;
    while (std::getline(file, line)) {
        ++number;
        line = line.substr(0, line.find('#'));
        if (line.find_first_not_of(" \t\r") == std::string::npos) continue;
        const auto equal = line.find('=');
        if (equal == std::string::npos) throw std::runtime_error("Missing '=' at config line " + std::to_string(number));
        std::istringstream keyStream(line.substr(0, equal)), valueStream(line.substr(equal+1));
        std::string key, extra;
        double value;
        if (!(keyStream >> key) || (keyStream >> extra) || !(valueStream >> value) ||
            (valueStream >> extra) || !std::isfinite(value))
            throw std::runtime_error("Invalid value at config line " + std::to_string(number));
        if (auto it = values.find(key); it != values.end()) *it->second = value;
        else if (auto integer = integers.find(key); integer != integers.end()) {
            if (value < 1 || value > 10000 || std::floor(value) != value)
                throw std::runtime_error("Expected positive integer: " + key);
            *integer->second = static_cast<int>(value);
        } else throw std::runtime_error("Unknown config key: " + key);
    }
    if (file.bad()) throw std::runtime_error("Failed to read config");
    c.validate();
    return c;
}

} // namespace golf::demo
