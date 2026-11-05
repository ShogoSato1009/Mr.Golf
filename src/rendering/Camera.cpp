#include "rendering/Camera.hpp"

#include <stdexcept>

namespace golf {

Camera::Camera(CameraSettings settings) : settings_(settings) {
    if (!isFinite(settings.offset) || length(cross(settings.offset, {0, 1, 0})) < 0.1 ||
        !std::isfinite(settings.response) || settings.response <= 0 ||
        !std::isfinite(settings.verticalFov) || settings.verticalFov < 10 || settings.verticalFov > 120)
        throw std::invalid_argument("Invalid camera settings");
}

void Camera::reset(Vec3 target) {
    if (!isFinite(target)) throw std::invalid_argument("Invalid camera target");
    target_ = target;
    distanceScale_ = 1.0;
}

void Camera::follow(Vec3 target, double frameTime, double distanceScale) {
    if (!isFinite(target) || !std::isfinite(frameTime) || frameTime < 0 ||
        !std::isfinite(distanceScale) || distanceScale <= 0)
        throw std::invalid_argument("Invalid camera update");
    const double blend = 1.0 - std::exp(-settings_.response * frameTime);
    target_ += blend * (target - target_);
    distanceScale_ += blend * (distanceScale - distanceScale_);
}

} // namespace golf
