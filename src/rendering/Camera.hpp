#pragma once

#include "core/Math.hpp"

namespace golf {

struct CameraSettings {
    Vec3 offset{-10.0, 5.0, 14.0};
    double response = 5.0;
    double verticalFov = 48.0;
};

// Receives presentation positions only; never owns or changes physics state.
class Camera {
public:
    explicit Camera(CameraSettings settings = {});
    void reset(Vec3 target);
    void follow(Vec3 target, double frameTime, double distanceScale = 1.0);
    Vec3 eye() const { return target_ + distanceScale_*settings_.offset; }
    Vec3 target() const { return target_; }
    double verticalFov() const { return settings_.verticalFov; }

private:
    CameraSettings settings_;
    Vec3 target_;
    double distanceScale_ = 1.0;
};

} // namespace golf
