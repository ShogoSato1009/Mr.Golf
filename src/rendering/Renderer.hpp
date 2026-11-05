#pragma once

#include "physics/BallState.hpp"
#include "rendering/Camera.hpp"
#include "world/Environment.hpp"
#include "world/Terrain.hpp"

#include <cstddef>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace golf {

struct RenderInfo {
    double time = 0;
    double duration = 0;
    double physicsDt = 0;
    int fps = 30;
    std::string phase;
    Environment environment;
    Vec3 launchPosition;
    std::optional<Vec3> landingPosition;
};

// Windows OpenGL offscreen bitmap. All scene data is read-only.
class Renderer {
public:
    Renderer(int width, int height, bool preview);
    ~Renderer();
    Renderer(const Renderer&) = delete;
    Renderer& operator=(const Renderer&) = delete;
    bool processEvents();
    void draw(const BallState& ball, double radius, const Terrain& terrain,
              const Camera& camera, const std::vector<Vec3>& trail, const RenderInfo& info);
    const unsigned char* pixels() const; // Bottom-up BGRX, tightly packed.
    std::size_t byteCount() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace golf
