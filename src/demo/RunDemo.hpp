#pragma once

#include <filesystem>

namespace golf::demo {

enum class Mode { Preview, Frames, Summary };
int run(const std::filesystem::path& configPath, Mode mode);

} // namespace golf::demo
