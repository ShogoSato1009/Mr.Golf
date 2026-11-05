#include "demo/RunDemo.hpp"

#include <exception>
#include <iostream>
#include <string_view>

int main(int argc, char* argv[]) {
    using golf::demo::Mode;
    const std::string_view mode = argc > 1 ? argv[1] : "--preview";
    if (mode == "--help" || argc > 3 || (mode != "--preview" && mode != "--frames" && mode != "--summary")) {
        std::cerr << "Usage: golf_demo [--preview|--summary|--frames] [configs/demo.cfg]\n";
        return mode == "--help" ? 0 : 1;
    }
    try {
        return golf::demo::run(argc > 2 ? argv[2] : "configs/demo.cfg",
                               mode == "--frames" ? Mode::Frames : mode == "--summary" ? Mode::Summary : Mode::Preview);
    } catch (const std::exception& e) {
        std::cerr << "golf_demo: " << e.what() << '\n';
        return 1;
    }
}
