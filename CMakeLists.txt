cmake_minimum_required(VERSION 3.20)
project(MrGolfPhysics VERSION 0.1.0 LANGUAGES CXX)

add_library(golf_physics
    src/physics/BallPhysics.cpp
    src/physics/Collision.cpp
    src/world/Terrain.cpp
)
target_include_directories(golf_physics PUBLIC src)
target_compile_features(golf_physics PUBLIC cxx_std_17)
set_target_properties(golf_physics PROPERTIES CXX_EXTENSIONS OFF)

function(golf_warnings target)
    if(MSVC)
        target_compile_options(${target} PRIVATE /W4 /permissive- /utf-8)
    else()
        target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic -Wconversion)
    endif()
endfunction()
golf_warnings(golf_physics)

option(GOLF_BUILD_DEMO "Build the Windows OpenGL visualization demo" ${WIN32})
if(GOLF_BUILD_DEMO)
    if(NOT WIN32)
        message(FATAL_ERROR "The visualization demo requires Windows")
    endif()
    add_library(golf_rendering
        src/rendering/Camera.cpp
        src/rendering/Renderer.cpp
    )
    target_include_directories(golf_rendering PUBLIC src)
    target_compile_features(golf_rendering PUBLIC cxx_std_17)
    target_link_libraries(golf_rendering PRIVATE golf_physics opengl32 gdi32 user32)
    golf_warnings(golf_rendering)

    add_library(golf_demo_simulation
        src/demo/DebugConfig.cpp
        src/demo/Simulation.cpp
    )
    target_link_libraries(golf_demo_simulation PUBLIC golf_physics golf_rendering)
    golf_warnings(golf_demo_simulation)

    add_executable(golf_demo src/main.cpp src/demo/RunDemo.cpp)
    target_link_libraries(golf_demo PRIVATE golf_demo_simulation)
    golf_warnings(golf_demo)
endif()

include(CTest)
if(BUILD_TESTING)
    add_executable(physics_tests tests/PhysicsTests.cpp)
    target_link_libraries(physics_tests PRIVATE golf_physics)
    golf_warnings(physics_tests)
    add_test(NAME physics_tests COMMAND physics_tests)
    if(GOLF_BUILD_DEMO)
        add_executable(demo_tests tests/DemoTests.cpp)
        target_link_libraries(demo_tests PRIVATE golf_demo_simulation)
        golf_warnings(demo_tests)
        add_test(NAME demo_tests COMMAND demo_tests configs/demo.cfg)
        set_tests_properties(demo_tests PROPERTIES WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}")
    endif()
endif()
