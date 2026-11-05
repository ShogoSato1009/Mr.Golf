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

include(CTest)
if(BUILD_TESTING)
    add_executable(physics_tests tests/PhysicsTests.cpp)
    target_link_libraries(physics_tests PRIVATE golf_physics)
    golf_warnings(physics_tests)
    add_test(NAME physics_tests COMMAND physics_tests)
endif()
