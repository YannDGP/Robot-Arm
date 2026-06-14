cmake_minimum_required(VERSION 3.16)
project(robot-arm LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Raylib
set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(BUILD_GAMES    OFF CACHE BOOL "" FORCE)
add_subdirectory(vendor/raylib)

# GLM (header-only)
add_subdirectory(vendor/glm)

# ── Renderer ────────────────────────────────────────────────────────────────
add_executable(robot-arm
    src/main.cpp
    src/Arm.cpp
)
target_include_directories(robot-arm PRIVATE src)
target_link_libraries(robot-arm PRIVATE raylib glm::glm)

# ── Angle controller (original, unchanged) ──────────────────────────────────
add_executable(arm-controller-angles
    src/controller/main.cpp
)
target_include_directories(arm-controller-angles PRIVATE src)

# ── IK Solver (static lib, GLM only, no raylib) ─────────────────────────────
add_library(ik-solver STATIC
    src/IKSolver.cpp
)
target_include_directories(ik-solver PUBLIC src)
target_link_libraries(ik-solver PUBLIC glm::glm)

# ── IK controller ───────────────────────────────────────────────────────────
add_executable(arm-controller-ik
    src/controller-ik/main.cpp
)
target_include_directories(arm-controller-ik PRIVATE src)
target_link_libraries(arm-controller-ik PRIVATE ik-solver)
