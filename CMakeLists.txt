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

# Executable
add_executable(robot-arm
    src/main.cpp
    src/Arm.cpp
)

target_include_directories(robot-arm PRIVATE src)

target_link_libraries(robot-arm PRIVATE
    raylib
    glm::glm
)

add_executable(arm-controller
    src/controller/main.cpp
)

target_include_directories(arm-controller PRIVATE src)
