# robot-arm

Real-time 3D simulation of a 3-DOF robotic arm in C++17, controlled via IPC (named pipe).

Two independent executables communicate in real time:
- `robot-arm` — 3D renderer (raylib + GLM), runs continuously
- `arm-controller` — control program, sends motion sequences

```
arm-controller  ──/tmp/arm_pipe──►  robot-arm
  (sequences)      (named pipe)     (3D renderer)
```

This architecture mirrors how ROS2 nodes communicate using decoupled processes exchanging messages through channels, where the renderer and controller can be developed, restarted, and debugged independently.

## Architecture

```
src/
├── Arm.hpp                    — kinematics (GLM 4x4 matrices, forward kinematics)
├── Arm.cpp
├── Pipe.hpp                   — shared IPC message structure
├── main.cpp                   — raylib renderer + pipe reader
└── controller/
    └── main.cpp               — motion sequences (Angles)
├── IKSolver.hpp               — kinematics (Damped Least Squares, inverse kinematics)
├── IKSolver.cpp               
└── controller-ik/
    └── main.cpp               — motion sequences (Targets)

vendor/
├── raylib/                    — 3D rendering (compiled with the project)
└── glm/                       — math library (header-only)
```

## How it works

### Kinematics

- **Forward Kinematics (FK):** Each joint holds a rotation (rx, ry) and a segment length. `jointPositions()` builds a chain of 4×4 transformation matrices, one per joint, multiplying them together from base to end effector. Each matrix encodes the cumulative rotation and translation up to that point, so joint 2 automatically inherits the orientation of joints 0 and 1.

- **Inverse Kinematics (IK):** The `IKSolver` implements a **Damped Least Squares (DLS)** algorithm. Given a 3D target position `(x, y, z)` in world space, it iteratively adjusts the joint angles to minimize the distance to the target. The damping factor ensures mathematical stability near singularities (e.g., when the arm is fully extended or collapsed).

### IPC

The controller opens `/tmp/arm_pipe` for writing. The renderer opens it for reading (`O_NONBLOCK` keeps running without a controller). Each `send()` call writes 24 bytes (3 joints × 2 floats × 4 bytes) into the pipe. The renderer reads incoming commands every frame and smoothly interpolates toward the target angles using a framerate-independent lerp.

## Dependencies

The following system libraries are required for windowing and input:

```bash
# Fedora
sudo dnf install gcc-c++ cmake libXinerama-devel libXcursor-devel libXi-devel libXrandr-devel

# Ubuntu / Debian
sudo apt install g++ cmake libxinerama-dev libxcursor-dev libxi-dev libxrandr-dev
```

All other dependencies (raylib, GLM) are vendored in the `vendor/` directory and compiled with the project, no system installation required.

## Build

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
```

To rebuild only the controller after modifying sequences:

```bash
cmake --build build --target arm-controller-(angles/ik)
```

## Run

In two separate terminals:

```bash
# Terminal 1 — start renderer first
./build/robot-arm

# Terminal 2 — start controller for Angles
./build/arm-controller-angles

# Terminal 2 — start controller for Inverse Kinetics
./build/arm-controller-ik

```

## Controls

| Input | Action |
|-------|--------|
| Hold `Shift` | Free camera (right-click + drag)-For mouvement (WASD/ZQSD) |
| `ESC` | Quit |

## Writing motion sequences (controller-angles)

In `src/controller/main.cpp`, each `send()` defines a target position:

```cpp
//         j0_rx   j0_ry  | j1_rx  j1_ry  | j2_rx  j2_ry  | ms
send(fd,   0.0f,   0.0f,    0.0f,  0.0f,    0.0f,  0.0f,   1000);
```

- `j0` / `j1` / `j2` — joint 0 (shoulder), joint 1 (elbow), joint 2 (wrist)
- `rx` — forward/backward rotation in degrees
- `ry` — left/right rotation in degrees
- `ms` — time before next command in milliseconds

Rotations are relative to the parent joint. The renderer interpolates smoothly to each target.

Voici le bloc complet mis à jour pour la section de l'Inverse Kinematics, rédigé exactement avec la même structure et le même style de commentaires et d'explications que ton bloc pour la Forward Kinematics :
Markdown

### Inverse Kinematics (controller-ik)

In `src/controller-ik/main.cpp`, each `moveTo()` defines a target position in world space:

```cpp
//           x      y      z     ms
moveTo(fd,  4.0f,  2.5f,  0.0f, 1500);