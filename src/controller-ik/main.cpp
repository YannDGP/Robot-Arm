#include <fcntl.h>
#include <unistd.h>
#include <cmath>
#include <iostream>
#include <glm/glm.hpp>
#include "Pipe.hpp"
#include "IKSolver.hpp"

// Warm-start: angles persist across calls so each solve starts
// from the previous solution — faster convergence, smoother motion.
static float g_angles[6] = {};

static void flushAngles(int fd, int ms) {
    ArmCommand cmd;
    for (int i = 0; i < 3; ++i)
        cmd.joints[i] = { g_angles[i * 2], g_angles[i * 2 + 1] };
    write(fd, &cmd, sizeof(cmd));
    usleep(ms * 1000);
}

// Move end-effector to world position (x, y, z).
static bool moveTo(int fd, float x, float y, float z, int ms) {
    glm::vec3 target(x, y, z);
    bool ok = IKSolver::solve(g_angles, target);
    if (!ok)
        std::cerr << "[IK] warning: did not fully converge for ("
                  << x << ", " << y << ", " << z << ")\n";
    else
        std::cout << "[IK] -> (" << x << ", " << y << ", " << z << ")\n";
    flushAngles(fd, ms);
    return ok;
}

int main() {
    int fd = open(PIPE_PATH, O_WRONLY);
    if (fd < 0) {
        std::cerr << "Cannot open pipe. Is robot-arm running?\n";
        return 1;
    }
    std::cout << "[IK Controller] Connected — Damped Least Squares IK\n";
    std::cout << "  Arm base: (0, 0.5, 0)  |  Total reach: 6 units\n\n";

    // Each moveTo() specifies a TARGET POSITION in world space.
    // The IK solver finds joint angles automatically.
    //
    // Arm base is at (0, 0.5, 0).
    // Rest pose (all angles = 0) has end-effector at (0, 6.5, 0).
    // Max reach from base ≈ 6 units.

    while (true) {
        //          x      y      z     ms
        moveTo(fd,  0.0f,  6.0f,  0.0f, 1500); // Straight up
        moveTo(fd,  4.0f,  2.5f,  0.0f, 1500); // Right
        moveTo(fd,  0.0f,  2.5f,  4.0f, 1500); // Front
        moveTo(fd, -4.0f,  2.5f,  0.0f, 1500); // Left
        moveTo(fd,  0.0f,  2.5f, -4.0f, 1500); // Back
        moveTo(fd,  3.0f,  1.0f,  3.0f, 1500); // Low front-right
        moveTo(fd, -3.0f,  4.5f, -3.0f, 1500); // High back-left
        moveTo(fd,  0.0f,  1.0f,  5.5f, 1500); // Far front, near max reach
        moveTo(fd,  5.0f,  0.8f,  0.0f, 1500); // Far right, near ground
        moveTo(fd, -5.0f,  0.8f,  0.0f, 1500); // Far left, near ground
        moveTo(fd,  2.5f,  5.0f,  2.5f, 1500); // High front-right
        moveTo(fd,  0.0f,  6.0f,  0.0f, 1500); // Back to top
    }

    close(fd);
    return 0;
}
