#pragma once
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <array>

// Inverse Kinematics solver for the 3-DOF robotic arm.
// Replicates Arm.cpp's FK exactly — same segment lengths, same
// rotation order (ry then rx), same base offset.
//
// angles layout: [rx0, ry0, rx1, ry1, rx2, ry2]
//
// Algorithm: Damped Least Squares (DLS / Levenberg-Marquardt)
//   Δθ = Jᵀ (JJᵀ + λ²I)⁻¹ · err
// Robust near singularities, works with full ±2π joint range.

class IKSolver {
public:
    static constexpr int DOF = 3;

    // Forward kinematics: returns [base, j1, j2, end-effector]
    static std::array<glm::vec3, DOF + 1> fk(const float angles[DOF * 2]);

    // Solve IK in-place.
    // angles is both the initial guess (warm-start) and the output.
    // Returns true when ||err|| < tolerance.
    static bool solve(float            angles[DOF * 2],
                      const glm::vec3& target,
                      int              maxIter   = 500,
                      float            tolerance = 0.04f);
};
