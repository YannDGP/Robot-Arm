#include "IKSolver.hpp"
#include <algorithm>
#include <cmath>

// ---------- constants matching Arm.cpp ----------
static constexpr float LENGTHS[IKSolver::DOF] = { 3.0f, 2.0f, 1.0f };
static constexpr float BASE_Y  =  0.5f;
static constexpr float MIN_ANG = -6.28318f; // -2π
static constexpr float MAX_ANG =  6.28318f; //  2π
// ------------------------------------------------

// Forward kinematics — must be a byte-for-byte replica of Arm::jointPositions()
// so that IK angles feed the renderer with zero drift.
std::array<glm::vec3, IKSolver::DOF + 1> IKSolver::fk(const float angles[DOF * 2]) {
    std::array<glm::vec3, DOF + 1> pos;

    glm::mat4 T = glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, BASE_Y, 0.0f));
    pos[0] = glm::vec3(T[3]);

    for (int i = 0; i < DOF; ++i) {
        float rx = angles[i * 2];
        float ry = angles[i * 2 + 1];
        // Rotation order matches Arm.cpp: ry first, then rx
        T = glm::rotate(T, ry, glm::vec3(0.0f, 1.0f, 0.0f));
        T = glm::rotate(T, rx, glm::vec3(1.0f, 0.0f, 0.0f));
        T = glm::translate(T, glm::vec3(0.0f, LENGTHS[i], 0.0f));
        pos[i + 1] = glm::vec3(T[3]);
    }
    return pos;
}

bool IKSolver::solve(float angles[DOF * 2], const glm::vec3& target,
                     int maxIter, float tolerance) {
    const float eps     = 0.001f; // Finite-difference step for numerical Jacobian
    const float lambda2 = 0.04f;  // DLS damping — prevents blow-up at singularities
    const float maxStep = 0.15f;  // Per-iteration angle clamp (rad)

    // Project target onto reachable workspace boundary if unreachable
    const glm::vec3 base(0.0f, BASE_Y, 0.0f);
    const float totalLen = LENGTHS[0] + LENGTHS[1] + LENGTHS[2];
    glm::vec3 tgt = target;
    float dist = glm::length(tgt - base);
    if (dist > totalLen * 0.99f)
        tgt = base + (tgt - base) * (totalLen * 0.99f / dist);

    for (int iter = 0; iter < maxIter; ++iter) {
        auto  pos = fk(angles);
        glm::vec3 ee  = pos[DOF];
        glm::vec3 err = tgt - ee;
        if (glm::length(err) < tolerance) return true;

        // ---------- build numerical Jacobian ----------
        // J is 3×6 (3 Cartesian outputs, 6 angle parameters).
        // J[j] stores the j-th column of J: ∂ee/∂angles[j].
        glm::vec3 J[DOF * 2];
        for (int j = 0; j < DOF * 2; ++j) {
            float p[DOF * 2];
            std::copy(angles, angles + DOF * 2, p);
            p[j] += eps;
            J[j] = (fk(p)[DOF] - ee) / eps;
        }

        // ---------- Damped Least Squares ----------
        // Build JJᵀ (3×3).  GLM mat3 is column-major: M[col][row].
        glm::mat3 JJT(0.0f);
        for (int j = 0; j < DOF * 2; ++j)
            for (int c = 0; c < 3; ++c)
                for (int r = 0; r < 3; ++r)
                    JJT[c][r] += J[j][r] * J[j][c];

        // Add damping to diagonal
        JJT[0][0] += lambda2;
        JJT[1][1] += lambda2;
        JJT[2][2] += lambda2;

        // Solve: x = (JJᵀ + λ²I)⁻¹ · err
        glm::vec3 x = glm::inverse(JJT) * err;

        // Apply: Δθ = Jᵀ · x
        for (int j = 0; j < DOF * 2; ++j) {
            float step = std::clamp(glm::dot(J[j], x), -maxStep, maxStep);
            angles[j]  = std::clamp(angles[j] + step, MIN_ANG, MAX_ANG);
        }
    }

    return glm::length(tgt - fk(angles)[DOF]) < tolerance;
}
