#include "Arm.hpp"
#include <algorithm>
#include <glm/gtc/matrix_transform.hpp>

Arm::Arm() {
    
    joints_[0] = { 3.0f,  0.0f, 0.0f,  -2*PI, 2*PI }; // Joint 0 — shoulder (longest segment, full rotation)
    joints_[1] = { 2.0f,  0.0f, 0.0f,  -2*PI, 2*PI }; // Joint 1 — elbow (medium segment, full rotation)
    joints_[2] = { 1.0f,  0.0f, 0.0f,  -2*PI, 2*PI }; // Joint 2 — wrist (shortest segment, full rotation)
}

void Arm::rotateJoint(int i, float delta) {
    joints_[i].ry = std::clamp(joints_[i].ry + delta,
                               joints_[i].minAngle, joints_[i].maxAngle);
}

void Arm::setAngles(int i, float rx, float ry) {
    joints_[i].rx = std::clamp(rx, joints_[i].minAngle, joints_[i].maxAngle);
    joints_[i].ry = std::clamp(ry, joints_[i].minAngle, joints_[i].maxAngle);
}

std::array<Vector3, DOF + 1> Arm::jointPositions() const {
    std::array<Vector3, DOF + 1> positions;

    // Start at base, slightly above ground
    glm::mat4 T = glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, 0.5f, 0.0f));

    positions[0] = { T[3][0], T[3][1], T[3][2] };

    for (int i = 0; i < DOF; ++i) {
        // Apply joint rotations in local frame
        T = glm::rotate(T, joints_[i].ry, glm::vec3(0.0f, 1.0f, 0.0f));
        T = glm::rotate(T, joints_[i].rx, glm::vec3(1.0f, 0.0f, 0.0f));

        // Advance along segment in local up direction
        T = glm::translate(T, glm::vec3(0.0f, joints_[i].length, 0.0f));

        positions[i + 1] = { T[3][0], T[3][1], T[3][2] };
    }

    return positions;
}