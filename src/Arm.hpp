#pragma once
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <array>
#include <raylib.h>
#include <raymath.h>

static constexpr int DOF = 3; // Degrees of Freedom

struct Joint {
    float length; // Segment length in world units
    float rx, ry; // Current rotation angles in radians (pitch, yaw)
    float minAngle, maxAngle; // Joint rotation limits
};

class Arm {
public:
    Arm();

    // Rotate joint i by delta radians (clamped to limits)
    void rotateJoint(int i, float delta);

    void setAngles(int i, float rx, float ry);

     // Returns world position of each joint (0 = base, DOF = end effector)
    std::array<Vector3, DOF + 1> jointPositions() const;


    const Joint& joint(int i) const { return joints_[i]; }

private:
    std::array<Joint, DOF> joints_;
};
