#pragma once

static constexpr const char* PIPE_PATH = "/tmp/arm_pipe";

struct JointCommand {
    float rx, ry;
};

struct ArmCommand {
    JointCommand joints[3];
};