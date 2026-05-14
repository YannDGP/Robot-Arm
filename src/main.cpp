#include <raylib.h>
#include <raymath.h>
#include "Arm.hpp"
#include <fcntl.h>
#include <unistd.h>
#include "Pipe.hpp"
#include <sys/stat.h>

static constexpr int   SCREEN_W   = 1280;
static constexpr int   SCREEN_H   = 720;
static constexpr float JOINT_SPEED = 1.2f;  // rad/s

// Colors
static const Color C_BG        = { 15,  15,  20,  255 };
static const Color C_BASE      = { 80,  80,  90,  255 };
static const Color C_SEGMENT0  = { 70,  130, 200, 255 };
static const Color C_SEGMENT1  = { 100, 180, 140, 255 };
static const Color C_SEGMENT2  = { 210, 120, 60,  255 };
static const Color C_EFFECTOR  = { 230, 230, 80,  255 };
static const Color C_JOINT     = { 240, 240, 240, 255 };

static const Color SEGMENT_COLORS[DOF] = { C_SEGMENT0, C_SEGMENT1, C_SEGMENT2 };

// Draw

void drawArm(const Arm& arm) {
    auto positions = arm.jointPositions();

    // Socle
    Vector3 baseBottom = { positions[0].x, positions[0].y - 0.3f, positions[0].z };
    DrawCylinder(baseBottom, 0.5f, 0.4f, 0.3f, 16, C_BASE);
    DrawCylinderWires(baseBottom, 0.5f, 0.4f, 0.3f, 16, DARKGRAY);

    // Segments
    for (int i = 0; i < DOF; ++i) {
        Vector3 from = positions[i];
        Vector3 to   = positions[i + 1];

        // Cylinder
        DrawCylinderEx(from, to, 0.18f, 0.14f, 12, SEGMENT_COLORS[i]);
        DrawCylinderWiresEx(from, to, 0.18f, 0.14f, 12, BLACK);

        DrawSphere(from, 0.22f, C_JOINT);
    }

     // End effector
    DrawSphere(positions[DOF], 0.26f, C_EFFECTOR);
    DrawSphereWires(positions[DOF], 0.27f, 8, 8, BLACK);
}


// Main

int main() {
    InitWindow(SCREEN_W, SCREEN_H, "robot-arm — 3DOF simulation");
    SetTargetFPS(60);

    // Camera
    Camera3D camera = {};
    camera.position   = { 8.0f, 8.0f, 8.0f };
    camera.target     = { 0.0f, 2.0f, 0.0f };
    camera.up         = { 0.0f, 1.0f, 0.0f };
    camera.fovy       = 45.0f;
    camera.projection = CAMERA_PERSPECTIVE;

    Arm arm;
    mkfifo(PIPE_PATH, 0666);
    int fd = open(PIPE_PATH, O_RDONLY | O_NONBLOCK);

    float targetAngles[6] = { 0.0f };

    while (!WindowShouldClose()) {
 
        if (IsKeyDown(KEY_LEFT_SHIFT))
            UpdateCamera(&camera, CAMERA_FREE);
         

        ArmCommand cmd;
        if (read(fd, &cmd, sizeof(cmd)) == sizeof(cmd)) {
        for (int i = 0; i < 3; ++i) {
        targetAngles[i*2]   = cmd.joints[i].rx;
        targetAngles[i*2+1] = cmd.joints[i].ry;
            }
        }   

        float dt = GetFrameTime();
        for (int i = 0; i < 3; ++i) {
        float rx = arm.joint(i).rx + (targetAngles[i*2]   - arm.joint(i).rx) * 5.0f * dt;
        float ry = arm.joint(i).ry + (targetAngles[i*2+1] - arm.joint(i).ry) * 5.0f * dt;
        arm.setAngles(i, rx, ry);
        }
        
        BeginDrawing();
            ClearBackground(C_BG);

            BeginMode3D(camera);
                // Ground
                DrawGrid(20, 1.0f);
                drawArm(arm);
            EndMode3D();

        EndDrawing();
    }

    CloseWindow();
    return 0;
}
