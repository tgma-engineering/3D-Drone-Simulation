#include <iostream>
#include "lib/eigen-3.4.0/Eigen/Dense"
#include "lib/raylib/src/raylib.h"
#include "lib/raylib/src/rlgl.h"
#include "lib/raylib/src/rcamera.h"
#include "Drone.hpp"

using std::cout;
using std::endl;

std::string returnVectorCoordinatesString(const Eigen::Vector3d vec) {
    std::string c = "";
    c.append("x= " + std::to_string(vec.x()) + " y= " + std::to_string(vec.y()) + " z= " + std::to_string(vec.z()));
    return c;
};

Vector3 getVector3FromStdVec(std::vector<float> vec) {
    Vector3 vec3 = { vec[0] , vec[1] , vec[2] };
    return vec3;
}

void drawVector(std::vector<float> vec) {
    DrawLine3D({0,0,0}, getVector3FromStdVec(vec), RED);
}

void drawDroneBody(const Drone& drone){
    // converting std::vetor to vector 3
    Eigen::Vector3d const dronePos = drone.position;
    Eigen::Vector3d pos = {0, 0, 0};
    Eigen::Vector3d rotAngle = drone.rotation.getRotationAngles();
    double x_rotation_angle = rotAngle.x() * 180 / PI;
    double y_rotation_angle = rotAngle.y() * 180 / PI;
    double z_rotation_angle = rotAngle.z() * 180 / PI;

    rlPushMatrix();
    // translating to change coordinate frame from ZXY to XYZ
    rlRotatef(-90, 1, 0, 0);
    rlRotatef(-90, 0, 0, 1);
    rlTranslatef(0, 0, 0);

    rlTranslatef(dronePos.x(), dronePos.y(), dronePos.z());
    rlRotatef(x_rotation_angle, 1, 0, 0);
    rlRotatef(y_rotation_angle, 0, 1, 0);
    rlRotatef(z_rotation_angle, 0, 0, 1);


    DrawLine3D({(float) pos.x(), (float) pos.y(), (float) pos.z()}, {(float) pos.x(), (float) pos.y(), (float) pos.z() + 2}, RED);
    DrawCube({(float) pos.x(), (float) pos.y(), (float) pos.z()}, 0.2f, 2.0f, 0.2f, BLUE);
    DrawCube({(float) pos.x(), (float) pos.y(), (float) pos.z()}, 2.0f, 0.2f, 0.2f, BLUE);
    DrawSphere({(float) pos.x(), (float) pos.y(), (float) pos.z()}, 0.15f, RED);

    // apply rotation in the drone frame
    // rlRotatef(x_rotation_angle, 1, 0, 0);
    // rlRotatef(y_rotation_angle, 0, 1, 0);
    // rlRotatef(z_rotation_angle, 0, 0, 1);

    // rlTranslatef(dronePos.x(), dronePos.y(), dronePos.z());

    rlPopMatrix();
}

int main()
{
    InitWindow(1920, 1080, "Drone Simulation");

    Model model = LoadModel("resources/drone.obj");
    Texture2D tex = LoadTexture("resources/duckt.png");
    model.materials[0].maps[MATERIAL_MAP_DIFFUSE].texture = tex;

    Camera cam = { 0 };
    cam.position = { 10.0f, 10.0f, 10.0f };
    cam.target = { 0.0f, 0.0f, 0.0f };
    cam.up = { 0.0f,1.0f,0.0f };
    cam.fovy = 90.0f;
    cam.projection = CAMERA_PERSPECTIVE;

    int fps = 60;
    SetTargetFPS(fps);
    int counter = 0;
    double dt = (double) 1.f/fps;
    Drone drone;

    while (!WindowShouldClose()) {

        Drone::update(drone, dt);

        UpdateCamera(&cam, CameraMode::CAMERA_FREE);
        BeginDrawing();
        ClearBackground(RAYWHITE);
        BeginMode3D(cam);

        // Drawing Drone
        drawDroneBody(drone);

        DrawGrid(100, 1.0f);

        EndMode3D();
        DrawFPS(10, 10);

        std::string text = "Drone Pos: " + returnVectorCoordinatesString(drone.getPosition());
        DrawText(text.c_str(), GetScreenWidth() - 600, 10, 25, LIME);
        text = "Drone Vel: " + returnVectorCoordinatesString(drone.getVelocity());
        DrawText(text.c_str(), GetScreenWidth() - 600, 35, 25, LIME);

        EndDrawing();

        counter = counter + 1;
    }

    UnloadTexture(tex);
    UnloadModel(model);
    CloseWindow();
}