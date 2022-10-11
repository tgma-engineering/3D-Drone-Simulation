#include "raylib.h"
#include <iostream>
#include <sstream>
#include <string>
#include "Drone.h"
#include "rlgl.h"
#include "Physics.h"

using std::cout; 
using std::endl;

std::string returnVectorCoordinatesString(Vector3 vec) {
	std::string c = "";
	double x = vec.x;
	float y = vec.y;
	float z = vec.z;
	c.append("x= " + std::to_string(x) + " y= " + std::to_string(y) + " z= " + std::to_string(z));
	return c;
};

Vector3 getVector3FromStdVec(std::vector<float> vec) {
	Vector3 vec3 = { vec[0] , vec[1] , vec[2] };
	return vec3;
}

void drawVector(std::vector<float> vec) {
	DrawLine3D({0,0,0}, getVector3FromStdVec(vec), RED);
}

void drawDroneBody(Drone drone) {
	// converting std::vetor to vector 3
	std::vector<float> vec = drone.getPosition();
	Vector3 pos = getVector3FromStdVec(vec);
	
	cout << "rotation angle x: " << drone.getCurrentRotationQuaternion().x_rotation_angle() << endl;
	cout << "rotation angle y: " << drone.getCurrentRotationQuaternion().y_rotation_angle() << endl;
	cout << "rotation angle z: " << drone.getCurrentRotationQuaternion().z_rotation_angle() << endl;
	
	float x_rotation_angle = drone.getCurrentRotationQuaternion().x_rotation_angle();
	float y_rotation_angle = drone.getCurrentRotationQuaternion().y_rotation_angle();
	float z_rotation_angle = drone.getCurrentRotationQuaternion().z_rotation_angle();

	//drone.getCurrentRotationQuaternion().printQuaternionInConsole();

	rlPushMatrix();
	// translating to change coordinate frame from ZXY to XYZ
	rlRotatef(-90, 1, 0, 0);
	rlRotatef(-90, 0, 0, 1);
	rlTranslatef(pos.x, pos.y, pos.z);

	DrawLine3D({0, 0, 0}, {1, 1 ,1}, RED);

	rlRotatef(x_rotation_angle, 1, 0, 0);
	rlRotatef(y_rotation_angle, 0, 1, 0);
	rlRotatef(z_rotation_angle, 0, 0, 1);
	
	DrawCube({0, 0 ,0}, 0.2f, 2.0f, 0.2f, BLUE);
	DrawCube({ 0, 0 ,0 }, 2.0f, 0.2f, 0.2f, BLUE);
	DrawSphere({ 0, 0 ,0 }, 0.15f, RED);

	
	rlPopMatrix();
	//DrawSphere(pos, 0.15f, GREEN);
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

	//Vector3 pos = { 0.0f, 0.0f, 0.0f };

	SetTargetFPS(60);
	SetCameraMode(cam, CAMERA_FREE);

	int fps = GetFPS();

	Drone drone = Drone();
	
	int counter = 0;
		
	while (!WindowShouldClose()) {
		//counter++;
		//cout << "counter " << counter << endl;
		
		/*
		if (counter <= 260) {
			drone.setRotation(Quaternion4::getQuaternionFromVectorAngle((double)counter, { 1, 1, 0}, true));
		}
		else if (counter <= 262) {
			drone.reset();
		}
		else if (counter <= 622) {
			drone.setRotation(Quaternion4::getQuaternionFromVectorAngle(counter, { 0, 1, 0 }, true));
		}
		else if (counter <= 624) {
			drone.reset();
		}
		else if (counter <= 720) {
			drone.setRotation(Quaternion4::getQuaternionFromVectorAngle(counter, { 0, 0, 1}, true));
		}
		*/

		counter++;
		if (counter > 60 && counter < 100) {
			drone.setRotation(Quaternion4::getQuaternionFromVectorAngle(45, { 1, 1, 1 }, true));
		}
		else if (counter > 100) {
			Constants::UNIT_VECTOR_Z = { 0, 0, 0 };
		}
	
		UpdateCamera(&cam);
		BeginDrawing();
		ClearBackground(RAYWHITE);
		BeginMode3D(cam);

		// Updating Physics
		Physics::updatePosition(&drone, 1.f / 60);

		// Drawing Drone
		drawDroneBody(drone);

		DrawGrid(100, 1.0f);

		EndMode3D();
		DrawFPS(10, 10);
		
		std::string text = "Drone Pos: " + returnVectorCoordinatesString(getVector3FromStdVec(drone.getPosition()));
		DrawText(text.c_str(), GetScreenWidth() - 600, 10, 25, LIME);
		text = "Drone Vel: " + returnVectorCoordinatesString(getVector3FromStdVec(drone.getVelocity()));
		DrawText(text.c_str(), GetScreenWidth() - 600, 35, 25, LIME);
		EndDrawing();
	}

	UnloadTexture(tex);
	UnloadModel(model);
	CloseWindow();	
}
