#include "Physics.h"

float Physics::fx(float thrustX, float velocity) {
	return thrustX / Constants::DRONE_MASS - velocity * DRAG_X / Constants::DRONE_MASS;
}

float Physics::fy(float thrustY, float velocity) {
	return (thrustY / Constants::DRONE_MASS) - velocity * (DRAG_Y / Constants::DRONE_MASS);
}

float Physics::fz(float thrustZ, float velocity) {
	return -1 * (Constants::GRAVITATION) + (thrustZ / Constants::DRONE_MASS) - (velocity * (DRAG_Z / Constants::DRONE_MASS));
}

// rk4 for calculating velocity, taveles Distance is then given by result * t + s0
// uj is velocity
template <typename Function>	
float Physics::rk4(float uj, float step, float thrust, Function func) {
	float k1 = func(thrust, uj);
	float k2 = func(thrust, uj + (step / 2) * k1);
	float k3 = func(thrust, uj + (step / 2) * k2);
	float k4 = func(thrust, uj + step * k3);
	return uj + step * (1.f / 6) * (k1 + 2 * k2 + 2 * k3 + k4);
}

void Physics::updatePosition(Drone* drone, float timeStep) {
	// fps ist die Schrittweite
	// berechne x, y und z
	std::vector<float> thrustVector = Quaternion4::calculateRotationAndReturnVector(drone->getCurrentRotationQuaternion(), Constants::UNIT_VECTOR_Z);
	float thrustX = thrustVector[0];
	float thrustY = thrustVector[1];
	float thrustZ = thrustVector[2];
	
	std::vector<float> currentDroneVelocity = drone->getVelocity();
	float currentVelocityX = currentDroneVelocity[0];
	float currentVelocityY = currentDroneVelocity[1];
	float currentVelocityZ = currentDroneVelocity[2];

	std::cout << "Current Drone Velocity: ";
	Quaternion4::printVectorParams(currentDroneVelocity);
	std::cout << "Current Thrust: ";
	Quaternion4::printVectorParams(thrustVector);

	Quaternion4::printVectorParams(drone->getVelocity());

	float newVelocityX = rk4(currentVelocityX, timeStep, thrustX, fx);
	float newVelocityY = rk4(currentVelocityY, timeStep, thrustY, fy);
	float newVelocityZ = rk4(currentVelocityZ, timeStep, thrustZ, fz);

	std::vector<float> currentPosition = drone->getPosition();
	float positionX = currentPosition[0];
	float positionY = currentPosition[1];
	float positionZ = currentPosition[2];
	
	float newPositionX = positionX + newVelocityX * timeStep;
	float newPositionY = positionY + newVelocityY * timeStep;
	float newPositionZ = positionZ + newVelocityZ * timeStep;

	std::cout << "New Position: ";
	std::vector<float> newPosition = { newPositionX , newPositionY, newPositionZ };
	Quaternion4::printVectorParams(newPosition);
	drone->setPosition(newPosition);

	std::cout << "New Velocity: ";
	std::vector<float> newVelocity = { newVelocityX ,newVelocityY, newVelocityZ};
	Quaternion4::printVectorParams(newVelocity);
	drone->setVelocity(newVelocity);
}

void Physics::updateRotation() {}