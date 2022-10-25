#include "Physics.h"

float Physics::fx(float thrustX, float velocity) {
	return thrustX / Constants::DRONE_MASS - velocity * DRAG_X / Constants::DRONE_MASS;
}

float Physics::fy(float thrustY, float velocity) {
	return (thrustY / Constants::DRONE_MASS) - velocity * (DRAG_Y / Constants::DRONE_MASS);
}

float Physics::fz(float thrustZ, float velocity) {
	return -1 * (Constants::GRAVITATION / Constants::DRONE_MASS) + (thrustZ / Constants::DRONE_MASS) - (velocity * (DRAG_Z / Constants::DRONE_MASS));
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
	float thrustNorm = drone->calculateThrust();
	
	//std::cout << thrustNorm << std::endl;
	
	float thrustX = thrustVector[0] * thrustNorm;
	float thrustY = thrustVector[1] * thrustNorm;
	float thrustZ = thrustVector[2] * thrustNorm;
	
	std::vector<float> currentDroneVelocity = drone->getVelocity();
	float currentVelocityX = currentDroneVelocity[0];
	float currentVelocityY = currentDroneVelocity[1];
	float currentVelocityZ = currentDroneVelocity[2];

	//std::cout << "Current Drone Velocity: ";
	//Quaternion4::printVectorParams(currentDroneVelocity);
	//std::cout << "Current Thrust: ";
	//Quaternion4::printVectorParams(thrustVector);

	//Quaternion4::printVectorParams(drone->getVelocity());

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

	if (newPositionZ < -1) {
		newPositionY = 0;
		newPositionX = 0;
		newPositionZ = 0;
		newVelocityX = 0;
		newVelocityY = 0;
		newVelocityZ = 0;
		drone->setRotation(Quaternion4(1, 0, 0, 0));
		drone->setPropellorSpeed(0, 0, 0, 0);
		drone->setAngularVelocity({0.0f, 0.0f, 0.0f});
	}

	//std::cout << "New Position: ";
	std::vector<float> newPosition = { newPositionX , newPositionY, newPositionZ };
	//Quaternion4::printVectorParams(newPosition);
	drone->setPosition(newPosition);

	//std::cout << "New Velocity: ";
	std::vector<float> newVelocity = { newVelocityX ,newVelocityY, newVelocityZ};
	//Quaternion4::printVectorParams(newVelocity);
	drone->setVelocity(newVelocity);
}

float Physics::wx(float torque, float angularVelocityY, float angularVelocityZ) {
	float inertiaX = Constants::INERTIA_X;
	float inertiaY = Constants::INERTIA_Y;
	float inertiaZ = Constants::INERTIA_Z;
	return (torque * (1 / inertiaX)) - ((inertiaY-inertiaZ) / (inertiaX)) * angularVelocityY * angularVelocityZ;
}

float Physics::wy(float torque, float angularVelocityX, float angularVelocityZ) {
	float inertiaX = Constants::INERTIA_X;
	float inertiaY = Constants::INERTIA_Y;
	float inertiaZ = Constants::INERTIA_Z;
	return (torque * (1 / inertiaY)) - ((inertiaZ - inertiaX) / (inertiaY)) * angularVelocityX * angularVelocityZ;
}

float Physics::wz(float torque, float angularVelocityX, float angularVelocityY) {
	float inertiaX = Constants::INERTIA_X;
	float inertiaY = Constants::INERTIA_Y;
	float inertiaZ = Constants::INERTIA_Z;
	return (torque * (1 / inertiaZ)) - ((inertiaX - inertiaY) / (inertiaZ)) * angularVelocityX * angularVelocityY;
}

template <typename Function>
float Physics::rk4q(float quat, float step, float angularVelocity, Function func) {
	float k1 = func(angularVelocity, quat);
	float k2 = func(angularVelocity, quat + (step / 2) * k1);
	float k3 = func(angularVelocity, quat + (step / 2) * k2);
	float k4 = func(angularVelocity, quat + step * k3);
	return quat + step * (1.f / 6) * (k1 + 2 * k2 + 2 * k3 + k4);
}

float funcquat(float angularVelocity, float quat) {
	return 0.5 * angularVelocity * quat;
}

void Physics::updateRotation(Drone* drone, float timeStep) {
	// getting params
	std::vector<float> angularVelocity = drone->getAngularVelocoty();
	std::vector<float> torque = drone->getTorqueVector();

	std::cout << "Current Angular Velocity: ";
	Quaternion4::printVectorParams(angularVelocity);

	std::cout << "Current Torque: ";
	Quaternion4::printVectorParams(angularVelocity);

	// calculate angular accleration
	float angularAccelerationX = wx(torque[0], angularVelocity[1], angularVelocity[2]);
	float angularAccelerationY = wy(torque[1], angularVelocity[0], angularVelocity[2]);
	float angularAccelerationZ = wz(torque[2], angularVelocity[0], angularVelocity[1]);

	std::vector<float> newAngularVel = {angularAccelerationX, angularAccelerationY, angularAccelerationZ};

	std::cout << "Angular Acceleration: ";
	Quaternion4::printVectorParams(newAngularVel);

	// Calculating new angular velocities
	float newAngularVelocityX = angularVelocity[0] + angularAccelerationX * timeStep;
	float newAngularVelocityY = angularVelocity[1] + angularAccelerationY * timeStep;
	float newAngularVelocityZ = angularVelocity[2] + angularAccelerationZ * timeStep;

	std::vector<float> newAngularVelocity = { newAngularVelocityX, newAngularVelocityY, newAngularVelocityZ };

	std::cout << "New Angular Velocity: ";
	Quaternion4::printVectorParams(newAngularVelocity);

	// updating drone angular velocity
	drone->setAngularVelocity(newAngularVelocity);

	// finding new Quaternion rotation values
	Quaternion4 currentRotation = drone->getCurrentRotationQuaternion();

	// calculating Rotation von hier an
	Quaternion4 angularVelQuat = Quaternion4(0, newAngularVelocityX, newAngularVelocityY, newAngularVelocityZ, false);

	// angular distance traveled = w * timeStep
	//angularVelQuat.multiply(timeStep);

	std::cout << "New Angular Velocity: ";
	angularVelQuat.printQuaternionInConsole();

	// Applying rotation traveled to Quanterion
	Quaternion4 qpunktQuaternion = Quaternion4::quaternionMultiplication(drone->getCurrentRotationQuaternion(), angularVelQuat);
	qpunktQuaternion.multiply(0.5);

	// q punkt mal timestamp rechnen
	qpunktQuaternion.multiply(timeStep);  // qpunkt ist jetzt roationsquat

	Quaternion4 newRotatedQuat = Quaternion4::addQuat(qpunktQuaternion, drone->getCurrentRotationQuaternion());

	drone->setRotation(newRotatedQuat);

	//float newW = rk4q(currentW, timeStep, angularVelQuat.getW(), funcquat);
	//float newX = rk4q(currentW, timeStep, angularVelQuat.getX(), funcquat);
	//float newY = rk4q(currentW, timeStep, angularVelQuat.getY(), funcquat);
	//float newZ = rk4q(currentW, timeStep, angularVelQuat.getZ(), funcquat);

	//drone->setRotation(Quaternion4(newW, newX, newY, newZ));
}