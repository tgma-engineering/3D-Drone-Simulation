#include <iostream>
#include "Drone.h"

using std::cout;

Propellor::Propellor() {
	speed = 0.0f;
	thrust = 0;
}

void Propellor::setSpeed(float newSpeed) {
	speed = newSpeed;
}

int Propellor::getSpeed() {
	return speed;
}

double Propellor::calculateThrust() {
 	return speed * speed * k;
}

Drone::Drone() {
	x_vec_drone_frame = Constants::UNIT_VECTOR_X;
	y_vec_drone_frame = Constants::UNIT_VECTOR_Y;
	z_vec_drone_frame = Constants::UNIT_VECTOR_Z;

	//cout << x_vec_drone_frame[0] << " " << x_vec_drone_frame[1] << " " << x_vec_drone_frame[2] << std::endl;
	//cout << y_vec_drone_frame[0] << " " << y_vec_drone_frame[1] << " " << y_vec_drone_frame[2] << std::endl;
	//cout << z_vec_drone_frame[0] << " " << z_vec_drone_frame[1] << " " << z_vec_drone_frame[2] << std::endl;

	propellors[0] = Propellor(); // Propellor 1
	propellors[1] = Propellor(); // Propellor 2
	propellors[2] = Propellor(); // Propellor 3
	propellors[3] = Propellor(); // Propellor 4

	// mark current rotation als no rotation
	currentRotation = Quaternion4();
}

double Drone::calculateThrust() {
	//cout << "Propellor 1 speed: " << propellors[0].getSpeed() << std::endl;
	//cout << "Propellor 2 speed: " << propellors[1].getSpeed() << std::endl;
	//cout << "Propellor 3 speed: " << propellors[2].getSpeed() << std::endl;
	//cout << "Propellor 4 speed: " << propellors[3].getSpeed() << std::endl;
	//cout << "Konstante k lautet: " << Constants::K_CONSTANT << std::endl;

	double thrust = 0;
	
	for (int i = 0; i < sizeof(propellors)/sizeof(propellors[0]); i++) {
	//	cout << "Thrust hier nach Berechnung: " << thrust << std::endl;
		thrust += propellors[i].calculateThrust();
	}

	return thrust;
}

void Drone::setPropellorSpeed(int propellor, float speed) {
	propellors[propellor].setSpeed(speed);
}

void Drone::setPropellorSpeed(float speed1, float speed2, float speed3, float speed4) {
	propellors[0].setSpeed(speed1);
	propellors[1].setSpeed(speed2);
	propellors[2].setSpeed(speed3);
	propellors[3].setSpeed(speed4);
}

Quaternion4 Drone::getCurrentRotationQuaternion() {
	return currentRotation;
}

std::vector<float> Drone::getPosition() {
	return position;
}

void Drone::setRotation(Quaternion4 quat) {
	currentRotation = quat;
}

void Drone::reset() {
	setRotation(Quaternion4());
	position = {0.0f, 0.0f, 0.0f};
	velocity = {0.0f, 0.0f, 0.0f};
}

std::vector<float> Drone::getVelocity() {
	return velocity;
}

void Drone::setPosition(std::vector<float> pos) {
	position = pos;
}

void Drone::setVelocity(std::vector<float> vel) {
	velocity = vel;
	Quaternion4::printVectorParams(velocity);
}

std::vector<float> Drone::getTorqueVector() {
	std::vector<float> torque = { 0.0f, 0.0f, 0.0f };

	torque[0] = Constants::DRONE_ARM_LENGTH * (propellors[0].getSpeed() * propellors[0].getSpeed() - propellors[2].getSpeed() * propellors[2].getSpeed());
	torque[1] = Constants::DRONE_ARM_LENGTH * (propellors[1].getSpeed() * propellors[1].getSpeed() - propellors[3].getSpeed() * propellors[3].getSpeed());
	float res = Constants::B_CONSTANT * (propellors[0].getSpeed() * propellors[0].getSpeed() - propellors[1].getSpeed() * propellors[1].getSpeed() + propellors[2].getSpeed() * propellors[2].getSpeed() - propellors[3].getSpeed() * propellors[3].getSpeed());
	torque[2] = res;

	return torque;
}

std::vector<float> Drone::getAngularVelocoty() {
	return angularVelocity;
}

void Drone::setAngularVelocity(std::vector<float> angvel) {
	angularVelocity = angvel;
}