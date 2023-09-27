#include "Drone.hpp"

Drone::Drone() {
    propellors[0] = Propellor(); // Propellor 1
    propellors[1] = Propellor(); // Propellor 2
    propellors[2] = Propellor(); // Propellor 3
    propellors[3] = Propellor(); // Propellor 4

    // reset();
    double speed = 10;
    propellors[0].setSpeed(speed);
    propellors[1].setSpeed(0);
    propellors[2].setSpeed(speed);
    propellors[3].setSpeed(speed);
}

void Drone::reset() {
    // Drone position
    position = {0.0, 0.0, 0.0};
    // Drone velocity
    velocity = {0.0, 0.0, 0.0};
    // Drone angular velocity
    angularVelocity = {0.0, 0.0, 0.0};
    // Rotation Ql,kjhbjklhöuaternion is in the direction of thrust vector
    rotation = Quaternion4<double>{Constants::UNIT_VECTOR_Z, 0};

    // set speed of all motors to hover speed
    setSpeedOfAllPropellors(Constants::HOVER_SPEED);
}

Eigen::Vector3d Drone::calculateTorqueVector() {
    // Propellor und Indiziesbelegung: 0 = 1, 1 = 2, 2=3, 3 = 4,
    double x = Constants::DRONE_ARM_LENGTH * Constants::K_CONSTANT * (propSpeedSquared(0) - propSpeedSquared(2));
    double y = Constants::DRONE_ARM_LENGTH * Constants::K_CONSTANT * (propSpeedSquared(1) - propSpeedSquared(3) );
    double z = Constants::B_CONSTANT * (propSpeedSquared(0) - propSpeedSquared(1) + propSpeedSquared(2) - propSpeedSquared(3));
    return Eigen::Vector3d{x, y, z};
}

Eigen::Vector3d Drone::calculateThrustVector() {
    // calculate Thrust
    double thrust = 0;
    for (int i = 0; i < sizeof(propellors)/sizeof(propellors[0]); i++) {
        thrust += propellors[i].calculateThrust();
    }

    Eigen::Vector3d thrustVec = thrust * Constants::UNIT_VECTOR_Z;
    // rotate thrust vector according to rotation quaternion
    thrustVec = rotation.rotateVectorAroundQuaternion(thrustVec);
    return thrustVec;
}

void Drone::setSpeedOfPropellor(int propellor, double speed) {
    propellors[propellor].setSpeed(speed);
}

void Drone::setSpeedOfAllPropellors(double speed) {
    propellors[0].setSpeed(speed);
    propellors[1].setSpeed(speed);
    propellors[2].setSpeed(speed);
    propellors[3].setSpeed(speed);
}

Propellor::Propellor() {
    speed = 0.0;
}

void Propellor::setSpeed(int newSpeed) {
    speed = Constants::MAX_ANGULAR_VELOCITY_MOTOR * newSpeed / 100;
}

double Propellor::getSpeed() {
    return speed;
}

double Propellor::calculateThrust() {
    return speed * speed * k;
}