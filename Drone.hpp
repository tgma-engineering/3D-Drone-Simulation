#include "Constants.hpp"
#include <array>
#include "Quaternion.hpp"

#ifndef DRONE
#define DRONE

class Propellor
{
private:
    double speed;
    double thrust;
    double k = Constants::K_CONSTANT;
public:
    Propellor();
    void setSpeed(double newSpeed); // set speed vlt von 0 bis 100 Prozent oder von 0 bis 255 an Arduino angelehnt, muss noch geändert werden
    double calculateThrust();
    double getSpeed();
};

class Drone
{
private:
    const double mass = Constants::DRONE_MASS;
    const double radius = Constants::PROPELLOR_RADIUS;
    // array with four propellors
    Propellor propellors[4];

    // Drone position
    Eigen::Vector3d position = {0.0, 0.0, 0.0};
    // Drone velocity
    Eigen::Vector3d velocity = {0.0, 0.0, 0.0};
    // Drone angular velocity
    Eigen::Vector3d angularVelocity = {0.0, 0.0, 0.0};
    // Rotation Quaternion is in the direction of thrust vector
    Quaternion4<double> rotation{Constants::UNIT_VECTOR_Z, 0};

    double propSpeedSquared(int propellor){
        return propellors[propellor].getSpeed() * propellors[propellor].getSpeed();
    };
public:
    // Initializes Drone
    Drone();

    void reset();
    // set speed of propellor, int value should be between 0 and 1
    void setSpeedOfPropellor(int propellor, double speed);
    void setSpeedOfAllPropellors(double speed);

    Eigen::Vector3d calculateThrustVector();
    Eigen::Vector3d calculateTorqueVector();

    Eigen::Vector3d getPosition(){
        return position;
    };

    Eigen::Vector3d getVelocity(){
        return velocity;
    };

    Quaternion4<double> getRotation(){
        return rotation;
    }

    // Euler step for updating state of drone
    static void update(Drone& drone, double dt){
        // update position
        drone.position = drone.position + drone.velocity * dt;

        // update velocity
        Eigen::Matrix3d friction;
        friction << Constants::DRAG_IN_X_DIRECTION, 0, 0,
                0, Constants::DRAG_IN_Y_DIRECTION, 0,
                0, 0, Constants::DRAG_IN_Z_DIRECTION;
        drone.velocity = drone.velocity + (dt / Constants::DRONE_MASS) * (Constants::GRAVITY_VECTOR - friction * drone.velocity + drone.calculateThrustVector());

        // update rotation quaternion
        Quaternion4<double> angularVelQuat = Quaternion4<double>{drone.angularVelocity};
        drone.rotation = drone.rotation + angularVelQuat * drone.rotation * 0.5 * dt;
        drone.rotation = drone.rotation.normalize();

        // update angular velocity
        Eigen::Vector3d torqueVec = drone.calculateTorqueVector();
        Eigen::Vector3d angVel = drone.angularVelocity;
        double ix = Constants::INERTIA_X;
        double iy = Constants::INERTIA_Y;
        double iz = Constants::INERTIA_Z;
        double angularVelX = torqueVec.x() / ix - ((iy - iz) / ix) * angVel.y() * angVel.z();
        double angularVelY = torqueVec.y() / iy - ((iz - ix) / iy) * angVel.x() * angVel.z();
        double angularVelZ = torqueVec.z() / iz - ((ix - iy) / iz) * angVel.x() * angVel.y();

        Eigen::Vector3d dAngularVel = Eigen::Vector3d{angularVelX, angularVelY, angularVelZ};
        drone.angularVelocity = drone.angularVelocity + dt * dAngularVel;

    }

    friend void drawDroneBody(const Drone& drone);
};


#endif // !DRONE