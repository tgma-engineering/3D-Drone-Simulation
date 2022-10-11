#include "Quaternion.h"
#include "Drone.h"

using std::cout;
using std::endl;

/*
int main(int argc, char const* argv[])
{
    // Testing Drone class
    Drone drone = Drone();
    double thrust = 0;
    thrust = drone.calculateThrust();
    cout << "Thrust: " << thrust << endl;
    drone.setPropellorSpeed(1, 1);
    thrust = drone.calculateThrust();
    cout << "Thrust: " << thrust << endl;
    drone.setPropellorSpeed(1, 1, 1, 1);
    thrust = drone.calculateThrust();
    cout << "Thrust: " << thrust << endl;

    Quaternion currentRotationQuaternion = drone.getCurrentRotationQuaternion();
    currentRotationQuaternion.printQuaternionInConsole();
    Quaternion quat = Quaternion::calculateRotation(currentRotationQuaternion, Constants::UNIT_VECTOR_Y);
    quat.printQuaternionInConsole();
    quat.printVectorParams();
    
    

    //std::vector<double> vec = Quaternion::calculateRotationAndReturnVector(, Constants::UNIT_VECTOR_X);

    // Testing quaternions
    /*
    std::vector<double> vec1 = { 1, 0, 0 };
    std::vector<double> vec2 = { 0, 1, 0 };
    std::vector<double> vec3 = { 0, 0, 1 };

    double angle = 90;
    std::vector<double> rotation_axix_x = { 1, 0, 0 };
    std::vector<double> rotation_axix_y = { 0, 1, 0 };
    std::vector<double> rotation_axix_z = { 0, 0, 1 };

    cout << "Starting..." << endl;

    Quaternion rotationQuaternion = Quaternion::getQuaternionFromVectorAngle(90, rotation_axix_x, true);
    rotationQuaternion.printQuaternionInConsole();
        
    Quaternion rotated;
    rotated = rotated.calculateRotation(rotationQuaternion, vec2);
    rotated.printQuaternionInConsole();
    std::vector<double> vec = rotated.getRealVector();
    rotated.printVectorParams(vec);
    rotated = rotated.calculateRotation(rotationQuaternion.getQuaternionFromVectorAngle(90, rotation_axix_y, true), vec);
    rotated.printQuaternionInConsole();
    //
}
*/