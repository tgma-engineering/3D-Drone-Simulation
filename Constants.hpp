//
// Created by rasalghul on 25.09.2023.
//

#ifndef DRONE_SIMULATION_CONSTANTS_HPP
#define DRONE_SIMULATION_CONSTANTS_HPP
#include <vector>
#include "lib/eigen-3.4.0/Eigen/Dense"

using paramType = double;

class Constants {
public:
    // Drone parameter Coefficients
    static constexpr paramType DRONE_MASS = 0.468; // Drone mass in kg
    static constexpr paramType PROPELLOR_RADIUS = 0.03;
    static constexpr paramType DRONE_ARM_LENGTH = 0.1;
    static constexpr paramType K_CONSTANT = 2.980e-6; // Die k konstante die gemeint ist um Torque einfacher auszurechnene
    static constexpr paramType B_CONSTANT = 1.140e-7; // Die b konstante für Ausrechnen des Drehmomentes um z achse
    static constexpr paramType GRAVITATION = 9.81; // Gravitational constant
    static constexpr paramType MAX_ANGULAR_VELOCITY_MOTOR = 3000;

    static constexpr float HOVER_SPEED = 650;

    // some Basic vectors
    inline static const Eigen::Vector3<paramType> UNIT_VECTOR_X{ 1.0, 0.0, 0.0 };
    inline static const Eigen::Vector3<paramType> UNIT_VECTOR_Y = { 0.0, 1.0, 0.0 };
    inline static Eigen::Vector3<paramType> UNIT_VECTOR_Z{ 0.0, 0.0, 1.0 };
    inline static const Eigen::Vector3<paramType> GRAVITY_VECTOR{0, 0, - DRONE_MASS * GRAVITATION};

    // Drag Coefficients
    static constexpr paramType DRAG_IN_X_DIRECTION = .01; //
    static constexpr paramType DRAG_IN_Y_DIRECTION = .01; //
    static constexpr paramType DRAG_IN_Z_DIRECTION = .01; //

    // mass inertias
    static constexpr paramType INERTIA_X = 4.856e-3;
    static constexpr paramType INERTIA_Y = 4.856e-3;
    static constexpr paramType INERTIA_Z = 4.856e-3;
};

#endif //DRONE_SIMULATION_CONSTANTS_HPP
