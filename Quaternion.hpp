//
// Created by rasalghul on 25.09.2023.
//

#ifndef DRONE_QUATERNION_HPP
#define DRONE_QUATERNION_HPP
#include <iostream>
#include "lib/eigen-3.4.0/Eigen/Dense"

template <typename T>
using vec = Eigen::Vector3<T>;

template <typename T>
class Quaternion4 {
private:
    T w, x, y, z;
public:
    // Constructor
    Quaternion4(T w, T x, T y, T z) : w(w), x(x), y(y), z(z) {}

    // Construct quaternion from vector
    explicit Quaternion4(vec<T> vec): w(0), x(vec.x()), y(vec.y()), z(vec.z()){}

    explicit Quaternion4(vec<T> vec, double rad){
        w = cos(rad / 2);
        x = sin(rad / 2) * vec.x();
        y = sin(rad / 2) * vec.y();
        z = sin(rad / 2) * vec.z();
    }

    // Adding two Quaternions
    Quaternion4<T> operator+(const Quaternion4<T>& other){
        return Quaternion4<T>(w + other.w, x + other.x, y + other.y, z + other.z);
    }

    // Subtracting two quaternions
    Quaternion4<T> operator-(const Quaternion4<T>& other){
        return Quaternion4<T>(w - other.w, x - other.x, y - other.y, z - other.z);
    }

    // Multiplying two quaternions
    Quaternion4<T> operator*(const Quaternion4<T>& other){
        T real_part = w * other.w - x * other.x - y * other.y - z * other.z;
        T i_part = w * other.x + x * other.w + y * other.z - z * other.y;
        T j_part = w * other.y + y * other.w + z * other.x - x * other.z;
        T k_part = w * other.z + z * other.w + x * other.y - y * other.x;
        return Quaternion4<T>(real_part, i_part, j_part, k_part);
    }

    Quaternion4<T> operator*(const double scalar){
        return Quaternion4<T>(w * scalar, x * scalar, y * scalar, z * scalar);
    }


    // Returns norm of quaternion
    T norm();
    // Returns Conjugate of quatenrion
    Quaternion4<T> conjugate();
    // Returns inverse of quaternion
    Quaternion4<T> inverse();
    // Returns quaternion normalized
    Quaternion4<T> normalize();

    // returns imaginary vector from quaternion
    vec<T> imaginaryVector();

    // returns rotated vector around rotation axis with given rad
    static vec<T> rotatedVector(vec<T> vector, vec<T> axisVector, double rad);

    vec<T> convertQuaternionToAxisAngleRepresentation(){
        T theta = 2 * acos(w);
        T divisor = sin(theta / 2);

        assert(theta != 0);
        return vec<T>{x / divisor, y / divisor, z / divisor};
    }

    vec<T> rotateVectorAroundQuaternion(vec<T> vector);

    vec<T> getRotationAngles() const;

    // operator for printing quaternion
    template <typename S>
    friend std::ostream& operator<<(std::ostream&os, const Quaternion4<S>& q);
};

template<typename T>
vec<T> Quaternion4<T>::getRotationAngles() const{
    double phi = atan2(2*(w*x + y*z), w*w-x*x-y*y-z*z);
    double theta = asin(2*(w*y - z*x));
    double psi = atan2(2*(w*z + x*y), w*w + x*x -y*y -z*z);
    return vec<T>{phi, theta, psi};
}

template<typename T>
vec<T> Quaternion4<T>::rotateVectorAroundQuaternion(vec<T> vector) {
    Quaternion4<T> rotatedQuaternion = *this * Quaternion4(vector) * this->inverse();
    return rotatedQuaternion.imaginaryVector();
}

template<typename T>
vec<T> Quaternion4<T>::rotatedVector(vec<T> vector, vec<T> axisVector, double rad) {
    if(axisVector.norm() != 0){
        // norm rotation vector
        axisVector /= axisVector.norm();

        Quaternion4<T> rotationQuaternion = Quaternion4(axisVector, rad);
        Quaternion4<T> vectorQuaternion = Quaternion4(vector);
        Quaternion4<T> rotationQuaternionInverse = rotationQuaternion.inverse();
        // std::cout << rotationQuaternion.convertQuaternionToAxisAngleRepresentation() << std::endl;
        Quaternion4<T> rotatedQuaternion = rotationQuaternion * vectorQuaternion * rotationQuaternionInverse;
        return rotatedQuaternion.imaginaryVector();
    } else {
        return vector;
    }
}

template <typename S>
std::ostream& operator<<(std::ostream&os, const Quaternion4<S>& q){
    std::cout << "w: " << q.w << " x: " << q.x << " y: " << q.y << " z: " << q.z <<std::endl;
    return os;
}

template<typename T>
vec<T> Quaternion4<T>::imaginaryVector() {
    return vec<T>{x, y, z};
}

template<typename T>
Quaternion4<T> Quaternion4<T>::normalize() {
    T norm = this->norm();
    T norm_w = w / norm;
    T norm_x = x / norm;
    T norm_y = y / norm;
    T norm_z = z / norm;
    return Quaternion4<T>(norm_w, norm_x, norm_y, norm_z);
}

template<typename T>
Quaternion4<T> Quaternion4<T>::inverse() {
    Quaternion4<T> conjugate = this->conjugate();
    T divisor = pow(norm(), 2);
    return Quaternion4<T>(conjugate.w / divisor, conjugate.x / divisor, conjugate.y / divisor, conjugate.z / divisor);
}

template<typename T>
Quaternion4<T> Quaternion4<T>::conjugate() {
    return Quaternion4<T>(w, -x, -y, -z);
}

template<typename T>
T Quaternion4<T>::norm() {
    return sqrt(w*w + x*x + y*y + z*z);
}

#endif //DRONE_QUATERNION_HPP
