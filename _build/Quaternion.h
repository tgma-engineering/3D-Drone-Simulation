#define _USE_MATH_DEFINES

#include <vector>
#include <math.h>
#include <iostream>
#include <cmath>

#ifndef QUAT
#define QUAT

class Quaternion4 {
	private:
		float w, x, y, z;
	public:
		Quaternion4(bool shouldNorm = true);
		Quaternion4(float w, float x, float y, float z, bool shouldNorm = true);
		Quaternion4 getConjugate();
		static Quaternion4 getQuaternionFromVectorAngle(double angle, std::vector<float> rotationVector, bool degree);
		std::vector<float> getRealVector();
		std::vector<float> getQuaternionVector();
		void printQuaternionInConsole();
		void printVectorParams();
		void static printVectorParams(std::vector<float> vec);
		void multiply(float number);
		void addQuaternion(Quaternion4 quat);

		float getW();
		float getX();
		float getY();
		float getZ();

		// Calculations
		static Quaternion4 quaternionMultiplication(Quaternion4 quaternion1, Quaternion4 quaternion2);
		static Quaternion4 calculateRotation(Quaternion4 rotationQuaternion, std::vector<float> vec);
		static std::vector<float> calculateRotationAndReturnVector(Quaternion4 rotationQuaternion, std::vector<float> vec);
		static Quaternion4 addQuat(Quaternion4 quat1, Quaternion4 quat2);

		// Neu und in Entwicklung
		float x_rotation_angle();
		float y_rotation_angle();
		float z_rotation_angle();
};

#endif