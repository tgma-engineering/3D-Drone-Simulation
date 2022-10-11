#include "Quaternion.h"

/**
 * Initializes Quternion object with (1,0,0,0)
 */
Quaternion4::Quaternion4() {
	Quaternion4 it = Quaternion4(1, 0, 0, 0);
	*this = it;
}

/**
 * Initializes normed Quternion object
 * @param shoudlNorm should be false if shouldNorm = false
 * @param w,x,y,z values of Quaternion
 */
Quaternion4::Quaternion4(float w, float x, float y, float z, bool shouldNorm) {
	// Normalizing Quaternion
	double vectorSum = w * w + x * x + y * y + z * z;
	double norm = sqrt(vectorSum);

	if (norm != 0 && shouldNorm) {
		this->w = w / norm;
		this->x = x / norm;
		this->y = y / norm;
		this->z = z / norm;
	}
	else {
		this->w = w;
		this->x = x;
		this->y = y;
		this->z = z;
	}
}

// Returns value of w
float Quaternion4::getW() {
	return w;
}

// Returns value of x
float Quaternion4::getX() {
	return x;
}

// Returns value of y
float Quaternion4::getY() {
	return y;
}

// Returns value of z
float Quaternion4::getZ() {
	return z;
}

/**
 * Method returns conjugated Quaternion 
 * 
 * @return conjugated Quaternion
 */
Quaternion4 Quaternion4::getConjugate() {
	return Quaternion4(w, -x, -y, -z);
};

/**
 * Method returns Quaternion containing information of rotation vector and rotaion angle
 *
 * @param angle angle around which rotation vector is to be rotated
 * @param rotationVector rotationVector, the vector around which the rotation takes place
 * @param degree if angle is given in degress, then true should be given here
 * @return Quaternion conjugated object
 */
Quaternion4 Quaternion4::getQuaternionFromVectorAngle(double angle, std::vector<float> rotationVector, bool degree) {
	// converting degree to rad
	if (degree) {
		angle = M_PI * angle / 180;
	}

	double half_angle = angle / 2;

	double norm = 0;
	for (int i = 0; i < rotationVector.size(); i++) {
		norm += rotationVector[i] * rotationVector[i];
	}

	norm = sqrt(norm);

	for (int i = 0; i < rotationVector.size(); i++) {
		rotationVector[i] = rotationVector[i] / norm;
	}

	double real_part = cos(half_angle);
	double i_part = sin(half_angle) * rotationVector[0];
	double j_part = sin(half_angle) * rotationVector[1];
	double k_part = sin(half_angle) * rotationVector[2];

	return Quaternion4(real_part, i_part, j_part, k_part);
};

/**
 * Method returns the real vector from a Quaternion
 *
 * @return std::vector containing x,y and z value of Quaternion
 */
std::vector<float> Quaternion4::getRealVector() {
	return std::vector<float> {x, y, z};
};

/**
 * Method returns w,x,y, and z parameter of Quaternion as std::vector object
 *
 * @return std::vector containing w,x,y and z value of Quaternion
 */
std::vector<float> Quaternion4::getQuaternionVector() {
	return std::vector<float> {w, x, y, z};
};

/**
 * Method returns the real vector from a Quaternion
 *
 * @param quaternion 
 * @return std::vector containing x,y and z value of Quaternion
 */
Quaternion4 Quaternion4::quaternionMultiplication(Quaternion4 quaternion1, Quaternion4 quaternion2) {
	double real_part = quaternion1.w * quaternion2.w - quaternion1.x * quaternion2.x - quaternion1.y * quaternion2.y - quaternion1.z * quaternion2.z;
	double i_part = quaternion1.w * quaternion2.x + quaternion1.x * quaternion2.w + quaternion1.y * quaternion2.z - quaternion1.z * quaternion2.y;
	double j_part = quaternion1.w * quaternion2.y + quaternion1.y * quaternion2.w + quaternion1.z * quaternion2.x - quaternion1.x * quaternion2.z;
	double k_part = quaternion1.w * quaternion2.z + quaternion1.z * quaternion2.w + quaternion1.x * quaternion2.y - quaternion1.y * quaternion2.x;

	return Quaternion4(real_part, i_part, j_part, k_part);
};

/**
 * Method returns new Quaternion after calculating rotation of vec
 *
 * @param rotainQuaternion, rotationQuaternion contains information about rotation, can be derived using getQuaternionFromVectorAngle()
 * @param vec, vector that the rotation should be applied to
 * 
 * @return Quaternion of vector after the rotation
 */
Quaternion4 Quaternion4::calculateRotation(Quaternion4 rotationQuaternion, std::vector<float> vec) {
	Quaternion4 rotationQuaternionInverse = rotationQuaternion.getConjugate();
	Quaternion4 quaternionVector = Quaternion4(0, vec[0], vec[1], vec[2], false);
	Quaternion4 q1 = Quaternion4::quaternionMultiplication(rotationQuaternion, quaternionVector);
	Quaternion4 result = Quaternion4::quaternionMultiplication(q1, rotationQuaternionInverse);
	float norm = sqrt(vec[0] * vec[0] + vec[1] * vec[1] + vec[2] * vec[2]);
	result.multiply(norm);
	return result;
};

/**
 * Method returns rotated vector
 *
 * @param rotainQuaternion, rotationQuaternion contains information about rotation, can be derived using getQuaternionFromVectorAngle()
 * @param vec, vector that the rotation should be applied to
 *
 * @return rotated vector 
 */
std::vector<float> Quaternion4::calculateRotationAndReturnVector(Quaternion4 rotationQuaternion, std::vector<float> vec) {
	return calculateRotation(rotationQuaternion, vec).getRealVector();
};

// Methods to print Quaternion in console
void Quaternion4::printQuaternionInConsole() {
	std::cout << "Quaternion values: w= " << w << " x= " << x << " y= " << y << " z= " << z << std::endl;
}

// method to print Vector from Quaternion
void Quaternion4::printVectorParams() {
	std::cout << "Vector values: x= " << x << " y= " << y << " z= " << z << std::endl;
}

float Quaternion4::x_rotation_angle() {
	double angle = atan2(2*(w*x + y * z), (1 - 2*(x*x + y*y)));
	return (float)angle * 180 / M_PI;
}

float Quaternion4::y_rotation_angle() {
	double value = 2 * (w * y - z * x);
	if (value > 1)
	{
		value = 1;
	}
	double angle = asin(value);
	return (float)angle * 180 / M_PI;
}

float Quaternion4::z_rotation_angle() {
	double angle = atan2(2 * (x * y + w * z), w * w - z * z - x * x - y * y);
	return (float)angle * 180 / M_PI;
}

void Quaternion4::multiply(float number) {
	this->w = w * number;
	this->x = x * number;
	this->y = y * number;
	this->z = z * number;
}

void Quaternion4::printVectorParams(std::vector<float> vec) {
	std::cout << "Vector values: x= " << vec[0] << " y= " << vec[1] << " z= " << vec[2] << std::endl;
}