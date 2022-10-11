#include "Contants.h"
#include <array>
#include "Quaternion.h"

#ifndef DRONE
#define DRONE

class Propellor
{
private:
	int speed;
	double thrust;
	double k = Constants::K_CONSTANT;
public:
	Propellor();
	void setSpeed(int newSpeed); // set speed vlt von 0 bis 100 Prozent oder von 0 bis 255 an Arduino angelehnt, muss noch geändert werden
	double calculateThrust();
	int getSpeed();	
};

class Drone
{
	private:
		const double mass = Constants::DRONE_MASS;
		const double radius = Constants::PROPELLOR_RADIUS;
		// array with four propellors
		Propellor propellors[4]; // <- wurden hier aber Propelloren jetzt initializiert?????

		// Vector of drone plane
		std::vector<float> x_vec_drone_frame;
		std::vector<float> y_vec_drone_frame;
		// Thrust Vector shows in z direction
		std::vector<float> z_vec_drone_frame;

		Quaternion4 currentRotation;

		// Drone position
		std::vector<float> position = {0.0f, 0.0f, 0.0f};

		// Drone velocity
		std::vector<float> velocity = {0.0f, 0.0f, 0.0f};
	public:
		Drone();
		double calculateThrust();
		void setPropellorSpeed(int propellor, int speed);
		void setPropellorSpeed(int speed1, int speed2, int speed3, int speed4);
		void reset();

		// getters
		Quaternion4 getCurrentRotationQuaternion();
		std::vector<float> getPosition();
		std::vector<float> getVelocity(); // returns rotated velocity

		// setetrs
		void setRotation(Quaternion4 quat);
		void setPosition(std::vector<float> pos);
		void setVelocity(std::vector<float> vel);
};


#endif // !DRONE