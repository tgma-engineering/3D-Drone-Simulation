#include <vector>
#include <array>

#ifndef CONSTANTS
#define CONSTANTS

class Constants {
	private:
	public:
		// Drone parameter Coefficients	
		static constexpr float DRONE_MASS = 1.0f; // Drone mass in kg
		static constexpr float PROPELLOR_RADIUS = 0.0f;
		static constexpr float PROPELLOR_ARM_LENGTH = 0.0f;
		static constexpr float K_CONSTANT = 1.0f; // Die k konstante die gemeint ist um Torque einfacher auszurechnene 
		static constexpr float GRAVITATION = 9.81f; // Gravitational constant

		// some Basic vectors
		inline static const std::vector<float> UNIT_VECTOR_X{ 1.0f, 0.0f, 0.0f };
		inline static const std::vector<float> UNIT_VECTOR_Y = { 0.0f, 1.0f, 0.0f };;
		inline static std::vector<float> UNIT_VECTOR_Z{ 0.0f, 0.0f, 20 };;

		// Drag Coefficients
		// Achtung hier auf null gesetzt
		static constexpr float DRAG_IN_X_DIRECTION = 0.01; //
		static constexpr float DRAG_IN_Y_DIRECTION = 0.01; //
		static constexpr float DRAG_IN_Z_DIRECTION = 0.01; //
};
#endif // !CONSTANTS