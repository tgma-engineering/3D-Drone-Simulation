#include <vector>
#include <array>

#ifndef CONSTANTS
#define CONSTANTS

class Constants {
	private:
	public:
		// Drone parameter Coefficients	
		static constexpr float DRONE_MASS = 0.468f; // Drone mass in kg
		static constexpr float PROPELLOR_RADIUS = 0.1f;
		static constexpr float DRONE_ARM_LENGTH = 0.1f;
		static constexpr float K_CONSTANT = 2.980e-6f; // Die k konstante die gemeint ist um Torque einfacher auszurechnene 
		static constexpr float B_CONSTANT = 1.140e-7f; // Die b konstante für Ausrechnen des Drehmomentes um z achse 
		static constexpr float GRAVITATION = 9.81f; // Gravitational constant

		static constexpr float HOVER_SPEED = 907.18607649f;
		
		// some Basic vectors
		inline static const std::vector<float> UNIT_VECTOR_X{ 1.0f, 0.0f, 0.0f };
		inline static const std::vector<float> UNIT_VECTOR_Y = { 0.0f, 1.0f, 0.0f };
		inline static std::vector<float> UNIT_VECTOR_Z{ 0.0f, 0.0f, 1.0f };

		// Drag Coefficients
		// Achtung hier auf null gesetzt
		static constexpr float DRAG_IN_X_DIRECTION = 0.01; //
		static constexpr float DRAG_IN_Y_DIRECTION = 0.01; //
		static constexpr float DRAG_IN_Z_DIRECTION = 0.01; //

		// mass inertias
		static constexpr float INERTIA_X = 4.856e-3f;
		static constexpr float INERTIA_Y = 4.856e-3f;
		static constexpr float INERTIA_Z = 4.856e-3f;

};
#endif // !CONSTANTS