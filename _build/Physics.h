#pragma once
#include "Drone.h"

class Physics {
	private:
		static float fx(float thrustX, float velocity);
		static float fy(float thrustY, float velocity);
		static float fz(float thrustZ, float velocity);
		static constexpr float DRAG_X = Constants::DRAG_IN_X_DIRECTION;
		static constexpr float DRAG_Y = Constants::DRAG_IN_Y_DIRECTION;
		static constexpr float DRAG_Z = Constants::DRAG_IN_Z_DIRECTION;
	public:	
		static void updatePosition(Drone* drone, float timeStep);
		static void updateRotation();
		template <typename Function>
		static float rk4(float uj, float step, float thrust, Function func);
};

