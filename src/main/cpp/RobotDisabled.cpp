/*
 * RobotDisabled.cpp
 *
 *  Created on: Feb 11, 2017
 *      Author: pinkenbu
 */

#include "Robot.h"

void
Robot::DisabledInit()
{
	CameraLightOff();
	talon_left_master.Set(0.0);
	talon_right_master.Set(0.0);
	talon_left_slave.Set(0.0);
	talon_right_slave.Set(0.0);
	talon_winch.Set(0.0);
	talon_ballpick.Set(0.0);
	talon_ballshoot.Set(0.0);
	colliding = false;
	accelX = 0.0;
	accelY = 0.0;
	lastAccelX = 0.0;
	lastAccelY = 0.0;
}
