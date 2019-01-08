/*
 * RobotTest.cpp
 *
 *  Created on: Jan 30, 2017
 *      Author: pinkenbu
 */

#include "Robot.h"

using namespace std;
void Robot::TestPeriodic() {


//	lw->Run();
	cout << "angle " << ahrs->GetAngle() << endl;

//	if (m_encoder_right.GetDistance() > 10) {
//		talon_left_master.Set(0.);
//		talon_right_master.Set(0.);
//	}
	talon_left_master.Set(0.0);
	talon_right_master.Set(0.0);
	talon_left_slave.Set(0.0);
	talon_right_slave.Set(0.0);
	float right = xboxcontroller0.GetY(Joystick::kRightHand)/3.;
	float left = xboxcontroller0.GetY(Joystick::kLeftHand)/3.;
	//talon_left_master.Set(left);
	//talon_left_slave.Set(left);
	//talon_right_master.Set(right);
	//talon_right_slave.Set(right);
	calzone.Set(right);
	frc::SmartDashboard::PutNumber("Encoder Distance L",
			m_encoder_left.GetDistance());
	frc::SmartDashboard::PutNumber("Encoder Distance R",
			m_encoder_right.GetDistance());

	// Retrieve the current rate of the encoder.
	frc::SmartDashboard::PutNumber("Encoder Rate L", m_encoder_left.GetRate());
	frc::SmartDashboard::PutNumber("Encoder Rate R", m_encoder_right.GetRate());
	frc::SmartDashboard::PutNumber("Angle", ahrs->GetAngle());

//		frc::Wait(kUpdatePeriod); // Wait a short bit before updating again.
}



