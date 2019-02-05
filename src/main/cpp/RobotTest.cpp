/*
 * RobotTest.cpp
 *
 *  Created on: Jan 30, 2017
 *      Author: pinkenbu
 */

#include "Robot.h"

using namespace std;
double last_accel_x;
double last_accel_y;
void Robot::TestInit() {
	navx->ZeroYaw();
}
void Robot::TestPeriodic() {

//	lw->Run();
	//cout << "angle " << navx->GetAngle() << endl;

//	if (m_encoder_right.GetDistance() > 10) {
//		FrontLeft.Set(0.);
//		FrontRight.Set(0.);
//	}
	//float right = xboxcontroller0.GetY(Joystick::kRightHand)/3.;
	//float left = xboxcontroller0.GetY(Joystick::kLeftHand)/3.;
	//FrontLeft.Set(.2);
	//BackLeft.Set(.2);
	//FrontRight.Set(.2);
	//BackRight.Set(.2);


	//frc::SmartDashboard::PutNumber("Encoder Distance L",
		//	m_encoder_left.GetDistance());
	//frc::SmartDashboard::PutNumber("Encoder Distance R",
		//	m_encoder_right.GetDistance());

	// Retrieve the current rate of the encoder.
	//frc::SmartDashboard::PutNumber("Encoder Rate L", m_encoder_left.GetRate());
	//frc::SmartDashboard::PutNumber("Encoder Rate R", m_encoder_right.GetRate());
	//frc::SmartDashboard::PutNumber("Angle", navx->GetAngle());
	bool collisionDetection = false;
	double curr_accel_x = navx->GetWorldLinearAccelX();
	double currentJerkX = curr_accel_x - last_accel_x;
	last_accel_x = curr_accel_x;
	double curr_accel_y = navx->GetWorldLinearAccelY();
	double currentJerkY = curr_accel_y - last_accel_y;
	last_accel_y = curr_accel_y;
	double Vx = navx->GetVelocityX();
	double Vy = navx->GetVelocityY();

	frc::SmartDashboard::PutNumber("X velocity", navx->GetWorldLinearAccelX());
	frc::SmartDashboard::PutNumber("Y velocity", navx->GetWorldLinearAccelY());

	frc::SmartDashboard::PutNumber("angle", navx->GetAngle());

	frc::Wait(kUpdatePeriod); //Wait a short bit before updating again.
}
