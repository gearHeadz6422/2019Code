/*
 * RobotTeleop.cpp
 *
 *  Created on: Jan 30, 2017
 *      Author: pinkenbu
 */
#include "Robot.h"
using namespace std;
double dpad = 0.0;

void Robot::TeleopInit() {
	double dpad = 0.0;
	CameraLightOff();
}

void Robot::TeleopPeriodic() {
	//smart dashboard
	for (int i = 0; i < 3; i++) {
		char name[100];
		sprintf(name, "Current Channel %d", i);
		frc::SmartDashboard::PutNumber(name, m_pdp.GetCurrent(i));
	}
	for (int i = 13; i < 16; i++) {
		char name[100];
		sprintf(name, "Current Channel %d", i);
		frc::SmartDashboard::PutNumber(name, m_pdp.GetCurrent(i));
	}

	frc::SmartDashboard::PutNumber("UltraSonic",
			ultrasonic.GetValue() * kValueToCM);
	frc::SmartDashboard::PutNumber("Angle", ahrs->GetAngle());
	frc::SmartDashboard::PutNumber("SpeedvalBall", ballpickspeedval);
	frc::SmartDashboard::PutNumber("SpeedvalWinch", winchspeedval);

	// driver controller

	//	talon_winch.Set(winchspeedval);
	float right = xboxcontroller0.GetY(frc::GenericHID::JoystickHand::kRightHand);
	float left = xboxcontroller0.GetY(frc::GenericHID::JoystickHand::kLeftHand);
	float rightpwr = reverse * right / 3.0;
	float leftpwr = reverse * left / 3.0;
//	cs::CvSource outputStream = CameraServer::GetInstance()->
//	PutVideo(*grip.GetHslThresholdOutput);

	frc::SmartDashboard::PutNumber("Linear accel X", accelX * 1000);
	frc::SmartDashboard::PutNumber("Linear accel Y", accelY * 1000);

	accelX = ahrs->GetWorldLinearAccelX() * 1000;
	accelY = ahrs->GetWorldLinearAccelY() * 1000;

	if (fabs(lastAccelX) - fabs(accelX) > 500 || fabs(lastAccelY) - fabs(accelY) > 500) {
		colliding = true;
	}

	frc::SmartDashboard::PutBoolean("Currently crashing into like a wall or some shit. Like just stop moving maybe?", colliding);

	lastAccelX = accelX;
	lastAccelY = accelY;
	if(xboxcontroller0.GetPOV() == 90 && dpad < 1){
		for (int i=0;i<5;i++){
		dpad = dpad + .01;
		}
	}
	if (xboxcontroller0.GetPOV() == 270 && dpad > 0) {
		for (int k=0;k<5;k++){
		dpad = dpad - .01;
		}
	}
	calzone.Set(dpad);//HOOK
	climber.Set(left);

	if (xboxcontroller0.GetRawButton(kTriggerleft)) {
		rightpwr = reverse * right;
		leftpwr = rightpwr;
		//MotorSpeedLeft(leftpwr);
		//MotorSpeedRight(rightpwr);

	} else {
		if (xboxcontroller0.GetRawButton(kReverseButton)) {
			if (!buttonpushed[kReverseButton]) {
				reverse = reverse * -1;
			}
			buttonpushed[kReverseButton] = true;
		} else {
			buttonpushed[kReverseButton] = false;
		}
		if (reverse > 0) {
		//	MotorSpeedLeft(leftpwr);
		//	MotorSpeedRight(rightpwr);
		} else {
		//	MotorSpeedLeft(rightpwr);
		//	MotorSpeedRight(leftpwr);
		}
	}
	//attachment controler

	if (xboxcontroller1.GetRawButton(kBallButton)) {
		if (!buttonpushed[kBallButton]) {
			ballpickspeedval -= 0.1;
		}
		buttonpushed[kBallButton] = true;
	} else {
		buttonpushed[kBallButton] = false;
	}

	if (xboxcontroller1.GetRawButton(kBallButtonOff)) {
		ballpickspeedval = 0.0;
	}

	CameraLightOn();
	talon_ballpick.Set(ballpickspeedval);

	if (xboxcontroller1.GetRawButton(kWinchButton)) {
		if (!buttonpushed[kWinchButton]) {
			winchspeedval += 0.1;
		}
		buttonpushed[kWinchButton] = true;
	} else {
		buttonpushed[kWinchButton] = false;
	}

	if (xboxcontroller1.GetRawButton(kWinchButtonOff)) {
		winchspeedval = 0;
	}

	if (xboxcontroller1.GetRawButton(kBallShootButton)) {
		if (!buttonpushed[kBallShootButton]) {
			talon_ballshoot.Set(1.0);
		}
		buttonpushed[kBallShootButton] = true;
	}else {
		buttonpushed[kBallShootButton] = false;
	}

if (buttonpushed[kBallShootButton])
{
	talon_ballshoot.Set(1.0);
	}
	if (xboxcontroller1.GetRawButton(kBallShootButtonOff)) {
		if (!buttonpushed[kBallShootButtonOff]) {
			talon_ballshoot.Set(0.0);
		}
		buttonpushed[kBallShootButtonOff] = true;
	}else {
		buttonpushed[kBallShootButtonOff] = false;
	}

//	float right1 = xboxcontroller1.GetY(Joystick::kRightHand);
	float right1 = xboxcontroller1.GetY(frc::GenericHID::JoystickHand::kRightHand);
	talon_winch.Set(right1);
	if(xboxcontroller0.GetAButton() == true) {
		Ground_Beef.Start();
	}
	if(xboxcontroller0.GetBButton() == true) {
		Ground_Beef.Stop();
	}
	if (xboxcontroller0.GetYButton() == true){
		Diced_Tomato.Set(frc::DoubleSolenoid::Value::kForward);
	}
	if (xboxcontroller0.GetXButton() == true){
		Diced_Tomato.Set(frc::DoubleSolenoid::Value::kReverse);
	}
}
