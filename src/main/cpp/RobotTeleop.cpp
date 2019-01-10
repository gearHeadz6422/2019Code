/*
 * RobotTeleop.cpp
 *
 *  Created on: Jan 30, 2017
 *      Author: pinkenbu
 */
#include "Robot.h"
#include <cmath>

using namespace std;

bool buttonpress = true;
bool rbump = true;
bool lbump = true;
double mpwr = 0;


int smartDashTimerTele = 0;

bool raisingRamp = false;
bool loweringRamp = false;

double desRampAngle = 0.0;
double rampAngleProgress = 0.0;

double previousRampAngle = 0.0;

double maxAccelX = 0.0;
double maxAccelY = 0.0;
double dpad = 0.0;

double driveType = 0.0;

void Robot::TeleopInit() {
	colliding = false;
	accelX = 0.0;
	accelY = 0.0;
	lastAccelX = 0.0;
	lastAccelY = 0.0;

	rightEncoder.Reset();
	leftEncoder.Reset();
	winchEncoder.Reset();
	climberEncoder.Reset();

	FrontLeft.Set(ControlMode::PercentOutput, 0);
	FrontRight.Set(ControlMode::PercentOutput, 0);
	BackLeft.Set(ControlMode::PercentOutput, 0);
	BackRight.Set(ControlMode::PercentOutput, 0);

	Climber.SetSelectedSensorPosition(0,0,0);
	driveType = frc::SmartDashboard::GetNumber("DB/Slider 3", 0.0);
}

void Robot::TeleopPeriodic() {
	if (fabs(ahrs->GetWorldLinearAccelX()) > maxAccelX) {
		maxAccelX = fabs(ahrs->GetWorldLinearAccelX()) * 100;
	}

	if (fabs(ahrs->GetWorldLinearAccelY()) > maxAccelY) {
		maxAccelY = fabs(ahrs->GetWorldLinearAccelX()) * 100;
	}

	frc::SmartDashboard::PutNumber("X velocity", maxAccelX);
	frc::SmartDashboard::PutNumber("Y velocity", maxAccelY);
	frc::SmartDashboard::PutNumber("angle", ahrs->GetAngle());

	frc::SmartDashboard::PutNumber("Timer", smartDashTimerTele);
	if (smartDashTimerTele < 10){
		smartDashTimerTele += 1;
	} else {
		smartDashTimerTele = 0;
			//smart dashboard
		if (fabs(m_pdp.GetCurrent(0)) > maxpwr[0]) {
			maxpwr[0] = fabs(m_pdp.GetCurrent(0));
		}
		if (fabs(m_pdp.GetCurrent(1)) > maxpwr[1]) {
			maxpwr[1] = fabs(m_pdp.GetCurrent(1));
		}
		if (fabs(m_pdp.GetCurrent(2)) > maxpwr[2]) {
			maxpwr[2] = fabs(m_pdp.GetCurrent(2));
		}
		if (m_pdp.GetCurrent(3) > maxpwr[3]) {
			maxpwr[3] = m_pdp.GetCurrent(3);
		}
		if (m_pdp.GetCurrent(14) > maxpwr[4]) {
			maxpwr[4] = m_pdp.GetCurrent(14);
		}
		if (m_pdp.GetCurrent(15) > maxpwr[5]) {
			maxpwr[5] = m_pdp.GetCurrent(15);
		}
		frc::SmartDashboard::PutNumber("Max current 0", maxpwr[0]);
		frc::SmartDashboard::PutNumber("Max current 1", maxpwr[1]);
		frc::SmartDashboard::PutNumber("Max current 2", maxpwr[2]);
		frc::SmartDashboard::PutNumber("Max current 3", maxpwr[3]);
		frc::SmartDashboard::PutNumber("Max current 14", maxpwr[4]);
		frc::SmartDashboard::PutNumber("Max current 15", maxpwr[5]);
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
		//left_master = front left
				//left_slave = back left
				//right_master = front right
				//right_slave = back right

		frc::SmartDashboard::PutNumber("Left encoder", leftEncoder.GetDistance());
		frc::SmartDashboard::PutNumber("Right encoder", rightEncoder.GetDistance());
		frc::SmartDashboard::PutNumber("Winch encoder", winchEncoder.GetDistance());
		frc::SmartDashboard::PutNumber("Climber encoder", climberEncoder.GetDistance());


	}

	bool stop0 = xboxcontroller0.GetStartButton();
	bool stop1 = xboxcontroller1.GetStartButton();

	//PILOT CONTROLLER BUTTONS
	double rightX1 = xboxcontroller0.GetX(frc::Joystick::kRightHand);
	double rightY1 = xboxcontroller0.GetY(frc::Joystick::kRightHand);
	double leftX1 = xboxcontroller0.GetX(frc::Joystick::kLeftHand);
	double leftY1 = xboxcontroller0.GetY(frc::Joystick::kLeftHand);

	if (rightX1 < 0) {
		rightX1 = ((rightX1 * rightX1) * -1);
	} else {
		rightX1 = (rightX1 * rightX1);
	}

	if (rightY1 < 0) {
			rightY1 = ((rightY1 * rightY1) * -1);
		} else {
			rightY1 = (rightY1 * rightY1);
		}


	bool leftBumper1 = xboxcontroller0.GetBumper(frc::Joystick::kLeftHand);
	bool rightBumper1 = xboxcontroller0.GetBumper(frc::Joystick::kRightHand);

	double rightTrigger1 = xboxcontroller0.GetTriggerAxis(frc::Joystick::kRightHand);
	double leftTrigger1 = xboxcontroller0.GetTriggerAxis(frc::Joystick::kLeftHand);

	bool startButton1 = xboxcontroller0.GetStartButton();

	float big = 0.0;
	float multiplier = leftTrigger1+1;

	//CO-PILOT CONTROLLER
	double rightY2 = xboxcontroller1.GetY(frc::Joystick::kRightHand);
	double rightX2 = xboxcontroller1.GetX(frc::Joystick::kRightHand);
	double leftY2 = xboxcontroller1.GetY(frc::Joystick::kLeftHand);
	double leftX2 = xboxcontroller1.GetX(frc::Joystick::kLeftHand);

	bool leftBumper2 = xboxcontroller1.GetBumper(frc::Joystick::kLeftHand);
	bool rightBumper2 = xboxcontroller1.GetBumper(frc::Joystick::kRightHand);

	bool xButton2 = xboxcontroller1.GetXButton();
	bool aButton2 = xboxcontroller1.GetAButton();
	bool yButton2 = xboxcontroller1.GetYButton();
	bool bButton2 = xboxcontroller1.GetBButton();

	double rightTrigger2 = xboxcontroller1.GetTriggerAxis(frc::Joystick::kRightHand);
	double leftTrigger2 = xboxcontroller1.GetTriggerAxis(frc::Joystick::kLeftHand);

	bool startButton2 = xboxcontroller1.GetStartButton();

	float rampMultiplier = leftTrigger2 +1;

	double climbRotations = Climber.GetSelectedSensorPosition(0)/4096.0;
	frc::SmartDashboard::PutNumber("ClimbRotations", climbRotations);
	if(stop1 == false && stop0 == false) {

		//PILOT CONTROLLER CODE
	if (fabs(rightX1) > fabs(rightY1) && fabs(rightX1) > fabs(leftX1)) {
		big = fabs(rightX1);
	} else if (fabs(rightY1) > fabs(rightX1) && fabs(rightY1) > fabs(leftX1) ) {
		big = fabs(rightY1);
	} else {
		big = fabs(leftX1);
	}

	if(xboxcontroller1.GetPOV() == 90 && dpad < 1){
		for (int i=0;i<5;i++){
		dpad = dpad + .01;
		}
	}
	if (xboxcontroller1.GetPOV() == 270 && dpad > 0) {
		for (int k=0;k<5;k++){
		dpad = dpad - .01;
		}
	}
	Hook.Set(dpad);

	if (startButton1){
		FrontLeft.Set(0.0);
		FrontRight.Set(0.0);
		BackLeft.Set(0.0);
		BackRight.Set(0.0);
		winchSpeed = 0;
		rampSpeed = 0;
	}
	else {
		FrontLeft.Set(rightY1/2.5);
		FrontRight.Set(leftY1/2.5);
		BackLeft.Set(rightY1/2.5);
		BackRight.Set(leftY1/2.5);
	}


		//CO-PILOT CONTROLLER CODE
	if (aButton2) {
		winchEncoder.Reset();
	}

	if (winchEncoder.GetDistance() != previousRampAngle) {
		if (previousRampAngle < winchEncoder.GetDistance() && !loweringRamp && leftY2 > -0.025) {
			Winch.Set(0.5);
		} else if (previousRampAngle < winchEncoder.GetDistance() && !raisingRamp && leftY2 < 0.05) {
			Winch.Set(-0.5);
		}
	}

	previousRampAngle = winchEncoder.GetDistance();

	if (rightBumper2) {
		raisingRamp = true;
		loweringRamp = false;
		desRampAngle = 1885;
	} else if (leftBumper2) {
		desRampAngle = 0;
		raisingRamp = false;
		loweringRamp = true;
	}

	if(raisingRamp || loweringRamp) {
		rampAngleProgress = winchEncoder.GetDistance();
		if (raisingRamp && !loweringRamp) {
			if (rampAngleProgress < desRampAngle) {
				Winch.Set(1);
			} else if (rampAngleProgress >= desRampAngle) {
				Winch.Set(0);
				raisingRamp = false;
				loweringRamp = false;
				desRampAngle = 0.0;
				rampAngleProgress = 0.0;
			}
		} else if (!raisingRamp && loweringRamp) {
			if (rampAngleProgress > desRampAngle) {
				Winch.Set(-1);
			} else if (rampAngleProgress <= desRampAngle) {
				Winch.Set(0);
				raisingRamp = false;
				loweringRamp = false;
				desRampAngle = 0.0;
				rampAngleProgress = 0.0;
			}
		}
	}

	if (startButton2){
		FrontLeft.Set(0.0);
		FrontRight.Set(0.0);
		BackLeft.Set(0.0);
		BackRight.Set(0.0);
		winchSpeed = 0;
		rampSpeed = 0;
	} else {
		rampSpeed = -rightY2/2;
		if (rampSpeed < 0) {
			Shooter1.Set(((rampSpeed * rampSpeed) * -1) * rampMultiplier);
		} else {
			Shooter1.Set((rampSpeed * rampSpeed) * rampMultiplier);
		}
		if (rampSpeed < 0) {
			Shooter2.Set(((rampSpeed * rampSpeed) * -1) * rampMultiplier);
		} else {
			Shooter2.Set((rampSpeed * rampSpeed) * rampMultiplier);
		} if (bButton2 == false){
		if(leftY2 >= 0.025 || leftY2 <= -0.025){
			if (raisingRamp || loweringRamp) {
				raisingRamp = false;
				loweringRamp = false;
				rampAngleProgress = 0.0;
				desRampAngle = 0.0;
			}
			if ((winchEncoder.GetDistance() < 3750 && leftY2 < 0) || (winchEncoder.GetDistance() > 0 && leftY2 > 0) || xButton2) {
				Winch.Set(-leftY2);
			}  else {
				Winch.Set(0);
			}
		} else if (!raisingRamp && !loweringRamp) {
			Winch.Set(0);
		}
		Climber.Set(0);
		}
		else if (bButton2){
			//if ((climberEncoder.GetDistance() <= 0 && climberEncoder.GetDistance() > -6650 )|| (climberEncoder.GetDistance() > 0 && leftY2 > .05) || (climberEncoder.GetDistance() < -6650 && leftY2 < -.05)){
			if(yButton2 || climbRotations > -3.35 || leftY2 > 0){
				Climber.Set(leftY2);
			}
			else {
				Climber.Set(0);
			}
			//}
			//else {
				//if (xboxcontroller0.GetTriggerAxis(frc::Joystick::kRightHand) > .05){
				//	Climber.Set(leftY2);
				//}
				//else {
					//Climber.Set(0);
				//}
			//}
		}
		//else if ((aButton2 && climberEncoder.GetDistance() >= 0) || (aButton2 && rightBumper2)){
			//Climber.Set(leftY2);
		//}
	}
	if(yButton2) {
		climberEncoder.Reset();
	}
	}
	else {
		FrontLeft.Set(0.0);
		FrontRight.Set(0.0);
		BackLeft.Set(0.0);
		BackRight.Set(0.0);
		Winch.Set(0.0);
		Climber.Set(0.0);
		Shooter1.Set(0.0);
		Shooter2.Set(0.0);
	}


/*	if (yButton2) {
		Climber.Set(0.5);
	} else if (bButton2) {
		Climber.Set(-0.5);
	}
	else
	{
		Climber.Set(0);
	}
	*/
	frc::SmartDashboard::PutNumber("ClimbEncoder", climberEncoder.GetDistance());
}

