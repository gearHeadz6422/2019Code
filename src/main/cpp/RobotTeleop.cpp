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

double hookSpeed = 0.0;

double driveType = 0.0;

bool joystickMode = true;

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
	bool stop2 = xboxcontroller2.GetStartButton();

	double rightX1 = 0.0;
	double rightY1 = 0.0;
	double leftX1 = 0.0;
	double leftY1 = 0.0;
	bool leftBumper1 = false;
	bool rightBumper1 = false;
	double rightTrigger1 = 0.0;
	double leftTrigger1 = 0.0;
	bool joyAccelButton = false;
	bool startButton1 = false;

	//PILOT CONTROLLER BUTTONS
	if (!joystickMode) {
		rightX1 = xboxcontroller0.GetX(frc::Joystick::kRightHand);
		rightY1 = xboxcontroller0.GetY(frc::Joystick::kRightHand);
		leftX1 = xboxcontroller0.GetX(frc::Joystick::kLeftHand);
		leftY1 = xboxcontroller0.GetY(frc::Joystick::kLeftHand);

		leftBumper1 = xboxcontroller0.GetBumper(frc::Joystick::kLeftHand);
		rightBumper1 = xboxcontroller0.GetBumper(frc::Joystick::kRightHand);

		rightTrigger1 = xboxcontroller0.GetTriggerAxis(frc::Joystick::kRightHand);
		leftTrigger1 = xboxcontroller0.GetTriggerAxis(frc::Joystick::kLeftHand);

		startButton1 = xboxcontroller0.GetStartButton();
	} else {
		// Only use left hand - the joystick axes line up with the left controller axes
		leftX1 = sqrt(abs(xboxcontroller0.GetTriggerAxis(frc::Joystick::kLeftHand)/1.5));
		if (xboxcontroller0.GetTriggerAxis(frc::Joystick::kLeftHand) < 0)
		{
			leftX1 *= -1;
		}
		rightY1 = 1.3 * sqrt(abs(xboxcontroller0.GetY(frc::Joystick::kLeftHand)));
		if (xboxcontroller0.GetY(frc::Joystick::kLeftHand) < 0) {
			rightY1 *= -1;
		}
		rightX1 = 1.3 * sqrt(abs(xboxcontroller0.GetX(frc::Joystick::kLeftHand)*1.5));
		if (xboxcontroller0.GetX(frc::Joystick::kLeftHand) < 0)
		{
			rightX1 *= -1;
		}
		joyAccelButton = xboxcontroller0.GetAButton();
		if (joyAccelButton) {
			leftTrigger1 = 1.0;
		} else {
			leftTrigger1 = 0.0;
		}
		startButton1 = xboxcontroller0.GetBButton();
		// rightX1 = xboxcontroller0.GetX(frc::Joystick::kLeftHand);
		// rightY1 = xboxcontroller0.GetY(frc::Joystick::kLeftHand);
	}

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

	float big = 0.0;
	float multiplier = leftTrigger1+1;

	double rightY2 = 0.0; 
	double rightX2 = 0.0;
	double leftY2 = 0.0;
	double leftX2 = 0.0;
	bool leftBumper2 = false;
	bool rightBumper2 = false;
	bool xButton2 = false; 
	bool aButton2 = false;
	bool yButton2 = false; 
	bool bButton2 = false;
	double rightTrigger2 = 0.0; 
	double leftTrigger2 = 0.0;
	bool startButton2 = false;
	int dpad = 0;

	//CO-PILOT CONTROLLER
	if (!joystickMode) {
		rightY2 = xboxcontroller1.GetY(frc::Joystick::kRightHand);
		rightX2 = xboxcontroller1.GetX(frc::Joystick::kRightHand);
		leftY2 = xboxcontroller1.GetY(frc::Joystick::kLeftHand);
		leftX2 = xboxcontroller1.GetX(frc::Joystick::kLeftHand);

		leftBumper2 = xboxcontroller1.GetBumper(frc::Joystick::kLeftHand);
		rightBumper2 = xboxcontroller1.GetBumper(frc::Joystick::kRightHand);

		xButton2 = xboxcontroller1.GetXButton();
		aButton2 = xboxcontroller1.GetAButton();
		yButton2 = xboxcontroller1.GetYButton();
		bButton2 = xboxcontroller1.GetBButton();

		rightTrigger2 = xboxcontroller1.GetTriggerAxis(frc::Joystick::kRightHand);
		leftTrigger2 = xboxcontroller1.GetTriggerAxis(frc::Joystick::kLeftHand);

		startButton2 = xboxcontroller1.GetStartButton();

		dpad = xboxcontroller1.GetPOV();
	} else {
		rightY2 = xboxcontroller2.GetY(frc::Joystick::kRightHand);
		rightX2 = xboxcontroller2.GetX(frc::Joystick::kRightHand);
		leftY2 = xboxcontroller2.GetY(frc::Joystick::kLeftHand);
		leftX2 = xboxcontroller2.GetX(frc::Joystick::kLeftHand);

		leftBumper2 = xboxcontroller2.GetBumper(frc::Joystick::kLeftHand);
		rightBumper2 = xboxcontroller2.GetBumper(frc::Joystick::kRightHand);

		xButton2 = xboxcontroller2.GetXButton();
		aButton2 = xboxcontroller2.GetAButton();
		yButton2 = xboxcontroller2.GetYButton();
		bButton2 = xboxcontroller2.GetBButton();

		rightTrigger2 = xboxcontroller2.GetTriggerAxis(frc::Joystick::kRightHand);
		leftTrigger2 = xboxcontroller2.GetTriggerAxis(frc::Joystick::kLeftHand);

		startButton2 = xboxcontroller2.GetStartButton();

		dpad = xboxcontroller2.GetPOV();
	}

	float rampMultiplier = leftTrigger2 +1;

	double climbRotations = Climber.GetSelectedSensorPosition(0)/4096.0;
	frc::SmartDashboard::PutNumber("ClimbRotations", climbRotations);
	if(stop2 == false && stop1 == false && stop0 == false) {

		//PILOT CONTROLLER CODE
	if (fabs(rightX1) > fabs(rightY1) && fabs(rightX1) > fabs(leftX1)) {
		big = fabs(rightX1);
	} else if (fabs(rightY1) > fabs(rightX1) && fabs(rightY1) > fabs(leftX1) ) {
		big = fabs(rightY1);
	} else {
		big = fabs(leftX1);
	}

	if (startButton1){
		FrontLeft.Set(0.0);
		FrontRight.Set(0.0);
		BackLeft.Set(0.0);
		BackRight.Set(0.0);
		winchSpeed = 0;
		rampSpeed = 0;
	}
	else {
		// Tank drive
		// FrontLeft.Set(rightY1/2.5);
		// FrontRight.Set(leftY1/2.5);
		// BackLeft.Set(rightY1/2.5);
		// BackRight.Set(leftY1/2.5);

		// Mecanum 
		FrontLeft.Set(((cos(atan2(rightY1, rightX1) - M_PI / 4) + leftX1 * (1 - rightTrigger1)) / 2) * big * multiplier);
		FrontRight.Set(((sin(atan2(rightY1, rightX1) - M_PI / 4) - leftX1 * (1 - rightTrigger1)) / 2) * big * multiplier);
		BackLeft.Set(((sin(atan2(rightY1, rightX1) - M_PI / 4) + leftX1 * (1 - rightTrigger1)) / 2) * big * multiplier);
		BackRight.Set(((cos(atan2(rightY1, rightX1) - M_PI / 4) - leftX1 * (1 - rightTrigger1)) / 2) * big * multiplier);
	}


		//CO-PILOT CONTROLLER CODE
	if (aButton2) {
		winchEncoder.Reset();
	}

	if(dpad == 90 && hookSpeed < 1){
		for (int i=0; i<5; i++){
			hookSpeed = hookSpeed + .01;
		}
	}

	if (dpad == 270 && hookSpeed > 0) {
		for (int k=0; k<5; k++){
			hookSpeed = hookSpeed - .01;
		}
	}
	Hook.Set(hookSpeed);

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

