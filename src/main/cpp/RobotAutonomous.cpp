#include "Robot.h"
#include <cmath>

using namespace std;

#define LEFT

void Robot::AutonomousInit() {
		// Set the initial state for our autonomous variables, and pull some data from the driver station
	autoDelayTimer = 0;
	startPosition = SmartDashboard::GetNumber("DB/Slider 0", 0.0);
	desAutoDelay = SmartDashboard::GetNumber("DB/Slider 1", 0.0);
	stepCount = 0;
	accelX = 0.0;
	accelY = 0.0;
	lastAccelX = 0.0;
	lastAccelY = 0.0;
	leftEncoder.Reset();
	rightEncoder.Reset();
	winchEncoder.Reset();
	rightEncoder.Reset();
	leftEncoder.Reset();
	winchEncoder.Reset();
	climberEncoder.Reset();
	ahrs->ZeroYaw();
	currentAnlge = 0;

		// I literally have no idea what this does... I wouldn't touch it
	FrontLeft.Set(ControlMode::PercentOutput, 0);
	FrontRight.Set(ControlMode::PercentOutput, 0);
	BackLeft.Set(ControlMode::PercentOutput, 0);
	BackRight.Set(ControlMode::PercentOutput, 0);
	Climber.SetSelectedSensorPosition(0, 0, 0);

		// This variable is used to track which side of the rocket we are trying to line up with
	autoLineUp = "none";
}

void Robot::AutonomousPeriodic() {
		// This is exclusively used for debuging
	// if (smartDashTimerAuto < 10) {
	// 	smartDashTimerAuto += 1;
	// } else {
	// 	smartDashTimerAuto = 0;
	// 	frc::SmartDashboard::PutNumber("Left encoder", leftEncoder.GetDistance());
	// 	frc::SmartDashboard::PutNumber("Right encoder", rightEncoder.GetDistance());
	// 	frc::SmartDashboard::PutNumber("Winch encoder", winchEncoder.GetDistance());
	// 	frc::SmartDashboard::PutNumber("Current step num", stepCount);
	// }

		// Now that we know the robot is running, we collect all of the data our sensors are spitting out and display some of it on the smart dashboard
	accelX = ahrs->GetWorldLinearAccelX() * 1000;
	accelY = ahrs->GetWorldLinearAccelY() * 1000;
	frc::SmartDashboard::PutNumber("desAutoDelay", desAutoDelay);
	frc::SmartDashboard::PutNumber("startPosition", startPosition);
	startPosition = frc::SmartDashboard::GetNumber("startPosition", 0.0);
	leftInches = rightEncoder.GetDistance() / (6 * M_PI); //changed to right, it should be left!
	rightInches = rightEncoder.GetDistance() / (6 * M_PI);
	if (fabs(lastAccelX) - fabs(accelX) > 500 || fabs(lastAccelY) - fabs(accelY) > 500) {
		colliding = true;
	}
	polyCount = frc::SmartDashboard::GetNumber("polyCount", 0);
	cameraOutput = frc::SmartDashboard::GetNumber("cameraOutput", 0);
	currentAnlge += ahrs->GetAngle() - prevAnlge;

		// We need to make sure that the robot always has an idea of the direction it's pointing, so we manipulate the angle measurement a little so that positive angles are always right, negative angles are always left, and that the angle is never above 180
	while (currentAnlge >= 360)	{
		currentAnlge -= 360;
	}
	while (currentAnlge <= -360) {
		currentAnlge += 360;
	}
	if (currentAnlge < -180 || currentAnlge > 180) {
		currentAnlge = -currentAnlge;
	}

		// This if block wraps all of the autonomous code. Driver inputs are not even measured, so be careful to keep that code outside (The end is marked with another comment)
	if (!autoCompleted) {
		if (autoDelayTimer < desAutoDelay){
			autoDelayTimer += 0.05;
		} else {
			if (startPosition == 1) {
				if (leftInches <= 220) {
					FrontLeft.Set(-0.35);
					FrontRight.Set(-0.35);
					BackLeft.Set(-0.35);
					BackRight.Set(-0.35);
				} else {
					FrontLeft.Set(0);
					FrontRight.Set(0);
					BackLeft.Set(0);
					BackRight.Set(0);
				}
			}
		}
		return;
	}

	// End auto code-----------------------------------------------------------------------------------------------------------------------------------------------------
}
