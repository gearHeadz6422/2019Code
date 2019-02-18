#include "Robot.h"
#include <cmath>

using namespace std;

#define LEFT

void Robot::AutonomousInit() {
	// 	// Set the initial state for our autonomous variables, and pull some data from the driver station
	// desAutoDelay = SmartDashboard::GetNumber("Auto delay", 0.0);
	// stepCount = 0;

	// 	// Set the initial state for our variables, and pull some data from the driver station
	// accelX = 0.0;
	// accelY = 0.0;
	// lastAccelX = 0.0;
	// lastAccelY = 0.0;
	// liftEncoderLow.Reset();

	// if (sensorBoardType == "navx") {
	// 	navx->ZeroYaw();
	// } else {
	// 	analogDev.Reset();
	// }

	// currentAnlge = 0;
	// prevAnlge = 0;

	// 	// Sets the way that the TALONs will recieve input
	// FrontLeft.Set(ControlMode::PercentOutput, 0);
	// FrontRight.Set(ControlMode::PercentOutput, 0);
	// BackLeft.Set(ControlMode::PercentOutput, 0);
	// BackRight.Set(ControlMode::PercentOutput, 0);
	// 	// I literally have no idea what this does... don't touch it
	// Climber.SetSelectedSensorPosition(0, 0, 0);

	// 	// Tracks the current step the auto-lineup algorithim is on
	// alignState = "none";
	// targetAngle = -1;

	// 	// Tracks the desired height of the hook
	// desElevatorPosition = "low";

	// 	// This sets up the ultrasonic sensor, and declares that it is hooked into analog port 0
	// frontUltraSonic = new AnalogInput(0);
	// rearUltraSonic = new AnalogInput(1);

	// // Used to define which talon is currently being tested later on
	// if (motorDebug) {
	// 	frc::SmartDashboard::PutNumber("testMotor", -1.0);
	// }
}

void Robot::AutonomousPeriodic() {
	// 	// This is the code that will cancel operations that are posing real world danger. This stuff is very important, so it comes first, and isn't dependent on any other systems actually working. If you need to change it for some reason, make sure it still works!
	// bool stop0 = xboxcontroller0.GetStartButton();
	// bool stop1 = xboxcontroller1.GetStartButton();
	// bool stickMoved = false; // This is used later to cancel out of autonomous movements when the driver moves the stick
	// if (stop0 || stop1) {
	// 	FrontLeft.Set(0.0);
	// 	FrontRight.Set(0.0);
	// 	BackLeft.Set(0.0);
	// 	BackRight.Set(0.0);
	// 	Hook.Set(0.0);
	// 	Winch.Set(0.0);

	// 	return;
	// }

	// if (autoMode) {
	// 		// Now, we collect all of the data our sensors are spitting out, process it, and display some of it on the smart dashboard
	// 	if (sensorBoardType == "navx") {
	// 		accelX = navx->GetWorldLinearAccelX() * 1000;
	// 		accelY = navx->GetWorldLinearAccelY() * 1000;
	// 	} else {
	// 		accelX = analogDev.GetAccelX() * 1000;
	// 		accelY = analogDev.GetAccelY() * 1000;
	// 	}
	// 	frontWallDistance = frontUltraSonic->GetValue();
	// 	rearWallDistance = rearUltraSonic->GetValue();
	// 	frc::SmartDashboard::PutNumber("Front wall distance", frontWallDistance);
	// 	frc::SmartDashboard::PutNumber("Rear wall distance", rearWallDistance);
	// 	frc::SmartDashboard::PutNumber("X velocity", accelX);
	// 	frc::SmartDashboard::PutNumber("Y velocity", accelX);
	// 	frc::SmartDashboard::PutNumber("desAutoDelay", desAutoDelay);
	// 	frc::SmartDashboard::PutNumber("startPosition", startPosition);
	// 	startPosition = frc::SmartDashboard::GetNumber("startPosition", 0.0);
	// 	if (fabs(lastAccelX) - fabs(accelX) > 500 || fabs(lastAccelY) - fabs(accelY) > 500) {
	// 		colliding = true;
	// 	} else {
	// 		colliding = false;
	// 	}
		
	// 	if (sensorBoardType == "navx") {
	// 		currentAnlge += navx->GetAngle() - prevAnlge;
	// 	} else {
	// 		currentAnlge += analogDev.GetAngle() - prevAnlge;
	// 	}

	// 		// We need to make sure that the robot always has an idea of the direction it's pointing, so we manipulate the angle measurement a little so that positive angles are always right, negative angles are always left, and that the angle is never above 180
	// 	while (currentAnlge >= 360)	{
	// 		currentAnlge -= 360;
	// 	}
	// 	while (currentAnlge <= -360) {
	// 		currentAnlge += 360;
	// 	}
	// 	if (currentAnlge < -180 || currentAnlge > 180) {
	// 		currentAnlge = -currentAnlge;
	// 	}
	// 	frc::SmartDashboard::PutNumber("Angle", currentAnlge);

	// 		// There variables are used to control where the robot moves. They are named to match the format of the Tele-Op
	// 	double rightX1 = 0.0;
	// 	double rightY1 = 0.0;
	// 	double leftX1 = 0.0;
	// 	float big = 0.0;

	// 	switch (stepCount) {
	// 		case 1:
	// 			// First auto command here
	// 			stepCount++;
	// 			break;

	// 			// Once we run out of automated stpes we set the robot up to run tele-op and exit the current loop of Auto code
	// 		default:
	// 			autoMode = false;
	// 			Robot::TeleopInit();
	// 			return;
	// 	}

	// 			// Now that we have all of the control movemnt set, we set the motors to their cooresponding variables
	// 	if (fabs(rightX1) > fabs(rightY1) && fabs(rightX1) > fabs(leftX1)) {
	// 		big = fabs(rightX1);
	// 	} else if (fabs(rightY1) > fabs(rightX1) && fabs(rightY1) > fabs(leftX1) ) {
	// 		big = fabs(rightY1);
	// 	} else {
	// 		big = fabs(leftX1);
	// 	}

	// 	if (mecanumDrive) {
	// 		// TODO: Test and make sure this method of driving actually works
	// 		FrontLeft.Set((cos(atan2(rightY1, rightX1) - 0.7853981633974483) + leftX1) * big);
	// 		FrontRight.Set((sin(atan2(rightY1, rightX1) - 0.7853981633974483) - leftX1) * big);
	// 		BackLeft.Set((sin(atan2(rightY1, rightX1) - 0.7853981633974483) + leftX1) * big);
	// 		BackRight.Set((cos(atan2(rightY1, rightX1) - 0.7853981633974483) - leftX1) * big);
	// 	} else {
	// 			// Tank drive code here
	// 		FrontLeft.Set(0.0);
	// 		FrontRight.Set(0.0);
	// 		BackLeft.Set(0.0);
	// 		BackRight.Set(0.0);
	// 	}

	// 	lastAccelX = accelX;
	// 	lastAccelY = accelY;
	// 	if (sensorBoardType == "navx") {
	// 		prevAnlge = navx->GetAngle();
	// 	} else {
	// 		prevAnlge = analogDev.GetAngle();
	// 	}

	// 		// Put your debugging code here
	// 	frc::SmartDashboard::PutString("alignStateString", alignState);
	// } else { // Once our auto code is comeplete we have the robot run as if it was just in Tele-Op
	// 	Robot::TeleopPeriodic();
	// }
}
