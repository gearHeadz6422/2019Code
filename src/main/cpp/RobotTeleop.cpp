#include "Robot.h"
#include <cmath>

using namespace std;

void Robot::TeleopInit() {
	// Set the initial state for our variables, and pull some data from the driver station
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

	if (sensorBoardType == "navx") {
		navx->ZeroYaw();
	} else {
		analogDev.Reset();
	}

	currentAnlge = 0;
	prevAnlge = 0;

		// Sets the way that the TALONs will recieve input
	FrontLeft.Set(ControlMode::PercentOutput, 0);
	FrontRight.Set(ControlMode::PercentOutput, 0);
	BackLeft.Set(ControlMode::PercentOutput, 0);
	BackRight.Set(ControlMode::PercentOutput, 0);
		// I literally have no idea what this does... I wouldn't touch it
	Climber.SetSelectedSensorPosition(0, 0, 0);

		// Tracks the current step the auto-lineup algorithim is on
	alignState = "none";
	targetAngle = -1;

		// Tracks the desired height of the hook
	desElevatorPosition = "low";

		// This sets up the ultrasonic sensor, and declares that it is hooked into analog port 0
	ultraSonic = new AnalogInput(0);

	frc::SmartDashboard::PutNumber("testMotor", -1.0);
}

void Robot::TeleopPeriodic() {
		// This is the code that will cancel operations that are posing real world danger. This stuff is very important, so it comes first, and isn't dependent on any other systems actually working. If you need to change it for some reason, make sure it still works!
	bool stop0 = xboxcontroller0.GetStartButton();
	bool stop1 = xboxcontroller1.GetStartButton();
	bool stickMoved = false; // This is used later to cancel out of autonomous movements when the driver moves the stick
	if (stop0 || stop1) {
		FrontLeft.Set(0.0);
		FrontRight.Set(0.0);
		BackLeft.Set(0.0);
		BackRight.Set(0.0);
		Hook.Set(0.0);
		Winch.Set(0.0);

		return;
	}

		// Now, we collect all of the data our sensors are spitting out, process it, and display some of it on the smart dashboard
	if (sensorBoardType == "navx") {
		accelX = navx->GetWorldLinearAccelX() * 1000;
		accelY = navx->GetWorldLinearAccelY() * 1000;
	} else {
		accelX = analogDev.GetAccelX() * 1000;
		accelY = analogDev.GetAccelY() * 1000;
	}
	wallDistance = ultraSonic->GetValue();
	frc::SmartDashboard::PutNumber("Wall distance", wallDistance);
	frc::SmartDashboard::PutNumber("X velocity", accelX);
	frc::SmartDashboard::PutNumber("Y velocity", accelX);
	frc::SmartDashboard::PutNumber("desAutoDelay", desAutoDelay);
	frc::SmartDashboard::PutNumber("startPosition", startPosition);
	startPosition = frc::SmartDashboard::GetNumber("startPosition", 0.0);
	leftInches = rightEncoder.GetDistance() / (6 * M_PI); //changed to right, it should be left!
	rightInches = rightEncoder.GetDistance() / (6 * M_PI);
	if (fabs(lastAccelX) - fabs(accelX) > 500 || fabs(lastAccelY) - fabs(accelY) > 500) {
		colliding = true;
	} else {
		colliding = false;
	}
	
	polyCount = frc::SmartDashboard::GetNumber("polyCount", 0);
	cameraOutput = frc::SmartDashboard::GetNumber("cameraOutput", 0);
	if (sensorBoardType == "navx") {
		currentAnlge += navx->GetAngle() - prevAnlge;
	} else {
		currentAnlge += analogDev.GetAngle() - prevAnlge;
	}

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
	frc::SmartDashboard::PutNumber("Angle", currentAnlge);

		// Next, we reset all of the gamepad inputs. Variables labeld 1 coordespond to the driver, and 2 to attatchments, regardless of the actual number for each gamepad
	double rightX1 = 0.0;
	double rightY1 = 0.0;
	double leftX1 = 0.0;
	double leftY1 = 0.0;
	bool leftBumper1 = false;
	bool rightBumper1 = false;
	bool xButton1 = false;
	bool yButton1 = false;
	bool aButton1 = false;
	bool bButton1 = false;
	double rightTrigger1 = 0.0;
	double leftTrigger1 = 0.0;
	bool startButton1 = false;
	double dpad1 = 0.0;
	float big = 0.0; // This isn't actually a controller input, but is rather used by the mecanum drive code

	double rightY2 = 0.0;
	double rightX2 = 0.0;
	double leftY2 = 0.0;
	double leftX2 = 0.0;
	bool leftBumper2 = false;
	bool rightBumper2 = false;
	bool xButton2 = false;
	bool yButton2 = false;
	bool aButton2 = false;
	bool bButton2 = false;
	double rightTrigger2 = 0.0;
	double leftTrigger2 = 0.0;
	bool startButton2 = false;
	double dpad2 = 0;

	if (!joystickMode)	{ // The non-joystick mode is no longer supported by the actual driver code. If you want to switch back you'll need to fix a few bugs
		/* 
		* During teleop, we use the variables set below to decide what power the motors should run with. We initially set them to the raw input
		* from the gamepad, however these values are then manipulated by the code that follows. As a result, if you ever need the raw values from the
		* gamepad later, use the nessecary getter function instead of these variables.
		*/

		rightX1 = xboxcontroller0.GetX(frc::Joystick::kRightHand);
		rightY1 = xboxcontroller0.GetY(frc::Joystick::kRightHand);
		leftX1 = xboxcontroller0.GetX(frc::Joystick::kLeftHand);
	leftY1 = xboxcontroller0.GetY(frc::Joystick::kLeftHand);

		leftBumper1 = xboxcontroller0.GetBumper(frc::Joystick::kLeftHand);
		rightBumper1 = xboxcontroller0.GetBumper(frc::Joystick::kRightHand);

		rightTrigger1 = xboxcontroller0.GetTriggerAxis(frc::Joystick::kRightHand);
		leftTrigger1 = xboxcontroller0.GetTriggerAxis(frc::Joystick::kLeftHand);

		if (fabs(rightX1) > fabs(rightY1) && fabs(rightX1) > fabs(leftX1)) {
			big = fabs(rightX1);
		} else if (fabs(rightY1) > fabs(rightX1) && fabs(rightY1) > fabs(leftX1) ) {
			big = fabs(rightY1);
		} else {
			big = fabs(leftX1);
		}
	} else { // This code is used to get input from the joystick, however we still use the xboxController library to do it, so the names of the getter functions do not match what values are actually being measured. A guide to translating xbox controller getters to joystick getters is included in the documentation.
		xButton1 = xboxcontroller0.GetXButton();
		yButton1 = xboxcontroller0.GetYButton();
		aButton1 = xboxcontroller0.GetAButton();
		bButton1 = xboxcontroller0.GetBButton();

		leftX1 = sqrt(abs(xboxcontroller0.GetTriggerAxis(frc::Joystick::kLeftHand)/1.5)); // All driver stick inputs are curved. These can be adjusted to your liking. I recomend using a graphing utility to visualize it, such as https://www.desmos.com/calculator

			// Measures the stick inputs related to moving and turning
		if (xboxcontroller0.GetTriggerAxis(frc::Joystick::kLeftHand) < 0) {
			leftX1 *= -1;
		}
		rightY1 = -1.3 * sqrt(abs(xboxcontroller0.GetY(frc::Joystick::kLeftHand)));
		if (xboxcontroller0.GetY(frc::Joystick::kLeftHand) < 0) {
			rightY1 *= -1; // Ensures that the curve doesn't screw up the sign of the stick input
		}
		rightX1 = 1.3 * sqrt(abs(xboxcontroller0.GetX(frc::Joystick::kLeftHand) * 1.5));
		if (xboxcontroller0.GetX(frc::Joystick::kLeftHand) < 0) {
			rightX1 *= -1;
		}

			// Checks if the driver is sending input to the robot. This will cancel out some autonomous processes
		stickMoved = false;
		if (abs(xboxcontroller0.GetTriggerAxis(frc::Joystick::kLeftHand)) > 0.25 || abs(xboxcontroller0.GetY(frc::Joystick::kLeftHand)) > 0.25 || abs(xboxcontroller0.GetX(frc::Joystick::kLeftHand)) > 0.25) {
			stickMoved = true;
		}

			// Again, curves the stick input, and makes sure the sign is not lost
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

		dpad1 = xboxcontroller0.GetPOV();
	}

		// Calculates the robot's speed multiplier
	leftTrigger1 = fabs(1 - (xboxcontroller0.GetTriggerAxis(frc::Joystick::kRightHand) / 2 + 0.5));
	if (aButton1) {
		leftTrigger1 = 1.0;
	}
	float multiplier = leftTrigger1 + 1;

		// Enables linear-only movement
	if (bButton1) {
		if (fabs(rightX1) >= fabs(rightY1)) {
			rightY1 = 0.0;
		} else {
			rightX1 = 0.0;
		}
	}

		// Determines the desired line based on the current direction of the robot, and the joystick dpad
	const int lineErrorMagrin = 5;
	if (dpad1 != -1) {
		alignState = "angle";
		if (dpad1 == 0) {
			if (currentAnlge >= 0) {
				targetAngle = 150;
			} else {
				targetAngle = -150;
			}
		} else if (dpad1 == 90 || dpad1 == 270) {
			if (currentAnlge >= 0) {
				targetAngle = 90;
			} else {
				targetAngle = -90;
			}
		} else {
			if (currentAnlge >= 0) {
				targetAngle = 30;
			} else {
				targetAngle = -30;
			}
		}
	}

	if (stickMoved)	{
		alignState = "none";
		targetAngle= -1;
	}

		//Turns the robot to be aligned with the selected line
	if (alignState == "angle" && (targetAngle - lineErrorMagrin >= currentAnlge || currentAnlge >= targetAngle + lineErrorMagrin)) {
		if (currentAnlge <= targetAngle) {
			leftX1 = 1.0;
		} else {
			leftX1 = -1.0;
		}
	} else if (alignState == "angle" && targetAngle != -1) {
		alignState = "slide";
	}

		 // Align the robot to a rocket
	if (alignState == "slide") {
			// Theres an inherent delay in camera updates, so we stop the robot to allow the network tables to update about once 1500ms
		alignLoopCount++;
		if (alignLoopCount >= 15) {
			if (!networkUpdating) {
				networkUpdating = true;
			} else {
				networkUpdating = false;
			}
			alignLoopCount = 0;
		}

			// Pulls camera outputs from the net tables, detirmines what direction to drive in, and scales the speed of the robot based on the distance
		if (cameraOutput > 10 && !networkUpdating) {
			rightX1 = -1.25;
			multiplier = fabs(cameraOutput) * 0.075;
		} else if (cameraOutput < -10 && !networkUpdating) {
			rightX1 = 1.25;
			multiplier = fabs(cameraOutput) * 0.075;
		} else if (!networkUpdating) {
			alignState = "straight";
		}

		frc::SmartDashboard::PutNumber("Auto mult", multiplier);

			// Limits the speed of the algorithim to prevent cases where the robot over draws current, slids too far, or is unable to slide due to low motor power
		if (multiplier >= 1.75) {
			multiplier = 1.75;
		} else if (multiplier <= 0.75) {
			multiplier = 0.75;
		}

			// Maintains the angle of the robot as it slides
		if (currentAnlge < targetAngle - lineErrorMagrin) {
			leftX1 = 1.0 / multiplier;
		} else if (currentAnlge > targetAngle + lineErrorMagrin) {
			leftX1 = -1.0 / multiplier;
		}
	} else if (alignState == "straight") {
		rightY1 = (sqrt(wallDistance - 350) / -20);
		if (rightY1 > 0.5) {
			rightY1 = 0;
		}
		if (wallDistance <= 350) {
			alignState = "lift";
		}

		// Maintains the angle of the robot as it drives forward
		// if (currentAnlge < targetAngle - lineErrorMagrin) {
		// 	leftX1 = 1.0 / multiplier;
		// } else if (currentAnlge > targetAngle + lineErrorMagrin) {
		// 	leftX1 = -1.0 / multiplier;
		// }

			// detect when we reach the rocket
		// if (colliding) {
		// 	alignState = "none";
		// 	targetAngle = -1;
		// }
	} else if (alignState == "lift") {
		rightY1 = -0.5;
	}

	// End driver code; Begin attatchment code---------------------------------------------------------------------------------------------------------------------------

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

		dpad2 = xboxcontroller1.GetPOV();
	} else {
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

		dpad2 = xboxcontroller1.GetPOV();
	}

		// Checks if the raise elevator button was just pressed, and if it was it changes the desired elevator position
	if (yButton2 && !buttonsPressed[1][3] && desElevatorPosition != "high") {
		if (desElevatorPosition == "mid") {
			desElevatorPosition = "high";
		} else {
			desElevatorPosition = "mid";
		}
	} else if (aButton2 && !buttonsPressed[1][0] && desElevatorPosition != "low") {
		if (desElevatorPosition == "mid") {
			desElevatorPosition = "low";
		} else {
			desElevatorPosition = "mid";
		}
	}

	if (desElevatorPosition == "high") {
		frc::SmartDashboard::PutBoolean("elevatorHigh", true);
		frc::SmartDashboard::PutBoolean("elevatorMid", false);
		frc::SmartDashboard::PutBoolean("elevatorLow", false);
	} else if (desElevatorPosition == "mid") {
		frc::SmartDashboard::PutBoolean("elevatorHigh", false);
		frc::SmartDashboard::PutBoolean("elevatorMid", true);
		frc::SmartDashboard::PutBoolean("elevatorLow", false);
	} else {
		frc::SmartDashboard::PutBoolean("elevatorHigh", false);
		frc::SmartDashboard::PutBoolean("elevatorMid", false);
		frc::SmartDashboard::PutBoolean("elevatorLow", true);
	}

	// End controller code-----------------------------------------------------------------------------------------------------------------------------------------------

		// Now that we have all of the controller inputs, we set the motors to their cooresponding veriables
	if (fabs(rightX1) > fabs(rightY1) && fabs(rightX1) > fabs(leftX1)) {
		big = fabs(rightX1);
	} else if (fabs(rightY1) > fabs(rightX1) && fabs(rightY1) > fabs(leftX1) ) {
		big = fabs(rightY1);
	} else {
		big = fabs(leftX1);
	}

	if (mecanumDrive) {
		FrontLeft.Set(((cos(atan2(rightY1, rightX1) - 0.7853981633974483) + leftX1 * (1 - rightTrigger1)) / 2) * big * multiplier);
		FrontRight.Set(((sin(atan2(rightY1, rightX1) - 0.7853981633974483) - leftX1 * (1 - rightTrigger1)) / 2) * big * multiplier);
		BackLeft.Set(((sin(atan2(rightY1, rightX1) - 0.7853981633974483) + leftX1 * (1 - rightTrigger1)) / 2) * big * multiplier);
		BackRight.Set(((cos(atan2(rightY1, rightX1) - 0.7853981633974483) - leftX1 * (1 - rightTrigger1)) / 2) * big * multiplier);
	} else {
			// Tank drive code here
		FrontLeft.Set(0.0);
		FrontRight.Set(0.0);
		BackLeft.Set(0.0);
		BackRight.Set(0.0);
	}

		// Finally update our variables that track data from previous robot ticks
	buttonsPressed[0][0] = aButton1;
	buttonsPressed[0][1] = bButton1;
	buttonsPressed[0][2] = xButton1;
	buttonsPressed[0][3] = yButton1;
	buttonsPressed[0][4] = rightBumper1;
	buttonsPressed[0][5] = leftBumper1;

	buttonsPressed[1][0] = aButton2;
	buttonsPressed[1][1] = bButton2;
	buttonsPressed[1][2] = xButton2;
	buttonsPressed[1][3] = yButton2;
	buttonsPressed[1][4] = rightBumper2;
	buttonsPressed[1][5] = leftBumper2;

	lastAccelX = accelX;
	lastAccelY = accelY;
	if (sensorBoardType == "navx") {
		prevAnlge = navx->GetAngle();
	} else {
		prevAnlge = analogDev.GetAngle();
	}

		// Put your debugging code here
	frc::SmartDashboard::PutString("alignStateString", alignState);

		// While in this mode the joystick's Y axis is used to control 1 specifiic talon by an id taken from the shuffleboard
	if (motorDebug) {
		double leftY1 = 0.0;
		leftY1 = xboxcontroller0.GetY(frc::Joystick::kLeftHand);
		testMotor = frc::SmartDashboard::GetNumber("testMotor", -1.0);

		switch (testMotor)	{
			case 1:
				FrontLeft.Set(leftY1/2);
				break;
			case 2:
				BackLeft.Set(leftY1/2);
				break;
			case 3:
				FrontRight.Set(leftY1/2);
				break;
			case 4:
				BackRight.Set(leftY1/2);
				break;
			case 5:
				Lift1.Set(leftY1/2);
				break;
			case 6:
				Lift2.Set(leftY1/2);
				break;
			case 7:
				Winch.Set(leftY1/2);
				break;
			case 8:
				Climber.Set(leftY1/2);
				break;

			default:
				FrontLeft.Set(0.0);
				BackLeft.Set(0.0);
				FrontRight.Set(0.0);
				BackRight.Set(0.0);
				Lift1.Set(0.0);
				Lift2.Set(0.0);
				Winch.Set(0.0);
				Climber.Set(0.0);
				break;
		}
	}
}