#include "Robot.h"
#include <cmath>

using namespace std;

// bool joystickMode = true;
// bool mecanumDrive = true;

// double prevAnlge = 0.0;
// double currentAnlge = 0.0;

// bool leftTurn = false;
// bool rightTurn = false;
// int targetAngle = -1;

// double polyCount = 0.0;
// double cameraOutput = 0.0;
// int alignLoopCount = 0;
// bool networkUpdating = false;
// std::string autoLineUp = "none";

// bool autoCompleted = false;

// int startPosition = 0;
// double desAutoDelay = 0;
// double smartDashTimerAuto = 0;
// double autoDelayTimer = 0;
// float leftInches = 0; //This variable accually uses the right encoder for now because of an emergancy fix
// float rightInches = 0;
// int stepCount = 0;
// float accelX = 0.0;
// float accelY = 0.0;
// float lastAccelX = 0.0;
// float lastAccelY = 0.0;
// bool colliding = false;
// // Network Tables
// nt::NetworkTableEntry cameraOut;

void Robot::TeleopInit() {
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

		// This variable is used to track which side of the rocket we are trying to line up with, or if we are centered
	autoLineUp = "none";
}

void Robot::TeleopPeriodic() {
		// This is the code that will cancel operations that are posing real world danger. This stuff is very important, so it comes first, and isn't dependent on any other systems actually working. If you need to change it for some reason, make sure it still works
	bool stop0 = xboxcontroller0.GetStartButton();
	bool stop1 = xboxcontroller1.GetStartButton();
	bool stop2 = xboxcontroller2.GetStartButton();
	bool stickMoved = false; // This is used later to cancel out of autonomous movements when the driver moves the stick
	if (stop0 || stop1 || stop2) {
		FrontLeft.Set(0.0);
		FrontRight.Set(0.0);
		BackLeft.Set(0.0);
		BackRight.Set(0.0);
		Hook.Set(0.0);
		Winch.Set(0.0);

		return;
	}

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

		leftX1 = sqrt(abs(xboxcontroller0.GetTriggerAxis(frc::Joystick::kLeftHand)/1.5)); // All driver stick inputs are curved. These can be adjusted to your liking. I recomend using some sort of visualizer like https://www.desmos.com/calculator

			// Measures the stick inputs related to moving and turning
		if (xboxcontroller0.GetTriggerAxis(frc::Joystick::kLeftHand) < 0) {
			leftX1 *= -1;
		}
		rightY1 = 1.3 * sqrt(abs(xboxcontroller0.GetY(frc::Joystick::kLeftHand)));
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

		// This later is synched with the multiplier
	leftTrigger1 = fabs(1 - (xboxcontroller0.GetTriggerAxis(frc::Joystick::kRightHand) / 2 + 0.5));
	if (aButton1) {
		leftTrigger1 = 1.0;
	}

	float multiplier = leftTrigger1 + 1;

	// Enables linear only movement
	if (bButton1) {
		if (fabs(rightX1) >= fabs(rightY1)) {
			rightY1 = 0.0;
		} else {
			rightX1 = 0.0;
		}
	}

		// Handles the 90 degree turn feature
	if (xButton1) {
		targetAngle = ahrs->GetAngle() - 85;
		leftTurn = false;
		rightTurn = true;
	} else if (yButton1) {
		targetAngle = ahrs->GetAngle() + 85;
		leftTurn = true;
		rightTurn = false;
	}
	if ((leftTurn || rightTurn) && ((rightTurn && ahrs->GetAngle() < targetAngle) || (leftTurn && ahrs->GetAngle() > targetAngle) || targetAngle == -1)) {
		targetAngle = -1;
		leftTurn = false;
		rightTurn = false;
	}
	if (leftTurn) {
		leftX1 = 1.2;
	} else if (rightTurn) {
		leftX1 = -1.2;
	}

		// Determines the desired line based on the current direction of the robot, and the joystick dpad
	if (dpad1 != -1) {
		// if (dpad1 == 0) {
		// 	autoLineUp = "top";
		// } else if (dpad1 == 90 || dpad1 == 270) {
		// 	autoLineUp = "side";
		// } else {
		// 	autoLineUp = "bottom";
		// }

		autoLineUp = "angled"; // TEMP
	}
	if (stickMoved) {
		autoLineUp = "none";
	}

		//Turns the robot to be aligned with the selected line
	// if (autoLineUp == "top") {
	// 	if (currentAnlge >= 0) {
	// 		if (118.75 - 5 >= currentAnlge || currentAnlge >= 118.75 + 5) {
	// 			if (currentAnlge <= 118.75) {
	// 				leftX1 = 1.0;
	// 			} else {
	// 				leftX1 = -1.0;
	// 			}
	// 		} else {
	// 			autoLineUp = "angled";
	// 		}	
	// 	} else {
	// 		if (-118.75 - 5 >= currentAnlge || currentAnlge >= -118.75 + 5) {
	// 			if (currentAnlge <= -118.75) {
	// 				leftX1 = 1.0;
	// 			} else {
	// 				leftX1 = -1.0;
	// 			}
	// 		} else {
	// 			autoLineUp = "angled";
	// 		}
	// 	}
	// } else if (autoLineUp == "side") {
	// 	if (currentAnlge >= 0) {
	// 		if (90 - 5 >= currentAnlge || currentAnlge >= 90 + 5) {
	// 			if (currentAnlge <= 90) {
	// 				leftX1 = 1.0;
	// 			} else {
	// 				leftX1 = -1.0;
	// 			}
	// 		} else {
	// 			autoLineUp = "angled";
	// 		}
	// 	} else {
	// 		if (-90 - 5 >= currentAnlge || currentAnlge >= -90 + 5) {
	// 			if (currentAnlge <= -90) {
	// 				leftX1 = 1.0;
	// 			} else {
	// 				leftX1 = -1.0;
	// 			}
	// 		} else {
	// 			autoLineUp = "angled";
	// 		}
	// 	}
	// } else if (autoLineUp == "bottom") {
	// 	if (currentAnlge >= 0) {
	// 		if (61.25 - 5 >= currentAnlge || currentAnlge >= 61.25 + 5) {
	// 			if (currentAnlge <= 61.25) {
	// 				leftX1 = 1.0;
	// 			} else {
	// 				leftX1 = -1.0;
	// 			}
	// 		} else {
	// 			autoLineUp = "angled";
	// 		}
	// 	} else {
	// 		if (-61.25 - 5 >= currentAnlge || currentAnlge >= -61.25 + 5) {
	// 			if (currentAnlge <= -61.25) {
	// 				leftX1 = 1.0;
	// 			} else {
	// 				leftX1 = -1.0;
	// 			}
	// 		} else {
	// 			autoLineUp = "angled";
	// 		}
	// 	}
	// }

		// Align the robot to a rocket
	frc::SmartDashboard::PutNumber("Angle", currentAnlge);
	if (autoLineUp == "angled") {
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
		if (cameraOutput > 50 && !networkUpdating) {
			rightX1 = -1.25;
			multiplier = fabs(cameraOutput) * 0.01;
		} else if (cameraOutput < -50 && !networkUpdating) {
			rightX1 = 1.25;
			multiplier = fabs(cameraOutput) * 0.01;
		} else if (!networkUpdating) {
			rightY1 = -1.25;
		}

			// Limits the maximum speed of the algorithim to prevent excess power draw and smooth movement
		if (multiplier >= 1.75) {
			multiplier = 1.75;
		}

			// Maintains the angle of the robot as it slides
		if (currentAnlge < -5) {
			leftX1 = 1.0 / multiplier;
		} else if (currentAnlge > 5) {
			leftX1 = -1.0 / multiplier;
		}
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

		dpad2 = xboxcontroller2.GetPOV();
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
	lastAccelX = accelX;
	lastAccelY = accelY;
	prevAnlge = ahrs->GetAngle();
}