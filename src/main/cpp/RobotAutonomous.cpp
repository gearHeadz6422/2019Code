/*
 * RobotAutonomous.cpp
 *
 *  Created on: Jan 30, 2017
 *      Author: pinkenbu
 */

#include "Robot.h"
#include <cmath>
using namespace std;

#define LEFT
int startPosition = 0;
double desAutoDelay = 0;
double smartDashTimerAuto = 0;
double autoDelayTimer = 0;
int seconds = 0;
std::string gameData = "";
bool foundGameData = false;
float leftInches = 0; //This variable acTually uses the right encoder because of an emergancy fix
float rightInches = 0;
int stepCount = 0;

/*
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable
 * chooser code works with the Java SmartDashboard. If you prefer the
 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
 * GetString line to get the auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the
 * SendableChooser make sure to add them to the chooser code above as well.
 */
void Robot::AutonomousInit() {
	autoDelayTimer = 0;
	desAutoDelay = 0.0;

	accelX = 0.0;
	accelY = 0.0;

	lastAccelX = 0.0;
	lastAccelY = 0.0;

	startPosition = SmartDashboard::GetNumber("DB/Slider 0", 0.0);
	desAutoDelay = SmartDashboard::GetNumber("DB/Slider 1", 0.0);
	cout << desAutoDelay;

	leftEncoder.Reset();
	rightEncoder.Reset();
	winchEncoder.Reset();

	ahrs->ZeroYaw();

	stepCount = 0;
}

void Robot::AutonomousPeriodic() {
	frc::SmartDashboard::PutNumber("Linear accel X", accelX * 1000);
	frc::SmartDashboard::PutNumber("Linear accel Y", accelY * 1000);

	accelX = ahrs->GetWorldLinearAccelX() * 1000;
	accelY = ahrs->GetWorldLinearAccelY() * 1000;

	//waitper = 0;
	frc::SmartDashboard::PutNumber("Gyro" , ahrs->GetAngle());
	frc::SmartDashboard::PutNumber("desAutoDelay", desAutoDelay);
	frc::SmartDashboard::PutNumber("startPosition", startPosition);
	frc::SmartDashboard::PutNumber("autoDelayTimer", autoDelayTimer);

	startPosition = frc::SmartDashboard::GetNumber("startPosition", 0.0);
	if (smartDashTimerAuto < 10) {
		smartDashTimerAuto += 1;
	} else {
		smartDashTimerAuto = 0;
		frc::SmartDashboard::PutNumber("Left encoder", leftEncoder.GetDistance());
		frc::SmartDashboard::PutNumber("Right encoder", rightEncoder.GetDistance());
		frc::SmartDashboard::PutNumber("Winch encoder", winchEncoder.GetDistance());
		frc::SmartDashboard::PutNumber("Current step numb", stepCount);
		frc::SmartDashboard::PutString("Game data", gameData);
	}

	if (!foundGameData) {
		gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();
	}

	if(gameData.length() > 0) {
		foundGameData = true;
	}

	leftInches = rightEncoder.GetDistance() / (6*M_PI); //changed to right, it should be left!
	rightInches = rightEncoder.GetDistance() / (6 * M_PI);

	if (autoDelayTimer < desAutoDelay){
		autoDelayTimer += 0.05;
		cout << autoDelayTimer;
		cout << desAutoDelay;
	} else {
		if (startPosition == 5){
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
		if(startPosition == 4) {
			switch (stepCount) {
				case 0:
					if (fabs(lastAccelX) - fabs(accelX) > 500 || fabs(lastAccelY) - fabs(accelY) > 500) {
						stepCount++;
					}
					FrontLeft.Set(-0.35);
					FrontRight.Set(-0.35);
					BackLeft.Set(-0.35);
					BackRight.Set(-0.35);
				break;
				case 1:
					FrontLeft.Set(-0.35);
					FrontRight.Set(-0.35);
					BackLeft.Set(-0.35);
					BackRight.Set(-0.35);
				break;
			}
		}
		if (startPosition == 1) {
			if (gameData[0] == 'L'){
				switch(stepCount) {
					case 0:
						if (leftInches <= 50){
							FrontLeft.Set(-0.35);
							FrontRight.Set(-0.35);
							BackLeft.Set(-0.35);
							BackRight.Set(-0.35);
						} else {
							stepCount++;
							FrontLeft.Set(0.0);
							FrontRight.Set(0.0);
							BackLeft.Set(0.0);
							BackRight.Set(0.0);
							leftEncoder.Reset();
							rightEncoder.Reset();
						}
					break;
					case 1:
						if (ahrs->GetAngle() > -80){
							FrontRight.Set(0.3);
							BackRight.Set(0.3);
							FrontLeft.Set(-0.3);
							BackLeft.Set(-0.3);
						}
						else {
							stepCount++;
							FrontLeft.Set(0.0);
							FrontRight.Set(0.0);
							BackLeft.Set(0.0);
							BackRight.Set(0.0);
							leftEncoder.Reset();
							rightEncoder.Reset();
						}
					break;
					case 2:
						if (leftInches <= 100){
							FrontLeft.Set(-0.35);
							FrontRight.Set(-0.35);
							BackLeft.Set(-0.35);
							BackRight.Set(-0.35);
						} else {
							stepCount++;
							FrontLeft.Set(0.0);
							FrontRight.Set(0.0);
							BackLeft.Set(0.0);
							BackRight.Set(0.0);
							leftEncoder.Reset();
							rightEncoder.Reset();
						}
					break;
					case 3:
						if (ahrs->GetAngle() < 0){
							FrontRight.Set(-0.3);
							BackRight.Set(-0.3);
							FrontLeft.Set(0.3);
							BackLeft.Set(0.3);
						} else {
							stepCount++;
							FrontLeft.Set(0.0);
							FrontRight.Set(0.0);
							BackLeft.Set(0.0);
							BackRight.Set(0.0);
							leftEncoder.Reset();
							rightEncoder.Reset();
						}
					break;
					case 4:
						if (fabs(lastAccelX) - fabs(accelX) > 500 || fabs(lastAccelY) - fabs(accelY) > 500) {
							stepCount++;
						}

						if (leftInches <= 94){
							FrontLeft.Set(-0.35);
							FrontRight.Set(-0.35);
							BackLeft.Set(-0.35);
							BackRight.Set(-0.35);
						} else {
							stepCount++;
							FrontLeft.Set(0.0);
							FrontRight.Set(0.0);
							BackLeft.Set(0.0);
							BackRight.Set(0.0);
							leftEncoder.Reset();
							rightEncoder.Reset();
						}

						if (winchEncoder.GetDistance() > -1880) {
							Winch.Set(-1);
						} else {
							Winch.Set(0.0);
						}
					break;
					case 5:
						Winch.Set(0.0);
						FrontRight.Set(0.0);
						FrontLeft.Set(0.0);
						BackRight.Set(0.0);
						BackLeft.Set(0.0);
						Shooter1.Set(-0.35);
						Shooter2.Set(-0.35);
					break;
				}
			}

			if (gameData[0] == 'R') {
				switch (stepCount){
					case 0:
						if (fabs(lastAccelX) - fabs(accelX) > 500 || fabs(lastAccelY) - fabs(accelY) > 500) {
							stepCount++;
						}

						if (leftInches <= 108){
							FrontLeft.Set(-0.25);
							FrontRight.Set(-0.25);
							BackLeft.Set(-0.25);
							BackRight.Set(-0.25);
						} else {
							stepCount ++;
							FrontLeft.Set(0.0);
							FrontRight.Set(0.0);
							BackLeft.Set(0.0);
							BackRight.Set(0.0);
						}
						if (winchEncoder.GetDistance() > -1900) {
							Winch.Set(-1);
						} else {
							Winch.Set(0);
						}
					break;
					case 1:
							Shooter1.Set(-0.35);
							Shooter2.Set(-0.35);
					break;
				}
			}
		} else if (startPosition == 0) {
			if (gameData[0] == 'L') {
				switch (stepCount){
					case 0:
						if (fabs(lastAccelX) - fabs(accelX) > 500 || fabs(lastAccelY) - fabs(accelY) > 500) {
							stepCount++;
						}

						if (leftInches <= 98){
							FrontLeft.Set(-0.35);
							FrontRight.Set(-0.35);
							BackLeft.Set(-0.35);
							BackRight.Set(-0.35);
						} else {
							leftEncoder.Reset();
							rightEncoder.Reset();
							stepCount ++;
							FrontLeft.Set(0.0);
							FrontRight.Set(0.0);
							BackLeft.Set(0.0);
							BackRight.Set(0.0);
						}
						if (winchEncoder.GetDistance() > -1900) {
							Winch.Set(-1);
						} else {
							Winch.Set(0);
						}
					break;
					case 1:
						if (fabs(lastAccelX) - fabs(accelX) > 500 || fabs(lastAccelY) - fabs(accelY) > 500) {
							stepCount++;
						}

						if (ahrs->GetAngle() < 15){
							FrontRight.Set(-0.35);
							BackRight.Set(-0.35);
						}
						else {
							FrontRight.Set(0.0);
							FrontLeft.Set(0.0);
							leftEncoder.Reset();
							rightEncoder.Reset();
							stepCount ++;
						}
					break;
					case 2:
						FrontRight.Set(0.0);
						FrontLeft.Set(0.0);
						BackRight.Set(0.0);
						BackLeft.Set(0.0);
						Shooter1.Set(-0.35);
						Shooter2.Set(-0.35);
					break;
				}
			} else if (gameData[0] == 'R') {
				switch(stepCount) {
					case 0:
						if (leftInches <= 58){
							FrontLeft.Set(-0.35);
							FrontRight.Set(-0.35);
							BackLeft.Set(-0.35);
							BackRight.Set(-0.35);
						} else {
							stepCount++;
							FrontLeft.Set(0.0);
							FrontRight.Set(0.0);
							BackLeft.Set(0.0);
							BackRight.Set(0.0);
							leftEncoder.Reset();
							rightEncoder.Reset();
						}
					break;
					case 1:
						if (ahrs->GetAngle() < 90){
							FrontRight.Set(-0.3);
							BackRight.Set(-0.3);
							FrontLeft.Set(0.3);
							BackLeft.Set(0.3);
						}
						else {
							stepCount++;
							FrontLeft.Set(0.0);
							FrontRight.Set(0.0);
							BackLeft.Set(0.0);
							BackRight.Set(0.0);
							leftEncoder.Reset();
							rightEncoder.Reset();
						}
					break;
					case 2:
						if (leftInches <= 134){
							FrontLeft.Set(-0.35);
							FrontRight.Set(-0.35);
							BackLeft.Set(-0.35);
							BackRight.Set(-0.35);
						} else {
							stepCount++;
							FrontLeft.Set(0.0);
							FrontRight.Set(0.0);
							BackLeft.Set(0.0);
							BackRight.Set(0.0);
							leftEncoder.Reset();
							rightEncoder.Reset();
						}
					break;
					case 3:
						if (ahrs->GetAngle() > 0){
							FrontRight.Set(0.3);
							BackRight.Set(0.3);
							FrontLeft.Set(-0.3);
							BackLeft.Set(-0.3);
						}
						else {
							stepCount++;
							FrontLeft.Set(0.0);
							FrontRight.Set(0.0);
							BackLeft.Set(0.0);
							BackRight.Set(0.0);
							leftEncoder.Reset();
							rightEncoder.Reset();
						}
					break;
					case 4:
						if (fabs(lastAccelX) - fabs(accelX) > 500 || fabs(lastAccelY) - fabs(accelY) > 500) {
							stepCount++;
						}

						if (leftInches <= 56){
							FrontLeft.Set(-0.35);
							FrontRight.Set(-0.35);
							BackLeft.Set(-0.35);
							BackRight.Set(-0.35);
						} else {
							stepCount++;
							FrontLeft.Set(0.0);
							FrontRight.Set(0.0);
							BackLeft.Set(0.0);
							BackRight.Set(0.0);
							leftEncoder.Reset();
							rightEncoder.Reset();
						}

						if (winchEncoder.GetDistance() > -1880) {
							Winch.Set(-1);
						} else {
							Winch.Set(0.0);
						}
					break;
					case 5:
						Winch.Set(0.0);
						FrontRight.Set(0.0);
						FrontLeft.Set(0.0);
						BackRight.Set(0.0);
						BackLeft.Set(0.0);
						Shooter1.Set(-0.25);
						Shooter2.Set(-0.25);
					break;
				}
			}
		} else if (startPosition == 2) {
			if (gameData[0] == 'L'){
				switch(stepCount) {
					case 0:
						if (leftInches <= 58){
							FrontLeft.Set(-0.35);
							FrontRight.Set(-0.35);
							BackLeft.Set(-0.35);
							BackRight.Set(-0.35);
						} else {
							stepCount++;
							FrontLeft.Set(0.0);
							FrontRight.Set(0.0);
							BackLeft.Set(0.0);
							BackRight.Set(0.0);
							leftEncoder.Reset();
							rightEncoder.Reset();
						}
					break;
					case 1:
						if (ahrs->GetAngle() > -85){
							FrontRight.Set(0.3);
							BackRight.Set(0.3);
							FrontLeft.Set(-0.3);
							BackLeft.Set(-0.3);
						}
						else {
							stepCount++;
							FrontLeft.Set(0.0);
							FrontRight.Set(0.0);
							BackLeft.Set(0.0);
							BackRight.Set(0.0);
							leftEncoder.Reset();
							rightEncoder.Reset();
						}
					break;
					case 2:
						if (leftInches <= 134){
							FrontLeft.Set(-0.35);
							FrontRight.Set(-0.35);
							BackLeft.Set(-0.35);
							BackRight.Set(-0.35);
						} else {
							stepCount++;
							FrontLeft.Set(0.0);
							FrontRight.Set(0.0);
							BackLeft.Set(0.0);
							BackRight.Set(0.0);
							leftEncoder.Reset();
							rightEncoder.Reset();
						}
					break;
					case 3:
						if (ahrs->GetAngle() < 0){
							FrontRight.Set(-0.3);
							BackRight.Set(-0.3);
							FrontLeft.Set(0.3);
							BackLeft.Set(0.3);
						} else {
							stepCount++;
							FrontLeft.Set(0.0);
							FrontRight.Set(0.0);
							BackLeft.Set(0.0);
							BackRight.Set(0.0);
							leftEncoder.Reset();
							rightEncoder.Reset();
						}
					break;
					case 4:
						if (fabs(lastAccelX) - fabs(accelX) > 500 || fabs(lastAccelY) - fabs(accelY) > 500) {
							stepCount++;
						}

						if (leftInches <= 64){
							FrontLeft.Set(-0.35);
							FrontRight.Set(-0.35);
							BackLeft.Set(-0.35);
							BackRight.Set(-0.35);
						} else {
							stepCount++;
							FrontLeft.Set(0.0);
							FrontRight.Set(0.0);
							BackLeft.Set(0.0);
							BackRight.Set(0.0);
							leftEncoder.Reset();
							rightEncoder.Reset();
						}

						if (winchEncoder.GetDistance() > -1880) {
							Winch.Set(-1);
						} else {
							Winch.Set(0.0);
						}
					break;
					case 5:
						Winch.Set(0.0);
						FrontRight.Set(0.0);
						FrontLeft.Set(0.0);
						BackRight.Set(0.0);
						BackLeft.Set(0.0);
						Shooter1.Set(-0.25);
						Shooter2.Set(-0.25);
					break;
				}
			} else if (gameData[0] == 'R') {
				switch (stepCount){
					case 0:
						if (fabs(lastAccelX) - fabs(accelX) > 500 || fabs(lastAccelY) - fabs(accelY) > 500) {
							stepCount++;
						}

						if (leftInches <= 98){
							FrontLeft.Set(-0.35);
							FrontRight.Set(-0.35);
							BackLeft.Set(-0.35);
							BackRight.Set(-0.35);
						} else {
							leftEncoder.Reset();
							rightEncoder.Reset();
							stepCount ++;
							FrontLeft.Set(0.0);
							FrontRight.Set(0.0);
							BackLeft.Set(0.0);
							BackRight.Set(0.0);
						}
						if (winchEncoder.GetDistance() > -1900) {
							Winch.Set(-1);
						} else {
							Winch.Set(0);
						}
					break;
					/*case 1:
						if (ahrs->GetAngle() < 15){
							FrontRight.Set(-0.35);
							BackRight.Set(-0.35);
						}
						else {
							FrontRight.Set(0.0);
							FrontLeft.Set(0.0);
							leftEncoder.Reset();
							rightEncoder.Reset();
							stepCount ++;
						}
					break;
					case 2:*/
					case 1:
						FrontRight.Set(0.0);
						FrontLeft.Set(0.0);
						BackRight.Set(0.0);
						BackLeft.Set(0.0);
						Shooter1.Set(-0.35);
						Shooter2.Set(-0.35);
					break;
				}
			}
		}
	}
	lastAccelX = accelX;
	lastAccelY = accelY;
}
