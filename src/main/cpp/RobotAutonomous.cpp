#include "Robot.h"
#include <cmath>
using namespace std;

#define LEFT
int startPosition = 0;
double desAutoDelay = 0;
double smartDashTimerAuto = 0;
double autoDelayTimer = 0;
float leftInches = 0; //This variable acTually uses the right encoder because of an emergancy fix
float rightInches = 0;
int stepCount = 0;
float accelX = 0.0;
float accelY = 0.0;
float lastAccelX = 0.0;
float lastAccelY = 0.0;
bool colliding = false;
// Network Tables
nt::NetworkTableEntry cameraOut;

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
	}

	leftInches = rightEncoder.GetDistance() / (6*M_PI); //changed to right, it should be left!
	rightInches = rightEncoder.GetDistance() / (6 * M_PI);

	if (fabs(lastAccelX) - fabs(accelX) > 500 || fabs(lastAccelY) - fabs(accelY) > 500) {
		colliding = true;
	}

	if (autoDelayTimer < desAutoDelay){
		autoDelayTimer += 0.05;
		cout << autoDelayTimer;
		cout << desAutoDelay;
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
	lastAccelX = accelX;
	lastAccelY = accelY;
}
