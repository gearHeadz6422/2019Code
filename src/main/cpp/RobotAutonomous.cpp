/*
 * RobotAutonomous.cpp
 *
 *  Created on: Jan 30, 2017
 *      Author: pinkenbu
 */

#include "Robot.h"
#include <Encoder.h>
#include <SampleRobot.h>
#include <SmartDashboard/SmartDashboard.h>
#include <Timer.h>
#include <cmath>
using namespace std;

#define LEFT
bool forwarddone = false;
float encoderDistanceReading = 0.0;
int autoPick = 0;
double maxAccelDiffX = 0.0;
double maxAccelDiffY = 0.0;
int x = 0;
double AccelDiffX = 0.0;
double AccelDiffY = 0.0;
double velX = 0.0;
double velY = 0.0;
bool posAccelX;
bool posAccelY;
bool posVelX;
bool posVelY;
float err = 0.0;

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
	CameraLightOn();
	autoPick = int(frc::SmartDashboard::GetNumber("DB/Slider 0", 0.0));
	/*
	 autoSelected = chooser.GetSelected();
	 std::string autoSelected = SmartDashboard::GetString("Auto Selector",
	 autoNameDefault);
	 std::cout << "Auto selected: " << autoSelected << std::endl;

	 if (autoSelected == autoNameCustom) {
	 // Custom Auto goes here
	 } else {
	 // Default Auto goes here
	 }
	 */
	encoderDistanceReading = 0.0;
	MotorSpeedRight(0.0);
	MotorSpeedLeft(0.0);
	m_encoder_left.Reset();
	m_encoder_right.Reset();
	ahrs->ZeroYaw();
	colliding = false;
	err = .25;
	x = 0;
}

void Robot::AutonomousPeriodic() {
	frc::SmartDashboard::PutNumber("Gyro" , ahrs->GetAngle());
	frc::SmartDashboard::PutNumber("Linear accel X", accelX * 1000);
	frc::SmartDashboard::PutNumber("Linear accel Y", accelY * 1000);
	accelX = ahrs->GetWorldLinearAccelX();
	accelY = ahrs->GetWorldLinearAccelY();
	velX = ahrs->GetVelocityX();
	velY = ahrs->GetVelocityY();

	frc::SmartDashboard::PutBoolean("Currently crashing into like a wall or some shit. Like just stop moving maybe?", colliding);
	AccelDiffX = fabs(lastAccelX) - fabs(accelX);
	AccelDiffY = fabs(lastAccelY) - fabs(accelY);
	/*if ((AccelDiffX < -0.05 || AccelDiffY < -0.05) && x > 0) {
		colliding = true;
	}
	*/
	if(AccelDiffX < maxAccelDiffX) {
		maxAccelDiffX = AccelDiffX;
	}
	if(AccelDiffY < maxAccelDiffY) {
		maxAccelDiffY = AccelDiffY;
	}
	if (x == 5) {
		if (velX > 0.05){
			posVelX = true;
		}
		else if (velX < -0.05){
			posVelX = false;
		}
		if (velY > 0.05){
			posVelY = true;
		}
		else if (velY < -0.05){
			posVelY = false;
		}
	}
	if (((posVelX && accelX < -err) || (posVelY && accelY < -err) || (!posVelX && accelX > err) || (!posVelY && accelY > err)) && x > 10) {
		colliding = true;
	}
	if (!colliding) {
		MotorSpeedLeft(0.3);
		MotorSpeedRight(0.3);
	}
	else {
		MotorSpeedLeft(0.0);
		MotorSpeedRight(0.0);
	}
	frc::SmartDashboard::PutNumber("maxAccelDiffX", maxAccelDiffX);
	frc::SmartDashboard::PutNumber("maxAccelDiffY", maxAccelDiffY);
	frc::SmartDashboard::PutNumber("accelY", accelY);
	frc::SmartDashboard::PutNumber("accelX", accelX);
	lastAccelX = accelX;
	lastAccelY = accelY;
	x = x + 1;
	/*if (ahrs->GetAngle() < 75) {
		 talon0.Set(0.05);
		 talon1.Set(0.05);
	 } else {
		 talon0.Set(0.0);
		 talon1.Set(0.0);
	 }
	 */


/*
	static int ksol = -9;
	if (ksol != frc::DoubleSolenoid::kForward) {
		m_doubleSolenoid.Set(frc::DoubleSolenoid::kReverse);
		ksol = frc::DoubleSolenoid::kForward;
		cout << "forward" << endl;
	}
*/
	//		m_doubleSolenoid.Set(frc::DoubleSolenoid::kReverse);
	  // Wait for a motor update time
	/*
	static float wheel_circumference = 6;
	static float pulse_per_revolution = 360;

	float distanceperpulse = (M_PI*wheel_circumference/pulse_per_revolution);
	if (autoPick <=1) {
#ifdef LEFT
	encoderDistanceReading = m_encoder_left.GetDistance();
		if (encoderDistanceReading < 35) {
			//35 before
			MotorSpeedRight(0.27);
			MotorSpeedLeft(0.27);
		} else {
			forwarddone=true;
			MotorSpeedRight(0.0);
			MotorSpeedLeft(0.0);
		}
#else
	encoderDistanceReading = m_encoder_right.GetDistance();
	if (encoderDistanceReading > -35) {
		MotorSpeedRight(0.27);
		MotorSpeedLeft(0.27);
	}
	else{
		forwarddone=true;
		MotorSpeedRight(0.0);
		MotorSpeedLeft(0.0);
	}
	}
#endif
}
else if(autoPick >= 1 && autoPick < 2) {
	if(m_encoder_left.GetDistance()< 35) {
		MotorSpeedLeft(0.3);
		MotorSpeedRight(0.3);
	}
	else{
		MotorSpeedLeft(0.0);
		//MotorSpeedRight(0.0);
		forwarddone = true;

	if(ahrs->GetAngle() < 45 && forwarddone == true) {
		MotorSpeedRight(0.3);
	}
	else {
		//MotorSpeedRight(0.0);

	if(m_encoder_left.GetDistance() < 50){
		MotorSpeedLeft(0.3);
		MotorSpeedRight(0.3);
	}
	else{
		MotorSpeedLeft(0.0);
		MotorSpeedRight(0.0);
	}
	}
	}
}
else if (autoPick >= 2 && autoPick < 3){
	if(m_encoder_left.GetDistance() < 45) {
			MotorSpeedLeft(0.3);
			MotorSpeedRight(0.3);
		}
		else{
			//MotorSpeedLeft(0.0);
			MotorSpeedRight(0.0);
			forwarddone = true;

		if(ahrs->GetAngle() > -55 && forwarddone == true) {
			MotorSpeedLeft(0.3);
		}
		else {
			//MotorSpeedLeft(0.0);

		if(m_encoder_left.GetDistance() < 60){
			MotorSpeedLeft(0.3);
			MotorSpeedRight(0.3);
		}
		else{
			MotorSpeedLeft(0.0);
			MotorSpeedRight(0.0);
		}
		}
		}
}
else if (autoPick >= 3) {
#ifdef LEFT
	encoderDistanceReading = m_encoder_left.GetDistance();
		if (encoderDistanceReading < 70) {
			//35 before
			MotorSpeedRight(0.3);
			MotorSpeedLeft(0.3);
		} else {
			forwarddone=true;
			MotorSpeedRight(0.0);
			MotorSpeedLeft(0.0);
		}
#else
	encoderDistanceReading = m_encoder_right.GetDistance();
	if (encoderDistanceReading > -35) {
		MotorSpeedRight(0.3);
		MotorSpeedLeft(0.3);
	}
	else{
		forwarddone=true;
		MotorSpeedRight(0.0);
		MotorSpeedLeft(0.0);
	}
	}
#endif
}
	frc::SmartDashboard::PutNumber("Encoder Left", encoderDistanceReading);
	frc::SmartDashboard::PutNumber("Encoder Right", m_encoder_right.GetDistance());

	if (autoSelected == autoNameCustom) {
		// Custom Auto goes here
	} else {
		// Default Auto goes here
	}
/*
	if (forwarddone)
	{
	if (ahrs->GetAngle() < 45) {
		MotorSpeedRight(0.2);
	}
	else {
		MotorSpeedRight(0.0);
	}
	}
*/

}


