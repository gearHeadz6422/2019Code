/*
 * Robot.h
 *
 *  Created on: Jan 21, 2017
 *      Author: pinkenbu
 */

#ifndef SRC_ROBOT_H_
#define SRC_ROBOT_H_

#include "AHRS.h"
#include <ctre/Phoenix.h>
#include <frc/WPIlib.h>
#include <iostream>
#include <memory>
#include <string>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
/*button codes
1 - A
2 - B
3 - X
4 - Y
5 - LB
6 - RB
7 - SELECT
8 - START
9 - L STICK
10 - R STICK
*/

const float kUpdatePeriod = 0.005;
const float kValueToCM = 0.144;

static bool reverseControl __attribute__((unused)) = false;
static double visionPwr __attribute__((unused)) = 0;
//static std::vector<std::vector<cv::Point> > B;

class Robot: public frc::TimedRobot

{

  public:
	Robot();
    virtual ~Robot(){}

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
//	  void StartCompetition()  override {}

	  void DisabledInit()  override;
	  void DisabledPeriodic()  override {}

	  // RobotInit is executed at power on
	  void RobotInit()  override;
      // RobotPeriodic is executed after the current mode Periodic is executed
	  // (means it is called in every mode)
	  void RobotPeriodic()  override {}

	  void AutonomousInit() override;
	  void AutonomousPeriodic() override;

	  void TeleopInit() override;
	  void TeleopPeriodic() override;

	  void TestInit() override;
  	  void TestPeriodic() override;

	//void AutonomousInit() override;

//	void Autonomous();

//	void AutonomousPeriodic();
//	void TeleopInit();
//	void TeleopPeriodic();
//	void TestPeriodic();
void InitEncoder(frc::Encoder &enc);
void CameraLightOn();
void CameraLightOff();


private:
static void VisionThread();
void MotorSpeedLeft(const float val);
void MotorSpeedRight(const float val);
int kUltrasonicPort = 0;
//int reverse;
//double maxpwr1 = 0;
//double maxpwr2 = 0;
double maxpwr[6];
int waitper = 0;
frc::Encoder leftEncoder { 2, 3, true, Encoder::k4X };
frc::Encoder rightEncoder { 0, 1, false, Encoder::k4X };
frc::Encoder winchEncoder { 4, 5, false, Encoder::k4X };
frc::Encoder climberEncoder { 6, 7, false, Encoder::k4X };

float winchSpeed = 0.0;
float rampSpeed = 0.0;

//Compressor *compressor;
//   frc::LiveWindow* lw = LiveWindow::GetInstance();
	frc::SendableChooser<std::string> chooser;

	const std::string autoNameDefault = "Default";
	const std::string autoNameCustom = "My Auto";
	std::string autoSelected;
	frc::XboxController xboxcontroller0{0};
	frc::XboxController xboxcontroller1{1};
	frc::XboxController xboxcontroller2{2};
bool buttonpushed[7];
frc::PowerDistributionPanel m_pdp;
	WPI_TalonSRX FrontLeft;
	WPI_TalonSRX BackLeft;
	WPI_TalonSRX FrontRight;
	WPI_TalonSRX BackRight;
	WPI_TalonSRX Shooter1;
	WPI_TalonSRX Shooter2;
	WPI_TalonSRX Winch;
	WPI_TalonSRX Climber;
	//frc::Encoder m_encoder_right { 0, 1, false, Encoder::k4X };
	cs::UsbCamera camera0;
	cs::UsbCamera camera1;
    AHRS *ahrs;
    float angle;
    Servo Hook;

    bool colliding;
    double accelX;
    double accelY;
    double lastAccelX;
    double lastAccelY;
};




#endif /* SRC_ROBOT_H_ */
