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
#include <AnalogInput.h>
#include <DoubleSolenoid.h>
#include <Compressor.h>
#include <Encoder.h>
#include <IterativeRobot.h>
#include <LiveWindow/LiveWindow.h>
#include <PowerDistributionPanel.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include <XboxController.h>
#include <iostream>
#include <memory>
#include <string>
#include <Encoder.h>
#include <Timer.h>
#include <opencv2/core/types.hpp>

/*button codes
 * 1 - A
 * 2 - B
 * 3 - X
 * 4 - Y
 * 5 - LB
 * 6 - RB
 * 7 - SELECT
 * 8 - START
 * 9 - L STICK
 * 10 - R STICK
*/

//Driver controller
const int kReverseButton = 4;
const int kForwardButton = 6;
const int kTriggerleft = 5;

//Attatchment controllerXs
const int kBallButton = 1;
const int kBallButtonOff = 2;
const int kWinchButton = 3;
const int kWinchButtonOff = 4;
const int kBallShootButton = 6;
const int kBallShootButtonOff = 5;

//Other
const float kUpdatePeriod = 0.005;
const float kValueToCM = 0.144;

static bool reverseControl __attribute__((unused)) = false;
static double visionPwr __attribute__((unused)) = 0;
static std::vector<std::vector<cv::Point> > B;

class Robot: public frc::IterativeRobot

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

	  void TestInit() override{}
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
int reverse;
//Compressor *compressor;
frc::Compressor Ground_Beef;
frc::DoubleSolenoid Diced_Tomato {6,7};
//   frc::LiveWindow* lw = LiveWindow::GetInstance();
	frc::SendableChooser<std::string> chooser;

	frc::Solenoid m_solenoid { 16, 0 };
	frc::Solenoid m_solenoid_gear { 16, 1 };
//	frc::DoubleSolenoid m_doubleSolenoid { 16, 0, 1 };
	const std::string autoNameDefault = "Default";
	const std::string autoNameCustom = "My Auto";
	std::string autoSelected;
//	frc::Joystick xboxcontroller0{0};
	frc::XboxController xboxcontroller0{0};
	frc::XboxController xboxcontroller1{1};
    frc::Encoder m_encoder_left { 2, 3, false, Encoder::k4X };
	frc::Encoder m_encoder_right { 0, 1, false, Encoder::k4X };
    AHRS *ahrs;                         // navX MXP
float angle;
float ballpickspeedval;
float winchspeedval;
bool buttonpushed[7];
bool colliding;
double accelX;
double accelY;
double lastAccelX;
double lastAccelY;
frc::PowerDistributionPanel m_pdp{17};
frc::AnalogInput ultrasonic { kUltrasonicPort };
	WPI_TalonSRX talon_left_master;
	WPI_TalonSRX talon_left_slave;
	WPI_TalonSRX talon_right_master;
	WPI_TalonSRX talon_right_slave;
	WPI_TalonSRX talon_winch;
	WPI_TalonSRX talon_ballpick;
	WPI_TalonSRX talon_ballshoot;
	WPI_TalonSRX climber;
	cs::UsbCamera camera0;
	cs::UsbCamera camera1;
	Servo calzone;

};




#endif /* SRC_ROBOT_H_ */
