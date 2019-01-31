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
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableInstance.h"
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

class Robot: public frc::TimedRobot

{

  public: 
	Robot();
    virtual ~Robot(){}

		// FRC built in functions - never delete thsese
	  void DisabledInit()  override;
	  void DisabledPeriodic()  override {}
	  void RobotInit()  override;
	  void RobotPeriodic()  override {}
	  void AutonomousInit() override;
	  void AutonomousPeriodic() override;
	  void TeleopInit() override;
	  void TeleopPeriodic() override;
	  void TestInit() override;
  	  void TestPeriodic() override;

	void InitEncoder(frc::Encoder &enc);
	void CameraLightOn();
	void CameraLightOff();

  private:
	static void VisionThread();
	void MotorSpeedLeft(const float val);
	void MotorSpeedRight(const float val);
	int kUltrasonicPort = 0;

	double maxpwr[6];
	int waitper = 0;
	frc::Encoder leftEncoder { 2, 3, true, Encoder::k4X };
	frc::Encoder rightEncoder { 0, 1, false, Encoder::k4X };
	frc::Encoder winchEncoder { 4, 5, false, Encoder::k4X };
	frc::Encoder climberEncoder { 6, 7, false, Encoder::k4X };

	float winchSpeed = 0.0;
	float rampSpeed = 0.0;

	frc::SendableChooser<std::string> chooser;

	const std::string autoNameDefault = "Default";
	const std::string autoNameCustom = "My Auto";
	std::string autoSelected;
	frc::XboxController xboxcontroller0{0};
	frc::XboxController xboxcontroller1{1};
	frc::XboxController xboxcontroller2{2};
	
	frc::PowerDistributionPanel m_pdp;
	WPI_TalonSRX FrontLeft;
	WPI_TalonSRX BackLeft;
	WPI_TalonSRX FrontRight;
	WPI_TalonSRX BackRight;
	WPI_TalonSRX Shooter1;
	WPI_TalonSRX Shooter2;
	WPI_TalonSRX Winch;
	WPI_TalonSRX Climber;

		// Driver station cameras get initialized here
	cs::UsbCamera camera0;
	cs::UsbCamera camera1;
	
    AHRS *ahrs;
    float angle;
    Servo Hook;
};

#endif /* SRC_ROBOT_H_ */