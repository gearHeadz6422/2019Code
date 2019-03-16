#include "Robot.h"
#include <unistd.h>

//static void VisionThread();

using namespace std;

Robot::Robot() :
	m_pdp(0),

	FrontLeft(1),
	BackLeft(2),
	FrontRight(3),
	BackRight(4),
	liftLow(5),
	grabberWinch(6),
	intake(7),
	// climber(8),
	liftHigh(9),

	testTalon0(0),
	testTalon1(1),
	testTalon2(2),
	testTalon3(3),
	testTalon4(4),
	testTalon5(5),
	testTalon6(6),
	testTalon7(7),
	testTalon8(8),
	testTalon9(9)
{

	// testTalons.push_back(testTalon0);
	// testTalons.push_back(testTalon1);
	// testTalons.push_back(testTalon2);
	// testTalons.push_back(testTalon3);
	// testTalons.push_back(testTalon4);
	// testTalons.push_back(testTalon5);
	// testTalons.push_back(testTalon6);
	// testTalons.push_back(testTalon7);
	// testTalons.push_back(testTalon8);
	// testTalons.push_back(testTalon9);

	try {
		navx = new AHRS(SPI::Port::kMXP);
		int n = 0;
		while (n < 20){
			n++;
			if (navx->IsCalibrating()) {
				sleep(1);
			} else {
				break;
			}
		}
	} catch (std::exception& ex) {
		std::string err_string = "Error instantiating navX MXP:  ";
		err_string += ex.what();
		DriverStation::ReportError(err_string.c_str());
	}

	FrontRight.SetInverted(true);
	BackRight.SetInverted(true);
	//InitEncoder(m_encoder_right);
//	talon_left_slave.SetControlMode(CANSpeedController::kFollower);
//	talon_left_slave.Set(talon_left_master.GetDeviceID());
//	talon_right_slave.SetControlMode(CANSpeedController::kFollower);
//	talon_right_slave.Set(talon_right_master.GetDeviceID());

	//talon_ballpick.SetNeutralMode(NeutralMode::Coast);

	InitEncoder(liftEncoderHigh);
	angle = navx->GetAngle();
	navx->ZeroYaw();
	memset(maxpwr,0,sizeof(maxpwr));
}

void Robot::RobotInit() {
		// Set up the initial state of our shuffleb	oard
	frc::SmartDashboard::PutNumber("desAutoDelay", 0);
}
int main() { return frc::StartRobot<Robot>(); }