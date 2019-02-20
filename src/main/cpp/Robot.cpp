#include "Robot.h"
#include <unistd.h>

//static void VisionThread();

using namespace std;

Robot::Robot() :
	//left_master = front left
	//left_slave = back left
	//right_master = front right
	//right_slave = back right
	//reverse(1),
			m_pdp(0),
	FrontLeft(1), BackLeft(2),
	FrontRight(3), BackRight(4),
	//sp(1),
	liftHigh(9), liftLow(5), intake(7), climber(8),
	Hook(0)
{


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

	liftTargetingTable *liftTargets;
	liftTargets = new liftTargetingTable();
}

void Robot::RobotInit() {
		// Set up the initial state of our shuffleboard
	frc::SmartDashboard::PutNumber("desAutoDelay", 0);
}
int main() { return frc::StartRobot<Robot>(); }