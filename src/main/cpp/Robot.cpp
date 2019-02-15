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
	Shooter1(5), Shooter2(6), Winch(7), Climber(8),
	Hook(0)
{


	try {
		/***********************************************************************
		 * navX-MXP:
		 * - Communication via RoboRIO MXP (SPI, I2C, TTL UART) and USB.
		 * - See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface.
		 *
		 * navX-Micro:
		 * - Communication via I2C (RoboRIO MXP or Onboard) and USB.
		 * - See http://navx-micro.kauailabs.com/guidance/selecting-an-interface.
		 *
		 * Multiple navX-model devices on a single robot are supported.
		 ************************************************************************/
		navx = new AHRS(SPI::Port::kMXP);
		int n = 0;
// wait till calibration is finished
		// check if navx board is still calibrating, if so
		// sleep 1 sec and try again
		// do this for 20 times before giving up
		// this prevents a hangup if the board never calibrates
		// [IsCalibrating() returns true]
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

	//InitEncoder(m_encoder_left);
	//I/nitEncoder(m_encoder_right);
	angle = navx->GetAngle();
	navx->ZeroYaw();
	memset(maxpwr,0,sizeof(maxpwr));
}

void Robot::RobotInit() {
	//  chooser.AddDefault(autoNameDefault, autoNameDefault);
	//  chooser.AddObject(autoNameCustom, autoNameCustom);
	//  frc::SmartDashboard::PutData("Auto Modes", &chooser);

	// camera0 = CameraServer::GetInstance()->StartAutomaticCapture(0);
	// camera0.SetResolution(160, 120);
	// camera0.SetFPS(30);

	// camera1 = CameraServer::GetInstance()->StartAutomaticCapture(1);
	// camera1.SetResolution(160, 120);
	// camera1.SetFPS(30);
//frc::StartRobot<Robot>();

	auto netTable = nt::NetworkTableInstance::GetDefault().GetTable("SmartDashboard");
	auto cameraOut = netTable->GetEntry("cameraOut");
	cameraOut.SetString("Left");
}
int main() { return frc::StartRobot<Robot>(); }
//START_ROBOT_CLASS(Robot)
