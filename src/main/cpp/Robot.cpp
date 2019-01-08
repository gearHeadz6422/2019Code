#include "Robot.h"
#include <unistd.h>

//static void VisionThread();

using namespace std;


Robot::Robot() :
		reverse(1),ahrs(nullptr), ballpickspeedval(0),
		winchspeedval(0), talon_left_master(0), talon_left_slave(1),
		talon_right_master(2), talon_right_slave(3),
		talon_winch(4),
		talon_ballpick(5),
		talon_ballshoot(6),
		climber(8),
		calzone(0),
		Diced_Tomato(0,6,7),
		Ground_Beef(5)
//		xboxctrl(0)
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
		ahrs = new AHRS(SPI::Port::kMXP);
		int n = 0;
// wait till calibration is finished
		// check if navx board is still calibrating, if so
		// sleep 1 sec and try again
		// do this for 20 times before giving up
		// this prevents a hangup if the board never calibrates
		// [IsCalibrating() returns true]
		while (n < 20)
		{
			n++;
			if (ahrs->IsCalibrating()) {
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
	talon_left_master.SetInverted(true);
	talon_left_slave.SetInverted(true);
//	talon_left_slave.SetControlMode(CANSpeedController::kFollower);
//	talon_left_slave.Set(talon_left_master.GetDeviceID());
//	talon_right_slave.SetControlMode(CANSpeedController::kFollower);
//	talon_right_slave.Set(talon_right_master.GetDeviceID());

	talon_ballpick.SetNeutralMode(NeutralMode::Coast);

	InitEncoder(m_encoder_left);
	InitEncoder(m_encoder_right);
	angle = ahrs->GetAngle();
	ahrs->ZeroYaw();
	memset(buttonpushed,0,sizeof(buttonpushed));
}

void Robot::RobotInit() {
	/*
	 chooser.AddDefault(autoNameDefault, autoNameDefault);
	 chooser.AddObject(autoNameCustom, autoNameCustom);
	 frc::SmartDashboard::PutData("Auto Modes", &chooser);
	 */
	camera0 = CameraServer::GetInstance()->StartAutomaticCapture(0);
	camera0.SetResolution(160, 120);
	camera0.SetFPS(10);
//	camera1 = CameraServer::GetInstance()->StartAutomaticCapture(1);
//	cs::UsbCamera camera =
//			CameraServer::GetInstance()->StartAutomaticCapture(1);
	// We need to run our vision program in a separate Thread.
	// If not, our robot program will not run
	std::thread visionThread(VisionThread);
	visionThread.detach();
}



START_ROBOT_CLASS(Robot)

