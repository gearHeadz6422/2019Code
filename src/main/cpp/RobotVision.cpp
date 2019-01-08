/*
 * RobotVision.cpp
 *
 *  Created on: Feb 20, 2017
 *      Author: pinkenbu
 */

#include "Robot.h"
#include "GripPipeline.h"

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/types.hpp>
#include <thread>
#include <cscore.h>
#include <CameraServer.h>
#include <IterativeRobot.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/types.hpp>
#include <iostream>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>

void Robot::VisionThread() {
	int cameranum = 1;
	double point1y;
	double point1x;
	double point0y;
	double point0x;
	double nbTest;
	double refreshes;
	int icnt = 1000;
	// Get the USB camera from CameraServer
	cs::UsbCamera camera1 = CameraServer::GetInstance()->StartAutomaticCapture(
			cameranum);
	// Set the resolution
	camera1.SetResolution(640, 480);
	camera1.SetFPS(15);
	//camera1.SetPixelFormat(cs::VideoMode::kMJPEG);

	// Get a CvSink. This will capture Mats from the Camera
	cs::CvSink cvSink = CameraServer::GetInstance()->GetVideo(camera1);
	// Setup a CvSource. This will send images back to the Dashboard
	cs::CvSource outputStream = CameraServer::GetInstance()->PutVideo(
			"Rectangle", 640, 480);

	grip::GripPipeline grp;
	// Mats are very memory expensive. Lets reuse this Mat.
	cv::Mat mat;
	//cv::VideoCapture cap(cameranum);
	//rectangle(*grp.gethslThresholdOutput(), *grp.getfilterContoursOutput());

	while (true) {
		// Tell the CvSink to grab a frame from the camera and put it
		// in the source mat.  If there is an error notify the output.
		if (cvSink.GrabFrame(mat) == 0) {
			// Send the output the error.
			outputStream.NotifyError(cvSink.GetError());
			// skip the rest of the current iteration
			continue;
		}
		grp.Process(mat);

		B = *grp.GetFilterContoursOutput();
		unsigned int nB = B.size();
		for (unsigned int i = 0; i < nB; i++) {
			std::vector<cv::Point> A = B[i];
			rectangle(*grp.GetHslThresholdOutput(), A[0], A[1],
					cv::Scalar(255, 255, 255), 5);
			point1y = A[1].y;
			point1x = A[1].x;
			point0y = A[0].y;
			point0x = A[0].x;

			rectangle(mat, A[1], A[0], cv::Scalar(255, 255, 255), 5);
		}
		nbTest = nB;
		if (icnt > 100) {
			/*std::cout << "----------refresh number " << refreshes << "----------"  << std::endl;
			 std::cout << "pt0Y: " << point1y << std::endl;
			 std::cout << "pt0X: " << point1x << std::endl;
			 std::cout << "pt1Y: " << point0x << std::endl;
			 std::cout << "pt1Y: " << point0y << std::endl;
			 std::cout << "nB test: " << nbTest  << std::endl;
			 icnt=0;
			 */
			/*
			 frc::SmartDashboard::PutNumber("pt0Y" , point0y);
			 frc::SmartDashboard::PutNumber("pt0X" , point0x);
			 frc::SmartDashboard::PutNumber("pt1Y" , point1y);
			 frc::SmartDashboard::PutNumber("pt1X" , point1x);
			 refreshes ++;
			 */
		}
		// Put a rectangle on the image
		// Give the output stream a new image to display
		outputStream.PutFrame(*grp.GetHslThresholdOutput());
		//outputStream.PutFrame(mat);
	}
}

