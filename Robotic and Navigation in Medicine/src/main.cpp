#define D_SCL_SECURE_NO_WARNINGS 
#include <iostream>
#include "Robot\UR5.h"
#include "Ultra Sound\USHandler.h"
#include <opencv2\highgui\highgui.hpp>
#include "Kinematic\directKinematics.h"
#include "Kinematic\JointAngles.h"
#include "Kinematic\inverseKinematics.h"

#include "boost\numeric\ublas\matrix.hpp"
#include "boost\numeric\ublas\io.hpp"
#include <stdlib.h>
#include <time.h>
#include <chrono>
#include <thread>
#include "Math\InvertMatrix.h"
#include <boost\math\constants\constants.hpp>
#include "Tools\CSVParser.hpp"
#include "Math\Window.h"

#define Z_OFFSET 0.3
#define PI boost::math::constants::pi<double>()

int main(int argc, char* argv[])
{
	//matrix<double> trans(4,4);
	////USHandler ush;
	//trans(0, 0) = -0.294;
	//trans(1, 0) = 0.2461;
	//trans(2, 0) = -0.9236;
	//trans(3, 0) = 0;

	//trans(0, 1) = -0.7791;
	//trans(1, 1) = -0.6214;
	//trans(2, 1) = 0.0825;
	//trans(3, 1) = 0;
	//
	//trans(0, 2) = -0.5536;
	//trans(1, 2) = 0.7438;
	//trans(2, 2) = 0.3745;
	//trans(3, 2) = 0;
	//
	//trans(0, 3) = -1.1666;
	//trans(1, 3) = 0.5527;
	//trans(2, 3) = 1.0429;
	//trans(3, 3) = 1;

	//matrix<double> targetCameraCoord(4,4);
	//targetCameraCoord(0, 0) = 0.304;
	//targetCameraCoord(0, 1) = -0.138;
	//targetCameraCoord(0, 2) = 0.943;
	//targetCameraCoord(0, 3) = -0.133;

	//targetCameraCoord(1, 0) = -0.267;
	//targetCameraCoord(1, 1) = -0.962;
	//targetCameraCoord(1, 2) = -0.055;
	//targetCameraCoord(1, 3) = -0.291;

	//targetCameraCoord(2, 0) = 0.914;
	//targetCameraCoord(2, 1) = -0.235;
	//targetCameraCoord(2, 2) = -0.330;
	//targetCameraCoord(2, 3) = -1.57;

	//targetCameraCoord(3, 0) = 0;
	//targetCameraCoord(3, 1) = 0;
	//targetCameraCoord(3, 2) = 0;
	//targetCameraCoord(3, 3) = 1;

	//matrix<double> targetRobotCoord(4,4);

	//targetRobotCoord = prod(trans, targetCameraCoord);

	//std::cout << targetRobotCoord << std::endl;

	// read files
	CSVParser csvParser;
	boost::numeric::ublas::matrix<double> robot_to_cam_transformation = csvParser.readHTM("C:\\Users\\rnm_grp4\\Documents\\Robotics in Medicine\\RoboticInMedicine\\rob2cam.csv");
	boost::numeric::ublas::matrix<double> robot_to_needle_transformation = csvParser.readHTM("C:\\Users\\rnm_grp4\\Documents\\Robotics in Medicine\\RoboticInMedicine\\rob2needle.csv");
	boost::numeric::ublas::matrix<double> marker_position = csvParser.readHTM("C:\\Users\\rnm_grp4\\Documents\\Robotics in Medicine\\RoboticInMedicine\\camHTM.csv");
	boost::numeric::ublas::matrix<double> pixel_to_probe = csvParser.readHTM("C:\\Users\\rnm_grp4\\Documents\\Robotics in Medicine\\RoboticInMedicine\\ImageToProbe.csv");
	boost::numeric::ublas::matrix<double> probe_pose = csvParser.readHTM("C:\\Users\\rnm_grp4\\Documents\\Robotics in Medicine\\RoboticInMedicine\\fileoutpos.txt");
	std::vector<boost::numeric::ublas::vector<double>> windowCoord = csvParser.readWindow("C:\\Users\\rnm_grp4\\Documents\\Robotics in Medicine\\RoboticInMedicine\\windowPoints.csv");

	
	
	UR5 robot;
	robot.connectToRobot(ROBOT_IP_LABOR, ROBOT_PORT);
	robot.setSpeed(10);
	robot.setRobotToCamTransformation(robot_to_cam_transformation);
	robot.setRobotToNeedleTransformation(robot_to_needle_transformation);
	robot.setPixelToProbeTransformation(pixel_to_probe);

	//USTEST
	vector<double> z(4);
	vector<double> test = robot.convertPixelToProbe(266, 143);
	z = prod(probe_pose, test);
	std::cout << "US Point" << z << std::endl << std::endl;

	Window w(windowCoord, z);

	vector<double> middle(3);
	vector<double> dir(3);

	middle = w.get_middle();
	dir = w.get_tumor_dir();

	matrix<double> homo_z(4,4);
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			if (i == j) {
				homo_z(i, j) = 1;
			}
			else if (j == 3){
				homo_z(i, j) = z(i);
			}
		}
	}
	
	matrix<double> homo_m(4, 4);

	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			if (i == j) {
				homo_m(i, j) = 1;
			}
			else if (j == 3){
				homo_m(i, j) = middle(i);
			}
		}
	}

	matrix<double> homo_d(4, 4);

	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			if (i == j) {
				homo_d(i, j) = 1;
			}
			else if (j == 3){
				homo_d(i, j) = dir(i);
			}
		}
	}
	matrix<double> robot_window = robot.convertCamToRobPose(homo_m);
	matrix<double> robot_target = robot.convertCamToRobPose(homo_z);

	robot.doNeedlePlacement(column(robot_target, 3), column(robot_window, 3), robot.getRobotToNeedleTransformation());

	vector<double> target(3), window(3);
	//matrix<double> needleTip(4, 4);
	matrix<double> needleTip = robot.getRobotToNeedleTransformation();
	matrix<double> targetM = robot.convertCamToRobPose(marker_position);
	matrix<double> targetM2(4,4);
	matrix<double> invertNeedle(4, 4);

	InvertMatrix(needleTip,invertNeedle);
	
	targetM2 = prod(targetM,invertNeedle);
	std::cout << "NeedleTip : " << invertNeedle << std::endl << std::endl;
	std::cout << "Target M : " << targetM << std::endl << std::endl;
	std::cout << "Target : " << targetM2 << std::endl;
	DirectKinematics dk;
	matrix<double> a(4, 4);
	matrix<double> b(4, 4);
	matrix<double> c(4, 4);
	matrix<double> d(4, 4);
	a = dk.getPositionOfJoint(6, robot.getJoints("rad"));
	b = prod(a, needleTip);
	//robot.moveToPosition(targetM2(0, 3), targetM2(1, 3), targetM2(2, 3));
	target(0) = 0.3;
	target(1) = 0.4;
	target(2) = 0.2;

	window(0) = 0.3;
	window(1) = 0.3;
	window(2) = 0.25;
	/*robot.moveToHomePosition();
	robot.waitUntilFinished(500);
	robot.doNeedlePlacement(target, window, needleTip);
	*///ush.processImage();
	
	

	//robot.moveToPosition(0.6, 0.2, 0.2);
	
	system("Pause");
	return 0;
}

