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

#include <boost\math\constants\constants.hpp>

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


	//ush.processImage();
	InverseKinematics ik;
	IKResult new, old;
	UR5 robot;
	robot.connectToRobot(ROBOT_IP_LOCAL, ROBOT_PORT);
	robot.setSpeed(10);

	double target_x, target_y, target_z;
	target_x = -0.6;
	target_y = -0.3;
	target_z = 0.2;

	vector<double> target_tumor(3);
	vector<double> target_window(3);
	vector<double> direction(3);

	direction = target_window - target_tumor;
	
	robot.orientateAlongVector(direction[0], direction[1], direction[2]);
	bool done = false;
	while (!done) {

	}
	
	robot.moveToPosition(target_x, target_y, target_z + Z_OFFSET);
	robot.moveToPosition(target_x, target_y, target_z);

	//robot.moveToPosition(0.6, 0.2, 0.2);
	
	system("Pause");
	return 0;
}

