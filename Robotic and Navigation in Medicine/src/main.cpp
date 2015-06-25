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
#define PI boost::math::constants::pi<double>()

int main(int argc, char* argv[])
{
	matrix<double> trans(4,4);
	//USHandler ush;
	trans(0, 0) = -0.294;
	trans(1, 0) = 0.2461;
	trans(2, 0) = -0.9236;
	trans(3, 0) = 0;

	trans(0, 1) = -0.7791;
	trans(1, 1) = -0.6214;
	trans(2, 1) = 0.0825;
	trans(3, 1) = 0;
	
	trans(0, 2) = -0.5536;
	trans(1, 2) = 0.7438;
	trans(2, 2) = 0.3745;
	trans(3, 2) = 0;
	
	trans(0, 3) = -1.1666;
	trans(1, 3) = 0.5527;
	trans(2, 3) = 1.0429;
	trans(3, 3) = 1;

	matrix<double> targetCameraCoord(4,4);
	targetCameraCoord(0, 0) = 0.304;
	targetCameraCoord(0, 1) = -0.138;
	targetCameraCoord(0, 2) = 0.943;
	targetCameraCoord(0, 3) = -0.133;

	targetCameraCoord(1, 0) = -0.267;
	targetCameraCoord(1, 1) = -0.962;
	targetCameraCoord(1, 2) = -0.055;
	targetCameraCoord(1, 3) = -0.291;

	targetCameraCoord(2, 0) = 0.914;
	targetCameraCoord(2, 1) = -0.235;
	targetCameraCoord(2, 2) = -0.330;
	targetCameraCoord(2, 3) = -1.57;

	targetCameraCoord(3, 0) = 0;
	targetCameraCoord(3, 1) = 0;
	targetCameraCoord(3, 2) = 0;
	targetCameraCoord(3, 3) = 1;

	matrix<double> targetRobotCoord(4,4);

	targetRobotCoord = prod(trans, targetCameraCoord);

	std::cout << targetRobotCoord << std::endl;


	//ush.processImage();

	UR5 robot;

	////connect to the robot
	robot.connectToRobot(ROBOT_IP_LABOR, ROBOT_PORT);
	//set the robot's joints

	robot.setSpeed(10);
	//robot.moveToPose(targetRobotCoord);
	//robot.moveAlongVector(0, 0, -0.1);
	robot.moveToPosition(-0.6, -0.2, 0.3);
	
	//robot.rotateEndEffector(0, -PI / 2, 0);
	/*robot.moveToHomePosition();
	std::this_thread::sleep_for(std::chrono::milliseconds(15000));*/
	
	//robot.moveToPose(0.50, -0.3,0.2,0,0,0);
	
	//std::this_thread::sleep_for(std::chrono::milliseconds(10000));
	//robot.orientateAlongVector(-1, 0, 0);
	//std::this_thread::sleep_for(std::chrono::milliseconds(10000));
	//robot.orientateAlongVector(0, 0, 1);
	//std::this_thread::sleep_for(std::chrono::milliseconds(10000));
	//robot.orientateAlongVector(0, 0, -1);
	//std::this_thread::sleep_for(std::chrono::milliseconds(10000));
	//robot.orientateAlongVector(1, 1, 1);
	//robot.rotateEndEffector(0, PI/2, 0);
	//std::this_thread::sleep_for(std::chrono::milliseconds(15000));
	//for (double y = 0; y < 0.2; y += 0.005){
	//	robot.moveAlongVector(0.005, 0, 0);
	//	std::this_thread::sleep_for(std::chrono::milliseconds(500));
	//}
	//robot.moveToHomePosition();
	
	cv::waitKey(0);
	system("Pause");
	return 0;
}