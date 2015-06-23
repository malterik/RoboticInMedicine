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

int main(int argc, char* argv[])
{

	//USHandler ush;


	//ush.processImage();
	srand(time(NULL));
	////set angles for the joints
	//std::array < double, 6 > temp = { rand() % 360, rand() % 360, rand() % 360, rand() % 360, rand() % 360, rand() % 360 };
	std::array<JointAngles, 8> angles;
	DirectKinematics dk;
	InverseKinematics ik;
	matrix<double> pos(4, 4);

	pos(0, 0) = 1;
	pos(0, 1) = 0;
	pos(0, 2) = 0;
	pos(0, 3) = 0.2;

	pos(1, 0) = 0;
	pos(1, 1) = 1;
	pos(1, 2) = 0;
	pos(1, 3) = 0.28;

	pos(2, 0) = 0;
	pos(2, 1) = 0;
	pos(2, 2) = 1;
	pos(2, 3) = 0;

	pos(3, 0) = 0;
	pos(3, 1) = 0;
	pos(3, 2) = 0;
	pos(3, 3) = 1;
	UR5 robot;

	////connect to the robot
	robot.connectToRobot(ROBOT_IP_LOCAL, ROBOT_PORT);
	//set the robot's joints
	
	//std::this_thread::sleep_for(std::chrono::milliseconds(10000));
	angles = ik.computeInverseKinematics(pos);
	robot.setJoints(angles[0]);
	robot.getJoints("rad");
	//std::array<float, 6> a = robot.getJoints("rad");
	
	cv::waitKey(0);
	system("Pause");
	return 0;
}