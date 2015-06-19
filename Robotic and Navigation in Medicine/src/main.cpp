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
int main(int argc, char* argv[])
{

	//USHandler ush;


	//ush.processImage();
	srand(time(NULL));
	////set angles for the joints
	std::array < double, 6 > temp = { rand() % 360, rand() % 360, rand() % 360, rand() % 360, rand() % 360, rand() % 360 };
	JointAngles angles(temp);
	DirectKinematics dk;
	InverseKinematics ik;
	

	
	UR5 robot;

	////connect to the robot
	robot.connectToRobot(ROBOT_IP_LOCAL, ROBOT_PORT);
	//set the robot's joints
	//robot.setJoints(angles);

	ik.computeInverseKinematics(dk.computeDirectKinematics(robot.getJoints("rad")));
	//std::array<float, 6> a = robot.getJoints("rad");
	
	cv::waitKey(0);
	system("Pause");
	return 0;
}