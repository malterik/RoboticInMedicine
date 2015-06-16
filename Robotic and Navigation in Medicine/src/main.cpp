
#include <iostream>
#include "Robot\UR5.h"
#include "Ultra Sound\USHandler.h"
#include <opencv2\highgui\highgui.hpp>
#include "Kinematic\directKinematics.h"
#include "Kinematic\JointAngles.h"
#include "Kinematic\inverseKinematics.h"

int main(int argc, char* argv[])
{

	//USHandler ush;


	//ush.processImage();

	////set angles for the joints
	JointAngles angles;
	DirectKinematics dk;
	InverseKinematics ik;

	ik.computeInverseKinematics(dk.computeDirectKinematics(angles));
	//UR5 robot;

	////connect to the robot
	//robot.connectToRobot(ROBOT_IP_LABOR, ROBOT_PORT);
	//set the robot's joints
	//robot.setJoints(angles);

	//std::array<float, 6> a = robot.getJoints("rad");
	
	cv::waitKey(0);
	system("Pause");
	return 0;
}