
#include <iostream>
#include "Robot\UR5.h"
#include "Ultra Sound\USHandler.h"
#include <opencv2\highgui\highgui.hpp>
#include "Kinematic\directKinematics.h"
int main(int argc, char* argv[])
{

	USHandler ush;


	//ush.processImage();

	////set angles for the joints
	std::array<double, 6> angles;
	angles = { -90, 90, -120, -46, 90, 0 };
	////this is the robot api
	UR5 robot;

	////connect to the robot
	robot.connectToRobot(ROBOT_IP_LOCAL, ROBOT_PORT);
	//set the robot's joints
	robot.setJoints(angles);

	std::array<double, 6> a = robot.getJoints("rad");


	for (unsigned int i = 0; i < a.size(); i++)
		std::cout << a[i] << std::endl;
	DirectKinematics dk;


	boost::numeric::ublas::matrix<double> m = dk.computeDirectKinematics(a);

	std::cout << m;

	cv::waitKey(0);
	system("Pause");
	return 0;
}