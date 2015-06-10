#include <iostream>
#include "Kinematic\KinematicMatrix.h"
#include "Robot\UR5.h"
#include "Ultra Sound\USHandler.h"
#include  <boost\numeric\ublas\vector.hpp>

using namespace boost::numeric::ublas;
#include <opencv2\highgui\highgui.hpp>
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

	std::array<double, 6> a = robot.getJoints("deg");


	for (unsigned int i = 0; i < a.size(); i++)
		std::cout << a[i] << std::endl;

	////A very small example of the kinematicMatrix class
	//KinematicMatrix k;
	//std::cout << k.toString() << std::endl << std::endl;
	cv::waitKey(0);
	system("Pause");
	return 0;
}