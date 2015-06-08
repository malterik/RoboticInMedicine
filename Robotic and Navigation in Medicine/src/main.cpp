#include <iostream>
#include "Kinematic\KinematicMatrix.h"
#include "Robot\UR5.h"
#include "Ultra Sound\USHandler.h"
#include "Kinematic\directKinematics.h"

#include <opencv2\highgui\highgui.hpp>
int main(int argc, char* argv[])
{

	USHandler ush;

	ush.processImage();

	//////set angles for the joints
	////std::array<double, 6> angles;
	////angles = { -90, 90, -120, -46, 90, 0 };
	//////this is the robot api
	////UR5 robot;

	//////connect to the robot
	////robot.connectToRobot(ROBOT_IP_LOCAL, ROBOT_PORT);
	//////set the robot's joints
	////robot.setJoints(angles);

	//////A very small example of the kinematicMatrix class
	////KinematicMatrix k;
	////std::cout << k.toString() << std::endl << std::endl;

	/*DirectKinematics direct;

	vector<double> q = vector<double>(6);
	q(0) = 1;
	q(1) = 2;
	q(2) = 3;
	q(3) = 4;
	q(4) = 5;
	q(5) = 6;

	matrix<double> A = direct.computeDirectKinematics(q);

	for (int i = 0; i <= 3; i++)
	{
		for (int j = 0; j <= 3; j++)
		{
			std::cout << A(i, j) << "\t";


		}
		std::cout << std::endl;
	}*/


	cv::waitKey(0);
	system("Pause");
	return 0;
}