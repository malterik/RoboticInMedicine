
#include <iostream>
#include "Kinematic\KinematicMatrix.h"
#include "Robot\UR5.h"

int main(int argc, char* argv[])
{
	//set angles for the joints
	std::array<double, 6> angles;
	angles.assign(0);
	
	//this is the robot api
	UR5 robot;

	//connect to the robot
	robot.connectToRobot();
	//set the robot's joints
	robot.setJoints(angles);

	//A very small example of the kinematicMatrix class
	KinematicMatrix k;
	std::cout << k.toString() << std::endl << std::endl;

	system("Pause");

	return 0;
}