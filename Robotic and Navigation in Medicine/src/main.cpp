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

	//USHandler ush;


	//ush.processImage();
	srand(time(NULL));
	////set angles for the joints
	//std::array < double, 6 > temp = { rand() % 360, rand() % 360, rand() % 360, rand() % 360, rand() % 360, rand() % 360 };
	std::array<JointAngles, 8> angles;
	DirectKinematics dk;
	InverseKinematics ik;
	matrix<double> pos(4, 4);

	UR5 robot;

	////connect to the robot
	robot.connectToRobot(ROBOT_IP_LOCAL, ROBOT_PORT);
	//set the robot's joints
	
	//std::this_thread::sleep_for(std::chrono::milliseconds(10000));
	//angles = ik.computeInverseKinematics(pos);
	//robot.setJoints(angles[0]);
	//robot.getJoints("rad");
	//std::array<float, 6> a = robot.getJoints("rad");
	//robot.moveToHomePosition();
	//std::this_thread::sleep_for(std::chrono::milliseconds(15000));
	robot.moveToPosition(0.5, 0.0,0.3);
	std::this_thread::sleep_for(std::chrono::milliseconds(10000));
	robot.rotateEndEffector(0, PI/2, 0);
	std::this_thread::sleep_for(std::chrono::milliseconds(15000));
	for (double y = 0; y < 0.2; y += 0.005){
		robot.moveAlongVector(0.005, 0, 0);
		std::this_thread::sleep_for(std::chrono::milliseconds(500));
	}
	robot.moveToHomePosition();
	
	cv::waitKey(0);
	system("Pause");
	return 0;
}