#define D_SCL_SECURE_NO_WARNINGS 
#include <iostream>
#include "Robot\UR5.h"
#include "Ultra Sound\USHandler.h"
#include <opencv2\highgui\highgui.hpp>
#include "Kinematic\directKinematics.h"
#include "Kinematic\JointAngles.h"
#include "Kinematic\inverseKinematics.h"
#include "Tools\MathTools.hpp"

#include "boost\numeric\ublas\matrix.hpp"
#include "boost\numeric\ublas\io.hpp"
#include <stdlib.h>
#include <time.h>
#include <chrono>
#include <thread>
#include "Math\InvertMatrix.h"
#include <boost\math\constants\constants.hpp>
#include "Tools\CSVParser.hpp"
#include "Math\Window.h"

#define Z_OFFSET 0.3
#define PI boost::math::constants::pi<double>()
#define INPUT_FOLDER "..\\Input\\"

int main(int argc, char* argv[])
{
	// READ FILES
	CSVParser csvParser;
	boost::numeric::ublas::matrix<double> robot_to_cam_transformation = csvParser.readHTM(std::string(INPUT_FOLDER) + "rob2cam.csv");
	boost::numeric::ublas::matrix<double> robot_to_needle_transformation = csvParser.readHTM(std::string(INPUT_FOLDER) + "rob2needle.csv");
	boost::numeric::ublas::matrix<double> marker_position = csvParser.readHTM(std::string(INPUT_FOLDER) + "camHTM.csv");
	boost::numeric::ublas::matrix<double> pixel_to_probe = csvParser.readHTM(std::string(INPUT_FOLDER) + "ImageToProbe.csv");
	boost::numeric::ublas::matrix<double> probe_pose = csvParser.readHTM(std::string(INPUT_FOLDER) + "fileoutpos.txt");
	boost::numeric::ublas::vector<double> tumor_position = csvParser.readVector3D(std::string(INPUT_FOLDER) + "tumorCenter.csv");
	std::vector<boost::numeric::ublas::vector<double>> window_points = csvParser.readWindow(std::string(INPUT_FOLDER) + "windowPoints.csv");
	
	// INITIALIZE ROBOT
	UR5 robot;
	robot.connectToRobot(ROBOT_IP_LOCAL, ROBOT_PORT);
	robot.setSpeed(10);
	robot.setRobotToCamTransformation(robot_to_cam_transformation);
	robot.setRobotToNeedleTransformation(robot_to_needle_transformation);
	robot.setPixelToProbeTransformation(pixel_to_probe);

	// PREPARE DATA FOR NEEDLE PLACEMENT (i.e calculate target and window center in robot coordinates)
	// transform tumor coordinates from camera to robot world
	boost::numeric::ublas::vector<double> tumor_position_rob = robot.convertCamToRobPose(tumor_position);
	std::cout << "tumor_position_rob: " << tumor_position_rob << std::endl;

	// transform window coordinates from camera to robot world and calculate middle
	std::vector<boost::numeric::ublas::vector<double>> window_points_rob;
	for (int i = 0; i != window_points.size(); i++) {
		window_points_rob.push_back(robot.convertCamToRobPose(window_points[i]));
	}

	// calculate middle of window
	boost::numeric::ublas::vector<double> window_middle_rob(3);
	window_middle_rob <<= 0, 0, 0;
	for (int i = 0; i != window_points_rob.size(); i++) {
		window_middle_rob[0] += window_points_rob[i](0);
		window_middle_rob[1] += window_points_rob[i](1);
		window_middle_rob[2] += window_points_rob[i](2);
	}
	window_middle_rob[0] /= window_points_rob.size();
	window_middle_rob[1] /= window_points_rob.size();
	window_middle_rob[2] /= window_points_rob.size();
	std::cout << "window_middle_rob: " << window_middle_rob << std::endl;

	// NEEDLE PLACEMENT
	robot.needlePlacement(tumor_position_rob, window_middle_rob, true);

	system("Pause");
	return 0;
}

