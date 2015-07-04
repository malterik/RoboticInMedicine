#pragma once
#define _CRT_SECURE_NO_WARNINGS
#include <boost\smart_ptr\shared_ptr.hpp>
#include <boost\numeric\ublas\vector.hpp>
#include "../Network/TcpClient.h"
#include <array>
#include "../Kinematic/JointAngles.h"
#include "../Kinematic/inverseKinematics.h"
#include "../Kinematic/directKinematics.h"
#include "../Kinematic/PathPlanner.h"
#include "../ALGLIB/linalg.h"
#include "../Tools/MathTools.hpp"
#include "../Tools/CSVParser.hpp"
#include "boost/numeric/ublas/matrix.hpp"
#include "boost/numeric/ublas/assignment.hpp"
#include <boost/geometry/geometry.hpp> 
#include <boost/geometry/arithmetic/dot_product.hpp> 
#include <chrono>
#include <thread>


#define ROBOT_IP_LOCAL "192.168.56.101"
#define ROBOT_IP_LABOR "134.28.45.95"
#define ROBOT_PORT 5005

#define Z_OFFSET_TO_TARGET 0.2
class UR5 {

public:
	UR5();
	~UR5();

	boost::numeric::ublas::matrix<double> getRobotToCamTransformation();
	boost::numeric::ublas::matrix<double> getRobotToNeedleTransformation();
	boost::numeric::ublas::matrix<double> getPixelToProbeTransformation();

	void setRobotToCamTransformation(boost::numeric::ublas::matrix<double> robot_to_cam_transformation);
	void setRobotToNeedleTransformation(boost::numeric::ublas::matrix<double> robot_to_needle_transformation_);
	void setPixelToProbeTransformation(boost::numeric::ublas::matrix<double> pixel_to_probe_transformation);

	bool connectToRobot(char* ip, int port);
	bool setJoints(JointAngles angles);
	JointAngles& getJoints(char* mode);
	/**
	* This Moves the endeffector to the given Position (leaves the orientation untouched
	*
	*/
	void moveToPosition(double x, double y, double z);
	void moveToPosition(vector<double> vec);
	void moveToPose(double x, double y, double z, double theta_x, double theta_y, double theta_z);
	void moveToPose(matrix<double> endPose);
	matrix<double> UR5::moveAndWait(void(UR5::* moveFunction)(vector<double>), vector<double> vec);
	matrix<double> UR5::moveAndWait(void(UR5::* moveFunction)(matrix<double>), matrix<double> mat);

	//Keep the position and rotate the effector by the given angles
	matrix<double> rotateEndEffector(double theta_x, double theta_y, double theta_z);

	matrix<double> moveAlongVector(double x, double y, double z);
	matrix<double> moveAlongVector(vector<double> vec);

	matrix<double> orientateAlongVector(double x, double y, double z);
	matrix<double> orientateAlongVector(vector<double> vec);

	void doNeedlePlacement(vector<double> target, vector<double> window, matrix<double> needleTip);
	void needlePlacement(vector<double> target, vector<double> window_center, bool log_movement);

	void setSpeed(double speedValue);
	void moveToHomePosition();
	void waitUntilFinished(int pollTime);


	boost::numeric::ublas::matrix<double> convertCamToRobPose(boost::numeric::ublas::matrix<double> camPose);
	boost::numeric::ublas::matrix<double> convertCamToRobPose(boost::numeric::ublas::matrix<double> camPose, bool use_orthogonalization);
	boost::numeric::ublas::vector<double> convertCamToRobPose(boost::numeric::ublas::vector<double> camPosition);

	vector<double> convertPixelToProbe(int x, int y);



private:
	boost::shared_ptr<TcpClient> tcp_client_;
	InverseKinematics inverse_kinematics_;
	DirectKinematics direct_kinematics_;
	PathPlanner path_planner_;
	boost::numeric::ublas::matrix<double> robot_to_cam_transformation_;
	boost::numeric::ublas::matrix<double> robot_to_needle_transformation_;
	boost::numeric::ublas::matrix<double> pixel_to_probe_transformation_;
};