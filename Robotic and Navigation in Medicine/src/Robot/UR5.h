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

	matrix<double> getRobotToCamTransformation();
	matrix<double> getRobotToNeedleTransformation();
	matrix<double> getPixelToProbeTransformation();
	matrix<double> getEndEffectorPose();
	matrix<double> getNeedlePose();
	JointAngles& getJoints(char* mode);

	void setRobotToCamTransformation(matrix<double> robot_to_cam_transformation);
	void setRobotToNeedleTransformation(matrix<double> robot_to_needle_transformation_);
	void setPixelToProbeTransformation(matrix<double> pixel_to_probe_transformation);
	
	bool setJoints(JointAngles angles);
	bool setSpeed(double speedValue);
	bool enableLinearMovement();
	bool disableLinearMovement();
	bool connectToRobot(char* ip, int port);

	bool moveToPosition(vector<double> vec);
	bool moveToPosition(double x, double y, double z);	
	bool moveToPose(double x, double y, double z, double theta_x, double theta_y, double theta_z);
	bool moveToPose(matrix<double> endPose);
	bool moveLinear(matrix<double> pose);
	bool moveAlongVector(vector<double> vec);
	bool moveAlongVector(double x, double y, double z);
	bool moveToHomePosition();
	bool UR5::moveAndWait(bool(UR5::* moveFunction)(vector<double>), vector<double> vec, matrix<double> &outMatrix);
	bool UR5::moveAndWait(bool(UR5::* moveFunction)(matrix<double>), matrix<double> mat, matrix<double> &outMatrix);

	//Keep the position and rotate the effector by the given angles
	bool rotateEndEffector(double theta_x, double theta_y, double theta_z);
	matrix<double> orientateAlongVector(double x, double y, double z);
	matrix<double> orientateAlongVector(vector<double> vec);




	void doNeedlePlacement(vector<double> target, vector<double> window, matrix<double> needleTip);
	bool needlePlacement(vector<double> target, vector<double> window_center, bool log_movement);

	
	void waitUntilFinished(int pollTime);

	bool checkCommandSuccess(const char* server_answer);


	matrix<double> convertCamToRobPose(matrix<double> camPose);
	matrix<double> convertCamToRobPose(matrix<double> camPose, bool use_orthogonalization);
	vector<double> convertCamToRobPose(vector<double> camPosition);
	matrix<double> UR5::convertNeedleToRobPose(matrix<double> needlePose);
	vector<double> convertPixelToProbe(int x, int y);



private:
	boost::shared_ptr<TcpClient> tcp_client_;
	InverseKinematics inverse_kinematics_;
	DirectKinematics direct_kinematics_;
	PathPlanner path_planner_;
	matrix<double> robot_to_cam_transformation_;
	matrix<double> robot_to_needle_transformation_;
	matrix<double> pixel_to_probe_transformation_;
};