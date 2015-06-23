#pragma once
#define _CRT_SECURE_NO_WARNINGS
#include <boost\smart_ptr\shared_ptr.hpp>
#include <boost\numeric\ublas\vector.hpp>
#include "../Network/TcpClient.h"
#include <array>
#include "../Kinematic/JointAngles.h"
#include "../Kinematic/inverseKinematics.h"
#include "../Kinematic/directKinematics.h"
#define ROBOT_IP_LOCAL "192.168.56.101"
#define ROBOT_IP_LABOR "134.28.45.95"
#define ROBOT_PORT 5005

class UR5 {

public:
	UR5();
	~UR5();

	bool connectToRobot(char* ip, int port);
	bool setJoints(JointAngles angles);
	JointAngles& getJoints(char* mode);
	/**
	* This Moves the endeffector to the given Position (leaves the orientation untouched
	*
	*/
	void moveToPosition(std::array<double, 3> pos);
	void moveToPosition(double x, double y, double z);
	//Keep the position and rotate the effector by the given angles
	void rotateEndEffector(double theta_x, double theta_y, double theta_z);
	void moveAlongVector(double x, double y, double z);
	void moveToHomePosition();



private:
	boost::shared_ptr<TcpClient> tcp_client_;
	InverseKinematics inverse_kinematics_;
	DirectKinematics direct_kinematics_;
};