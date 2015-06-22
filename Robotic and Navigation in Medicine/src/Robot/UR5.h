#pragma once
#define _CRT_SECURE_NO_WARNINGS
#include <boost\smart_ptr\shared_ptr.hpp>
#include <boost\numeric\ublas\vector.hpp>
#include "../Network/TcpClient.h"
#include <array>
#include "../Kinematic/JointAngles.h"

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



private:
	boost::shared_ptr<TcpClient> tcp_client_;
};