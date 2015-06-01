#pragma once
#define _CRT_SECURE_NO_WARNINGS
#include <boost\smart_ptr\shared_ptr.hpp>
#include "../Network/TcpClient.h"
#include <array>

#define ROBOT_IP_LOCAL "192.168.56.101"
#define ROBOT_IP_LABOR "134.45.95.75"
#define ROBOT_PORT 5005

class UR5 {

public:
	UR5();
	~UR5();

	bool connectToRobot(char* ip, int port);
	bool setJoints(std::array<double,6> &angles);
	std::array<double,6>& getJoints();

	//todo
	std::array<double,6> getJoints();


private:
	boost::shared_ptr<TcpClient> tcp_client_;
	

};