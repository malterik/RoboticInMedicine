#include "UR5.h"


UR5::UR5() :tcp_client_(new TcpClient)
{
}

UR5::~UR5(){}

bool UR5::connectToRobot(char* ip, int port){
	tcp_client_->connect(ip, port);
	tcp_client_->command("Hello Robot");

	//todo return if connection is established or not
	return true;
}

bool UR5::setJoints(std::array<double,6> &angles) {
	char jointString[100];

	sprintf(jointString, "MovePTPJoints %f %f %f %f %f %f ", angles.at(0), angles.at(1), angles.at(2), angles.at(3), angles.at(4), angles.at(5));
	std::cout << tcp_client_->command(jointString) << std::endl;
	//todo return if transmission was successfull or not
	return true;

}

//std::array<double, 6> UR5::getJoints() {
//	std::array<double, 6> ret;
//
//	return ret;
//}


