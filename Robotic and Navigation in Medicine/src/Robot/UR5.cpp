#include "UR5.h"

#include <boost/math/constants/constants.hpp>
#define PI boost::math::constants::pi<double>()

UR5::UR5() :tcp_client_(new TcpClient)
{
}

UR5::~UR5(){}

bool UR5::connectToRobot(char* ip, int port){
	tcp_client_->connect(ip, port);
	std::cout << tcp_client_->read() << std::endl;
	std::cout << tcp_client_->command("Hello Robot") << std::endl;
	

	//todo return if connection is established or not
	return true;
}

bool UR5::setJoints(std::array<float,6> &angles) {
	char jointString[100];

	sprintf(jointString, "MovePTPJoints %f %f %f %f %f %f ", angles.at(0), angles.at(1), angles.at(2), angles.at(3), angles.at(4), angles.at(5));
	std::cout << tcp_client_->command(jointString) << std::endl;
	//todo return if transmission was successfull or not
	return true;

}

    std::array<float,6>& UR5::getJoints(char* mode) {
	std::array<float, 6> ret;
	double test = 1;
	const char* jointString;
	jointString = tcp_client_->command("GetPositionJoints");
	std::cout <<"string: " << jointString << std::endl;
	sscanf(jointString, "%f %f %f %f %f %f", &ret[0], &ret[1], &ret[2], &ret[3], &ret[4], &ret[5]);
	
	if (mode == "deg") {
		//nothing to do because the values are already in degree	
	}
	else if(mode == "rad") {
		//convert to radians
		for (unsigned int i = 0; i < ret.size(); i++) {
			ret[i] = ret[i] * (PI / 180);
		}
	}
	else {
		std::cout << "False mode! Choose rad or deg" << std::endl;
	}
	
	return ret;
	
}


