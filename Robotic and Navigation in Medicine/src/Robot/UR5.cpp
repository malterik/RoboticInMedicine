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

bool UR5::setJoints(JointAngles angles) {
	char jointString[100];

	sprintf(jointString, "MovePTPJoints %f %f %f %f %f %f ", angles[0] * (180 / PI), angles[1] * (180 / PI), angles[2] * (180 / PI), angles[3] * (180 / PI), angles[4] * (180 / PI), angles[5] * (180 / PI));
	std::cout << tcp_client_->command(jointString) << std::endl;
	std::cout << jointString << std::endl;
	//todo return if transmission was successfull or not
	return true;

}

JointAngles& UR5::getJoints(char* mode) {
	std::array<double, 6> ret;
	double test = 1;
	const char* jointString;
	jointString = tcp_client_->command("GetPositionJoints");
	std::cout <<"string: " << jointString << std::endl;
	sscanf(jointString, "%lf %lf %lf %lf %lf %lf", &ret[0], &ret[1], &ret[2], &ret[3], &ret[4], &ret[5]);
	
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
	JointAngles temp(ret);
	return temp;
	
}

void UR5::moveToPosition(std::array<double, 3> pos){

	matrix<double> endPose(4,4);
	matrix<double> currentPose(4, 4);
	std::vector<JointAngles> endPoseJoints;
	currentPose = direct_kinematics_.computeDirectKinematics(getJoints("rad"));
	//Set the endpose. Keep orientation and set position
	endPose = currentPose;
	endPose(0, 3) = pos[0];
	endPose(1, 3) = pos[1];
	endPose(2, 3) = pos[2];
	endPoseJoints = inverse_kinematics_.computeInverseKinematics(endPose);
	if (endPoseJoints.size() > 0) {
		setJoints(endPoseJoints[0]); //Todo: Pathplanning
	}
	else{
		std::cout << "Error: No IK Solution Found" << std::endl;
	}



}

void UR5::moveToPosition(double x, double y, double z){

	matrix<double> endPose(4, 4);
	matrix<double> currentPose(4, 4);
	std::vector<JointAngles> endPoseJoints;
	currentPose = direct_kinematics_.computeDirectKinematics(getJoints("rad"));
	//Set the endpose. Keep orientation and set position
	endPose = currentPose;
	endPose(0, 3) = x;
	endPose(1, 3) = y;
	endPose(2, 3) = z;
	endPoseJoints = inverse_kinematics_.computeInverseKinematics(endPose);
	if (endPoseJoints.size() > 0) {
		setJoints(endPoseJoints[0]); //Todo: Pathplanning
	}
	else{
		std::cout << "Error: No IK Solution Found" << std::endl;
	}
	


}

void UR5::rotateEndEffector(double theta_x, double theta_y, double theta_z) {
	matrix<double> rotX(3, 3);
	matrix<double> rotY(3, 3);
	matrix<double> rotZ(3, 3);
	matrix<double> endOrientation(3, 3);
	matrix<double> endPose(4, 4);
	matrix<double> currentPose(4, 4);
	std::vector<JointAngles> endPoseJoints;

	currentPose = direct_kinematics_.computeDirectKinematics(getJoints("rad"));
	//fill rotX
	rotX(0, 0) = 1;
	rotX(0, 1) = 0;
	rotX(0, 2) = 0;

	rotX(1, 0) = 0;
	rotX(1, 1) = cos(theta_x);
	rotX(1, 2) = -sin(theta_x);
	
	rotX(2, 0) = 0;
	rotX(2, 1) = sin(theta_x);
	rotX(2, 2) = cos(theta_x);

	//fill rotY
	rotY(0, 0) = cos(theta_y);
	rotY(0, 1) = 0;
	rotY(0, 2) = sin(theta_y);

	rotY(1, 0) = 0;
	rotY(1, 1) = 1;
	rotY(1, 2) = 0;

	rotY(2, 0) = -sin(theta_y);
	rotY(2, 1) = 0;
	rotY(2, 2) = cos(theta_y);

	//fill rotZ
	rotZ(0, 0) = cos(theta_z);
	rotZ(0, 1) = -sin(theta_z);
	rotZ(0, 2) = 0;

	rotZ(1, 0) = sin(theta_z);
	rotZ(1, 1) = cos(theta_z);
	rotZ(1, 2) = 0;

	rotZ(2, 0) = 0;
	rotZ(2, 1) = 0;
	rotZ(2, 2) = 1;

	matrix<double> temp(3, 3);
	temp = prod(rotY, rotZ);
	endOrientation = prod(rotX, temp);

	
	//Keep the current position
	endPose = currentPose;
	//put the orientation matrix into the endPose
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			endPose(i, j) = endOrientation(i, j);
		}
	}

	endPoseJoints = inverse_kinematics_.computeInverseKinematics(endPose);

	if (endPoseJoints.size() > 0) {
		setJoints(endPoseJoints[0]);
	}

	
}

void UR5::moveAlongVector(double x, double y, double z) {
	matrix<double> currentPose(4, 4);
	matrix<double> endPose(4, 4);
	std::vector<JointAngles> endPoseJoints;

	currentPose = direct_kinematics_.computeDirectKinematics(getJoints("rad"));

	endPose = currentPose;
	endPose(0, 3) += x;
	endPose(1, 3) += y;
	endPose(2, 3) += z;

	endPoseJoints = inverse_kinematics_.computeInverseKinematics(endPose);

	if (endPoseJoints.size() > 0) {
		setJoints(endPoseJoints[0]);
	}
}

void UR5::moveToHomePosition(){
	JointAngles homePos;

	setJoints(homePos);
}




