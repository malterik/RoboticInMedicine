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



void UR5::moveToPosition(double x, double y, double z){

	matrix<double> endPose(4, 4);
	matrix<double> currentPose(4, 4);
	std::vector<JointAngles> endPoseJoints;
	JointAngles currentAngles;



	currentAngles = getJoints("rad");
	currentPose = direct_kinematics_.computeDirectKinematics(currentAngles);
	//Set the endpose. Keep orientation and set position
	endPose = currentPose;
	endPose(0, 3) = x;
	endPose(1, 3) = y;
	endPose(2, 3) = z;
	endPoseJoints = inverse_kinematics_.computeInverseKinematics(endPose);
	if (endPoseJoints.size() > 0) {
		setJoints(path_planner_.chooseNearest(currentAngles,path_planner_.checkForValidConfigurations(endPoseJoints))); 
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
	JointAngles currentAngles;

	currentAngles = getJoints("rad");
	currentPose = direct_kinematics_.computeDirectKinematics(currentAngles);
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
		setJoints(path_planner_.chooseNearest(currentAngles, path_planner_.checkForValidConfigurations(endPoseJoints)));
	}
	else{
		std::cout << "Error: No IK Solution Found" << std::endl;
	}

	
}

void UR5::moveAlongVector(double x, double y, double z) {
	matrix<double> currentPose(4, 4);
	matrix<double> endPose(4, 4);
	std::vector<JointAngles> endPoseJoints;
	JointAngles currentAngles;
	currentAngles = getJoints("rad");
	currentPose = direct_kinematics_.computeDirectKinematics(currentAngles);

	endPose = currentPose;
	endPose(0, 3) += x;
	endPose(1, 3) += y;
	endPose(2, 3) += z;

	endPoseJoints = inverse_kinematics_.computeInverseKinematics(endPose);

	if (endPoseJoints.size() > 0) {
		setJoints(path_planner_.chooseNearest(currentAngles, path_planner_.checkForValidConfigurations(endPoseJoints)));
	}
	else{
		std::cout << "Error: No IK Solution Found" << std::endl;
	}
}

void UR5::moveToHomePosition(){
	JointAngles homePos;

	setJoints(homePos);
}

void UR5::waitUntilFinished(){
	const char *respString;
	tcp_client_->write("GetQueueLength");
}

void UR5::setSpeed(double speedValue) {
	char jointString[100];

	sprintf(jointString, "SetSpeed %lf ",speedValue);
	std::cout << tcp_client_->command(jointString) << std::endl;
}

void UR5::moveToPose(double x, double y, double z, double theta_x, double theta_y, double theta_z) {
	matrix<double> rotX(3, 3);
	matrix<double> rotY(3, 3);
	matrix<double> rotZ(3, 3);

	matrix<double> endOrientation(3, 3);
	matrix<double> endPose(4, 4);
	matrix<double> currentPose(4, 4);
	std::vector<JointAngles> endPoseJoints;
	JointAngles currentAngles;

	currentAngles = getJoints("rad");
	currentPose = direct_kinematics_.computeDirectKinematics(currentAngles);
	endPose = currentPose;

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

	
	//put the orientation matrix into the endPose
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			endPose(i, j) = endOrientation(i, j);
		}
	}

	endPose(0, 3) = x;
	endPose(1, 3) = y;
	endPose(2, 3) = z;

	endPoseJoints = inverse_kinematics_.computeInverseKinematics(endPose);
	if (endPoseJoints.size() > 0) {
		setJoints(path_planner_.chooseNearest(currentAngles, path_planner_.checkForValidConfigurations(endPoseJoints)));
	}
	else{
		std::cout << "Error: No IK Solution Found" << std::endl;
	}
}

void UR5::moveToPose(matrix<double> endPose) {
	std::vector<JointAngles> endPoseJoints;
	JointAngles currentAngles;

	currentAngles = getJoints("rad");
	endPoseJoints = inverse_kinematics_.computeInverseKinematics(endPose);
	if (endPoseJoints.size() > 0) {
		setJoints(path_planner_.chooseNearest(currentAngles, path_planner_.checkForValidConfigurations(endPoseJoints)));
	}
	else{
		std::cout << "Error: No IK Solution Found" << std::endl;
	}
}

void UR5::orientateAlongVector(double x, double y, double z){
	double theta_x, theta_y, theta_z;

	boost::numeric::ublas::vector<double> vector(3);

	
	//Einheitsvektoren
	boost::numeric::ublas::vector<double> e_x(3);
	boost::numeric::ublas::vector<double> e_y(3);
	boost::numeric::ublas::vector<double> e_z(3);

	vector.insert_element(0, x);
	vector.insert_element(1, y);
	vector.insert_element(2, z);
	
	for (int i = 0; i < 3; i++) {
		e_x.insert_element(i, 0);
		e_y.insert_element(i, 0);
		e_z.insert_element(i, 0);
	}

	e_x[0] = 1;
	e_y[1] = 1;
	e_z[2] = 1;

	theta_x = acos(inner_prod(e_x, vector) / norm_2(e_x) * norm_2(vector));
	theta_y = acos(inner_prod(e_y, vector) / norm_2(e_y) * norm_2(vector));
	theta_z = acos(inner_prod(e_z, vector) / norm_2(e_z) * norm_2(vector));

	rotateEndEffector(theta_x, theta_y, theta_z);
}