#include "UR5.h"

#include <boost/math/constants/constants.hpp>

#define PI boost::math::constants::pi<double>()
#define AE_INT_T int

UR5::UR5() :tcp_client_(new TcpClient)
{
	robot_to_cam_transformation_ = boost::numeric::ublas::matrix<double>(4,4);
	endeffector_to_needletip_transformation_ = boost::numeric::ublas::matrix<double>(4, 4);
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
	temp = prod(rotX, rotY);
	endOrientation = prod(temp, rotZ);

	
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

/// <summary>
/// Waits the until robot has finished movement.
/// </summary>
/// <remarks>
/// The robot movement is finished when no commands are left in its queue. Therefore the method waits for an empty queue.
/// </remarks>
/// <param name="pollTime">The poll time.</param>
void UR5::waitUntilFinished(int pollTime){
	const char* respString;
	int queueLength = INT_MAX;

	// get queue length until queue is empty
	while (queueLength > 0)
	{
		respString = tcp_client_->command("GetQueueLength");
		sscanf(respString, "%d", &queueLength);
		std::this_thread::sleep_for(std::chrono::milliseconds(pollTime));
	}
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
	

	double yzLength = sqrt(pow(y, 2) + pow(z, 2));
	double xAngle = 0;
	if (yzLength != 0) {
		xAngle = acos(z / yzLength);
	}
	
	
	double vecLength = norm_2(vector);

	double yAngle = acos(yzLength / vecLength);
	std::cout << xAngle << "  " << yAngle << std::endl;
	rotateEndEffector((2*PI) - xAngle, yAngle, 0);
}

boost::numeric::ublas::matrix<double> UR5::convertCamToRobPose(boost::numeric::ublas::matrix<double> camPose)
{
	return convertCamToRobPose(camPose, true);
}

boost::numeric::ublas::matrix<double> UR5::convertCamToRobPose(boost::numeric::ublas::matrix<double> camPose, bool use_orthogonalization)
{	
	if (use_orthogonalization)
	{
		// orthonormalize pose	
		boost::numeric::ublas::matrix<double> transformationMatrix(4, 4);

		alglib::real_1d_array w;
		alglib::real_2d_array u;
		alglib::real_2d_array vt;

		alglib::real_2d_array a;
		a.setlength(3, 3);

		boost::numeric::ublas::matrix<double> pose = prod(robot_to_cam_transformation_, camPose);

		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < 3; j++)
			{
				a[i][j] = pose(i, j);
			}
		}


		alglib::rmatrixsvd(a, a.cols(), a.rows(), 2, 2, 2, w, u, vt);

		int uCols = u.cols();
		int uRows = u.rows();
		int vtCols = vt.cols();
		int vtRows = vt.rows();

		boost::numeric::ublas::matrix<double> uBoost(uCols, uRows);
		boost::numeric::ublas::matrix<double> vtBoost(vtCols, vtRows);

		for (int i = 0; i < u.cols(); i++)
		{
			for (int j = 0; j < u.rows(); j++)
			{
				uBoost(i, j) = u[i][j];
			}
		}

		for (int i = 0; i < vt.cols(); i++)
		{
			for (int j = 0; j < vt.rows(); j++)
			{
				vtBoost(i, j) = vt[i][j];
			}
		}

		boost::numeric::ublas::matrix<double> rot = prod(uBoost, vtBoost);

		transformationMatrix <<= rot(0, 0), rot(0, 1), rot(0, 2), pose(0, 3),
			rot(1, 0), rot(1, 1), rot(1, 2), pose(1, 3),
			rot(2, 0), rot(2, 1), rot(2, 2), pose(2, 3),
			0, 0, 0, 1;

		return transformationMatrix;
	}

	else
	{
		// take transformation matrix as is	

		return prod(robot_to_cam_transformation_, camPose);
	}	
	
}

void UR5::setRobotToCamTransformation(boost::numeric::ublas::matrix<double> robot_to_cam_transformation)
{
	robot_to_cam_transformation_ = robot_to_cam_transformation;
}