#include "UR5.h"

#include <boost/math/constants/constants.hpp>
#include "../Math/InvertMatrix.h"
#define PI boost::math::constants::pi<double>()
#define AE_INT_T int
#define SIMULATION_OUTPUT_FOLDER "..\\Output\\"

UR5::UR5() :tcp_client_(new TcpClient)
{
	robot_to_cam_transformation_ = matrix<double>(4,4);
	robot_to_needle_transformation_ = matrix<double>(4, 4);
}

UR5::~UR5(){}

matrix<double> UR5::getRobotToCamTransformation()
{
	return robot_to_cam_transformation_;
}

matrix<double> UR5::getRobotToNeedleTransformation()
{
	return robot_to_needle_transformation_;
}

matrix<double> UR5::getPixelToProbeTransformation() {
	return pixel_to_probe_transformation_;
}

JointAngles& UR5::getJoints(char* mode) {
	std::array<double, 6> ret;
	double test = 1;
	const char* jointString;
	jointString = tcp_client_->command("GetPositionJoints");
	std::cout << "string: " << jointString << std::endl;
	sscanf(jointString, "%lf %lf %lf %lf %lf %lf", &ret[0], &ret[1], &ret[2], &ret[3], &ret[4], &ret[5]);

	if (mode == "deg") {
		//nothing to do because the values are already in degree	
	}
	else if (mode == "rad") {
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


void UR5::setRobotToCamTransformation(matrix<double> robot_to_cam_transformation)
{
	robot_to_cam_transformation_ = robot_to_cam_transformation;
}

void UR5::setRobotToNeedleTransformation(matrix<double> robot_to_needle_transformation)
{
	robot_to_needle_transformation_ = robot_to_needle_transformation;
}

void UR5::setPixelToProbeTransformation(matrix<double> pixel_to_probe_transformation){
	pixel_to_probe_transformation_ = pixel_to_probe_transformation;
}

bool UR5::setJoints(JointAngles angles) {
	char commandString[100];

	sprintf(commandString, "MovePTPJoints %f %f %f %f %f %f ", angles[0] * (180 / PI), angles[1] * (180 / PI), angles[2] * (180 / PI), angles[3] * (180 / PI), angles[4] * (180 / PI), angles[5] * (180 / PI));
	std::cout << commandString << std::endl;

	return checkCommandSuccess(tcp_client_->command(commandString));
}

bool UR5::connectToRobot(char* ip, int port){
	tcp_client_->connect(ip, port);
	std::cout << tcp_client_->read() << std::endl;
	std::cout << tcp_client_->command("Hello Robot") << std::endl;

	//todo return if connection is established or not
	return true;
}



bool UR5::moveToPosition(vector<double> vec){
	return moveToPosition(vec[0], vec[1], vec[2]);
}

bool UR5::moveToPosition(double x, double y, double z){

	matrix<double> endPose(4, 4);
	matrix<double> currentPose(4, 4);
	IKResult endPoseJoints;
	JointAngles currentAngles;

	currentAngles = getJoints("rad");
	currentPose = direct_kinematics_.computeDirectKinematics(currentAngles);
	//Set the endpose. Keep orientation and set position
	endPose = currentPose;
	endPose(0, 3) = x;
	endPose(1, 3) = y;
	endPose(2, 3) = z;
	endPoseJoints = inverse_kinematics_.computeInverseKinematics(endPose);

	if (endPoseJoints.solutions.size() > 0) {
		IKResult* result = path_planner_.chooseNearest(currentAngles, path_planner_.checkForValidConfigurations(endPoseJoints));
		if (result)
		{
			return setJoints(result->nearestSolution);
		}
	}

	return false;
}

bool UR5::moveToPose(double x, double y, double z, double theta_x, double theta_y, double theta_z) {
	matrix<double> rotX(3, 3);
	matrix<double> rotY(3, 3);
	matrix<double> rotZ(3, 3);

	matrix<double> endOrientation(3, 3);
	matrix<double> endPose(4, 4);
	matrix<double> currentPose(4, 4);
	IKResult endPoseJoints;
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
	if (endPoseJoints.solutions.size() > 0) {
		IKResult* result = path_planner_.chooseNearest(currentAngles, path_planner_.checkForValidConfigurations(endPoseJoints));
		if (result)
		{
			return setJoints(result->nearestSolution);
		}
	}

	return false;
}

bool UR5::moveToPose(matrix<double> endPose) {
	IKResult endPoseJoints;
	JointAngles currentAngles;

	currentAngles = getJoints("rad");
	endPoseJoints = inverse_kinematics_.computeInverseKinematics(endPose);
	if (endPoseJoints.solutions.size() > 0) {
		IKResult* result = path_planner_.chooseNearest(currentAngles, path_planner_.checkForValidConfigurations(endPoseJoints));
		if (result)
		{
			return setJoints(result->nearestSolution);
		}
	}

	return false;
}

bool UR5::moveAlongVector(vector<double> vec){
	return moveAlongVector(vec[0], vec[1], vec[2]);
}

bool UR5::moveAlongVector(double x, double y, double z) {
	matrix<double> currentPose(4, 4);
	matrix<double> endPose(4, 4);
	IKResult endPoseJoints;
	JointAngles currentAngles;
	currentAngles = getJoints("rad");
	currentPose = direct_kinematics_.computeDirectKinematics(currentAngles);

	endPose = currentPose;
	endPose(0, 3) += x;
	endPose(1, 3) += y;
	endPose(2, 3) += z;

	endPoseJoints = inverse_kinematics_.computeInverseKinematics(endPose);

	if (endPoseJoints.solutions.size() > 0) {
		IKResult* result = path_planner_.chooseNearest(currentAngles, path_planner_.checkForValidConfigurations(endPoseJoints));
		if (result)
		{
			return setJoints(result->nearestSolution);
		}
	}

	return false;
}

bool UR5::moveLinear(matrix<double> pose)
{
	InverseKinematics inverseKinematics;
	IKResult ikResult = inverseKinematics.computeInverseKinematics(pose);
	JointAngles angles;

	if (ikResult.solutions.size() > 0) {
		IKResult* result = path_planner_.chooseNearest(getJoints("rad"), path_planner_.checkForValidConfigurations(ikResult));
		if (result)
		{
			angles = result->nearestSolution;
			char commandString[100];
			sprintf(commandString, "MoveLINJoints %f %f %f %f %f %f ", angles[0] * (180 / PI), angles[1] * (180 / PI), angles[2] * (180 / PI), angles[3] * (180 / PI), angles[4] * (180 / PI), angles[5] * (180 / PI));
			std::cout << commandString << std::endl;

			enableLinearMovement();
			bool success = checkCommandSuccess(tcp_client_->command(commandString));
			disableLinearMovement();

			return success;
		}
	}

	return false;
}

bool UR5::moveLinear(JointAngles angles)
{
	char commandString[100];
	sprintf(commandString, "MoveLINJoints %f %f %f %f %f %f ", angles[0] * (180 / PI), angles[1] * (180 / PI), angles[2] * (180 / PI), angles[3] * (180 / PI), angles[4] * (180 / PI), angles[5] * (180 / PI));
	std::cout << commandString << std::endl;

	enableLinearMovement();
	bool success = checkCommandSuccess(tcp_client_->command(commandString));
	disableLinearMovement();

	return success;
}

bool UR5::moveToHomePosition(){
	JointAngles homePos;

	return setJoints(homePos);
}

bool UR5::moveAndWait(bool(UR5::* moveFunction)(vector<double>), vector<double> vec, matrix<double> &outMatrix)
{
	if ((this->*moveFunction)(vec))
	{
		waitUntilFinished(500);
		JointAngles jointAngles = getJoints("rad");
		DirectKinematics directKinematics;
		outMatrix = directKinematics.computeDirectKinematics(jointAngles);
		return true;
	}

	return false;
}

bool UR5::moveAndWait(bool(UR5::* moveFunction)(matrix<double>), matrix<double> mat, matrix<double> &outMatrix)
{
	if ((this->*moveFunction)(mat))
	{
		waitUntilFinished(500);
		JointAngles jointAngles = getJoints("rad");
		DirectKinematics directKinematics;
		outMatrix = directKinematics.computeDirectKinematics(jointAngles);
		std::cout << outMatrix << std::endl;
		return true;
	}

	return false;
}

bool UR5::moveAndWait(bool(UR5::* moveFunction)(JointAngles), JointAngles jointAngles, matrix<double> &outMatrix)
{
	if ((this->*moveFunction)(jointAngles))
	{
		waitUntilFinished(500);
		JointAngles jointAngles = getJoints("rad");
		DirectKinematics directKinematics;
		outMatrix = directKinematics.computeDirectKinematics(jointAngles);
		return true;
	}

	return false;
}

bool UR5::rotateEndEffector(double theta_x, double theta_y, double theta_z) {
	matrix<double> rotX(3, 3);
	matrix<double> rotY(3, 3);
	matrix<double> rotZ(3, 3);
	matrix<double> endOrientation(3, 3);
	matrix<double> endPose(4, 4);
	matrix<double> currentPose(4, 4);
	IKResult endPoseJoints;
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

	if (endPoseJoints.solutions.size() > 0) {
		IKResult* result = path_planner_.chooseNearest(currentAngles, path_planner_.checkForValidConfigurations(endPoseJoints));
		if (result)
		{
			return setJoints(result->nearestSolution);
		}
	}
	
	return false;
}

bool UR5::interpolateLine(JointAngles startAngles, vector<double> endPosition, double max_change_in_rotation, double step_size, std::vector<JointAngles> &lineJointAngles)
{
	InverseKinematics inverseKinematics;
	DirectKinematics directKinematics;
	bool success = true;

	matrix<double> rob_pose = directKinematics.computeDirectKinematics(startAngles);
	matrix<double> needle_pose = prod(rob_pose, robot_to_needle_transformation_);

	matrix<double> endeEffectorRotation = MathTools::getRotation(rob_pose);
	vector<double> endeEffectorTranslation = MathTools::getTranslation(rob_pose);
	matrix<double> needleRotation = MathTools::getRotation(needle_pose);
	vector<double> direction = endPosition - MathTools::getTranslation(needle_pose);
	vector<double> direction_normed = direction / norm_2(direction);

	// adjust step_size to end exactly in target
	int steps = (int)(ceil(norm_2(direction) / step_size));
	step_size = norm_2(direction) / ((double)steps);

	lineJointAngles = std::vector<JointAngles>();
	JointAngles lastAngles = startAngles;

	for (int i = 1; i < (steps + 1); i++)
	{
		vector<double> position_rob = endeEffectorTranslation + direction_normed*i*step_size;
		matrix<double> pose_rob = MathTools::composeMatrix(endeEffectorRotation, position_rob);

		IKResult ikResult = inverseKinematics.computeInverseKinematics(pose_rob);

		if (ikResult.solutions.size() > 0) {
			IKResult* result = path_planner_.chooseNearest(lastAngles, path_planner_.checkForValidConfigurations(ikResult));
			if (result)
			{
				JointAngles jA = result->nearestSolution;

				double angleChange = MathTools::getChangeInRotation(jA, lastAngles);
				if (angleChange > max_change_in_rotation)
				{
					// max angle constraint violated
					return false;
				}

				lineJointAngles.push_back(jA);
				lastAngles = jA;
			}
			else
			{
				return false;
			}
		}
	}

	return true;
}





/// <summary>
/// Waits the until robot has finished movement.
/// </summary>
/// <remarks>
/// The robot movement is finished when no commands are left in its queue. Therefore the method waits for an empty queue.
/// </remarks>
/// <param name="pollTime">The poll time.</param>
void UR5::waitUntilFinished(int pollTime){
	std::string respString;
	std::string respString_old;
	int queueLength = INT_MAX;
	bool finished = false;

	// get queue length until queue is empty
	while (!finished)
	{
		respString = std::string(tcp_client_->command("GetPositionJoints"));
		if (respString.compare(respString_old) != 0) {
			respString_old = respString;
			std::this_thread::sleep_for(std::chrono::milliseconds(pollTime));
			continue;			
		}
		else {
			finished = true;
			std::cout << "Finished" << std::endl;
		}
	}
}

bool UR5::checkCommandSuccess(const char* server_answer)
{	
	if (boost::starts_with(server_answer, "true"))
	{
		return true;
	}
	
	return false;
}

bool UR5::setSpeed(double speedValue) {
	char commandString[100];
	sprintf(commandString, "SetSpeed %lf ", speedValue);

	return checkCommandSuccess(tcp_client_->command(commandString));
}

bool UR5::enableLinearMovement()
{
	return checkCommandSuccess(tcp_client_->command("EnableLin"));
}

bool UR5::disableLinearMovement() 
{
	return checkCommandSuccess(tcp_client_->command("DisableLin"));
}





/*matrix<double> UR5::orientateAlongVector(double x, double y, double z){
	double theta_x, theta_y, theta_z;

	vector<double> vector(3);


	//Einheitsvektoren
	vector<double> e_x(3);
	vector<double> e_y(3);
	vector<double> e_z(3);

	vector.insert_element(0, x);
	vector.insert_element(1, y);
	vector.insert_element(2, z);


	double yzLength = sqrt(pow(y, 2) + pow(z, 2));
	double xAngle = 0;
	if (yzLength != 0) {
	xAngle = acos(z / yzLength);
	}
	if (y < 0) {
	xAngle -= PI;
	}

	double yAngle2 = 0;
	double vecLength = norm_2(vector);
	if (vecLength != 0) {
	yAngle2 = acos(yzLength / vecLength);
	}
	if (x < 0) {
	yAngle2 -= PI;
	}

	return rotateEndEffector((2*PI) - xAngle, yAngle2, 0);
}*/

matrix<double> UR5::orientateAlongVector(double x, double y, double z){
	// create z unit vector
	vector<double> e_z(3);
	e_z <<= 0, 0, 1;

	/*matrix<double> e_z_pose(identity_matrix<double>(4));
	e_z_pose(2, 3) = 1;
	e_z_pose = prod(e_z_pose,robot_to_needle_transformation_);
	e_z(0) = e_z_pose(0, 3);
	e_z(1) = e_z_pose(1, 3);
	e_z(2) = e_z_pose(2, 3);*/

	// create vector containing direction
	vector<double> direction(3);
	direction <<= x, y, z;
	direction /= norm_2(direction);

	// calculate axis angle representation of vector between e_z and direction
	double angle = acos(inner_prod(e_z, direction));
	vector<double> axis = MathTools::crossProduct(e_z, direction);

	// convert to rotation matrix
	matrix<double> rotation = MathTools::convertAxisAngleToRotationMatrix(axis, angle);

	return rotation;
}

vector<double> UR5::convertCamToRobPose(vector<double> camPosition)
{
	// create 4x4 matrix with given cam position and dummy rotation matrix
	boost::numeric::ublas::identity_matrix<double> eye(4);
	matrix<double> camPose(eye);
	camPose(0, 3) = camPosition(0);
	camPose(1, 3) = camPosition(1);
	camPose(2, 3) = camPosition(2);

	// transform matrix
	matrix<double> camPose_rob = convertCamToRobPose(camPose, true);

	// reduce result to translation vector
	vector<double> camPosition_rob(3);
	camPosition_rob(0) = camPose_rob(0, 3);
	camPosition_rob(1) = camPose_rob(1, 3);
	camPosition_rob(2) = camPose_rob(2, 3);

	return camPosition_rob;
}

matrix<double> UR5::convertNeedleToRobPose(matrix<double> needlePose)
{
	matrix<double> needle2rob(4,4);
	InvertMatrix(getRobotToNeedleTransformation(), needle2rob);
	return prod(needlePose,needle2rob);
}

matrix<double> UR5::convertCamToRobPose(matrix<double> camPose)
{
	return convertCamToRobPose(camPose, true);
}

matrix<double> UR5::convertCamToRobPose(matrix<double> camPose, bool use_orthogonalization)
{	
	if (use_orthogonalization)
	{
		// orthonormalize pose	
		matrix<double> transformationMatrix(4, 4);

		alglib::real_1d_array w;
		alglib::real_2d_array u;
		alglib::real_2d_array vt;

		alglib::real_2d_array a;
		a.setlength(3, 3);

		matrix<double> pose = prod(robot_to_cam_transformation_, camPose);

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

		matrix<double> uBoost(uCols, uRows);
		matrix<double> vtBoost(vtCols, vtRows);

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

		matrix<double> rot = prod(uBoost, vtBoost);

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

matrix<double> UR5::orientateAlongVector(vector<double> vec){
	return orientateAlongVector(vec[0], vec[1], vec[2]);
}





void UR5::doNeedlePlacement(vector<double> target, vector<double> window, matrix<double> needleTip) {
	IKResult result, oldResult;
	vector<double> direction(3);
	vector<double> homoTarget(4);
	vector<double> homoWindow(4);
	vector<double> homoDirection(4);
	vector<double> nextPos(4);
	matrix<double> pose(4, 4);
	matrix<double> inverseNeedleTip(4,4);
	JointAngles angles;
	bool done = false;

	InvertMatrix(needleTip, inverseNeedleTip);

	direction = target - window;
	std::cout << "Direction 1:" << direction << std::endl;
	direction = direction / norm_2(direction);		//normalize direction vector
	std::cout << "Direction 2:" << direction << std::endl;

	//init Homogenous coordinates
	
	for (int i = 0; i < 4; i++) {
		if (i == 3) {
			homoTarget(i) = 1;
			homoWindow(i) = 1;
			homoDirection(i) = 1;
		}
		else{
			homoTarget(i) = target(i);
			homoWindow(i) = window(i);
			homoDirection(i) = direction(i);
		}
	}
	//homoTarget = prod(inverseNeedleTip, homoTarget);
	//homoWindow = prod(inverseNeedleTip, homoWindow);
	moveToPosition(window(0), window(1), window(2) + Z_OFFSET_TO_TARGET);
	waitUntilFinished(500);
	pose = orientateAlongVector(direction);
	pose(2, 3) -= Z_OFFSET_TO_TARGET;
	waitUntilFinished(500);
	moveToPosition(window - 0.2*direction);

	//determine the farest pose away from the target that is reachable with the same joint configuration
	//result = path_planner_.chooseNearest(getJoints("rad"), path_planner_.checkForValidConfigurations(inverse_kinematics_.computeInverseKinematics(pose)));
	//while (!done) {
	//	oldResult = result;
	//	nextPos = column(pose, 3) - (homoDirection * 0.05);
	//	for (int i = 0; i < 3; i++)
	//		pose(i, 3) = nextPos(i);
	//	result = path_planner_.checkForValidConfigurations(inverse_kinematics_.computeInverseKinematics(pose));
	//	
	//	//this is very bad coding...
	//	
	//		for (int j = 0; j < result.configuration.size(); j++) {
	//			std::cout << "  Old Result: " << oldResult.nearestConfig << "  Result : " << result.configuration[j] << "  Distance : " << norm_2((column(pose, 3) - homoTarget)) << std::endl;
	//			if (oldResult.nearestConfig(0) == result.configuration[j](0) &&
	//				oldResult.nearestConfig(1) == result.configuration[j](1) &&
	//				oldResult.nearestConfig(2) == result.configuration[j](2)) {
	//				done = false;
	//				result.nearestConfig = oldResult.nearestConfig;
	//				angles = result.solutions[j];
	//				
	//			}
	//			else {
	//				done = true;
	//			}
	//	}
	//	if (norm_2((column(pose, 3) - homoTarget)) > 0.3) {
	//		done = true;
	//	}
	//	
	//}
	//setJoints(angles);

}

vector<double> UR5::convertPixelToProbe(int x, int y) {

	vector<double> result(4);
	vector<double> pos(4);

	pos(0) = x*0.0000819;
	pos(1) = y*0.0000833;
	pos(2) = 0;
	pos(3) = 1;

	result = prod(pixel_to_probe_transformation_, pos);

	return result;
}

matrix<double> UR5::getEndEffectorPose()
{
	JointAngles jointAngles = getJoints("rad");
	DirectKinematics directKinematics;
	return directKinematics.computeDirectKinematics(jointAngles);
}

matrix<double> UR5::getNeedlePose()
{
	JointAngles jointAngles = getJoints("rad");
	DirectKinematics directKinematics;
	return prod(getEndEffectorPose(), robot_to_needle_transformation_);
}


/// <summary>
/// Places the needle.
/// </summary>
/// <param name="target">The target.</param>
/// <param name="window_center">The window_center.</param>
/// <param name="log_movement">Movement is logged to CSV files after being complete dif set to <c>true</c> [log_movement].</param>
bool UR5::needlePlacementTwo(vector<double> target, std::vector<vector<double>> window, vector<double> window_center, bool log_movement, bool move_interpolated)
{
	CSVParser csvParser;
	InverseKinematics inverseKinematics;
	DirectKinematics directKinematics;
	bool success = true;

	// PLANNING
	double distance = 0.05; // minimum distance from window to needle for outside position
	double max_change_in_rotation = 20 * (PI / 180); // maximum sum of joint angles between two poses for line interpolation
	double step_size = 0.005; // stepsize in m for line interpolation

	std::vector<vector<double>> window_points;
	window_points.push_back(window_center);
	window_points.push_back((window_center + window[0]) / 2);
	window_points.push_back((window_center + window[1]) / 2);
	window_points.push_back((window_center + window[2]) / 2);
	window_points.push_back((window_center + window[3]) / 2);
	window_points.push_back((window_points[0] + window_points[1]) / 2);
	window_points.push_back((window_points[1] + window_points[2]) / 2);
	window_points.push_back((window_points[2] + window_points[3]) / 2);
	window_points.push_back((window_points[3] + window_points[1]) / 2);
	

	matrix<double> pose_rob;
	std::vector<JointAngles> lineJointAngles;
	bool pathFound = false;
	for (int j = 0; j < window_points.size(); j++)
	{
		vector<double> window_to_target = target - window_points[j]; // vector from middle to tumor point

		vector<double> direction = (-window_to_target) / norm_2(window_to_target);	// normed vector from window center pointing away from target
		matrix<double> rot = orientateAlongVector(window_to_target); // desired needle rotation for outside pose

		// calculate pose with increased distance from window center
		vector<double> position = window_center + direction*distance;

		// convert to robot coordinates
		pose_rob = convertNeedleToRobPose(MathTools::composeMatrix(rot, position));

		// comput inverse kinematics
		IKResult ikResult = inverseKinematics.computeInverseKinematics(pose_rob);
		JointAngles angles;

		if (ikResult.solutions.size() > 0) {
			IKResult* result = path_planner_.chooseNearest(getJoints("rad"), path_planner_.checkForValidConfigurations(ikResult));
			if (result)
			{
				angles = (result->nearestSolution);
			}
		}

		// find path
		pathFound = interpolateLine(angles, target, max_change_in_rotation, step_size, lineJointAngles);

		if (pathFound)
		{
			// take first solution
			pathFound = true;
			break;
		}
	}


	if (!pathFound)
	{
		std::cout << "Target cannot be reached with linear movement" << std::endl;
		return false;
	}

	// STEP 1: MOVE TO POSE OUTSIDE OF BOX
	std::cout << "outside_pose_pre: " << pose_rob << std::endl;
	//success = moveAndWait(&UR5::moveToPose, outside_pose, outside_pose);
	success = moveAndWait(&UR5::setJoints, lineJointAngles[0], pose_rob);
	if (!success)
	{
		return false;
	}
	if (log_movement)
	{
		csvParser.writeHTM(pose_rob, std::string(SIMULATION_OUTPUT_FOLDER) + "sim_outside_matrix.csv");
	}
	std::cout << "outside_pose: " << pose_rob << std::endl;

	// STEP 2: MOVE INTO TUMOR ON STRAIGHT LINE
	matrix<double> final_pose;

	if (move_interpolated)
	{
		// move on interpolated line
		for (int i = 0; i < lineJointAngles.size(); i++)
		{
			success = setJoints(lineJointAngles[i]);
			if (!success)
			{
				return false;
			}
			/*if (log_movement)
			{
			csvParser.writeHTM(final_pose, std::string(SIMULATION_OUTPUT_FOLDER) + std::string("sim_final_matrix") + std::to_string(i) + std::string(".csv"));
			}*/
		}
		waitUntilFinished(500);
		JointAngles jointAngles = getJoints("rad");
		DirectKinematics directKinematics;
		final_pose = directKinematics.computeDirectKinematics(jointAngles);
		if (log_movement)
		{
			csvParser.writeHTM(final_pose, std::string(SIMULATION_OUTPUT_FOLDER) + "sim_final_matrix0.csv");
		}
	}
	else
	{
		// use linear movement function provided by robot
		final_pose = convertNeedleToRobPose(MathTools::composeMatrix(MathTools::getRotation(getNeedlePose()), target));
		success = moveAndWait(&UR5::moveLinear, final_pose, final_pose);
		if (!success)
		{
			return false;
		}
		if (log_movement)
		{
			csvParser.writeHTM(final_pose, std::string(SIMULATION_OUTPUT_FOLDER) + "sim_final_matrix0.csv");
		}
	}

	return true;
}


/// <summary>
/// Places the needle.
/// </summary>
/// <param name="target">The target.</param>
/// <param name="window_center">The window_center.</param>
/// <param name="log_movement">Movement is logged to CSV files after being complete dif set to <c>true</c> [log_movement].</param>
bool UR5::needlePlacement(vector<double> target, vector<double> window_center, bool log_movement, bool move_interpolated)
{
	CSVParser csvParser;
	InverseKinematics inverseKinematics;
	DirectKinematics directKinematics;
	bool success = true;
	
	// PLANNING
	double distance = 0.05; // minimum distance from window to needle for outside position
	double max_change_in_rotation = 20 * (PI / 180); // maximum sum of joint angles between two poses for line interpolation
	double step_size = 0.005; // stepsize in m for line interpolation
	vector<double> window_to_target = target - window_center; // vector from middle to tumor point

	vector<double> direction = (-window_to_target) / norm_2(window_to_target);	// normed vector from window center pointing away from target
	matrix<double> rot = orientateAlongVector(window_to_target); // desired needle rotation for outside pose
	matrix<double> outside_pose; // holds the final pose after calculation

	// calculate pose with increased distance from window center
	vector<double> position = window_center + direction*distance;

	// convert to robot coordinates
	matrix<double> pose_rob = convertNeedleToRobPose(MathTools::composeMatrix(rot, position));

	// comput inverse kinematics
	IKResult ikResult = inverseKinematics.computeInverseKinematics(pose_rob);

	// find joint angles that allow linear movement
	std::vector<JointAngles> lineJointAngles;
	bool pathFound = false;
	for (int i = 0; ikResult.solutions.size(); i++)
	{
		// check whether linear movement is possible or not
		pathFound = interpolateLine(ikResult.solutions[i], target, max_change_in_rotation, step_size, lineJointAngles);

		if (pathFound)
		{
			// take first solution
			pathFound = true;
			break;
		}
	}

	if (!pathFound)
	{
		std::cout << "Target cannot be reached with linear movement" << std::endl;
		return false;
	}

	// STEP 1: MOVE TO POSE OUTSIDE OF BOX
		std::cout << "outside_pose_pre: " << pose_rob << std::endl;
		//success = moveAndWait(&UR5::moveToPose, outside_pose, outside_pose);
		success = moveAndWait(&UR5::setJoints, lineJointAngles[0], outside_pose);
		if (!success)
		{
			return false;
		}
		if (log_movement)
		{
			csvParser.writeHTM(outside_pose, std::string(SIMULATION_OUTPUT_FOLDER) + "sim_outside_matrix.csv");
		}
		std::cout << "outside_pose: " << outside_pose << std::endl;

	// STEP 2: MOVE INTO TUMOR ON STRAIGHT LINE
		matrix<double> final_pose;

		if (move_interpolated)
		{
			// move on interpolated line
			for (int i = 0; i < lineJointAngles.size(); i++)
			{
				success = setJoints(lineJointAngles[i]);
				if (!success)
				{
					return false;
				}
				/*if (log_movement)
				{
					csvParser.writeHTM(final_pose, std::string(SIMULATION_OUTPUT_FOLDER) + std::string("sim_final_matrix") + std::to_string(i) + std::string(".csv"));
				}*/
			}
			waitUntilFinished(500);
			JointAngles jointAngles = getJoints("rad");
			DirectKinematics directKinematics;
			final_pose = directKinematics.computeDirectKinematics(jointAngles);
			if (log_movement)
			{
				csvParser.writeHTM(final_pose, std::string(SIMULATION_OUTPUT_FOLDER) + "sim_final_matrix0.csv");
			}
		}
		else
		{
			// use linear movement function provided by robot
			final_pose = convertNeedleToRobPose(MathTools::composeMatrix(MathTools::getRotation(getNeedlePose()), target));
			success = moveAndWait(&UR5::moveLinear, final_pose, final_pose);
			if (!success)
			{
				return false;
			}
			if (log_movement)
			{
				csvParser.writeHTM(final_pose, std::string(SIMULATION_OUTPUT_FOLDER) + "sim_final_matrix0.csv");
			}
		}

	return true;
}


