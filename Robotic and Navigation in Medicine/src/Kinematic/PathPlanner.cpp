#include "PathPlanner.h"
#include <iostream>
#include <boost/math/constants/constants.hpp>
#define PI boost::math::constants::pi<double>()

PathPlanner::PathPlanner(){

}

IKResult* PathPlanner::chooseNearest(JointAngles currentAngles, IKResult configs){
	// JointAngles result;
	IKResult res;

	double minDist,currentDist;
	if (configs.configuration.empty()) {
		std::cout << "No Solution found!" << std::endl;
		res.nearestSolution = currentAngles;
		return NULL;
	}

	minDist = std::numeric_limits<double>::max();
	int temp = -1;
	currentDist = 0;
	for (int i = 0; i < configs.solutions.size(); i++) {
		currentDist = currentAngles - configs.solutions[i];
		if (currentDist < minDist) {
			minDist = currentDist;
			res.nearestSolution = configs.solutions[i];
			res.nearestConfig = configs.configuration[i];
			temp = i;
		}
	}
	std::cout << "Pathplanner decided for Path " << temp << std::endl;
	return &res;
}

IKResult PathPlanner::checkForValidConfigurations(IKResult configs){

	IKResult result;
	matrix<double> jointPosition(4, 4);
	int value = 0;

	double maxWrist2Angle = 90 * (PI / 180);
	double minWrist2Angle = -maxWrist2Angle;

	for (int i = 0; i < configs.solutions.size(); i++) {
		value = 0;
		std::cout << configs.solutions[i].getWrist1Angle() << std::endl;

		if (-180 <= configs.solutions[i].getWrist1Angle() && configs.solutions[i].getWrist1Angle() <= 0 /*&& minWrist2Angle <= configs.solutions[i].getWrist2Angle() && configs.solutions[i].getWrist2Angle() <= maxWrist2Angle*/){
			for (int j = 0; j < 6; j++) {
				jointPosition = direct_kinematics_.getPositionOfJoint(j, configs.solutions[i]);
				//If the z coordinate of the joint is above the table
				if (jointPosition(2, 3) > 0.05) {
					value++;
				}
			}
			if (value == 6){
				result.solutions.push_back(configs.solutions[i]);
				result.configuration.push_back(configs.configuration[i]);
			}
		}
		
	}

	return result;
}

