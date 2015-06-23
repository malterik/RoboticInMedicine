#include "PathPlanner.h"
#include <iostream>
PathPlanner::PathPlanner(){

}

JointAngles PathPlanner::chooseNearest(JointAngles currentAngles, std::vector<JointAngles>& configs){
	JointAngles result;
	double minDist,currentDist;
	if (configs.empty()) {
		std::cout << "No Solution found!" << std::endl;
		return currentAngles;
	}

	minDist = std::numeric_limits<double>::max();
	int temp = -1;
	currentDist = 0;
	for (int i = 0; i < configs.size(); i++) {
		currentDist = currentAngles - configs[i];
		if (currentDist < minDist) {
			minDist = currentDist;
			result = configs[i];
			temp = i;
		}
	}
	std::cout << "Pathplanner decided for Path " << temp << std::endl;
	return result;
}

std::vector<JointAngles> PathPlanner::checkForValidConfigurations(std::vector<JointAngles>& configs){

	std::vector<JointAngles> result;
	matrix<double> jointPosition(4, 4);
	int value = 0;

	for (int i = 0; i < configs.size(); i++) {
		value = 0;
		for (int j = 0; j < 6; j++) {
			jointPosition = direct_kinematics_.getPositionOfJoint(j, configs[i]);
			//If the z coordinate of the joint is above the table
			if (jointPosition(2, 3) > 0) {
				value++;
			}
		}
		if (value == 6){
			result.push_back(configs[i]);
		}
	}

	return result;
}