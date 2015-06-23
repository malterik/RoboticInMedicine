#pragma once
#include "../Kinematic/JointAngles.h"
#include "../Kinematic/directKinematics.h"
#include <vector>
class PathPlanner
{
public:
	PathPlanner();
	JointAngles chooseNearest(JointAngles currentAngles, std::vector<JointAngles>& configs);
	std::vector<JointAngles> checkForValidConfigurations(std::vector<JointAngles>& configs);

private:
	DirectKinematics direct_kinematics_;
};
