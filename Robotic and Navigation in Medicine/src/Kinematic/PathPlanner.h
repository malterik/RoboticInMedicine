#pragma once
#include "../Kinematic/JointAngles.h"
#include "../Kinematic/directKinematics.h"
#include "../Kinematic/inverseKinematics.h"
#include "../Robot/UR5.h"

#define Z_OFFSET_TO_TARGET 0.2
#include <vector>
class PathPlanner
{
public:
	PathPlanner();
	JointAngles chooseNearest(JointAngles currentAngles, std::vector<JointAngles>& configs);
	std::vector<JointAngles> checkForValidConfigurations(std::vector<JointAngles>& configs);
	void doNeedlePlacement(vector<double> target,vector<double> window,matrix<double> needleTip);

private:
	DirectKinematics direct_kinematics_;
	InverseKinematics inverse_kinematics_;
};
