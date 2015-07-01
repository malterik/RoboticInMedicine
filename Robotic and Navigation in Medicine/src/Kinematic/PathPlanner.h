#pragma once
#include "../Kinematic/JointAngles.h"
#include "../Kinematic/directKinematics.h"
#include "../Kinematic/inverseKinematics.h"

#include <vector>
class PathPlanner
{
public:
	PathPlanner();
	IKResult chooseNearest(JointAngles currentAngles, IKResult configs);
	IKResult checkForValidConfigurations(IKResult configs);
	void doNeedlePlacement(vector<double> target,vector<double> window,matrix<double> needleTip);

private:
	DirectKinematics direct_kinematics_;
	InverseKinematics inverse_kinematics_;
};
