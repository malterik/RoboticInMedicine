#pragma once
#pragma warning(disable: 4996)
#include <boost\numeric\ublas\matrix.hpp>
#include <boost\numeric\ublas\io.hpp>
#include "KinematicMatrix.h"
#include "JointAngles.h"
#include "../Robot/DenavitHartenberg.h"
using namespace boost::numeric::ublas;

class DirectKinematics
{
public:
	DirectKinematics();
	//~DirectKinematics();

	matrix<double> computeDirectKinematics(JointAngles q);
	matrix<double> getPositionOfJoint(int jointNumber, JointAngles jointAngles);

private:
	std::array<float, 6> a;
	std::array<float, 6> d;
	std::array<float, 6> alpha;
	DenavitHartenberg denavit_hartenberg_;
	matrix<float> A;
};



