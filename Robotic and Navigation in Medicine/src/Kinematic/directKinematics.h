#pragma once
#pragma warning(disable: 4996)
#include <boost\numeric\ublas\matrix.hpp>
#include <boost\numeric\ublas\io.hpp>
#include "KinematicMatrix.h"
#include "JointAngles.h"
using namespace boost::numeric::ublas;

class DirectKinematics
{
public:
	DirectKinematics();
	//~DirectKinematics();

	boost::numeric::ublas::matrix<double> computeDirectKinematics(JointAngles q);

private:
	std::array<double,6> a;
	std::array<double, 6> d;
	std::array<double, 6> alpha;

	matrix<double> A;
};



