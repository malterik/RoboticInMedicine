#pragma once
#pragma warning(disable: 4996)
#include <boost\numeric\ublas\matrix.hpp>
#include <boost\numeric\ublas\io.hpp>
#include "KinematicMatrix.h"
using namespace boost::numeric::ublas;

class DirectKinematics
{
public:
	DirectKinematics();
	//~DirectKinematics();

	KinematicMatrix computeDirectKinematics(std::array<float,6> q);

private:
	std::array<float,6> a;
	std::array<float, 6> d;
	std::array<float, 6> alpha;

	matrix<float> A;
};



