#pragma once
#pragma warning(disable: 4996)
#include <boost\numeric\ublas\matrix.hpp>
#include <boost\numeric\ublas\io.hpp>

using namespace boost::numeric::ublas;

class DirectKinematics
{
public:
	DirectKinematics();
	//~DirectKinematics();

	matrix<double> computeDirectKinematics(std::array<double,6> q);

private:
	std::array<double,6> a;
	std::array<double, 6> d;
	std::array<double, 6> alpha;

	matrix<double> A;
};



