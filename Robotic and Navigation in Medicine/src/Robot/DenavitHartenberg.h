#pragma once
#include <array>
#include <boost\numeric\ublas\matrix.hpp>

using namespace boost::numeric::ublas;
class DenavitHartenberg
{
public:
	DenavitHartenberg();
	//~DenavitHartenberg();
	std::array<double, 6> a;
	std::array<double, 6> d;
	std::array<double, 6> alpha;
	matrix<double> getTransformation(int i, double theta_i);

private:

};

