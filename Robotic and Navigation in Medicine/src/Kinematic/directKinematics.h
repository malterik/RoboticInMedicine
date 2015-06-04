#ifndef _DIRECTKINEMATICS_
#define _DIRECTKINEMATICS_

#include <boost\numeric\ublas\vector.hpp>
#include <boost\numeric\ublas\matrix.hpp>
#include <boost\numeric\ublas\io.hpp>

using namespace boost::numeric::ublas;

class DirectKinematics
{
public:
	DirectKinematics();
	//~DirectKinematics();

	matrix<double> computeDirectKinematics(vector<double> q);

private:
	vector<double> a;
	vector<double> d;
	vector<double> alpha;

	matrix<double> A;





};



#endif