#ifndef _INVERSEKINEMATICS_
#define _INVERSEKINEMATICS_

#include <vector>
#include <boost\numeric\ublas\matrix.hpp>
#include <boost\numeric\ublas\vector.hpp>
#include "JointAngles.h"
#include "../Robot/DenavitHartenberg.h"

#define NUMBER_OF_SOLUTIONS 4
struct IKResult {
	IKResult() :
		nearestConfig(3), solutions(), configuration(),nearestSolution(){};
	JointAngles nearestSolution;
	boost::numeric::ublas::vector<double> nearestConfig;
	std::vector<JointAngles> solutions;
	std::vector<boost::numeric::ublas::vector<int>> configuration;
};

class InverseKinematics {
  public:
    InverseKinematics ();
    IKResult computeInverseKinematics (boost::numeric::ublas::matrix<double> endPose);
  private:
	  std::array<signed int, NUMBER_OF_SOLUTIONS> SHOULDER;
	  std::array<signed int, NUMBER_OF_SOLUTIONS> ELBOW;
	  std::array<signed int, NUMBER_OF_SOLUTIONS> WRIST;
	DenavitHartenberg dh;


};

#endif
