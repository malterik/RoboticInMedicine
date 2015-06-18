#ifndef _INVERSEKINEMATICS_
#define _INVERSEKINEMATICS_

#include <array>
#include <boost\numeric\ublas\matrix.hpp>
#include "JointAngles.h"
#include "../Robot/DenavitHartenberg.h"
class InverseKinematics {
  public:
    InverseKinematics ();
    std::array<JointAngles, 8> computeInverseKinematics (boost::numeric::ublas::matrix<double> endPose);
  private:
	std::array<signed int, 8> SHOULDER;
	std::array<signed int, 8> ELBOW;
	std::array<signed int, 8> WRIST;
	DenavitHartenberg dh;


};

#endif
