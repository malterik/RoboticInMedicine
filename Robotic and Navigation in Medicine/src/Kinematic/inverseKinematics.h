#ifndef _INVERSEKINEMATICS_
#define _INVERSEKINEMATICS_

#include <array>
#include <boost\numeric\ublas\matrix.hpp>
#include "JointAngles.h"

class InverseKinematics {
  public:
    InverseKinematics ();
    std::array<JointAngles, 8> computeInverseKinematics (boost::numeric::ublas::matrix<double> endPose);
  private:
    std::array<double, 6> a;
    std::array<double, 6> d;
    std::array<double, 6> alpha;
	std::array<signed int, 8> ARM;
	std::array<signed int, 8> ELBOW;
	std::array<signed int, 8> WRIST;


};

#endif
