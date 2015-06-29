#ifndef _INVERSEKINEMATICS_
#define _INVERSEKINEMATICS_

#include <vector>
#include <boost\numeric\ublas\matrix.hpp>
#include "JointAngles.h"
#include "../Robot/DenavitHartenberg.h"

#define NUMBER_OF_SOLUTIONS 8
class InverseKinematics {
  public:
    InverseKinematics ();
    std::vector<JointAngles> computeInverseKinematics (boost::numeric::ublas::matrix<double> endPose);
  private:
	  std::array<signed int, NUMBER_OF_SOLUTIONS> SHOULDER;
	  std::array<signed int, NUMBER_OF_SOLUTIONS> ELBOW;
	  std::array<signed int, NUMBER_OF_SOLUTIONS> WRIST;
	DenavitHartenberg dh;


};

#endif
