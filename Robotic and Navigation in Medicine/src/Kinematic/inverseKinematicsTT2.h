#ifndef _INVERSEKINEMATICS_
#define _INVERSEKINEMATICS_

#include <array>
#include <boost\numeric\ublas\matrix.hpp>

class InverseKinematics {
  public:
    InverseKinematics ();
    std::array<std::array<float, 8>, 6> computeInverseKinematics (boost::numeric::ublas::matrix<float> endPose);
  private:
    std::array<float, 6> a;
    std::array<float, 6> d;
    std::array<float, 6> alpha;


};

#endif
