#ifndef _INVERSEKINEMATICS_
#define _INVERSEKINEMATICS_

class InverseKinematics {
  public:
    InverseKinematics ();
    std::array<std::array<float, 6>, 8> computeInverseKinematics (matrix<float> endPose);
  private:
    std::array<float, 6> a;
    std::array<float, 6> d;
    std::array<float, 6> alpha;

};

#endif
