#include "inverseKinematics.h"
#include <Math>
#define PI boost::math::constants::pi<float>()

namespace InverseKinematics {

  InverseKinematics () {
    a[0] = 0;
    a[1] = -0.4250;
    a[2] = -0.39225;
    a[3] = 0;
    a[4] = 0;
    a[5] = 0;

    d[0] = 0.089159;
    d[1] = 0;
    d[2] = 0;
    d[3] = 0.10915;
    d[4] = 0.09465;
    d[5] = 0.0823;

    alpha[0] = PI/2;
    alpha[1] = 0;
    alpha[2] = 0;
    alpha[3] = PI / 2;
    alpha[4] = - PI / 2;
    alpha[5] = 0;
  }

  float atan2 (float x, float y) {

    if (y > 0 && a < 0) {
      return arctan (x / y) + 2 * PI;
    }
    else if (y > 0 && x >= 0) {
      return arctan (x / y);
    }
    else if (y == 0 && x > 0) {
      return PI / 2;
    }
    else if (y == 0 && x == 0) {
      return 0;
    }
    else if (y == 0 && x < 0) {
      return 3 * PI / 2;
    }
    else if (y < 0 && x >= 0) {
      return atan (x / y) + PI;
    }
    else if (y < 0 && x < 0) {
      return atan (x / y) + PI;
    }

    return 0;
  }

  std::array<std::array<float, 6>, 8> computeInverseKinematics (matrix<float> endPose) {
    std::array<std::array<float, 6>, 8> configs = new std::array < std::array<float, 6, 8>;
    std::array<float, 3> y = new std::array<float, 3> ();
    std::array<float, 3> z = new std::array<float, 3> ();
    std::array<float, 3> handPos = new std::array<float, 3> ();
    std::array<float, 2> theta_1 = new std::array<float, 2> ();
    std::array<float, 2> theta_2 = new std::array<float, 2> ();
    std::array<float, 2> theta_3 = new std::array<float, 2> ();
    std::array<float, 2> theta_4 = new std::array<float, 2> ();
    std::array<float, 2> theta_5 = new std::array<float, 2> ();
    std::array<float, 2> theta_6 = new std::array<float, 2> ();

    // Remove hand offset from position
    x (0) = endPose (0, 0);
    x (1) = endPose (1, 0);
    x (2) = endPose (2, 0);
    y (0) = endPose (0, 1);
    y (1) = endPose (1, 1);
    y (2) = endPose (2, 1);
    z (0) = endPose (0, 2);
    z (1) = endPose (1, 2);
    z (2) = endPose (2, 2);

    handPos (0) = endPose (0, 4) - d[5] * handRot (0);
    handPos (1) = endPose (1, 4) - d[5] * handRot (1);
    handPos (2) = endPose (2, 4) - d[5] * handRot (2);

    // Calculate theta1 aiming at 2 solutions
    theta_1 (0) = atan2 (handPos (1), handPos (0))
        + acos (d[3] / sqrt (handPos (1) * handPos (1) + handPos (0) * handPos (0))) + PI/ 2;
    theta_1 (1) = atan2 (handPos (1), handPos (0))
        - acos (d[3] / sqrt (handPos (1) * handPos (1) + handPos (0) * handPos (0))) + PI/ 2;

    theta_5 (0) = acos (
        (endPose (0, 3) * sin (theta_1 (0)) - endPose (1, 3) * cos (theta_1 (1)) - d[3]) / d[5]);
    theta_5 (1) = -acos (
        (endPose (0, 3) * sin (theta_1 (0)) - endPose (1, 3) * cos (theta_1 (1)) - d[3]) / d[5]);

    theta_6 (0) = atan2 (
        (-y (0) * sin (theta_1 (0)) + y (1) * cos (theta_1 (0))) / sin (theta_5 (0)),
        -(-x (0) * sin (theta_1 (0)) + x (1) * cos (theta_1 (0))) / sin (theta_5 (0)));
    theta_6 (1) = atan2 (
        (-y (0) * sin (theta_1 (1)) + y (1) * cos (theta_1 (1))) / sin (theta_5 (1)),
        -(-x (0) * sin (theta_1 (1)) + x (1) * cos (theta_1 (1))) / sin (theta_5 (1)));

    return configs;
  }
}
