#include "inverseKinematics.h"
#include <math.h>
#include <boost/math/constants/constants.hpp>
#define PI boost::math::constants::pi<float>()
#include <boost/numeric/ublas/matrix_proxy.hpp>
#include <boost\numeric\ublas\vector.hpp>
#include <boost\numeric\ublas\io.hpp>
using namespace boost::numeric::ublas;


  InverseKinematics::InverseKinematics () {
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

	ARM =	{ 1, 1, 1, 1, -1, -1, -1, -1 };
	ELBOW = { 1, 1, -1, -1, 1, 1, -1, -1 };
	WRIST = { 1, -1, 1, -1, 1, -1, 1, -1 };
  }


  std::array<JointAngles, 8> InverseKinematics::computeInverseKinematics (boost::numeric::ublas::matrix<double> endPose) {
	  std::array<JointAngles, 8> configs;

	  for (unsigned int i = 0; i < 1; i++) {


		  /// Theta 1 
		  std::array<double, 8> theta_1;
		  vector<double> vec1(4);
		  vector<double> p05(4);
		  double psi, phi;
		  //fill vec1 with zeros and -d6
		  for (int i = 0; i < vec1.size(); i++) {
			  if (i == 2) {
				  vec1[i] = -d[5];
			  }
			  else if (i == 3) {
				  vec1[i] = 1;
			  }
			  else {
				  vec1[i] = 0;
			  }
		  }
		  p05 = prod(endPose, vec1);
		  psi = atan2(p05(0), p05(1));
		  phi = /*+-*/ acos(d[3] / sqrt(pow(p05(0), 2) + pow(p05(1), 2)));
		  theta_1[i] = phi + psi + PI / 2;

		  std::cout << "p05: " << p05 << std::endl;
		  std::cout << "psi: " << psi << std::endl;
		  std::cout << "phi: " << phi << std::endl;
		  std::cout << "theta_" << i << ": " << theta_1[i] << std::endl;

		  ///Theta 5
		  std::array<double, 8> theta_5;
		  vector<double> p06(4);
		  vector<double> p16(4);
		  p06 = column(endPose, 3);

		  p16(2) = p06(0) * sin(theta_1[i]) - p06(0) * cos(theta_1[i]);
		  theta_5[i] = /*+-*/ acos(p16(2) - d[3] / d[5]);


	  }
	
	return configs;
  }	

