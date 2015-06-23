
#include "inverseKinematics.h"
#include <math.h>
#include <boost/math/constants/constants.hpp>
#define PI boost::math::constants::pi<float>()
#include <boost/numeric/ublas/matrix_proxy.hpp>
#include <boost\numeric\ublas\vector.hpp>
#include <boost\numeric\ublas\io.hpp>
#include "../Math/InvertMatrix.h"
using namespace boost::numeric::ublas;


InverseKinematics::InverseKinematics() {

	SHOULDER =	{{ 1, 1, 1, 1, -1, -1, -1, -1 }};
	ELBOW =		{{ 1, 1, -1, -1, 1, 1, -1, -1 }};
	WRIST =		{{ 1, -1, 1, -1, 1, -1, 1, -1 }};
  }


  std::vector<JointAngles> InverseKinematics::computeInverseKinematics (boost::numeric::ublas::matrix<double> endPose) {
	  std::vector<JointAngles> configs;
	  for (unsigned int i = 0; i < 8; i++) {


		  /// Theta 1 
		  std::array<double, 8> theta_1;
		  boost::numeric::ublas::vector<double> vec1(4);
		  boost::numeric::ublas::vector<double> e4(4);
		  boost::numeric::ublas::vector<double> p05(4);
		  double psi, phi;
		  //fill vec1 with zeros and -d6
		  for (unsigned int j = 0; j < vec1.size(); j++) {
			  
			  if (j == 2) {
				  vec1[j] = -dh.d[5];
				  e4[j] = 0;
			  }
			  else if (j == 3) {
				  vec1[j] = 1;
				  e4[j] = 1;
			  }
			  else {
				  vec1[j] = 0;
				  e4[j] = 0;
			  }
		  }
		  p05 = prod(endPose, vec1) - e4;
		  psi = atan2(p05(1), p05(0));
		  double arg = (dh.d[3] / sqrt(pow(p05(0), 2) + pow(p05(1), 2)));
		  if (arg > 1 && arg <= 1.01) {
			  arg = 1;
		  }
		  else if (arg < -1 && arg >= -1.01) {
			  arg = -1;
		  }
		  else if(arg > 1.01 || arg < -1.01) {
			 // continue;
		  }
		  phi = SHOULDER[i] * acos(arg);
		  
		  theta_1[i] = phi + psi + PI / 2;
		 

		  ///Theta 5
		  std::array<double, 8> theta_5;
		  boost::numeric::ublas::vector<double> p06(4);
		  boost::numeric::ublas::vector<double> p16(4);
		  p06 = column(endPose, 3);
		  p16(2) = p06(0) * sin(theta_1[i]) - p06(1) * cos(theta_1[i]);
		  theta_5[i] = WRIST[i] * acos((p16(2) - dh.d[3]) / dh.d[5]);
		  

		  ///Theta 6
		  //Transformation matrix from frame 0 to 1
		  std::array<double, 8> theta_6;
		  matrix<double> T61(4, 4);
		  matrix<double> T10(4, 4);
		  matrix<double> T16(4, 4);
		  matrix<double> T01(4, 4);

		  T01 = dh.getTransformation(1, theta_1[i]);
		  
		  InvertMatrix(T01, T10);
		  T16 = prod(T10, endPose);
		  InvertMatrix(T16, T61);
		  
		  theta_6[i] = atan2(-(T61(1, 2) / sin(theta_5[i])), (T61(0, 2) / sin(theta_5[i]) ));
		  

		  ///Theta 3
		  std::array<double, 8> theta_3;
		  matrix<double> T14(4, 4);
		  matrix<double> T45(4, 4);
		  matrix<double> T56(4, 4);
		  matrix<double> T46(4, 4);
		  matrix<double> T64(4, 4);
		  vector<double> p13(4);
		  vector<double> vec2(4);

		  vec2(0) = 0;
		  vec2(1) = -dh.d[3];
		  vec2(2) = 0;
		  vec2(3) = 1;

		  //std::cout << "T16 :" << T16 << std::endl;
		  T45 = dh.getTransformation(5, theta_5[i]);
		  T56 = dh.getTransformation(6, theta_6[i]);
		  T46 = prod(T45, T56);
		  InvertMatrix(T46, T64);
		  T14 = prod(T16, T64);
		  p13 = prod(T14, vec2) - e4;
		 // std::cout << "p13:  " << p13 << std::endl;
		  //std::cout << norm_2(p13) << std::endl;
		  double arg2 = (pow(norm_2(p13), 2) - pow(dh.a[1], 2) - pow(dh.a[2], 2)) / (2 * dh.a[1] * dh.a[2]);
		  //std::cout << "arg acos " << arg2 << std::endl;
		  if (arg2 > 1 && arg2 < 1.01) {
			  arg2 = 1;
		  }
		  else if (arg2 < -1 && arg2 > -1.01) {
			  arg2 = -1;
		  }
		  else if (arg2 > 1.01 || arg2 < -1.01) {
			 // continue;
		  }
		  theta_3[i] = ELBOW[i] * acos( arg2 );
		  
		  //std::cout << std::endl << std::endl;

		  ///Theta 2
		  std::array<double, 8> theta_2;
		  theta_2[i] = -atan2(p13(1), -p13(0)) + asin((dh.a[2] * sin(theta_3[i])) / norm_2(p13) );
		  

		  ///Theta 4 
		  std::array<double, 8> theta_4;
		  matrix<double> T13(4, 4);
		  matrix<double> T31(4, 4);
		  matrix<double> T12(4, 4);
		  matrix<double> T23(4, 4);
		  matrix<double> T34(4, 4);

		  T12 = dh.getTransformation(2, theta_2[i]);
		  T23 = dh.getTransformation(3, theta_3[i]);

		  T13 = prod(T12, T23);
		  InvertMatrix(T13, T31);
		  T34 = prod(T31, T14);
		  
		  theta_4[i] = atan2(T34(1, 0), T34(0, 0));
		  //std::cout << "theta_1 " << i << ": " << theta_1[i] << "\t" << "deg: " << theta_1[i] * (180 / PI) << std::endl;
		  //std::cout << "theta_2 " << i << ": " << theta_2[i] << "\t" << "deg: " << theta_2[i] * (180 / PI) << std::endl;
		  //std::cout << "theta_3 " << i << ": " << theta_3[i] << "\t" << "deg: " << theta_3[i] * (180 / PI) << std::endl;
		  //std::cout << "theta_4 " << i << ": " << theta_4[i] << "\t" << "deg: " << theta_4[i] * (180 / PI) << std::endl;
		  //std::cout << "theta_5 " << i << ": " << theta_5[i] << "\t" << "deg: " << theta_5[i] * (180 / PI) << std::endl;
		  //std::cout << "theta_6 " << i << ": " << theta_6[i] << "\t" << "deg: " << theta_6[i] * (180 / PI) << std::endl;
		  //std::cout << std::endl << std::endl;

		  JointAngles config_i(theta_1[i], theta_2[i], theta_3[i], theta_4[i], theta_5[i], theta_6[i]);
		  configs.push_back(config_i);
}
	
	return configs;
  }	

