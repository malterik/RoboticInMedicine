
#include "inverseKinematics.h"
#include <math.h>
#include <boost/math/constants/constants.hpp>
#define PI boost::math::constants::pi<float>()
#include <boost/numeric/ublas/matrix_proxy.hpp>
#include <boost\numeric\ublas\vector.hpp>
#include <boost\numeric\ublas\io.hpp>
#include "../Math/InvertMatrix.h"
using namespace boost::numeric::ublas;


  InverseKinematics::InverseKinematics () {

	SHOULDER =	{ 1, 1, 1, 1, -1, -1, -1, -1 };
	ELBOW = { 1, 1, -1, -1, 1, 1, -1, -1 };
	WRIST = { 1, -1, 1, -1, 1, -1, 1, -1 };
  }


  std::array<JointAngles, 8> InverseKinematics::computeInverseKinematics (boost::numeric::ublas::matrix<double> endPose) {
	  std::array<JointAngles, 8> configs;
	  std::cout << "endPose " << endPose << std::endl;
	  for (unsigned int i = 0; i < configs.size(); i++) {


		  /// Theta 1 
		  std::array<double, 8> theta_1;
		  boost::numeric::ublas::vector<double> vec1(4);
		  boost::numeric::ublas::vector<double> vec2(4);
		  boost::numeric::ublas::vector<double> p05(4);
		  double psi, phi;
		  //fill vec1 with zeros and -d6
		  for (unsigned int j = 0; j < vec1.size(); j++) {
			  
			  if (j == 2) {
				  vec1[j] = -dh.d[5];
			  }
			  else if (j == 3) {
				  vec1[j] = 1;
				  vec2[j] = 1;
			  }
			  else {
				  vec1[j] = 0;
				  vec2[j] = 0;
			  }
		  }
		  p05 = prod(endPose, vec1) - vec2;
		  psi = atan2(p05(1), p05(0));
		  double arg = (dh.d[3] / sqrt(pow(p05(0), 2) + pow(p05(1), 2)));
		  if (arg <= 1){
			  phi = SHOULDER[i] * acos(arg);
		  }
		  else {
			  std::cout << "warning" << std::endl;
			  phi = 0;
		  }
		  
		  theta_1[i] = phi + psi + PI / 2;
		  std::cout << "theta_1 " << i << ": " << theta_1[i] << "\t" << "deg: " << theta_1[i] * (180 / PI) << std::endl;

		  ///Theta 5
		  std::array<double, 8> theta_5;
		  boost::numeric::ublas::vector<double> p06(4);
		  boost::numeric::ublas::vector<double> p16(4);
		  p06 = column(endPose, 3);
		  p16(2) = p06(0) * sin(theta_1[i]) - p06(1) * cos(theta_1[i]);
		  theta_5[i] = WRIST[i] * acos((p16(2) - dh.d[3]) / dh.d[5]);
		  std::cout << "theta_5 " << i << ": " << theta_5[i] << "\t" << "deg: " << theta_5[i] * (180 / PI) << std::endl;

		  ///Theta 6
		  //Transformation matrix from frame 0 to 1
		  std::array<double, 8> theta_6;
		  matrix<double> T61(4, 4);
		  matrix<double> T10(4, 4);
		  matrix<double> tempMat(4, 4);
		  matrix<double> T01(4, 4);

		  T01 = dh.getTransformation(1, theta_1[i]);
		  
		  InvertMatrix(T01, T10);
		  tempMat = prod(T10, endPose);
		  InvertMatrix(tempMat, T61);
		  
		  theta_6[i] = atan2(-(T61(1, 2) / sin(theta_5[i])), (T61(0, 2) / sin(theta_5[i]) ));
		  std::cout << "theta_6 " << i << ": " << theta_6[i] << "\t" << "deg: " << theta_6[i] * (180 / PI) << std::endl;

		  std::cout << std::endl << std::endl;

	  }
	
	return configs;
  }	

