#include "DenavitHartenberg.h"
#include <boost\math\constants\constants.hpp>

#define PI boost::math::constants::pi<double>()

DenavitHartenberg::DenavitHartenberg() {
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

	alpha[0] = PI / 2;
	alpha[1] = 0;
	alpha[2] = 0;
	alpha[3] = PI / 2;
	alpha[4] = -PI / 2;
	alpha[5] = 0;
}

matrix<double> DenavitHartenberg::getTransformation(int i, double theta_i){
	matrix<double> transformationMatrix(4, 4);

	transformationMatrix(0, 0) = cos(theta_i);
	transformationMatrix(0, 1) = -cos(alpha[i-1]) * sin(theta_i);
	transformationMatrix(0, 2) = sin(alpha[i-1]) * sin(theta_i);
	transformationMatrix(0, 3) = a[i-1] /* (PI / 180) */* cos(theta_i);

	transformationMatrix(1, 0) = sin(theta_i);
	transformationMatrix(1, 1) = cos(alpha[i - 1]) * cos(theta_i);
	transformationMatrix(1, 2) = -sin(alpha[i-1]) * cos(theta_i);
	transformationMatrix(1, 3) = a[i-1] /* (PI / 180) */ *sin(theta_i);

	transformationMatrix(2, 0) = 0;
	transformationMatrix(2, 1) = sin(alpha[i-1]);
	transformationMatrix(2, 2) = cos(alpha[i-1]);
	transformationMatrix(2, 3) = d[i-1];

	transformationMatrix(3, 0) = 0;
	transformationMatrix(3, 1) = 0;
	transformationMatrix(3, 2) = 0;
	transformationMatrix(3, 3) = 1;

	return transformationMatrix;
}