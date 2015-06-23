#include "directKinematics.h"
#include <boost/math/constants/constants.hpp>
#define PI boost::math::constants::pi<double>()
#include <boost\numeric\ublas\matrix.hpp>
#include "JointAngles.h"

using namespace boost::numeric::ublas;
DirectKinematics::DirectKinematics()
{

	


	A = matrix<double>(4, 4);

}

matrix<double> DirectKinematics::computeDirectKinematics(JointAngles q)
{

	matrix<double> T_z = matrix<double>(4, 4);
	matrix<double> R_z = matrix<double>(4, 4);
	matrix<double> T_x = matrix<double>(4, 4);
	matrix<double> R_x = matrix<double>(4, 4);

	//Init A
	for (int i = 0; i <= 3; i++)
	{
		for (int j = 0; j <= 3; j++)
		{
			if (i == j)
			{
				A.insert_element(i, j, 1);
			}
			else
			{
				A.insert_element(i, j, 0);
			}
		}
	}



	for (int k = 0; k <= 5; k++)
	{
		//Init T_z
		for (int i = 0; i <= 3; i++)
		{
			for (int j = 0; j <= 3; j++)
			{
				if (i == j)
				{
					T_z.insert_element(i, j, 1);
				}
				else
				{
					T_z.insert_element(i, j, 0);
				}
			}
		}

		//Init R_z
		for (int i = 0; i <= 3; i++)
		{
			for (int j = 0; j <= 3; j++)
			{
				if (i == j)
				{
					R_z.insert_element(i, j, 1);
				}
				else
				{
					R_z.insert_element(i, j, 0);
				}
			}
		}

		//Init T_x
		for (int i = 0; i <= 3; i++)
		{
			for (int j = 0; j <= 3; j++)
			{
				if (i == j)
				{
					T_x.insert_element(i, j, 1);
				}
				else
				{
					T_x.insert_element(i, j, 0);
				}
			}
		}

		//Init R_x
		for (int i = 0; i <= 3; i++)
		{
			for (int j = 0; j <= 3; j++)
			{
				if (i == j)
				{
					R_x.insert_element(i, j, 1);
				}
				else
				{
					R_x.insert_element(i, j, 0);
				}
			}
		}

		T_z(2, 3) = denavit_hartenberg_.d[k];

		R_z(0, 0) = cos(q[k]);
		R_z(0, 1) = -sin(q[k]);
		R_z(1, 0) = sin(q[k]);
		R_z(1, 1) = cos(q[k]);

		T_x(0, 3) = denavit_hartenberg_.a[k];

		R_x(1, 1) = cos(denavit_hartenberg_.alpha[k]);
		R_x(1, 2) = -sin(denavit_hartenberg_.alpha[k]);
		R_x(2, 1) = sin(denavit_hartenberg_.alpha[k]);
		R_x(2, 2) = cos(denavit_hartenberg_.alpha[k]);

		//In one expression too complex for prod()
		matrix<double> B = prod(T_x, R_x);
		B = prod(R_z, B);
		B = prod(T_z, B);
		A = prod(A, B);
		//TODO make more performant, for example dont use all those matrices

	}

	return A;

}

matrix<double> DirectKinematics::getPositionOfJoint(int jointNumber, JointAngles jointAngles){
	matrix<double> result(4, 4);
	result = denavit_hartenberg_.getTransformation(1, jointAngles[0]);

	for (int i = 2; i <= jointNumber; i++) {
		result = prod(result, denavit_hartenberg_.getTransformation(i, jointAngles[i - 1]));
	}

	return result;
}





