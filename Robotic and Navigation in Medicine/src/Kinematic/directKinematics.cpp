#include "directKinematics.h"
#include <boost/math/constants/constants.hpp>
#define PI boost::math::constants::pi<float>()

DirectKinematics::DirectKinematics()
{
	////a = vector<float>(6);
	//a.insert_element(0, 0);
	//a.insert_element(1, -0.4250);
	//a.insert_element(2, -0.39225);
	//a.insert_element(3, 0);
	//a.insert_element(4, 0);
	//a.insert_element(5, 0);
	a[0] = 0;
	a[1] = -0.4250;
	a[2] = -0.39225;
	a[3] = 0;
	a[4] = 0;
	a[5] = 0;
	

	/*d = vector<float>(6);
	d.insert_element(0, 0.089159);
	d.insert_element(1, 0);
	d.insert_element(2, 0);
	d.insert_element(3, 0.10915);
	d.insert_element(4, 0.09465);
	d.insert_element(5, 0.0823);
*/
	d[0] = 0.089159;
	d[1] = 0;
	d[2] = 0;
	d[3] = 0.10915;
	d[4] = 0.09465;
	d[5] = 0.0823;
	
	/*alpha = vector<float>(6);
	alpha.insert_element(0, PI/2);
	alpha.insert_element(1, 0);
	alpha.insert_element(2, 0);
	alpha.insert_element(3, PI/2);
	alpha.insert_element(4, -PI/2);
	alpha.insert_element(5, 0);*/

	alpha[0] = PI/2;
	alpha[1] = 0;
	alpha[2] = 0;
	alpha[3] = PI / 2;
	alpha[4] = - PI / 2;
	alpha[5] = 0;

	A = matrix<float>(4, 4);
	
}

KinematicMatrix DirectKinematics::computeDirectKinematics(std::array<float,6> q)
{
	
	matrix<float> T_z = matrix<float>(4, 4);
	matrix<float> R_z = matrix<float>(4, 4);
	matrix<float> T_x = matrix<float>(4, 4);
	matrix<float> R_x = matrix<float>(4, 4);

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

		T_z(2, 3) = d[k];

		R_z(0, 0) = cos(q[k]);
		R_z(0, 1) = -sin(q[k]);
		R_z(1, 0) = sin(q[k]);
		R_z(1, 1) = cos(q[k]);

		T_x(0, 3) = a[k];

		R_x(1, 1) = cos(alpha[k]);
		R_x(1, 2) = -sin(alpha[k]);
		R_x(2, 1) = sin(alpha[k]);
		R_x(2, 2) = cos(alpha[k]);

		//In one expression too complex for prod()
		matrix<float> B = prod(T_x, R_x);
		B = prod(R_z, B);
		B = prod(T_z, B);
		A = prod(A, B);
		//TODO make more performant, for example dont use all those matrices

	}
	
	
	Vector3<float> posV;
	posV = Vector3<float>(A(0, 3), A(1, 3), A(2, 3));

	RotationMatrix rotM(	A(0,0), A(0,1), A(0,2),
							A(1,0), A(1,1), A(1,2),
							A(2,0), A(2,1), A(2,2)
						);

	KinematicMatrix kinM(rotM, posV);


	return kinM;
	

}






