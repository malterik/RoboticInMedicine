#pragma once

#include "MathTools.hpp"

matrix<double> MathTools::composeMatrix(matrix<double> rotation, vector<double> translation)
{
	identity_matrix<double> eye(4);
	matrix<double> pose(eye);

	pose = MathTools::setRotation(pose, rotation);
	pose = MathTools::setTranslation(pose, translation);

	return pose;
}

vector<double> MathTools::crossProduct(vector<double> a, vector<double> b)
{
	vector<double> cross(3);

	double ax = a(0);
	double ay = a(1);
	double az = a(2);

	double bx = b(0);
	double by = b(1);
	double bz = b(2);

	cross(0) = ay*bz - az*by;
	cross(1) = az*bx - ax*bz;
	cross(2) = ax*by - ay*bx;

	return cross;
}

matrix<double> MathTools::convertAxisAngleToRotationMatrix(vector<double> axis, double angle)
{
	matrix<double> rotation(3, 3);

	vector<double> temp_axis(axis);
	temp_axis /= norm_2(temp_axis);

	double x = temp_axis(0);
	double y = temp_axis(1);
	double z = temp_axis(2);

	double c = cos(angle);
	double s = sin(angle);
	double t = 1.0 - c;

	rotation(0, 0) = c + x*x*t;
	rotation(1, 1) = c + y*y*t;
	rotation(2, 2) = c + z*z*t;


	double tmp1 = x*y*t;
	double tmp2 = z*s;
	rotation(1, 0) = tmp1 + tmp2;
	rotation(0, 1) = tmp1 - tmp2;

	tmp1 = x*z*t;
	tmp2 = y*s;
	rotation(2, 0) = tmp1 - tmp2;
	rotation(0, 2) = tmp1 + tmp2;

	tmp1 = y*z*t;
	tmp2 = x*s;
	rotation(2, 1) = tmp1 + tmp2;
	rotation(1, 2) = tmp1 - tmp2;

	return rotation;
}

vector<double> MathTools::getTranslation(matrix<double> mat)
{
	vector<double> vec(3);

	vec <<= mat(0, 3), mat(1, 3), mat(2, 3);

	return vec;
}

matrix<double> MathTools::getRotation(matrix<double> mat)
{
	matrix<double> rotation(3,3);

	rotation(0, 0) = mat(0, 0);
	rotation(0, 1) = mat(0, 1);
	rotation(0, 2) = mat(0, 2);
	rotation(1, 0) = mat(1, 0);
	rotation(1, 1) = mat(1, 1);
	rotation(1, 2) = mat(1, 2);
	rotation(2, 0) = mat(2, 0);
	rotation(2, 1) = mat(2, 1);
	rotation(2, 2) = mat(2, 2);

	return rotation;
}

matrix<double> MathTools::setRotation(matrix<double> mat, matrix<double> rotation)
{
	matrix<double> newMat(mat);
	
	newMat(0, 0) = rotation(0, 0);
	newMat(0, 1) = rotation(0, 1);
	newMat(0, 2) = rotation(0, 2);
	newMat(1, 0) = rotation(1, 0);
	newMat(1, 1) = rotation(1, 1);
	newMat(1, 2) = rotation(1, 2);
	newMat(2, 0) = rotation(2, 0);
	newMat(2, 1) = rotation(2, 1);
	newMat(2, 2) = rotation(2, 2);


	return newMat;
}

matrix<double> MathTools::setTranslation(matrix<double> mat, vector<double> translation)
{
	matrix<double> newMat(mat);

	newMat(0, 3) = translation(0);
	newMat(1, 3) = translation(1);
	newMat(2, 3) = translation(2);

	return newMat;
}