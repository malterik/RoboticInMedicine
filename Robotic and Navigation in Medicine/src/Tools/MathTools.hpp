#pragma once

#include "boost\numeric\ublas\matrix.hpp"
#include "boost\numeric\ublas\vector.hpp"
#include "boost\numeric\ublas\assignment.hpp"

using namespace boost::numeric::ublas;

class MathTools
{
public:
	static matrix<double> composeMatrix(matrix<double> rotation, vector<double> translation);
	static vector<double> MathTools::crossProduct(vector<double> a, vector<double> b);
	static matrix<double> MathTools::convertAxisAngleToRotationMatrix(vector<double> axis, double angle);
	static vector<double> MathTools::getTranslation(matrix<double> mat);
	static matrix<double> MathTools::getRotation(matrix<double> mat);
	static matrix<double> MathTools::setRotation(matrix<double> mat, matrix<double> rotation);
	static matrix<double> MathTools::setTranslation(matrix<double> mat, vector<double> translation);
};

