#pragma once

#include "CSVParser.hpp"
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include "boost/numeric/ublas/matrix.hpp"
#include "boost/numeric/ublas/assignment.hpp"

/// <summary>
/// Initializes a new instance of the <see cref="CalibrationReader"/> class.
/// </summary>
/// <param name="fileName">Name of the file.</param>
CSVParser::CSVParser()
{
}


/// <summary>
/// Finalizes an instance of the <see cref="CalibrationReader" /> class.
/// </summary>
CSVParser::~CSVParser()
{
}

/// <summary>
/// Imports homogenous transformation matrix from CSV file.
/// </summary>
/// <remarks>
///	The file is expected to have the matrix entries listed in 4 lines with 4 entries. This can be achieved using MATLAB's bulit-in csvwrite command on a 4x4 matrix.
/// </remarks>
/// <returns>A homogenous transformation matrix</returns>
boost::numeric::ublas::matrix<double> CSVParser::readHTM(std::string fileName)
{
	boost::numeric::ublas::matrix<double> htm(4,4);
	std::vector<std::vector<double>> data;
	std::ifstream infile(fileName);

	// parse data into two-dimensional float array
	while (infile)
	{
		std::string s;
		if (!getline(infile, s)) break;

		std::istringstream ss(s);
		std::vector<double> record;

		while (ss)
		{
			std::string s;
			if (!getline(ss, s, ',')) break;
			record.push_back(std::strtod(s.c_str(), NULL));
		}

		data.push_back(record);
	}

	int numLines = data.size();
	int numRows = data[0].size();

	// create matrix from float array.
	if (numLines == 4 && numRows == 4)
	{
			htm <<= data[0][0], data[0][1], data[0][2], data[0][3],
					data[1][0], data[1][1], data[1][2], data[1][3],
					data[2][0], data[2][1], data[2][2], data[2][3],
					data[3][0], data[3][1], data[3][2], data[3][3];
	}
	else
	{
		std::cerr << "Invalid dimensions for HTM initialization.";
	}
	return htm;
}

/// <summary>
/// Imports vector from CSV file.
/// </summary>
/// <remarks>
///	The file is expected to have the vector listed in one line containing the three vector elements. This can be achieved using MATLAB's bulit-in csvwrite command on a 3x1 or 1x3 vector.
/// </remarks>
/// <returns>A homogenous transformation matrix</returns>
boost::numeric::ublas::vector<double> CSVParser::readVector3D(std::string fileName)
{
	boost::numeric::ublas::vector<double> vector(3);
	std::vector<std::vector<double>> data;
	std::ifstream infile(fileName);

	// parse data into two-dimensional float array
	while (infile)
	{
		std::string s;
		if (!getline(infile, s)) break;

		std::istringstream ss(s);
		std::vector<double> record;

		while (ss)
		{
			std::string s;
			if (!getline(ss, s, ',')) break;
			record.push_back(std::strtod(s.c_str(), NULL));
		}

		data.push_back(record);
	}

	int numLines = data.size();
	int numRows = data[0].size();

	// create vector from float array.
	if (numLines == 1 && numRows == 3)
	{
		vector <<= data[0][0], data[0][1], data[0][2];
	}
	else if (numLines == 3 && numRows == 1)
	{
		vector <<= data[0][0], data[1][0], data[2][0];
	}
	else
	{
		std::cerr << "Invalid dimensions for vector initialization.";
	}
	return vector;
}

/// <summary>
/// Imports window points from CSV file.
/// </summary>
/// <remarks>
///	The file is expected to have four vector listed in single lines containing the three vector elements. This can be achieved using MATLAB's bulit-in csvwrite command on a 3x4 vector.
/// </remarks>
/// <returns>A homogenous transformation matrix</returns>
std::vector<boost::numeric::ublas::vector<double>> CSVParser::readWindow(std::string fileName)
{
	std::vector<boost::numeric::ublas::vector<double>> windowPoints;
	std::vector<std::vector<double>> data;
	std::ifstream infile(fileName);

	// parse data into two-dimensional float array
	while (infile)
	{
		std::string s;
		if (!getline(infile, s)) break;

		std::istringstream ss(s);
		std::vector<double> record;

		while (ss)
		{
			std::string s;
			if (!getline(ss, s, ',')) break;
			record.push_back(std::strtod(s.c_str(), NULL));
		}

		data.push_back(record);
	}

	int numRows = data.size();
	int numCols = data[0].size();

	// create matrix from float array.
	if (numRows == 4 && numCols == 3)
	{
		for (int i = 0; i < numRows; i++)
		{
			boost::numeric::ublas::vector<double> point(3);
			point <<= data[i][0], data[i][1], data[i][2];
			windowPoints.push_back(point);
		}
	}
	else
	{
		std::cerr << "Invalid dimensions for HTM initialization.";
	}
	return windowPoints;
}