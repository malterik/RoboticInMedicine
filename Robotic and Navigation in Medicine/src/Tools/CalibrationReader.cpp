#pragma once

#include "CalibrationReader.hpp"
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

using namespace std;

/// <summary>
/// Initializes a new instance of the <see cref="CalibrationReader"/> class.
/// </summary>
/// <param name="fileName">Name of the file.</param>
CalibrationReader::CalibrationReader(string fileName)
{
	fileName_ = fileName;
}


/// <summary>
/// Finalizes an instance of the <see cref="CalibrationReader" /> class.
/// </summary>
CalibrationReader::~CalibrationReader()
{
}

/// <summary>
/// Imports homogenous transformation matrix from CSV file.
/// </summary>
/// <remarks>
///	The file is expected to have the matrix entries listed in 4 lines with 4 entries. This can be achieved using MATLAB's bulit-in csvwrite command on a 4x4 matrix.
/// </remarks>
/// <returns>A homogenous transformation matrix</returns>
KinematicMatrix CalibrationReader::importFromCSVFile()
{
	KinematicMatrix kinematicMatrix;
	vector<vector<float>> data;
	ifstream infile(fileName_);

	// parse data into two-dimensional float array
	while (infile)
	{
		string s;
		if (!getline(infile, s)) break;

		istringstream ss(s);
		vector<float> record;

		while (ss)
		{
			string s;
			if (!getline(ss, s, ',')) break;
			record.push_back(std::strtod(s.c_str(), NULL));
		}

		data.push_back(record);
	}

	// create KinematicMatrix from float array.
	if (data.size() == 4)
	{
		if (data[0].size() == 4)
		{
			Vector3<float> translationVector(data[0][3], data[1][3], data[2][3]);
			RotationMatrix rotationMatrix(data[0][0], data[0][1], data[0][2], data[1][0], data[1][1], data[1][2], data[2][0], data[2][1], data[2][2]);
			kinematicMatrix = KinematicMatrix(rotationMatrix, translationVector);
		}
	}
	else
	{
		cerr << "Invalid dimensions for HTM conversion.";
	}


	return kinematicMatrix;
}
