#pragma once

#include <string>
#include "boost\numeric\ublas\matrix.hpp"

/// <summary>
/// Provides methods to import calibration data.
/// </summary>
/// <remarks>
///	Calibration data can be imported to transform between different coordinate systems (i.e ultra sound, needle, camera, robot).
/// </remarks>
class CSVParser
{
public:
	CSVParser();
	~CSVParser();

	boost::numeric::ublas::matrix<double> readHTM(std::string fileName);
	boost::numeric::ublas::vector<double> CSVParser::readVector3D(std::string fileName);
};

