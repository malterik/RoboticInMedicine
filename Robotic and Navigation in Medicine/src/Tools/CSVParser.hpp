#pragma once

#include <string>
#include "boost\numeric\ublas\matrix.hpp"
#include <boost/algorithm/string/join.hpp>

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

	boost::numeric::ublas::matrix<double> CSVParser::readHTM(std::string fileName);
	boost::numeric::ublas::vector<double> CSVParser::readVector3D(std::string fileName);
	std::vector<boost::numeric::ublas::vector<double>> CSVParser::readWindow(std::string fileName);

	void CSVParser::writeHTM(boost::numeric::ublas::matrix<double> htm, std::string fileName);
};

