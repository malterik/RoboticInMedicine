#pragma once

#include "../Kinematic/KinematicMatrix.h"
#include <string>

/// <summary>
/// Provides methods to import calibration data.
/// </summary>
/// <remarks>
///	Calibration data can be imported to transform between different coordinate systems (i.e ultra sound, needle, camera, robot).
/// </remarks>
class CalibrationReader
{
public:
	CalibrationReader(std::string fileName);
	~CalibrationReader();

	KinematicMatrix importFromCSVFile();

private:
	std::string fileName_; ///< The filename that contains the calibration data.
};

