#pragma once
#pragma warning(disable: 4996)

#include <boost\numeric\ublas\vector.hpp>
#include <boost\numeric\ublas\io.hpp>
using namespace boost::numeric::ublas;

class Window
{
public:
	Window(std::vector<vector<double>> window_vec);
	Window(std::vector<vector<double>> window_vec, vector<double> tumor_vec);

	vector<double> get_middle();
	vector<double> get_tumor_dir(vector<double> tum);
	vector<double> get_tumor_dir();

private:
	vector<double> calc_middle();
	vector<double> vec1;
	vector<double> vec2;
	vector<double> vec3;
	vector<double> vec4;
	vector<double> middle;
	vector<double> tumor;

};