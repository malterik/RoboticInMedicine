#include "Window.h"

Window::Window(std::vector<vector<double>> window_vec)
{
	vec1= window_vec[0];
	vec2 = window_vec[1];
	vec3 = window_vec[2];
	vec4 = window_vec[3];

	middle = calc_middle();

	tumor = vector<double>(3);
	tumor[0] = 0;
	tumor[1] = 0;
	tumor[2] = 0;
}

Window::Window(std::vector<vector<double>> window_vec, vector<double> tumor_vec)
{
	vec1 = window_vec[0];
	vec2 = window_vec[1];
	vec3 = window_vec[2];
	vec4 = window_vec[3];

	middle = calc_middle();

	tumor = tumor_vec;

}

vector<double> Window::get_middle()
{
	return middle;
}

vector<double> Window::get_tumor_dir(vector<double> tum)
{
	vector<double> tumor_dir(3);

	tumor_dir = tum - middle;

	return tumor_dir;
}

vector<double> Window::get_tumor_dir()
{
	vector<double> tumor_dir(3);

	tumor_dir = tumor - middle;

	return tumor_dir;
}

vector<double> Window::calc_middle()
{
	vector<double> mid(3);
	mid[0] = (vec1[0] + vec2[0] + vec3[0] + vec4[0]) / 4;
	mid[1] = (vec1[1] + vec2[1] + vec3[1] + vec4[1]) / 4;
	mid[2] = (vec1[2] + vec2[2] + vec3[2] + vec4[2]) / 4;
	return mid;
}