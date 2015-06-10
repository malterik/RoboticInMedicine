#pragma once
#include <vector>

#include <opencv2\core\core.hpp>

#define IMG_PATH "fileout_0.jpg"
class USHandler
{
public:
	USHandler();
	~USHandler();

	std::vector<cv::Vec2i> processImage();
private:

	cv::Mat getImage(const char* path);

	cv::Mat calib_img_;
	cv::Mat img_;
};

