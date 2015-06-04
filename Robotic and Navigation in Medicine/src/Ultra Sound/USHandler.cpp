#include "USHandler.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>

using namespace cv;

USHandler::USHandler():
calib_img_(), img_()
{
}


USHandler::~USHandler()
{
}

std::vector<Vec2i> USHandler::processImage() {
	std::vector<Vec2i> ret;
	int thresh = 100 ;
	img_ = getImage(IMG_PATH);
	if (!img_.data) {
		std::cout << "couldn't load file" << std::endl;
		return ret;
	}

	blur(img_, img_, Size(2, 2));
	Mat threshold_output;
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;

	threshold(img_, threshold_output, 90, 255, THRESH_BINARY);
	//closing
	dilate(threshold_output, threshold_output, Mat(), Point(-1, -1), 2, 1, 1);
	erode(threshold_output, threshold_output, Mat(), Point(-1, -1), 2, 1, 1);
	imshow(IMG_PATH, threshold_output);
	/// Find contours
	/// Find contours
	findContours(threshold_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

	/// Approximate contours to polygons + get bounding rects and circles
	vector<vector<Point> > contours_poly(contours.size());
	vector<Rect> boundRect(contours.size());
	vector<Point2f>center(contours.size());
	vector<float>radius(contours.size());

	for (int i = 0; i < contours.size(); i++)
	{
		approxPolyDP(Mat(contours[i]), contours_poly[i], 3, true);
		boundRect[i] = boundingRect(Mat(contours_poly[i]));
		minEnclosingCircle((Mat)contours_poly[i], center[i], radius[i]);

	}

	/// Draw polygonal contour + bonding rects + circles
	Mat drawing = Mat::zeros(threshold_output.size(), CV_8UC3);
	for (int i = 0; i< contours.size(); i++)
	{
		if (boundRect[i].area() < 100){
			Scalar color = Scalar(0, 0, 255);
			drawContours(drawing, contours_poly, i, color, 1, 8, vector<Vec4i>(), 0, Point());
			rectangle(drawing, boundRect[i].tl(), boundRect[i].br(), color, 2, 8, 0);
			circle(drawing, center[i], (int)radius[i], color, 2, 8, 0);
		}

	}

	/// Show in a window
	namedWindow("Contours", CV_WINDOW_AUTOSIZE);
	imshow("Contours", drawing);




	
	
	
	return ret;
	

}

cv::Mat USHandler::getImage(const char* path){
	return cvLoadImage(path, CV_LOAD_IMAGE_GRAYSCALE);

}