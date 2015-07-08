#include "JointAngles.h"
#include <boost/math/constants/constants.hpp>
#define PI boost::math::constants::pi<double>()
#include <iostream>

JointAngles::JointAngles()
{
	angles_[0] = 0;
	angles_[1] = -PI/2;
	angles_[2] = 0;
	angles_[3] = -PI/2;
	angles_[4] = 0;
	angles_[5] = 0;
}

JointAngles::JointAngles(std::array<double, 6> angles) {
	angles_ = angles;
}
JointAngles::JointAngles(double theta_1, double theta_2, double theta_3, double theta_4, double theta_5, double theta_6) {
	angles_[0] = theta_1;
	angles_[1] = theta_2;
	angles_[2] = theta_3;
	angles_[3] = theta_4;
	angles_[4] = theta_5;
	angles_[5] = theta_6;

}

char* JointAngles::toString() {
	char jointString[60];
	sprintf(jointString, "%lf %lf %lf %lf %lf %lf", angles_[0], angles_[1], angles_[2], angles_[3], angles_[4], angles_[5]);
	//sprintf(jointString, "Hallo na");
	std::cout << jointString << std::endl;
	return jointString;
}

JointAngles::~JointAngles()
{
}

double JointAngles::getBaseAngle() const{
	return angles_[0];
}

double JointAngles::getShoulderAngle() const{
	return angles_[1];
}

double JointAngles::getElbowAngle() const{
	return angles_[2];
}

double JointAngles::getWrist1Angle() const{
	return angles_[3];
}

double JointAngles::getWrist2Angle() const{
	return angles_[4];
}

double JointAngles::getWrist3Angle() const{
	return angles_[5];
}

std::array<double, 6> JointAngles::getArray(){
	return angles_;
}

void JointAngles::setArray(std::array<double, 6> arr) {
	angles_ = arr;
}

void JointAngles::setAngles(JointAngles angles) {
	angles_ = angles.getArray();
}

void JointAngles::setIndex(int index, double value) {
	angles_[index] = value;
}

int JointAngles::size() const{
	return angles_.size();
}