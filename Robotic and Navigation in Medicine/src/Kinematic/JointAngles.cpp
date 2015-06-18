#include "JointAngles.h"
#include <boost/math/constants/constants.hpp>
#define PI boost::math::constants::pi<double>()

JointAngles::JointAngles()
{
	angles_[0] = 0;
	angles_[1] = -PI/2;
	angles_[2] = 0;
	angles_[3] = -PI / 2;
	angles_[4] = 0;
	angles_[5] = 0;
}

JointAngles::JointAngles(std::array<double, 6> angles) {
	angles_ = angles;
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

int JointAngles::size() const{
	return angles_.size();
}
