#pragma once
#include <array>
class JointAngles
{
public:
	JointAngles();
	JointAngles(std::array<double,6> angles);
	~JointAngles();

	double getBaseAngle() const;
	double getShoulderAngle() const;
	double getElbowAngle() const;
	double getWrist1Angle() const;
	double getWrist2Angle() const;
	double getWrist3Angle() const;
	std::array<double, 6> getArray();
	void setArray(std::array<double, 6> arr);
	void setAngles(JointAngles angles);
	int size() const;
	
	double operator[](int i) const {
		return angles_[i];
	}
	void operator=(JointAngles T) {
		angles_ = T.getArray();
	}
private:
	std::array<double,6> angles_;
};

