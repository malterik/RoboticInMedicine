#pragma once
#include <array>
class JointAngles
{
public:
	JointAngles();
	JointAngles(std::array<double,6> angles);
	JointAngles(double theta_1, double theta_2, double theta_3, double theta_4, double theta_5, double theta_6);
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
	void setIndex(int index, double value);
	int size() const;
	char* toString();

	double operator[](int i) const {
		return angles_[i];
	}
	void operator=(JointAngles T) {
		angles_ = T.getArray();
	}

	double operator-(JointAngles T) {
		double distance = 0;
		for (int i = 0; i < 6; i++) {
			distance += pow(angles_[i] - T.getArray()[i],2);
		}
		return distance;
	}

private:
	std::array<double,6> angles_;
};

