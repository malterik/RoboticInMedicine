
#include <iostream>
#include "Network\TcpClient.h"
#include "Kinematic\KinematicMatrix.h"

int main(int argc, char* argv[])
{

	KinematicMatrix k;
	std::cout << k.toString() << std::endl <<  std::endl;

	TcpClient roboCom;

	roboCom.connect("192.168.56.101", 5005);
	std::cout << roboCom.read() << std::endl;
	std::cout << roboCom.command("Hello Robot") << std::endl;
	std::cout << roboCom.command("MovePTPJoints 0 -50 -100 -90 100 0") << std::endl;
	std::cout << roboCom.command("GetPositionJoints") << std::endl;

	system("Pause");

	return 0;
}