#include <RKD/simInterface.h>
#include <iostream>
#include <chrono>
#include <thread>
#include <Eigen/Dense>

using namespace RKD;

int main (int, char*[])

{

	std::cout << "------------[START Client]------------" << std::endl;
	
	simInterface simInt;

	// if(!simInt.clear())
	// 	std::cout << "Clear() error" << std::endl;

	// Load Robot

	while(!simInt.generateURDFRobot("/home/valerio/libraries/vtk_simulator/urdf/panda_arm_hand.urdf"))
		std::cout << "ERROR in loading the robot" << std::endl;

	// if(!simInt.clear())
	// 	std::cout << "Clear() error" << std::endl;
	
	// Load PointCloudc
	Eigen::Matrix4f transf = Eigen::Matrix4f::Identity();

	transf(0, 3) = .5;

	while (!simInt.loadPCD("/home/valerio/libraries/vchomp/pcd/plane.pcd", transf))
		std::cout << "ERROR in loading the PC" << std::endl;

	// Draw trajectory

	std::vector<Eigen::VectorXd> points;

	Eigen::Vector3d p = {0, 0, 0};
	points.push_back(p);
	p = {1, 0, 0};
	points.push_back(p);
	p = {2, 0, 0};
	points.push_back(p);
	p = {2, 1, 1};
	points.push_back(p);
	p = {2, 3, 1};
	points.push_back(p);
	p = {2, 0, 0};
	points.push_back(p);

	while (!simInt.drawLine(points, new double[3]{0.2, 0.8, 0.2}))
		std::cout << "ERROR in drawing the line" << std::endl;

	// std::vector<Eigen::Vector3d> points;

	// Eigen::Vector3d pos = {1, 0, 1};
	// Eigen::Vector3d orient = {90, 0, 45};
	// Eigen::Vector3d size = {0.1, 0.2, 0.1};

	// while (!simInt.drawEllipsoid(pos, orient, size, 0.8, new double[3]{0.8, 0.1, 0.1}))
	// 	std::cout << "ERROR in drawing the Ellipsoid" << std::endl;

	// // Move the robot

 // 	Eigen::VectorXd q(9);
 // 	q << 0, 0, 0, 0, 0, 0, 0, 0, 0;

 // 	for (int i = 0; i < 1000; ++i){
 // 		q(6) = 2.0 * M_PI * sin(i * M_PI / 180.0);
 // 		simInt.updateRobot(q);
 // 		std::this_thread::sleep_for (std::chrono::milliseconds(20));  
 // 	}

	// std::vector<Eigen::VectorXd> qvec;

 // 	Eigen::VectorXd qG(9);
 // 	qG << 0, M_PI / 2, 0, 0, 0, 0, 0, 0, 0;
 	
 // 	qvec.push_back(qG);

 // 	qG(0) = M_PI / 2;

 // 	qvec.push_back(qG);

	// qG(0) = M_PI;
 // 	qG(7) = 0.03;
 // 	qG(8) = -0.03;
	
	// qvec.push_back(qG); 	

	// simInt.robotTrajectory(qvec, 0.8);

	// std::this_thread::sleep_for (std::chrono::milliseconds(20));  


	// // Move the robot

 // 	Eigen::VectorXd q(9);
 // 	q << 0, 0, 0, 0, 0, 0, 0, 0, 0;

 // 	for (int i = 0; i < 1000; ++i){
 // 		q(6) = 2.0 * M_PI * sin(i * M_PI / 180.0);
 // 		simInt.updateRobot(q);
 // 		std::this_thread::sleep_for (std::chrono::milliseconds(20));  
 // 	}


	return EXIT_SUCCESS;
}
