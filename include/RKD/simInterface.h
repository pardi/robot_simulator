// Author: Tommaso Pardi
// Date: 07/03/2020

#ifndef _simInterface_header_
#define _simInterface_header_

// Socket Lib
#include <iostream>
#include <RKD/simClientSocket.h>
#include <nlohmann/json.hpp>
#include <Eigen/Dense>
#include <chrono>
#include <thread>

// for convenience
using json = nlohmann::json;

namespace RKD{

class simInterface : public simClientSocket{

public:
	simInterface(){}
	~simInterface(){}

  	// Update the robot
	bool updateRobot(const Eigen::VectorXd&);
	// Generate URDF robot
	bool generateURDFRobot(const std::string&);
	// Load PCD
	bool loadPCD(const std::string&, const Eigen::Matrix4f transf = Eigen::Matrix4f::Identity());
	// Draw Line
	bool drawLine(const std::vector<Eigen::VectorXd>&, double* colour = new double[3]{1.0, 1.0, 1.0});
	// Draw Ellipsoid - parametrisation: ZYX
	bool drawEllipsoid(const Eigen::Vector3d&, const Eigen::Vector3d&, const Eigen::Vector3d&, const double&, double* colour = new double[3]{1.0, 1.0, 1.0});
	// Draw a ghost robot
	bool ghostRobot(const Eigen::VectorXd&, const double& opacity = 0.8);
	// Robot Trajectory
	bool robotTrajectory(const std::vector<Eigen::VectorXd>&, const double& opacity = 0.8);
	// Draw a Vector
	bool drawVector(const Eigen::Vector3d&, const Eigen::Vector3d&, const double& scale = 0.1);
	// Draw Frame
	bool drawFrame(const Eigen::Vector3d&, const Eigen::Vector3d&);
	// Draw PointFrame
	bool drawPoint(const Eigen::VectorXd&, const double& size = 4, const double* colour = new double[3]{1.0, 1.0, 1.0});
	// Colour background
	bool setBackground(const double&, const double&, const double&);
	// Clear scene
	bool clear();

};
}


#endif

