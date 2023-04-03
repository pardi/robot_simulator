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
private:

    const std::array<double, 3> defaultColour_{1.0, 1.0, 1.0};

    enum class cmdType{
        UPDATE_ROBOT,
        GENERATE_URDF_ROBOT,
        LOAD_PCD,
        DRAW_LINE,
        DRAW_ELLIPSOID,
        GHOST_ROBOT,
        ROBOT_TRAJECTORY,
        DRAW_VECTOR,
        DRAW_FRAME,
        DRAW_POINT,
        SET_BACKGROUND,
        CLEAR
    };

    std::unordered_map<cmdType, std::string> cmdMap_{
            {cmdType::UPDATE_ROBOT, "updateRobot"},
            {cmdType::GENERATE_URDF_ROBOT, "generateURDFRobot"},
            {cmdType::LOAD_PCD, "loadPCD"},
            {cmdType::DRAW_LINE, "drawLine"},
            {cmdType::DRAW_ELLIPSOID, "drawEllipsoid"},
            {cmdType::GHOST_ROBOT, "ghostRobot"},
            {cmdType::ROBOT_TRAJECTORY, "robotTrajectory"},
            {cmdType::DRAW_VECTOR, "drawVector"},
            {cmdType::DRAW_FRAME, "drawFrame"},
            {cmdType::DRAW_POINT, "drawPoint"},
            {cmdType::SET_BACKGROUND, "setBackground"},
            {cmdType::CLEAR, "clear"}
    };
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
	bool drawLine(const std::vector<Eigen::VectorXd>&, const std::array<double, 3> colour = defaultColour_);
	// Draw Ellipsoid - parametrisation: ZYX
	bool drawEllipsoid(const Eigen::Vector3d&, const Eigen::Vector3d&, const Eigen::Vector3d&, const double, const std::array<double, 3> colour = defaultColour_);
	// Draw a ghost robot
	bool ghostRobot(const Eigen::VectorXd&, const double opacity = 0.8);
	// Robot Trajectory
	bool robotTrajectory(const std::vector<Eigen::VectorXd>&, const double opacity = 0.8);
	// Draw a Vector
	bool drawVector(const Eigen::Vector3d&, const Eigen::Vector3d&, const double scale = 0.1);
	// Draw Frame
	bool drawFrame(const Eigen::Vector3d&, const Eigen::Vector3d&);
	// Draw PointFrame
	bool drawPoint(const Eigen::VectorXd&, const double size = 4, const std::array<double, 3> colour = defaultColour_);
	// Colour background
	bool setBackground(const double r, const double g, const double b);
	// Clear scene
	bool clear();

};
}


#endif

