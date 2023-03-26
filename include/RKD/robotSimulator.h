// Author: Tommaso Pardi
// Date: 07/03/2020

#ifndef _robotSimulatorVTK_header_
#define _robotSimulatorVTK_header_

#define RED_COLOR new double[3]{1.0, 0.298, 0.298}
#define WHITE_COLOR new double[3]{1.0, 1.0, 1.0}

// VTK lib
#include <vtkActor.h>
#include <vtkAxesActor.h>
#include <vtkCamera.h>
#include <vtkCaptionActor2D.h>
#include <vtkNamedColors.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkSmartPointer.h>
#include <vtkSphereSource.h>
#include <vtkTextProperty.h>
#include <vtkTransform.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkBYUReader.h>
#include <vtkOBJReader.h>
#include <vtkPLYReader.h>
#include <vtkPolyDataReader.h>
#include <vtkSTLReader.h>
#include <vtkXMLPolyDataReader.h>
#include <vtkPlanes.h>
#include <vtkHull.h>
#include <vtkProperty.h>
#include <vtkPointSource.h>
#include <vtkCommand.h>
#include <vtkActor2D.h>
#include <vtkHull.h>
#include <vtkTextMapper.h>
#include <vtkTimerLog.h>
#include <vtksys/SystemTools.hxx>
#include <vtkCleanPolyData.h>
#include <vtkLineSource.h>
#include <vtkPoints.h>
#include <vtkCellArray.h>
#include <vtkCellData.h>
#include <vtkLine.h>
#include <vtkParametricEllipsoid.h>
#include <vtkParametricFunctionSource.h>
#include <vtkArrowSource.h>
#include <vtkMath.h>
#include <vtkCallbackCommand.h>

// Boost
#include <boost/thread.hpp>
// URDF
#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>
// Eigen
#include <Eigen/Dense>
// STD
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <cstddef>

// Json
#include <nlohmann/json.hpp>
// RKD lib
#include <RKD/simLink.h>
#include <RKD/simJoint.h>
#include <RKD/simServerSocket.h>
#include <RKD/simvtkTimerCallback.h>
#include <RKD/simPointCloud.h>
#include <RKD/simInterface.h>

// for convenience
using json = nlohmann::json;

namespace RKD{


class simRobot{

public:	
	simRobot():joint_number_(0){
	}

	~simRobot(){}

	void clear(){
		rootLinkPtr_.reset();
	}

	void print(){

		rootLinkPtr_->print();
	}

	void makeCopy(const simRobot& r, vtkSmartPointer<vtkRenderer>& renderer, double opacity = 1.0){

		this->joint_number_ = r.joint_number_;

		this->rootLinkPtr_ = std::make_shared<simLink>(opacity);

		simLink::makeCopyLink(r.rootLinkPtr_, rootLinkPtr_, NULL, renderer, opacity);

	}


	std::shared_ptr<simLink> rootLinkPtr_;

	int joint_number_;
};

class robotSimulator: public std::enable_shared_from_this<robotSimulator>, public simServerSocket{

public:

	// Constructor
	robotSimulator();
	// Destructor
	~robotSimulator(){
		th_.interrupt();
	}

protected:
	
	// Update the robot
	void updateRobot(const Eigen::VectorXd&, const simRobot&);
	// Generate URDF robot
	bool generateURDFRobot(const std::string&);
	// Start the interaction loop
  	void InteractorStart();
	// Draw the floor
	void drawFloor(double* colour = new double[3]{0.3, 0.3, 0.3});
	// Draw a mesh
	vtkSmartPointer<vtkActor> drawMesh(const std::string&, const double*, const double*, double* colour = new double[3]{1.0, 1.0, 1.0});
	// Draw the plane
	void drawPlane(const double&, const double&, const double&, const double&, const double&, const double&, double* colour = new double[3]{0.3, 0.3, 0.3});
	// Create the Robot from the tree
	bool createRobotTree(urdf::ModelInterfaceSharedPtr);
	// Create each joint/link
	bool createJointLink(const urdf::LinkSharedPtr&, std::shared_ptr<simLink>&, const int, std::map<std::string, urdf::LinkSharedPtr>&, std::map<std::string, urdf::JointSharedPtr>&);
	// Create Mesh Obj
	vtkSmartPointer<vtkActor> createMeshObj(const std::string&, double* colour = new double[3]{1.0, 1.0, 1.0});
	// Get model from the URDF
  	urdf::ModelInterfaceSharedPtr getURDFModel(const std::string&);
  	// Read the mesh from file
	vtkSmartPointer<vtkPolyData> ReadPolyData(const char *);
	// Draw origin frame
	void drawOriginFrame();
	// Draw a frame
	vtkSmartPointer<vtkAxesActor> drawFrame(const double*, const double*);
	// Update Actor pose - vtkActor
	void updateActorPose(vtkSmartPointer<vtkActor>&, const Eigen::Vector3d&, const Eigen::Vector3d&);
	// Update Actor pose - vtkAxesActor
	void updateActorPose(vtkSmartPointer<vtkAxesActor>&, const Eigen::Vector3d&, const Eigen::Vector3d&);
	// Update Robot meshes
	void updateRobotMesh(const Eigen::VectorXd&, int&, const std::shared_ptr<simJoint>&, Eigen::MatrixXd);
	// Get rotation matrix
	Eigen::MatrixXd getRPYRotationMatrix(const Eigen::Vector3d&);
	// Get RPY from rotation matrix
	Eigen::Vector3d getRPYfromRotationMatrix(const Eigen::MatrixXd&);
	// Draw a ghost robot
	void ghostRobot(const Eigen::VectorXd&, const double&);
	// Get the new command from the client
	bool getCMDFromClient();
	// Select the function from the json
	void selectFunction(const json);
	// Load Point Cloud - PCD
	void loadPCD(const std::string&, const Eigen::Matrix4f transf = Eigen::Matrix4f::Identity());
	// Draw a line
	vtkSmartPointer<vtkActor> drawLine(const std::vector<Eigen::Vector3d>&, double* colour = new double[3]{1.0, 1.0, 1.0});
	// Draw a ellipsoid - parametrisation: ZYX
	vtkSmartPointer<vtkActor> drawEllipsoid(const Eigen::Vector3d&, const Eigen::Vector3d&, const Eigen::Vector3d&, const double&, double* colour = new double[3]{1.0, 1.0, 1.0});
	// Draw a robot trajectory
	void robotTrajectory(const std::vector<Eigen::VectorXd>&, const double&);
	// Draw a vector
	void drawVector(const Eigen::Vector3d&, const Eigen::Vector3d&, const double& scale = 0.1);
	// Draw a point
	void drawPoint(const Eigen::Vector3d&, const double& size = 2, double* colour = new double[3]{1.0, 1.0, 1.0} );
	// Set Background colour
	void setBackground(const double*);
	// Clear all the scene
	void clear();
	// Clear the Ghost Robots
	void clearGhostRobot();


	inline double rad2deg(const double angle){
		return angle * 180.0 / M_PI;
	}	
	
	inline double deg2rad(const double angle){
		return angle / 180.0 * M_PI;
	}	

	inline Eigen::Vector3d rad2deg(const Eigen::Vector3d& angles){
		return angles * 180.0 / M_PI;
	}	
	
	inline Eigen::Vector3d deg2rad(const Eigen::Vector3d& angles){
		return angles / 180.0 * M_PI;
	}	

	// Vars
	simRobot simRob;
	std::vector<simPointCloud> simPCs_;

	std::vector<vtkSmartPointer<vtkActor> > Actors_;

	std::vector<simRobot> simRobGhosts_;
	
	vtkSmartPointer<vtkRenderer> renderer_;
	vtkSmartPointer<vtkRenderWindow> renderWindow_;
	vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor_;

	boost::mutex muxRS_;
	boost::thread th_;

	friend class simvtkTimerCallback;

};
}


#endif