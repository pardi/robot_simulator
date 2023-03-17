// Author: Tommaso Pardi
// Date: 07/03/2020

#ifndef _simLinkVTK_header_
#define _simLinkVTK_header_

#include <iostream>
#include <memory>
#include <vector>

// VTK
#include <vtkActor.h>
#include <vtkSmartPointer.h>
#include <vtkAxesActor.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkObjectFactory.h>
#include <vtkDataObject.h>
#include <vtkRenderer.h>
#include <vtkProperty.h>





#include <Eigen/Dense>

#include <RKD/simJoint.h>

namespace RKD{

class simJoint;

class simLink{

public:	
	simLink(const double& opacity = 1.0){
		opacity_ = opacity;
	}
	simLink(const simLink&);
	~simLink(){
		parent_jointPtr_.reset();

		for (auto &childPtr: child_jointPtr_)
			childPtr.reset();
	}

	void makeCopy(const simLink&);
	static void makeCopyLink(const std::shared_ptr<simLink>&, std::shared_ptr<simLink>&, const std::shared_ptr<simJoint>&, vtkSmartPointer<vtkRenderer>&, const double&);
	void AddActor(vtkSmartPointer<vtkRenderer>&);

	void print(const int& i = 0);

	std::shared_ptr<simJoint> parent_jointPtr_;
	std::vector<std::shared_ptr<simJoint> > child_jointPtr_;

	std::string name_;
	Eigen::Vector3d link_pos_;
	Eigen::Vector3d link_rot_;

	vtkSmartPointer<vtkActor> linkActor_;
	vtkSmartPointer<vtkAxesActor> linkFrameActor_;

	double opacity_;

};


}
#endif