// Author: Tommaso Pardi
// Date: 07/03/2020

#ifndef _simJointVTK_header_
#define _simJointVTK_header_

#include <iostream>
#include <memory>

#include <Eigen/Dense>

#include <RKD/simLink.h>

namespace RKD{
class simLink;

class simJoint{

public:	
	simJoint(){}
	simJoint(const simJoint&);
	~simJoint(){
		if (std::shared_ptr<simLink> parentLnkPtr = parent_linkPtr_.lock()) {
			parentLnkPtr.reset();
		}
		child_linkPtr_.reset();
	}

	void makeCopy(const simJoint&);
	static void makeCopyJoint(const std::shared_ptr<simJoint>&, std::shared_ptr<simJoint>&, const std::shared_ptr<simLink>&, vtkSmartPointer<vtkRenderer>&, const double);

	void print(const int i = 0);
	
	enum {FIXED, PRISMATIC, REVOLUTE} type_;

	std::weak_ptr<simLink> parent_linkPtr_;
	std::shared_ptr<simLink> child_linkPtr_;

	std::string name_;
	Eigen::Vector3d joint_pos_;
	Eigen::Vector3d joint_rot_;
	double lower_limit_{0};
	double upper_limit_{0};
};
}
#endif