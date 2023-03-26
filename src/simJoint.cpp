#include <RKD/simJoint.h>

using namespace RKD;

void simJoint::print(const int i){

	for (int n_tab = 0; n_tab < i; ++n_tab)
		std::cout << " ";
	
	std::cout << "> " << name_ << std::endl;

	if (child_linkPtr_)
		child_linkPtr_->print(i + 1);
}


simJoint::simJoint(const simJoint& in){

	type_ = in.type_;

	parent_linkPtr_ = in.parent_linkPtr_;
	child_linkPtr_ = in.child_linkPtr_;

	name_ = in.name_;
	joint_pos_ = in.joint_pos_;
	joint_rot_ = in.joint_rot_;

	lower_limit_ = in.lower_limit_;
	upper_limit_ = in.upper_limit_;

}

void simJoint::makeCopy(const simJoint& in){

	this->type_ = in.type_;

	this->name_ = in.name_;
	this->joint_pos_ = in.joint_pos_;
	this->joint_rot_ = in.joint_rot_;

	this->lower_limit_ = in.lower_limit_;
	this->upper_limit_ = in.upper_limit_;

}


void simJoint::makeCopyJoint(const std::shared_ptr<simJoint>& j, std::shared_ptr<simJoint>& jcopy, const std::shared_ptr<simLink>& lcopyParent, vtkSmartPointer<vtkRenderer>& renderer, const double& opacity){

	if(!j)
		return;

	jcopy->makeCopy(*j);

	jcopy->parent_linkPtr_ = lcopyParent;

	jcopy->child_linkPtr_ = std::make_shared<simLink>();
	simLink::makeCopyLink(j->child_linkPtr_, jcopy->child_linkPtr_, jcopy, renderer, opacity);

}


