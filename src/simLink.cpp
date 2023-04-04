#include <RKD/simLink.h>

using namespace RKD;

void simLink::print(const int i){

	for (int n_tab = 0; n_tab < i; ++n_tab)
		std::cout << " ";

	std::cout << "- " << name_ << " " << std::endl;

	for (int n_joint = 0; n_joint < child_jointPtr_.size(); ++n_joint){
		if (child_jointPtr_[n_joint])
			child_jointPtr_[n_joint]->print(i + 1);
	}
}

simLink::simLink(const simLink& in){

	parent_jointPtr_ = in.parent_jointPtr_;
	child_jointPtr_ = in.child_jointPtr_;

	name_ = in.name_;
	link_pos_ = in.link_pos_;
	link_rot_ = in.link_rot_;
	
	linkActor_ = in.linkActor_;
	linkFrameActor_ = in.linkFrameActor_;
}

void simLink::makeCopy(const simLink& in){

	this->name_ = in.name_;
	this->link_pos_ = in.link_pos_;
	this->link_rot_ = in.link_rot_;
	this->opacity_ = in.opacity_;

	
	if (in.linkActor_){
		auto polyData = vtkPolyData::SafeDownCast(in.linkActor_->GetMapper()->GetInput());

		// Create mapper
		auto mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
		mapper->SetInputData(polyData);

		// Create Actor
	    this->linkActor_ = vtkSmartPointer<vtkActor>::New();
	    this->linkActor_->SetMapper(mapper);
	

	}else
		this->linkActor_ = in.linkActor_;


	if(in.linkFrameActor_){

		this->linkFrameActor_ = vtkSmartPointer<vtkAxesActor>::New();

		// The axes are positioned with a user transform
		this->linkFrameActor_->SetUserTransform(in.linkFrameActor_->GetUserTransform ());
		this->linkFrameActor_->SetTotalLength (.05, .05, .05);
		this->linkFrameActor_->AxisLabelsOff();

	}
	else
		this->linkFrameActor_ = in.linkFrameActor_;

}

void simLink::makeCopyLink(const std::shared_ptr<simLink>& l, std::shared_ptr<simLink>& lcopy, const std::shared_ptr<simJoint>& jcopyParent, vtkSmartPointer<vtkRenderer>& renderer, const double opacity){

	if(!l)
		return;

	lcopy->makeCopy(*l);
	lcopy->opacity_ = opacity;
	lcopy->AddActor(renderer);
	
	if (jcopyParent)
		lcopy->parent_jointPtr_ = jcopyParent;

	for (int i = 0; i < l->child_jointPtr_.size(); ++i){

		lcopy->child_jointPtr_.push_back(std::make_shared<simJoint>());

		simJoint::makeCopyJoint(l->child_jointPtr_[i], lcopy->child_jointPtr_[i], lcopy, renderer, opacity);
	}

	return;
}


void simLink::AddActor(vtkSmartPointer<vtkRenderer>& renderer){

	if (this->linkActor_){
		
		auto property = this->linkActor_->GetProperty();
		property->SetOpacity(this->opacity_);
		this->linkActor_->SetProperty(property);

		renderer->AddActor(this->linkActor_);
	}

	if (this->linkFrameActor_)
		renderer->AddActor(this->linkFrameActor_);
}