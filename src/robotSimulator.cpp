#include <RKD/robotSimulator.h>

using namespace RKD;

robotSimulator::robotSimulator():simServerSocket(){

	/*
	*   Define the render and the render Window
	*/

	renderer_ = vtkSmartPointer<vtkRenderer>::New();
	renderWindow_ = vtkSmartPointer<vtkRenderWindow>::New();

	renderWindow_->SetWindowName("robotSimulator");
	renderWindow_->AddRenderer(renderer_);
	renderWindow_->SetSize(1280, 720);

	// Render properties
	vtkSmartPointer<vtkNamedColors> colours = vtkSmartPointer<vtkNamedColors>::New();
	renderer_->SetBackground(colours->GetColor3d("Wheat").GetData());
	renderer_->UseHiddenLineRemovalOn();

	// // Define an interactor
	renderWindowInteractor_ = vtkSmartPointer<vtkRenderWindowInteractor>::New();
	renderWindowInteractor_->SetRenderWindow(renderWindow_);
	renderWindowInteractor_->Initialize();
    renderWindowInteractor_->CreateRepeatingTimer(1);

	/*
	*   Draw floor
	*/

	double floor_clr[3] = {0.5, 0.5, 0.5};

	drawFloor(floor_clr);

	/*
	*   Draw origin frame
	*/

	drawOriginFrame();

	/*
	*   Set the camera 
	*/

	renderer_->ResetCamera();
	renderer_->GetActiveCamera()->Azimuth(0);
	renderer_->GetActiveCamera()->Elevation(-45);
	
	/*
	*   Render
	*/

  	renderWindow_->Render();

	/*
	*  Activate Timer callback
	*/

	auto callBack = vtkSmartPointer<simvtkTimerCallback>::New();
	callBack->robSimObj_ = this;
	renderWindowInteractor_->AddObserver(vtkCommand::TimerEvent, callBack);
	callBack->timerId_ = renderWindowInteractor_->CreateRepeatingTimer(100);


	/*
	*  Start interactor
	*/
	vtkSmartPointer<vtkInteractorStyleTrackballCamera> style = vtkSmartPointer<vtkInteractorStyleTrackballCamera>::New();
	renderWindowInteractor_->SetInteractorStyle( style );
	InteractorStart();
}


vtkSmartPointer<vtkPolyData> robotSimulator::ReadPolyData(const char *fileName) {
	
	vtkSmartPointer<vtkPolyData> polyData;
	std::string extension = vtksys::SystemTools::GetFilenameLastExtension(std::string(fileName));

	// Drop the case of the extension
	std::transform(extension.begin(), extension.end(), extension.begin(), ::tolower);

	if (extension == ".ply")
	{
		auto reader = vtkSmartPointer<vtkPLYReader>::New();
		reader->SetFileName (fileName);
		reader->Update();
		polyData = reader->GetOutput();
	}
	else if (extension == ".vtp")
	{
		auto reader = vtkSmartPointer<vtkXMLPolyDataReader>::New();
		reader->SetFileName (fileName);
		reader->Update();
		polyData = reader->GetOutput();
	}
	else if (extension == ".obj")
	{
		auto reader = vtkSmartPointer<vtkOBJReader>::New();
		reader->SetFileName (fileName);
		reader->Update();
		polyData = reader->GetOutput();
	}
	else if (extension == ".stl")
	{
		auto reader = vtkSmartPointer<vtkSTLReader>::New();
		reader->SetFileName (fileName);
		reader->Update();
		polyData = reader->GetOutput();
	}
	else if (extension == ".vtk")
	{
		auto reader = vtkSmartPointer<vtkPolyDataReader>::New();
		reader->SetFileName (fileName);
		reader->Update();
		polyData = reader->GetOutput();
	}
	else if (extension == ".g")
	{
		auto reader = vtkSmartPointer<vtkBYUReader>::New();
		reader->SetGeometryFileName (fileName);
		reader->Update();
		polyData = reader->GetOutput();
	}
	else
	{
		auto source = vtkSmartPointer<vtkSphereSource>::New();
		source->Update();
		polyData = source->GetOutput();
	}

	return polyData;
}


vtkSmartPointer<vtkAxesActor> robotSimulator::drawFrame(const double* position, const double* orientation){

	vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
	transform->Translate(position);

	transform->RotateZ(orientation[2]);
	transform->RotateY(orientation[1]);
	transform->RotateX(orientation[0]);


	vtkSmartPointer<vtkAxesActor> frame = vtkSmartPointer<vtkAxesActor>::New();

	// The axes are positioned with a user transform
	frame->SetUserTransform(transform);

	renderer_->AddActor(frame);

	std::cout << "[INFO] Frame added" << std::endl;

	return frame;
}

void robotSimulator::drawOriginFrame(){

	vtkSmartPointer<vtkAxesActor> origin_frame = drawFrame(new double[3]{-2.25, -2.25, 0.0}, new double[3]{0.0, 0.0, 0.0});

	origin_frame->SetTotalLength (1, 1, 1);

	renderer_->AddActor(origin_frame);

	std::cout << "[INFO] Origin added" << std::endl;
}

void robotSimulator::drawFloor(double* colour){

	double floor_minx = -2.5;
	double floor_maxx = 2.5; 
	double floor_miny = -2.5;
	double floor_maxy = 2.5;
	double floor_minz = -0.001;
	double floor_maxz = 0.001;

	drawPlane(floor_minx, floor_maxx, floor_miny, floor_maxy, floor_minz, floor_maxz, colour);

	std::cout << "[INFO] Floor added" << std::endl;
}

void robotSimulator::drawPlane(const double& plane_minx, const double& plane_maxx, const double& plane_miny, const double& plane_maxy, const double& plane_minz, const double& plane_maxz, double* colour){

	// Define the planes
	vtkSmartPointer<vtkPlanes> plane = vtkSmartPointer<vtkPlanes>::New();

	plane->SetBounds(plane_minx, plane_maxx, plane_miny, plane_maxy, plane_minz, plane_maxz);

	vtkSmartPointer<vtkTextProperty> textProperty = vtkSmartPointer<vtkTextProperty>::New();
	textProperty->SetFontSize(16);
	textProperty->SetJustificationToCentered();

	vtkSmartPointer<vtkHull> hull;
	vtkSmartPointer<vtkPolyData> pd;
	vtkSmartPointer<vtkPolyDataMapper> mapper;
	vtkSmartPointer<vtkActor> actor;
	vtkSmartPointer<vtkTextMapper> textMapper;
	vtkSmartPointer<vtkActor2D> textActor;

	hull = vtkSmartPointer<vtkHull>::New();
	hull->SetPlanes(plane);

	pd = vtkSmartPointer<vtkPolyData>::New();

	hull->GenerateHull(pd, -200, 200, -200, 200, -200, 200);

	mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	mapper->SetInputData(pd);

	actor = vtkSmartPointer<vtkActor>::New();
	actor->SetMapper(mapper);
	actor->GetProperty()->SetColor(colour);
	actor->GetProperty()->SetSpecular(0.8);
	actor->GetProperty()->SetSpecularPower(30);

	renderer_->AddActor(actor);

}

vtkSmartPointer<vtkActor> robotSimulator::createMeshObj(const std::string& mesh_path, double* colour){
	
	// Load the mesh
	auto polyData = ReadPolyData(mesh_path.c_str());

	// Create mapper
	auto mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	mapper->SetInputData(polyData);

	/*   
	*	Background colour
	*/

	auto backProp = vtkSmartPointer<vtkProperty>::New();
	backProp->SetSpecular(.6);
	backProp->SetSpecularPower(30);

	auto actor = vtkSmartPointer<vtkActor>::New();
	actor->SetMapper(mapper);
	actor->SetBackfaceProperty(backProp);
	actor->GetProperty()->SetDiffuseColor(colour);
	actor->GetProperty()->SetSpecular(.3);
	actor->GetProperty()->SetSpecularPower(30);

	renderer_->AddActor(actor);

	std::cout << "[INFO] Create mesh" << std::endl;

	return actor;
}

void robotSimulator::updateActorPose(vtkSmartPointer<vtkActor>& actor, const Eigen::Vector3d& position, const Eigen::Vector3d& orientation){

	vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
	
	// Translation valuues
	transform->Translate(position(0), position(1), position(2));

	transform->RotateZ(orientation(2));
	transform->RotateY(orientation(1));
	transform->RotateX(orientation(0));

	actor->SetUserTransform(transform);

}

void robotSimulator::updateActorPose(vtkSmartPointer<vtkAxesActor>& actor, const Eigen::Vector3d& position, const Eigen::Vector3d& orientation){

	vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
	
	// Translation valuues
	transform->Translate(position(0), position(1), position(2));

	transform->RotateZ(orientation(2));
	transform->RotateY(orientation(1));
	transform->RotateX(orientation(0));

	// The axes are positioned with a user transform
	actor->SetUserTransform(transform);

}

vtkSmartPointer<vtkActor> robotSimulator::drawMesh(const std::string& mesh_path, const double* position, const double* orientation, double* colour){

	// Load mesh
	auto polyData = ReadPolyData(mesh_path.c_str());

	// Create mapper
	auto mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	mapper->SetInputData(polyData);

	
	// *   Background colour
	

	auto backProp = vtkSmartPointer<vtkProperty>::New();
	// backProp->SetDiffuseColor(colours->GetColor3d("Banana").GetData());
	backProp->SetSpecular(.6);
	backProp->SetSpecularPower(30);

	auto actor = vtkSmartPointer<vtkActor>::New();
	actor->SetMapper(mapper);
	actor->SetBackfaceProperty(backProp);
	actor->GetProperty()->SetDiffuseColor(colour);
	actor->GetProperty()->SetSpecular(.3);
	actor->GetProperty()->SetSpecularPower(30);

	vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
	transform->Translate(position);

	transform->RotateX(orientation[0]);
	transform->RotateY(orientation[1]);
	transform->RotateZ(orientation[2]);

	actor->SetUserTransform(transform);

	renderer_->AddActor(actor);

	std::cout << "[INFO] Mesh added" << std::endl;

	return actor;
}


vtkSmartPointer<vtkActor> robotSimulator::drawLine(const std::vector<Eigen::Vector3d>& points, double* colour){

	// Create points object
	vtkSmartPointer<vtkPoints> objPoints = vtkSmartPointer<vtkPoints>::New();


	for (auto &point: points){
		double v_point[3];
		v_point[0] = point(0);
		v_point[1] = point(1);
		v_point[2] = point(2);

		objPoints->InsertNextPoint(v_point);
	}

	// Create line object
	vtkSmartPointer<vtkCellArray> objLines = vtkSmartPointer<vtkCellArray>::New();

	for(unsigned int i = 0; i < points.size() - 1; i++)
	{
		vtkSmartPointer<vtkLine> line =	vtkSmartPointer<vtkLine>::New();

		line->GetPointIds()->SetId(0, i);
		line->GetPointIds()->SetId(1, i + 1);
		objLines->InsertNextCell(line);
	}

	// Create a polydata to store everything in
	vtkSmartPointer<vtkPolyData> linesPolyData = vtkSmartPointer<vtkPolyData>::New();

	// Add the points to the dataset
	linesPolyData->SetPoints(objPoints);

	// Add the lines to the dataset
	linesPolyData->SetLines(objLines);

	// Setup actor and mapper
	vtkSmartPointer<vtkPolyDataMapper> mapper =	vtkSmartPointer<vtkPolyDataMapper>::New();

	mapper->SetInputData(linesPolyData);

	vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
	actor->SetMapper(mapper);
	actor->GetProperty()->SetLineWidth(2);
	actor->GetProperty()->SetColor(colour);

	renderer_->AddActor(actor);

	std::cout << "[INFO] Line added" << std::endl;

	Actors_.push_back(actor);

	return actor;
}

vtkSmartPointer<vtkActor> robotSimulator::drawEllipsoid(const Eigen::Vector3d& position, const Eigen::Vector3d& orientation, const Eigen::Vector3d& size, const double& opacity, double* colour){

	// Create Object Ellipsoid
	vtkSmartPointer<vtkParametricEllipsoid> objEllips = vtkSmartPointer<vtkParametricEllipsoid>::New();

	objEllips->SetXRadius(size(0));
	objEllips->SetYRadius(size(1));
	objEllips->SetZRadius(size(2));

	vtkSmartPointer<vtkParametricFunctionSource> parametricFunctionSource = vtkSmartPointer<vtkParametricFunctionSource>::New();

	parametricFunctionSource->SetParametricFunction(objEllips);
	parametricFunctionSource->Update();

	vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	mapper->SetInputConnection(parametricFunctionSource->GetOutputPort());

	vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
	actor->SetMapper(mapper);
	actor->GetProperty()->SetColor(colour);
	actor->GetProperty()->SetOpacity(opacity);

	renderer_->AddActor(actor);

	updateActorPose(actor, position, orientation);

	std::cout << "[INFO] Ellipsoid added" << std::endl;

	Actors_.push_back(actor);

	return actor;

}

urdf::ModelInterfaceSharedPtr robotSimulator::getURDFModel(const std::string& urdf_path){
  
	std::ifstream urdf_file(urdf_path); //taking file as inputstream
	std::string urdf_str;

	urdf::ModelInterfaceSharedPtr model; 

	if(urdf_file) {
		std::ostringstream ss;

		ss << urdf_file.rdbuf(); 

		urdf_str = ss.str();

		model = urdf::parseURDF(urdf_str);
	}
	else
		std::cout << "[ERROR] The URDF file does not exist!!!" << std::endl;

	return model;
}


bool robotSimulator::createRobotTree(urdf::ModelInterfaceSharedPtr model){


	// Get links and joints in the chain
	std::map<std::string, urdf::LinkSharedPtr> model_links = model->links_;
	std::map<std::string, urdf::JointSharedPtr> model_joints = model->joints_;


	std::map<std::string, urdf::LinkSharedPtr>::iterator it = model_links.begin();

	urdf::LinkSharedPtr root;
	
	do{ 
		if(!it->second->parent_joint){
			root = it->second;
			simRob.rootLinkPtr_ = std::make_shared<simLink>();

			// Store data
			simRob.rootLinkPtr_->name_ = root->name;

		}
		++it;
	}while(it != model_links.end() && !simRob.rootLinkPtr_);

	if (!simRob.rootLinkPtr_)
		return false;

	bool status = true;
	
	if (root->child_joints.size() != 0){

		// Get Link actor
		if (root->visual){

			// Get the mesh
			urdf::MeshSharedPtr mesh = urdf::dynamic_pointer_cast<urdf::Mesh>(root->visual->geometry);	

			// Draw the mesh
			simRob.rootLinkPtr_->linkActor_ = createMeshObj("../" + mesh->filename);	 

			simRob.rootLinkPtr_->link_pos_(0) = root->visual->origin.position.x;
			simRob.rootLinkPtr_->link_pos_(1) = root->visual->origin.position.y;
			simRob.rootLinkPtr_->link_pos_(2) = root->visual->origin.position.z;
			root->visual->origin.rotation.getRPY(simRob.rootLinkPtr_->link_rot_(0), simRob.rootLinkPtr_->link_rot_(1), simRob.rootLinkPtr_->link_rot_(2));

		}
		else{
			// Draw the mesh
			simRob.rootLinkPtr_->linkFrameActor_ = drawFrame(new double[3]{0.0, 0.0, 0.0}, new double[3]{0.0, 0.0, 0.0});
			simRob.rootLinkPtr_->linkFrameActor_->SetTotalLength (.05, .05, .05);
			simRob.rootLinkPtr_->linkFrameActor_->AxisLabelsOff();
		}

		
		for (int joint_idx = 0; joint_idx < root->child_joints.size(); ++joint_idx)
			status = status && createJointLink(root, simRob.rootLinkPtr_, joint_idx, model_links, model_joints);

		// Check for errors
		if (!status)
			return false;
	}

	return true;

}

bool robotSimulator::createJointLink(const urdf::LinkSharedPtr& link_parent_ptr, std::shared_ptr<simLink>& simLinkParentPtr, const int i, std::map<std::string, urdf::LinkSharedPtr>& model_links, std::map<std::string, urdf::JointSharedPtr>& model_joints){

	/*
	* --------------- JOINT --------------- 
	*/

	// Create new Joint
	urdf::JointSharedPtr joint_ptr;
	std::shared_ptr<simJoint> simJointPtr = std::make_shared<simJoint>();

	// Assign urdf joint to a pointer
	joint_ptr = link_parent_ptr->child_joints[i];


	// Store data in simJoint pointer
	simJointPtr->name_ = joint_ptr->name;
	simJointPtr->joint_pos_(0) = joint_ptr->parent_to_joint_origin_transform.position.x;
	simJointPtr->joint_pos_(1) = joint_ptr->parent_to_joint_origin_transform.position.y;
	simJointPtr->joint_pos_(2) = joint_ptr->parent_to_joint_origin_transform.position.z;

	joint_ptr->parent_to_joint_origin_transform.rotation.getRPY(simJointPtr->joint_rot_(0), simJointPtr->joint_rot_(1), simJointPtr->joint_rot_(2));

	switch(joint_ptr->type){
		case urdf::Joint::PRISMATIC:{
			simJointPtr->type_ = simJoint::PRISMATIC;
			simRob.joint_number_++;
		}break;
		case urdf::Joint::REVOLUTE:{
			simJointPtr->type_ = simJoint::REVOLUTE;
			simRob.joint_number_++;
		}break;
		default: { //FIXED:
			simJointPtr->type_ = simJoint::FIXED;
		}
	}

	// Store Limits for the joint
	if (simJointPtr->type_ != simJoint::FIXED){
		simJointPtr->lower_limit_ =  joint_ptr->limits->lower;
		simJointPtr->upper_limit_ =  joint_ptr->limits->upper;
	}

	// Store parent link in simJoint pointer
	simJointPtr->parent_linkPtr_ = simLinkParentPtr;

	// Connect parent Link with the new Joint
	// simLinkParentPtr->child_jointPtr_[i] = simJointPtr;
	simLinkParentPtr->child_jointPtr_.push_back(simJointPtr);

	// No link after a joint!!!
	if (joint_ptr->child_link_name.empty()){
		std::cout << "[ERROR] No Link after a joint!" << std::endl;
		return false;
	}


	/*
	* --------------- LINK --------------- 
	*/
	urdf::LinkSharedPtr link_child_ptr;
	std::shared_ptr<simLink> simLinkChildPtr = std::make_shared<simLink>();

	// Connect child link with the parent joint
	simLinkChildPtr->parent_jointPtr_ = simJointPtr;

	// Assign urdf link to the joint's child pointer
	link_child_ptr = model_links[joint_ptr->child_link_name.c_str()];

	// Store data in simLink pointer
	simLinkChildPtr->name_ = link_child_ptr->name;

	// Connect parent joint with the child link
	simJointPtr->child_linkPtr_ = simLinkChildPtr;

	// Get Link actor
	if (link_child_ptr->visual){

		// Get the mesh
		urdf::MeshSharedPtr mesh = urdf::dynamic_pointer_cast<urdf::Mesh>(link_child_ptr->visual->geometry);	

		// Draw the mesh
		simLinkChildPtr->linkActor_ = createMeshObj("../" + mesh->filename);	

		// Get Link position and orientation
		simLinkChildPtr->link_pos_(0) = link_child_ptr->visual->origin.position.x;
		simLinkChildPtr->link_pos_(1) = link_child_ptr->visual->origin.position.y;
		simLinkChildPtr->link_pos_(2) = link_child_ptr->visual->origin.position.z;
		link_child_ptr->visual->origin.rotation.getRPY(simLinkChildPtr->link_rot_(0), simLinkChildPtr->link_rot_(1), simLinkChildPtr->link_rot_(2)); 
	}
	else{
		// Draw the mesh
		simLinkChildPtr->linkFrameActor_ = drawFrame(new double[3]{0.0, 0.0, 0.0}, new double[3]{0.0, 0.0, 0.0});
		simLinkChildPtr->linkFrameActor_->SetTotalLength (.05, .05, .05);
		simLinkChildPtr->linkFrameActor_->AxisLabelsOff();
	}


	if (link_parent_ptr->child_joints.size() == 0){
		std::cout << "NO child joints" << std::endl;
		return false;
	}
	
	bool status = true;

	for (int joint_idx = 0; joint_idx < link_child_ptr->child_joints.size(); ++joint_idx)
		status = status && createJointLink(link_child_ptr, simLinkChildPtr, joint_idx, model_links, model_joints);

	return status;

}


void robotSimulator::loadPCD(const std::string& pointcloud_path, const Eigen::Matrix4f transf){

	simPointCloud simPC;

	if (simPC.loadPCD(pointcloud_path, transf))
		renderer_->AddActor(simPC.PCLActor_);
	else
		std::cout << "[ERROR] Error Loading the PointCloud" << std::endl;
	// Update the render
  	renderWindow_->Render();

  	simPCs_.push_back(simPC);

	std::cout << "[INFO] loaded PCD" << std::endl;  	
}


Eigen::MatrixXd robotSimulator::getRPYRotationMatrix(const Eigen::Vector3d& rpy){

	Eigen::AngleAxisd rollAngle(rpy(0), Eigen::Vector3d::UnitX());
	Eigen::AngleAxisd yawAngle(rpy(1), Eigen::Vector3d::UnitY());
	Eigen::AngleAxisd pitchAngle(rpy(2), Eigen::Vector3d::UnitZ());

	Eigen::Quaternion<double> q = rollAngle * yawAngle * pitchAngle;

	return q.matrix();

}

Eigen::Vector3d robotSimulator::getRPYfromRotationMatrix(const Eigen::MatrixXd& rotMatrix){

	Eigen::Vector3d rpy;

	rpy(0) = atan2(rotMatrix(2, 1), rotMatrix(2, 2));
	rpy(1) = atan2(-rotMatrix(2, 0), sqrt(pow(rotMatrix(2, 1), 2) + pow(rotMatrix(2, 2), 2)));
	rpy(2) = atan2(rotMatrix(1, 0), rotMatrix(0, 0));

	return rpy;

}

void robotSimulator::updateRobotMesh(const Eigen::VectorXd& q, int& i, const std::shared_ptr<simJoint>& simJointPtr, Eigen::MatrixXd root_T_Jnew){

	// The Joint does not exist
	if (!simJointPtr)
		return;

	std::shared_ptr<simLink> simLinkPtr = simJointPtr->child_linkPtr_;

	Eigen::Vector3d position = simJointPtr->joint_pos_ + simLinkPtr->link_pos_;
	Eigen::Vector3d orientation = simJointPtr->joint_rot_ + simLinkPtr->link_rot_;

	// Limits
	bool limit_reached = false;

	if (simJointPtr->type_ != simJoint::FIXED){
		if (q(i) < simJointPtr->lower_limit_){
			// Rotation angle, configuration
			orientation(2) += simJointPtr->lower_limit_;
			limit_reached = true;
		}else{
			if (q(i) > simJointPtr->upper_limit_){
				// Rotation angle, configuration
				orientation(2) += simJointPtr->upper_limit_;
				limit_reached = true;
			}
			else{
				// Rotation angle, configuration
				orientation(2) += q(i);
			}
		}

		i++;
	}

	// Generate i-th transformation
	Eigen::MatrixXd Jold_T_Jnew = Eigen::MatrixXd::Zero(4, 4);
	
	Jold_T_Jnew.block<3, 1>(0, 3) = position;
	Jold_T_Jnew.block<3, 3>(0, 0) = getRPYRotationMatrix(orientation);		
	Jold_T_Jnew(3, 3) = 1;		

	// Root to Jnew
	root_T_Jnew = root_T_Jnew * Jold_T_Jnew;

	// Obtain mesh position and orientation
	Eigen::Vector3d mesh_position = root_T_Jnew.block<3, 1>(0, 3);
	Eigen::Vector3d mesh_rotation = rad2deg(getRPYfromRotationMatrix(root_T_Jnew.block<3, 3>(0, 0)));

	// Update the robot mesh
	if (simLinkPtr->linkActor_){
		if (limit_reached)
			simLinkPtr->linkActor_->GetProperty()->SetDiffuseColor(RED_COLOR);
		else
			simLinkPtr->linkActor_->GetProperty()->SetDiffuseColor(WHITE_COLOR);

		updateActorPose(simLinkPtr->linkActor_, mesh_position, mesh_rotation);
	}
	else{
		updateActorPose(simLinkPtr->linkFrameActor_, mesh_position, mesh_rotation);
	}

	for (int n_joint = 0; n_joint < simLinkPtr->child_jointPtr_.size(); ++n_joint )
		updateRobotMesh(q, i, simLinkPtr->child_jointPtr_[n_joint], root_T_Jnew);
}

void robotSimulator::clearGhostRobot(){

	
}



void robotSimulator::updateRobot(const Eigen::VectorXd& q, const simRobot& sR){

	// Print first link

	std::shared_ptr<simLink> simLinkPtr = sR.rootLinkPtr_; 
	std::shared_ptr<simJoint> simJointPtr; 

	Eigen::Vector3d position = {0, 0, 0};
	Eigen::Vector3d orientation = {0, 0, 0};
	
	updateActorPose(simLinkPtr->linkActor_, position, orientation);

	Eigen::MatrixXd root_T_Jnew(4, 4);
	root_T_Jnew << 	1, 0, 0, 0,
					0, 1, 0, 0,
					0, 0, 1, 0,
					0, 0, 0, 1;

	// Update the robot configuration
	int i = 0;
	for (int n_joint = 0; n_joint < simLinkPtr->child_jointPtr_.size(); ++n_joint )
		updateRobotMesh(q, i, simLinkPtr->child_jointPtr_[n_joint], root_T_Jnew);


}


void robotSimulator::ghostRobot(const Eigen::VectorXd& q, const double& opacity){

	simRobot simGhost;
	simGhost.makeCopy(simRob, renderer_, opacity);

	updateRobot(q, simGhost);

	simRobGhosts_.push_back(simGhost);
	
	std::cout << "[INFO] Ghost Robot added" << std::endl;

}

void robotSimulator::robotTrajectory(const std::vector<Eigen::VectorXd>& qvec, const double& opacity){

	for(auto &q: qvec){
		ghostRobot(q, opacity);
	}

	std::cout << "[INFO] Robot Trajectory added" << std::endl;
}



void robotSimulator::InteractorStart(){
	
  	renderWindowInteractor_->Start();  
}


bool robotSimulator::generateURDFRobot(const std::string& urdf_path){

	urdf::ModelInterfaceSharedPtr model = getURDFModel(urdf_path);

	if (!model)
		return false;

	bool status;
	status = createRobotTree(model);

	if (!status)
		return false;

	// Set the robot
	Eigen::VectorXd q = Eigen::VectorXd::Zero(simRob.joint_number_);
	
	updateRobot(q, simRob);

	std::cout << "[INFO] Robot Model generated" << std::endl;

}

void robotSimulator::drawVector(const Eigen::Vector3d& p, const Eigen::Vector3d& v, const double& scale){
  
	//Create an arrow.
	vtkSmartPointer<vtkArrowSource> arrowSource = vtkSmartPointer<vtkArrowSource>::New();

	// Rotation vector
	Eigen::Vector3d rotations;

	// Rotation on Z-axis
	double vxy_norm = sqrt(pow(v(0), 2) + pow(v(1), 2));
	rotations(0) = acos(v(0) / vxy_norm);
	
	// Rotation on Y-axis
	rotations(1) = acos(vxy_norm / v.norm());

	vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
	
	// Translation valuues
	transform->Translate(p(0), p(1), p(2));

	transform->RotateZ(rad2deg(rotations(0)));
	transform->RotateY(rad2deg(-rotations(1)));
	
	// Set Scale
	transform->Scale(scale, scale, scale);

	vtkSmartPointer<vtkNamedColors> colors = vtkSmartPointer<vtkNamedColors>::New();

	//Create a mapper and actor for the arrow
	vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
	mapper->SetInputConnection(arrowSource->GetOutputPort());
	actor->SetMapper(mapper);
	actor->GetProperty()->SetColor(colors->GetColor3d("White").GetData());
	actor->SetUserTransform(transform);

	renderer_->AddActor(actor);
	
}


void robotSimulator::clear(){

	std::cout << "CXXX" << std::endl;


	// Delete robot
	simRob.clear();

	std::cout << "XXX" << std::endl;


	// Remove all the actors from the scene
	renderer_->RemoveAllViewProps();

	std::cout << "XXX1" << std::endl;

	// Reset scene
	/*
	*   Draw floor
	*/

	double floor_clr[3] = {0.5, 0.5, 0.5};

	drawFloor(floor_clr);

	/*
	*   Draw origin frame
	*/

	drawOriginFrame();

	/*
	*   Set the camera 
	*/

	renderer_->ResetCamera();
	renderer_->GetActiveCamera()->Azimuth(0);
	renderer_->GetActiveCamera()->Elevation(-45);

	std::cout << "XXX2" << std::endl;


}

void robotSimulator::drawPoint(const Eigen::Vector3d& p, const double& size, double* colour){

	double pvec[3] = {p(0), p(1), p(2)};

	// Create a points structure
	vtkSmartPointer<vtkPoints> points =  vtkSmartPointer<vtkPoints>::New();

	// Create the topology of the point (a vertex)
	vtkSmartPointer<vtkCellArray> vertices = vtkSmartPointer<vtkCellArray>::New();

	// We need an an array of point id's for InsertNextCell.
	vtkIdType pid[1];
	pid[0] = points->InsertNextPoint(pvec);
	vertices->InsertNextCell(1, pid);

	// Create a polydata object
	vtkSmartPointer<vtkPolyData> point = vtkSmartPointer<vtkPolyData>::New();

	// Set the points and vertices we created as the geometry and topology of the polydata
	point->SetPoints(points);
	point->SetVerts(vertices);

 	// Visualise
	vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	mapper->SetInputData(point);

	vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
	actor->SetMapper(mapper);
	actor->GetProperty()->SetPointSize(size);
	actor->GetProperty()->SetColor(colour);

	renderer_->AddActor(actor);
	
}


void robotSimulator::setBackground(const double* colour){

	renderer_->SetBackground(colour[0], colour[1], colour[2]);
}


bool robotSimulator::getCMDFromClient(){
	
	json cmd_json;
	bool is_new_json = false;

	// Lock the variable
	mux_.lock();

	if (!msg_in_.empty()){

		// Parse the message and convert it to a json
		cmd_json = json::parse(msg_in_.front());

		// Remove first message
		msg_in_.pop_front();

		is_new_json = true;
	}

	mux_.unlock();

	// Check if a new json is available
	if (is_new_json)
		selectFunction(cmd_json);


	// Update the render
  	renderWindow_->Render();

}

void robotSimulator::selectFunction(const json cmd_json){

	std::string func_str = cmd_json["fcn"];

	/*
	* UPDATE ROBOT FUNCTION
	*/
	
	if (func_str.compare("updateRobot") == 0){

		if (!simRob.rootLinkPtr_){
			std::cout << "Robot not initialised yet" << std::endl;
			return;
		}

		if (cmd_json["number_joint"] != simRob.joint_number_){
			std::cout << "Number of joints mismatch! The right number is: " << simRob.joint_number_ << " Please check the tree: " << std::endl;
			simRob.print();
			return;
		}

		// Get new configuration
		Eigen::VectorXd q(simRob.joint_number_);
	
		for (int i = 0; i < simRob.joint_number_; ++i)
			q(i) = cmd_json["q" + std::to_string(i)];
			
		// Execute configuration
		updateRobot(q, simRob);

		return;
	}

	/*
	* GENERATE THE ROBOT FUNCTION
	*/

	if (func_str.compare("generateURDFRobot") == 0){

		generateURDFRobot(cmd_json["urdf_path"]);

		return;
	}

	/*
	* LOAD POINT CLOUD
	*/

	if (func_str.compare("loadPCD") == 0){

		Eigen::Matrix4f transf;

		transf << 	cmd_json["T00"], cmd_json["T01"], cmd_json["T02"], cmd_json["T03"],
					cmd_json["T10"], cmd_json["T11"], cmd_json["T12"], cmd_json["T13"],
					cmd_json["T20"], cmd_json["T21"], cmd_json["T22"], cmd_json["T23"],
					cmd_json["T30"], cmd_json["T31"], cmd_json["T32"], cmd_json["T33"];

		loadPCD(cmd_json["PCD_path"], transf);
		
		return;
	}

	/*
	* DRAW LINES
	*/

	if (func_str.compare("drawLine") == 0){

		std::vector<Eigen::Vector3d> points;

		for (int i = 0; i < cmd_json["n_ponts"]; ++i){
			Eigen::Vector3d p;
			
			p(0) = cmd_json["px" + std::to_string(i)];
			p(1) = cmd_json["py" + std::to_string(i)];
			p(2) = cmd_json["pz" + std::to_string(i)];

			points.push_back(p);
		}

		double colour[3];
		
		colour[0] = cmd_json["r"];
		colour[1] = cmd_json["g"];
		colour[2] = cmd_json["b"];
		
		drawLine(points, colour);
		
		return;
	}

	/*
	* DRAW ELLIPSOID
	*/

	if (func_str.compare("drawEllipsoid") == 0){

		Eigen::Vector3d position, orientation, size;

		position(0) = cmd_json["p_x"];
		position(1) = cmd_json["p_y"];
		position(2) = cmd_json["p_z"];

		orientation(0) = cmd_json["o_x"];
		orientation(1) = cmd_json["o_y"];
		orientation(2) = cmd_json["o_z"];

		size(0) = cmd_json["s_x"];
		size(1) = cmd_json["s_y"];
		size(2) = cmd_json["s_z"];

		double colour[3];
		
		colour[0] = cmd_json["r"];
		colour[1] = cmd_json["g"];
		colour[2] = cmd_json["b"];

		double opacity = cmd_json["opacity"];
		
		drawEllipsoid(position, orientation, size, opacity, colour);
		
		return;
	}

	/*
	* PLOT GHOST CONFIGURATION
	*/

	if (func_str.compare("ghostRobot") == 0){

		if (!simRob.rootLinkPtr_){
			std::cout << "Robot not initialised yet" << std::endl;
			return;
		}

		if (cmd_json["number_joint"] != simRob.joint_number_){
			std::cout << "Number of joints mismatch! The right number is: " << simRob.joint_number_ << " Please check the tree: " << std::endl;
			simRob.print();
			return;
		}

		// Get new configuration
		Eigen::VectorXd q(simRob.joint_number_);
	
		for (int i = 0; i < simRob.joint_number_; ++i)
			q(i) = cmd_json["q" + std::to_string(i)];

		double opacity = cmd_json["opacity"];
			
		// Execute configuration
		ghostRobot(q, opacity);

		return;
	}

	/*
	* DRAW ROBOT TRAJECTORY
	*/

	if (func_str.compare("robotTrajectory") == 0){

		if (!simRob.rootLinkPtr_){
			std::cout << "Robot not initialised yet" << std::endl;
			return;
		}

		if (cmd_json["number_joint"] != simRob.joint_number_){
			std::cout << "Number of joints mismatch! The right number is: " << simRob.joint_number_ << " Please check the tree: " << std::endl;
			simRob.print();
			return;
		}

		std::vector<Eigen::VectorXd> qvec;

		for (int k = 0; k < cmd_json["n_ponts"]; ++k){

			// Get new configuration
			Eigen::VectorXd q(simRob.joint_number_);
	
			for (int i = 0; i < simRob.joint_number_; ++i)
				q(i) = cmd_json["q" + std::to_string(i) + "_" + std::to_string(k)];

			qvec.push_back(q);
		}

		double opacity = cmd_json["opacity"];
		
		robotTrajectory(qvec, opacity);

		return;
	}

	/*
	* DRAW VECTOR
	*/

	if (func_str.compare("drawVector") == 0){
		
		Eigen::Vector3d p;
		Eigen::Vector3d v;
		double scale;

		// Get position
		p(0) = cmd_json["px"];
		p(1) = cmd_json["py"];
		p(2) = cmd_json["pz"];

		// Get vector 
		v(0) = cmd_json["vx"];
		v(1) = cmd_json["vy"];
		v(2) = cmd_json["vz"];

		// Get scle
		scale = cmd_json["scale"];

		drawVector(p, v, scale);

		return;
	}

	/*
	* DRAW FRAME
	*/

	if (func_str.compare("drawFrame") == 0){
		
		double position[3];
		double orientation[3];

		// Get position
		position[0] = cmd_json["px"];
		position[1] = cmd_json["py"];
		position[2] = cmd_json["pz"];

		// Get orientation
		orientation[0] = cmd_json["rZ"];
		orientation[1] = cmd_json["rY"];
		orientation[2] = cmd_json["rX"];

		drawFrame(position, orientation);

		return;
	}

	/*
	* SET BACKGROUND
	*/
	
	if (func_str.compare("setBackground") == 0){
		
		double colour[3];

		colour[0] = cmd_json["r"];
		colour[1] = cmd_json["g"];
		colour[2] = cmd_json["b"];

		setBackground(colour);

		return;
	}

	/*
	* DRAW POINT
	*/

	if (func_str.compare("drawPoint") == 0){

		Eigen::Vector3d p;
		
		p(0) = cmd_json["px"];
		p(1) = cmd_json["py"];
		p(2) = cmd_json["pz"];

		double size;
		
		size = cmd_json["s"];

		double colour[3];
		
		colour[0] = cmd_json["r"];
		colour[1] = cmd_json["g"];
		colour[2] = cmd_json["b"];
		
		drawPoint(p, size, colour);
		
		return;
	}

	/*
	* CLEAR
	*/

	if (func_str.compare("clear") == 0){
		
		clear();
		
		return;
	}	
}




