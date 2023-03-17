#include <RKD/simInterface.h>

using namespace RKD;

bool simInterface::updateRobot(const Eigen::VectorXd& q){

	json cmd;

	// add a number that is stored as double (note the implicit conversion of j to an object)
	cmd["fcn"] = "updateRobot";

	int i = 0;

	cmd["number_joint"] = q.size();

	std::string name_q = "q";
	
	for (int i = 0; i < q.size(); ++i)
		cmd[name_q + std::to_string(i)] = q(i);

	// Send message via Socket
	bool ret = sendMSG(cmd.dump());

	return ret;	
}

bool simInterface::generateURDFRobot(const std::string& urdf_path){

	json cmd;

	// add a number that is stored as double (note the implicit conversion of j to an object)
	cmd["fcn"] = "generateURDFRobot";

	int i = 0;

	cmd["urdf_path"] = urdf_path;

	// Send message via Socket
	bool ret = sendMSG(cmd.dump());

	return ret;	

}

bool simInterface::loadPCD(const std::string& PCD_path, const Eigen::Matrix4f transf){

	json cmd;

	cmd["fcn"] = "loadPCD";

	int i = 0;

	cmd["PCD_path"] = PCD_path;

	cmd["T00"] = transf(0, 0);
	cmd["T01"] = transf(0, 1);
	cmd["T02"] = transf(0, 2);
	cmd["T03"] = transf(0, 3);

	cmd["T10"] = transf(1, 0);
	cmd["T11"] = transf(1, 1);
	cmd["T12"] = transf(1, 2);
	cmd["T13"] = transf(1, 3);

	cmd["T20"] = transf(2, 0);
	cmd["T21"] = transf(2, 1);
	cmd["T22"] = transf(2, 2);
	cmd["T23"] = transf(2, 3);

	cmd["T30"] = transf(3, 0);
	cmd["T31"] = transf(3, 1);
	cmd["T32"] = transf(3, 2);
	cmd["T33"] = transf(3, 3);

	// Send message via Socket
	bool ret = sendMSG(cmd.dump());

	return ret;	

}

bool simInterface::drawLine(const std::vector<Eigen::VectorXd>& points, double* colour){

	json cmd;

	cmd["fcn"] = "drawLine";

	cmd["n_ponts"] = points.size();

	int i = 0;

	for (auto &p: points){
		cmd["px" + std::to_string(i)] = p(0);
		cmd["py" + std::to_string(i)] = p(1);
		cmd["pz" + std::to_string(i)] = p(2);
		i++;
	}

	cmd["r"] = colour[0];
	cmd["g"] = colour[1];
	cmd["b"] = colour[2];

	// Send message via Socket
	bool ret = sendMSG(cmd.dump());

	return ret;	
}

bool simInterface::drawEllipsoid(const Eigen::Vector3d& position, const Eigen::Vector3d& orientation, const Eigen::Vector3d& size, const double& opacity, double* colour){

	json cmd;

	cmd["fcn"] = "drawEllipsoid";

	cmd["p_x"] = position(0);
	cmd["p_y"] = position(1);
	cmd["p_z"] = position(2);

	cmd["o_x"] = orientation(0);
	cmd["o_y"] = orientation(1);
	cmd["o_z"] = orientation(2);

	cmd["s_x"] = size(0);
	cmd["s_y"] = size(1);
	cmd["s_z"] = size(2);

	cmd["opacity"] = opacity;

	cmd["r"] = colour[0];
	cmd["g"] = colour[1];
	cmd["b"] = colour[2];

	// Send message via Socket
	bool ret = sendMSG(cmd.dump());

	return ret;	
}

bool simInterface::ghostRobot(const Eigen::VectorXd& q, const double& opacity){
	
	json cmd;

	// add a number that is stored as double (note the implicit conversion of j to an object)
	cmd["fcn"] = "ghostRobot";

	int i = 0;

	cmd["number_joint"] = q.size();

	std::string name_q = "q";
	
	for (int i = 0; i < q.size(); ++i)
		cmd[name_q + std::to_string(i)] = q(i);

	cmd["opacity"] = opacity;

	// Send message via Socket
	bool ret = sendMSG(cmd.dump());

	return ret;	

}

bool simInterface::robotTrajectory(const std::vector<Eigen::VectorXd>& qvec, const double& opacity){

	json cmd;

	cmd["fcn"] = "robotTrajectory";

	if (qvec.size() == 0)
		return false;

	cmd["n_ponts"] = qvec.size();

	int number_joint = qvec[0].size();
	cmd["number_joint"] = number_joint;

	int k = 0;

	for (auto &q: qvec){
		for (int i = 0; i < q.size(); ++i)
			cmd["q" + std::to_string(i) + "_" + std::to_string(k)] = q(i);
		k++;
	}

	cmd["opacity"] = opacity;
	
	// Send message via Socket
	bool ret = sendMSG(cmd.dump());

	return ret;

}

bool simInterface::drawVector(const Eigen::Vector3d& p, const Eigen::Vector3d& v, const double& scale){

	json cmd;

	cmd["fcn"] = "drawVector";

	// Set position
	cmd["px"] = p(0);
	cmd["py"] = p(1);
	cmd["pz"] = p(2);

	// Set vector 
	cmd["vx"] = v(0);
	cmd["vy"] = v(1);
	cmd["vz"] = v(2);

	// Set scle
	cmd["scale"] = scale;

	// Send message via Socket
	bool ret = sendMSG(cmd.dump());

	return ret;
}

bool simInterface::drawFrame(const Eigen::Vector3d& p, const Eigen::Vector3d& r){

	json cmd;

	cmd["fcn"] = "drawFrame";

	// Set position
	cmd["px"] = p(0);
	cmd["py"] = p(1);
	cmd["pz"] = p(2);

	// Set orientation 
	cmd["rZ"] = r(0);
	cmd["rY"] = r(1);
	cmd["rX"] = r(2);

	// Send message via Socket
	bool ret = sendMSG(cmd.dump());

	return ret;
}


bool simInterface::drawPoint(const Eigen::VectorXd& p, const double& size, const double* colour){

	json cmd;

	cmd["fcn"] = "drawPoint";

	// Set position
	cmd["px"] = p(0);
	cmd["py"] = p(1);
	cmd["pz"] = p(2);

	// Set size
	cmd["s"] = size;

	// SetColour
	cmd["r"] = colour[0];
	cmd["g"] = colour[1];
	cmd["b"] = colour[2];

	// Send message via Socket
	bool ret = sendMSG(cmd.dump());

	return ret;
}


bool simInterface::setBackground(const double& r, const double& g, const double& b){

	json cmd;

	cmd["fcn"] = "setBackground";

	// Set position
	cmd["r"] = r;
	cmd["g"] = g;
	cmd["b"] = b;

	// Send message via Socket
	bool ret = sendMSG(cmd.dump());

	return ret;
}

bool simInterface::clear(){

	json cmd;

	cmd["fcn"] = "clear";

	// Send message via Socket
	bool ret = sendMSG(cmd.dump());

	return ret;
}