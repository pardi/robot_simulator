#include <RKD/robotSimulator.h>
#include <boost/thread.hpp>
#include <chrono>
#include <RKD/simServerSocket.h>
#include <RKD/simClientSocket.h>

#include <nlohmann/json.hpp>

// for convenience
using json = nlohmann::json;

using namespace RKD;

int main (int, char*[])

{

	std::cout << "------------[START Server]------------" << std::endl;
	


	robotSimulator rSim;
	// rSim.generateURDFRobot("/home/solaris/PhD/libraries/vtk_simulator/urdf/panda_arm.urdf");
  	
	// Eigen::VectorXd q(7);

	// q << 0, 0, 0, 0, M_PI/2, 0, 0;

	// rSim.updateRobot(q);

	// std::cout << "XXX" << std::endl;

	
    // Define lambda function
  // 	auto f = [&rSim]() {

	 //    std::this_thread::sleep_for (std::chrono::seconds(5));  


  // 		Eigen::VectorXd q(7);

  // 		q << 0, M_PI/2, -M_PI/2, 0, M_PI/2, 0, M_PI/2;

		// rSim.updateRobot(q);
		// std::cout << "XXX" << std::endl;

	 //    std::this_thread::sleep_for (std::chrono::seconds(5));  


  // 	};

	// // Start thread
	// boost::thread thread;
	// thread = boost::thread(boost::bind<void>(f)); 

	// rSim.InteractorStart();




	return EXIT_SUCCESS;

}
