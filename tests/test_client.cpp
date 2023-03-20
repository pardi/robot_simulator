#include <gtest/gtest.h>
#include <RKD/simInterface.h>

using namespace RKD;

// Demonstrate some basic assertions.
TEST(RobotTest, ClassInitialisation) {
  
	simInterface simInt;

	// Load Robot

    EXPECT_FALSE(simInt.generateURDFRobot("../urdf/no.urdf"));

}