#include <gtest/gtest.h>

#include "../IMUPostProcess.h"
#include "ros/ros.h"
#include "ros/time.h"



IMUPostProcess* initialize()
{
	IMUPostProcess *imupostprocess;
	return imupostprocess;
}
TEST(IMUPostProcess,Initialization)
{
}
int main(int argc, char **argv){
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}

