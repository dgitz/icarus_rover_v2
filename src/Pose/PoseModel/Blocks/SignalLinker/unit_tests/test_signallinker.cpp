#include <gtest/gtest.h>

#include "../LinearAccelerationLinker.h"
#include "ros/ros.h"
#include "ros/time.h"



LinearAccelerationLinker* initialize()
{
	LinearAccelerationLinker *linker;
	return linker;
}
TEST(LinearAccelerationLinker,Initialization)
{
}
int main(int argc, char **argv){
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}

