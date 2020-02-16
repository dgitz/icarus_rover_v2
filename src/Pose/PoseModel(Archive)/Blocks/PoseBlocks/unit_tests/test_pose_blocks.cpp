#include <gtest/gtest.h>
#include "ros/ros.h"
#include "ros/time.h"
#include "../PoseAccelerationBlock.h"



PoseAccelerationBlock* initialize()
{
	PoseAccelerationBlock *poseacc_block;
	return poseacc_block;
}
TEST(PoseAccelerationBlock,Initialization)
{
}
int main(int argc, char **argv){
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}

