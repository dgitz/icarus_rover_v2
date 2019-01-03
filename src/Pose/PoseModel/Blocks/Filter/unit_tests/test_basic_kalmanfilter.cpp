#include <gtest/gtest.h>

#include "../BasicKalmanFilter.h"
#include "ros/ros.h"
#include "ros/time.h"



BasicKalmanFilter* initialize()
{
	BasicKalmanFilter *timecomp;
	return timecomp;
}
TEST(BasicKalmanFilter,Initialization)
{
}
int main(int argc, char **argv){
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}

