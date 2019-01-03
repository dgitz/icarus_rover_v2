#include <gtest/gtest.h>
#include "ros/ros.h"
#include "ros/time.h"
#include "../TimeCompensate.h"



TimeCompensate* initialize()
{
	TimeCompensate *timecomp;
	return timecomp;
}
TEST(TimeCompensate,Initialization)
{
}
int main(int argc, char **argv){
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}

