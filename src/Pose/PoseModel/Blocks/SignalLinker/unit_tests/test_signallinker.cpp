#include <gtest/gtest.h>

#include "../LinearAccelerationLinker.h"
#include "ros/ros.h"
#include "ros/time.h"



LinearAccelerationLinker* initialize_linearacceleration_linker(std::string name)
{
	eros::diagnostic diag;
	LinearAccelerationLinker *linker;
	linker = new LinearAccelerationLinker;
	EXPECT_TRUE(linker->initialize_object(name,diag));
	return linker;
}
TEST(LinearAccelerationLinker,Execution)
{
	{//Initialize with 2 IMU's of data
		LinearAccelerationLinker *xacc_linker = initialize_linearacceleration_linker("X_Acc");
		{
			EXPECT_TRUE(xacc_linker->get_inputsignals().size() == xacc_linker->get_outputsignals().size());
			EXPECT_TRUE(xacc_linker->get_inputsignals().size() > 0);
		}
		LinearAccelerationLinker *yacc_linker = initialize_linearacceleration_linker("Y_Acc");
		{
			EXPECT_TRUE(yacc_linker->get_inputsignals().size() == yacc_linker->get_outputsignals().size());
			EXPECT_TRUE(yacc_linker->get_inputsignals().size() > 0);
		}
		LinearAccelerationLinker *zacc_linker = initialize_linearacceleration_linker("Z_Acc");
		{
			EXPECT_TRUE(zacc_linker->get_inputsignals().size() == zacc_linker->get_outputsignals().size());
			EXPECT_TRUE(zacc_linker->get_inputsignals().size() > 0);
		}
	}
	

}

int main(int argc, char **argv){
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}

