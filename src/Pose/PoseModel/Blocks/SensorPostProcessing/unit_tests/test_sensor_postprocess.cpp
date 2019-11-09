#include <gtest/gtest.h>

#include "../IMUPostProcess.h"
#include "../../../../Development/DummyData/CreateDummyData.h"
#include "ros/ros.h"
#include "ros/time.h"



IMUPostProcess* initialize()
{
	IMUPostProcess *imupostprocess;
	imupostprocess = new IMUPostProcess;
	return imupostprocess;
}
TEST(IMUPostProcess,Execution)
{
	CreateDummyData dummydata_obj;
	std::vector<TimedSignal> dummydata = dummydata_obj.Create_TimedSignal(CreateDummyData::DummyDataType::SINWAVE);
	IMUPostProcess* imupostprocess = initialize();
	imupostprocess->init(dummydata.at(0).signal.name);
	for(std::size_t i = 0; i < dummydata.size(); ++i)
	{
		PostProcessedSignal output = imupostprocess->new_signal(dummydata.at(i));
		EXPECT_TRUE(output.signal_class == SIGNALCLASS_PROCESSEDSIGNAL);
	}	
}
int main(int argc, char **argv){
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}

