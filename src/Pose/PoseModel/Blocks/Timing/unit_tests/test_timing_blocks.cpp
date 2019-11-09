#include <gtest/gtest.h>
#include "ros/ros.h"
#include "ros/time.h"
#include "../../../../Development/DummyData/CreateDummyData.h"
#include "../TimeCompensate.h"

SensorSignal get_latest_input(double log_start_time,double time,std::vector<SensorSignal> input_vector,uint64_t* current_index)
{
	SensorSignal input;
	if(*current_index == 0)
	{
		*current_index=1;
	}
	if((log_start_time+time) > input_vector.at(*current_index).signal.tov)
	{
		uint64_t new_index = *current_index;
		while((log_start_time+time) > input_vector.at(new_index).signal.tov)
		{
			new_index++;
			if(new_index > (input_vector.size()-1))
			{
				new_index--;
				break;
			}

		}
		if(new_index < input_vector.size())
		{
			*current_index=new_index;
		}
	}	
	input = input_vector.at(*current_index);
	return input;
}
TimeCompensate* initialize(TimeCompensate::SamplingMethod sample_method)
{
	TimeCompensate *timecomp;
	timecomp = new TimeCompensate;
	timecomp->init("Test_TimeCompensator",sample_method);
	return timecomp;
}
TEST(TimeCompensate,Execution_SinWave)
{
	CreateDummyData dummydata_obj;
	std::vector<SensorSignal> dummydata = dummydata_obj.Create_SensorSignal(CreateDummyData::DummyDataType::SINWAVE);
	double log_start_time = dummydata.at(0).signal.tov;
	{ // Sample and Hold
		TimeCompensate *tc = initialize(TimeCompensate::SamplingMethod::SAMPLEANDHOLD);
		double elap_time = 0.0;
		double time_to_run = 100.0;
		double dt = 0.05;
		std::vector<TimedSignal> out_signal;
		uint64_t current_index = 0;
		while(elap_time <= time_to_run)
		{
			uint64_t prev_index = current_index;
			SensorSignal input = get_latest_input(log_start_time,elap_time,dummydata,&current_index);
			
			TimedSignal signal = tc->new_signal(elap_time,input);
			if(prev_index != current_index)
			{
				EXPECT_TRUE(signal.signal.status == SIGNALSTATE_UPDATED);
			}
			else
			{
				EXPECT_TRUE(signal.signal.status == SIGNALSTATE_HOLD);
			}
			EXPECT_TRUE(tc->buffer_size() <= TimeCompensate::BUFFER_LENGTH);
			EXPECT_TRUE(tc->buffer_size() > 0);
			elap_time += dt;
		}
	}
	{ // Linear Extrapolate
		TimeCompensate *tc = initialize(TimeCompensate::SamplingMethod::LINEAREXTRAPOLATE);
		double elap_time = 0.0;
		double time_to_run = 35.0;
		double dt = 0.05;
		std::vector<TimedSignal> out_signal;
		uint64_t current_index = 0;
		while(elap_time <= time_to_run)
		{
			uint64_t prev_index = current_index;
			SensorSignal input = get_latest_input(log_start_time,elap_time,dummydata,&current_index);
			TimedSignal signal = tc->new_signal(elap_time,input);
			if(prev_index != current_index)
			{
				EXPECT_TRUE(signal.signal.status == SIGNALSTATE_UPDATED);
			}
			else
			{
				if(tc->buffer_size() < TimeCompensate::BUFFER_LENGTH)
				{
					EXPECT_TRUE(signal.signal.status == SIGNALSTATE_HOLD);
				}
				else
				{
					EXPECT_TRUE(signal.signal.status == SIGNALSTATE_EXTRAPOLATED);
				}
				
			}
			EXPECT_TRUE(tc->buffer_size() <= TimeCompensate::BUFFER_LENGTH);
			EXPECT_TRUE(tc->buffer_size() > 0);
			elap_time += dt;
		}
	}
}
int main(int argc, char **argv){
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}

