#include "CreateDummyData.h"
CreateDummyData::CreateDummyData()
{
}   
CreateDummyData::~CreateDummyData()
{
}
std::vector<SensorSignal> CreateDummyData::Create_SensorSignal(DummyDataType dummydata_type)
{
    std::vector<SensorSignal> signal_vector;
	SensorSignal input;
	input.signal_class = SIGNALCLASS_SENSORSIGNAL;
    input.id = 1;
	input.sequence_number = 1234;
	input.signal.name = "DummyData";
	input.signal.tov = 0.0;
	input.signal.type = SIGNALTYPE_ACCELERATION;
	input.signal.value = 0.0;
	input.signal.status = SIGNALSTATE_UPDATED;
	uint64_t counter = 0;
	double signal_time = 75.0;
	double dt = 1;
	double elap_time = 0.0;
	while(elap_time < signal_time)
	{
		input.signal.tov = elap_time;
		input.sequence_number++;
		if(dummydata_type == DummyDataType::SAWTOOTH)
		{
  			input.signal.value = counter;
		}
		else if(dummydata_type == DummyDataType::SINWAVE)
		{
			input.signal.value = sin(elap_time/4);
		}
  		signal_vector.push_back(input);
		counter = counter+1;
		if(counter > 10)
		{
			counter = 0;
		}
		double t_rand = (rand() % 100)/100.0;
  		elap_time = elap_time+dt+t_rand;
    }
    return signal_vector;
}