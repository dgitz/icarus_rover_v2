#include "CreateDummyData.h"
CreateDummyData::CreateDummyData()
{
}   
CreateDummyData::~CreateDummyData()
{
}
std::vector<eros::signal> CreateDummyData::create_sensorsignal(DummyDataType dummydata_type)
{
	std::vector<eros::signal> signal_vector;
	eros::signal signal;
	signal.name = "DummyData";
	signal.tov = 0.0;
	signal.type = SIGNALTYPE_ACCELERATION;
	signal.value = 0.0;
	signal.status = SIGNALSTATE_UPDATED;
	uint64_t counter = 0;
	double signal_time = 75.0;
	double dt = 1;
	double elap_time = 0.0;
	while(elap_time < signal_time)
	{
		signal.tov = elap_time;
		if(dummydata_type == DummyDataType::SAWTOOTH)
		{
  			signal.value = counter;
		}
		else if(dummydata_type == DummyDataType::SINWAVE)
		{
			signal.value = sin(elap_time/4);
		}
  		signal_vector.push_back(signal);
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
std::vector<TimedSignal> CreateDummyData::Create_TimedSignal(DummyDataType dummydata_type)
{
    std::vector<TimedSignal> signal_vector;
	TimedSignal input;
	input.signal_class = SIGNALCLASS_TIMEDSIGNAL;
	std::vector<eros::signal> data_vector = create_sensorsignal(dummydata_type);
	for(std::size_t i = 0; i < data_vector.size(); ++i)
	{
		input.signal = data_vector.at(i);
		input.signal_class = SIGNALCLASS_TIMEDSIGNAL;
		signal_vector.push_back(input);
	}
	return signal_vector;
}
std::vector<SensorSignal> CreateDummyData::Create_SensorSignal(DummyDataType dummydata_type)
{
    std::vector<SensorSignal> signal_vector;
	SensorSignal input;
	input.signal_class = SIGNALCLASS_SENSORSIGNAL;
	input.sequence_number = 1234;
	std::vector<eros::signal> data_vector = create_sensorsignal(dummydata_type);
	for(std::size_t i = 0; i < data_vector.size(); ++i)
	{
		input.signal = data_vector.at(i);
		input.signal_class = SIGNALCLASS_SENSORSIGNAL;
		input.sequence_number++;
		signal_vector.push_back(input);
	}
	return signal_vector;
}