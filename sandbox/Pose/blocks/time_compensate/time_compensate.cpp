#include "time_compensate.h"
TimeCompensate::TimeCompensate()
{
	method = NONE;
	buffer_limit = 0;
}   
TimeCompensate::~TimeCompensate()
{

}
Extended_Signal TimeCompensate::sample(Basic_Signal raw,Extended_Signal signal,double run_time)
{
	if(method == SAMPLEANDHOLD)
	{
		if(raw.computed_signal == false)
		{
			if(run_time > raw.timestamp)
			{
				signal.timestamp = run_time;
				signal.value = raw.value;
				signal.rms = raw.rms;
				signal.status = raw.status;
			}
			else
			{
				signal.status = SIGNALSTATE_HOLD;
			}
		}
	}
	signal.value_buffer.push_back(signal.value);
	if(signal.value_buffer.size() > buffer_limit)
	{
		signal.value_buffer.erase(signal.value_buffer.begin());
	}
	return signal;
}
