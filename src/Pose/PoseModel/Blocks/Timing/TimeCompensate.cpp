#include "TimeCompensate.h"
TimeCompensate::TimeCompensate()
{
	method = TimeCompensate::NONE;
	output.status = SIGNALSTATE_INITIALIZING;
	valid_buffer.clear();
	name = "";
	prev_sequence_number = 0;
}
TimeCompensate::~TimeCompensate()
{
}
TimedSignal TimeCompensate::new_signal(double current_time, SensorSignal input)
{
	bool sensor_updated = false;
	if (input.sequence_number != prev_sequence_number)
	{
		sensor_updated = true;
	}
	prev_sequence_number = input.sequence_number;
	TimedSignal output;
	output.signal_class = SIGNALCLASS_TIMEDSIGNAL;
	if (sensor_updated == true)
	{
		output.signal.tov = current_time;
		output.signal.value = input.signal.value;
		output.signal.rms = input.signal.rms;
		output.signal.status = input.signal.status;
		StorageBufferElement new_element;
		new_element.time = current_time;
		new_element.value = input.signal.value;
		valid_buffer.push_back(new_element);
		if((int)valid_buffer.size() > BUFFER_LENGTH)
		{
			valid_buffer.erase(valid_buffer.begin());
		}
	}
	else
	{
		if (method == TimeCompensate::SamplingMethod::SAMPLEANDHOLD)
		{
			output.signal.status = SIGNALSTATE_HOLD;
			output.signal.tov = current_time;
			output.signal.value = input.signal.value;
			output.signal.rms = input.signal.rms;
		}
		else if(method == TimeCompensate::SamplingMethod::LINEAREXTRAPOLATE)
		{
			if(valid_buffer.size() < BUFFER_LENGTH)
			{
				output.signal.status = SIGNALSTATE_HOLD;
				output.signal.tov = current_time;
				output.signal.value = input.signal.value;
				output.signal.rms = input.signal.rms;
			}
			else
			{
				output.signal.status = SIGNALSTATE_EXTRAPOLATED;
				double t_mean = valid_buffer.at(valid_buffer.size()-1).time-valid_buffer.at(valid_buffer.size()-2).time;
				double v_mean = valid_buffer.at(valid_buffer.size()-1).value-valid_buffer.at(valid_buffer.size()-2).value;
				double m = v_mean/t_mean;
				output.signal.tov = current_time;
				output.signal.value = m*(current_time-valid_buffer.at(valid_buffer.size()-1).time) + valid_buffer.at(valid_buffer.size()-1).value;
				output.signal.rms = input.signal.rms;
			}	
		}
	}
	return output;
}