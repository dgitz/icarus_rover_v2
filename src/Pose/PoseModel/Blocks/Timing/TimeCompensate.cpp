#include "TimeCompensate.h"
TimeCompensate::TimeCompensate()
{
	method = NONE;
	buffer_limit = 0;
	output.status = SIGNALSTATE_INITIALIZING;
}   
TimeCompensate::~TimeCompensate()
{

}
icarus_rover_v2::signal TimeCompensate::new_signal(ros::Time current_time,icarus_rover_v2::signal input)
{
	if(method == SAMPLEANDHOLD)
	{

		output = input;
	}
	else if(method == EXTRAPOLATE)
	{
		if(buffer.size() == 0)
		{
			StorageBufferElement element;
			element.timestamp = input.tov.toSec();
			element.state = SIGNALSTATE_INITIALIZING;
			element.value = input.value;
			buffer.push_back(element);
			output = input;
		}
		else if(input.tov.toSec() > buffer.back().timestamp)
		{
			StorageBufferElement element;
			element.timestamp = input.tov.toSec();
			element.state = input.status;
			element.value = input.value;
			output = input;
		}
		else if(buffer.size() <= BUFFER_LENGTH)
		{
			icarus_rover_v2::signal out = input;
			out.status = SIGNALSTATE_HOLD;
			output = out;
		}
		else
		{
			double t_sum = compute_timesum(buffer,buffer.size()-BUFFER_LENGTH,buffer.size());
			double value_sum = compute_valuesum(buffer,buffer.size()-BUFFER_LENGTH,buffer.size());
			double t_mean = t_sum/(double)(BUFFER_LENGTH);
			double value_mean = value_sum/(double)(BUFFER_LENGTH);
			double B0 = ((buffer.back().timestamp - t_mean) * (buffer.back().value-value_mean))/(buffer.back().timestamp-(t_mean*t_mean));
			double B1 = value_mean-B1*t_mean;
			icarus_rover_v2::signal out;
			out.tov = current_time;
			out.value = B0 + B1*out.tov.toSec();
			out.status = SIGNALSTATE_EXTRAPOLATED;
			output = out;
		}
		if(buffer.size() > BUFFER_LENGTH)
		{
			buffer.erase(buffer.begin());
		}
	}
	return output;
}
double TimeCompensate::compute_timesum(std::vector<StorageBufferElement> buf,uint32_t start_index,uint32_t stop_index)
{
	double sum = 0.0;
	for(std::size_t i = start_index; i < stop_index; ++i)
	{
		sum += buf.at(i).timestamp;
	}
	return sum;
}
double TimeCompensate::compute_valuesum(std::vector<StorageBufferElement> buf,uint32_t start_index,uint32_t stop_index)
{
	double sum = 0.0;
	for(std::size_t i = start_index; i < stop_index; ++i)
	{
		sum += buf.at(i).value;
	}
	return sum;
}
