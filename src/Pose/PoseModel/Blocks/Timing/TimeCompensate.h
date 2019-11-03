#ifndef TIMECOMPENSATE_H
#define TIMECOMPENSATE_H

#include <stdio.h>
#include <iostream>
#include "Definitions.h"
#include <eros/signal.h>
class TimeCompensate
{

public:
	const uint8_t BUFFER_LENGTH = 5;
	enum SamplingMethod
	{
		NONE = 0,
		SAMPLEANDHOLD = 1,
		EXTRAPOLATE = 2
	};
	TimeCompensate();
	~TimeCompensate();



	//Initialization Functions
	//Attribute Functions
	void set_samplingmethod(SamplingMethod method_)
	{
		method = method_;
	}
	//Message Functions
	eros::signal new_signal(ros::Time current_time,eros::signal input);
	eros::signal get_output()
	{
		eros::signal out = output;
		output.status = SIGNALSTATE_HOLD;
		return out;
	}
	//Support Functions

	//Printing Functions


private:
	struct StorageBufferElement
	{
		double timestamp;
		double value;
		uint8_t state;
	};
	double compute_timesum(std::vector<StorageBufferElement> buf,uint32_t start_index,uint32_t stop_index);
	double compute_valuesum(std::vector<StorageBufferElement> buf,uint32_t start_index,uint32_t stop_index);
	eros::signal output;
	std::vector<StorageBufferElement> buffer;
	SamplingMethod method;
	int buffer_limit;
};

#endif
