#ifndef TIMECOMPENSATE_H
#define TIMECOMPENSATE_H

#include "Definitions.h"
#include <stdio.h>
#include <iostream>
#include "../initializers/initializers.h"
//#include "../../pose_node_process.h"

class TimeCompensate
{

public:
	enum SamplingMethod
	{
		NONE = 0,
		SAMPLEANDHOLD = 1
	};

	TimeCompensate();
	~TimeCompensate();
    
	void set_samplingmethod(SamplingMethod method_)
	{
		method = method_;
	}
	void set_bufferlimit(int v)
	{
		buffer_limit = v;
	}
	Extended_Signal sample(Basic_Signal raw,Extended_Signal signal,double run_time);

private:
	SamplingMethod method;
	int buffer_limit;
};

#endif
