#ifndef TIMECOMPENSATE_H
#define TIMECOMPENSATE_H

#include <stdio.h>
#include <iostream>
#include "../../Definitions/PoseDefinitions.h"
#include "Definitions.h"
#include <eros/signal.h>
class TimeCompensate
{

public:
	const static uint8_t BUFFER_LENGTH = 10;
	enum SamplingMethod
	{
		NONE = 0,
		SAMPLEANDHOLD = 1,
		LINEAREXTRAPOLATE = 2
	};
	TimeCompensate();
	~TimeCompensate();



	//Initialization Functions
	void init(std::string _name,SamplingMethod _method)
	{
		name = _name;
		method = _method;
	}
	//Attribute Functions
	uint16_t buffer_size() { return (int)valid_buffer.size(); }
	std::string get_name() { return name; }
	//Message Functions
	TimedSignal new_signal(double current_time,SensorSignal input);
	/*eros::signal get_output()
	{
		eros::signal out = output;
		output.status = SIGNALSTATE_HOLD;
		return out;
	}
	*/
	//Support Functions

	//Printing Functions


private:
	struct StorageBufferElement
	{
		double time;
		double value;
	};
	std::string name;
	eros::signal output;
	std::vector<StorageBufferElement> valid_buffer;
	SamplingMethod method;
	uint64_t prev_sequence_number;
};

#endif
