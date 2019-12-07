#ifndef CREATEDUMMYDATA_H
#define CREATEDUMMYDATA_H

#include <stdio.h>
#include <iostream>
#include "../../PoseModel/Definitions/PoseDefinitions.h"
#include <eros/signal.h>
class CreateDummyData
{

public:
	enum DummyDataType
	{
		NONE = 0,
		SAWTOOTH = 1,
		SINWAVE = 2
	};
	CreateDummyData();
    ~CreateDummyData();
	//Initialization Functions
	//Attribute Functions
	//Message Functions
	//Support Functions
    std::vector<SensorSignal> Create_SensorSignal(DummyDataType dummydata_type);
	std::vector<TimedSignal> Create_TimedSignal(DummyDataType dummydata_type);
	std::vector<std::vector<PostProcessedSignal> > Create_ProcessedSignalVector();
	//Printing Functions


private:
	std::vector<eros::signal> create_sensorsignal(bool rand_time,std::string name, uint8_t signal_type,DummyDataType dummydata_type);
};

#endif
