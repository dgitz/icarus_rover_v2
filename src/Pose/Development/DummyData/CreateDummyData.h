#ifndef CREATEDUMMYDATA_H
#define CREATEDUMMYDATA_H

#include <stdio.h>
#include <iostream>
#include "../../PoseModel/Definitions/PoseDefinitions.h"
#include "Definitions.h"
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
	//Printing Functions


private:
};

#endif
