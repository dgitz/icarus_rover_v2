#ifndef INITIALIZERS_H
#define INITIALIZERS_H

#include "Definitions.h"
#include <stdio.h>
#include <stdint.h>
#include <vector>
#include <iostream>

struct Basic_Signal //Used for *_Raw and to extend to *_Signals and Sensor_Signals
{
	std::string name;
	std::string units;
	std::string type;
	std::string sensorname;
	uint8_t sensorindex;
	bool computed_signal;
	std::string sensorsource;
	double timestamp;
	double value;
	uint8_t status;
	double rms;
};

struct Extended_Signal : public Basic_Signal
{
	std::vector<double> value_buffer;
};

struct Sensor_Signal : public Extended_Signal
{
	bool initialized;
	bool compute_rms;
	double rms_temp1;

};
#endif
