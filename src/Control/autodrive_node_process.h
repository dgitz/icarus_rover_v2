#ifndef AUTODRIVENODEPROCESS_H
#define AUTODRIVENODEPROCESS_H

#include "Definitions.h"
#include <sys/time.h>
#include <stdio.h>
#include <iostream>
#include <ctime>
#include <fstream>
#include <string>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include "icarus_rover_v2/diagnostic.h"
#include "icarus_rover_v2/device.h"
#include "icarus_rover_v2/command.h"
#include "icarus_rover_v2/pin.h"
#include "icarus_rover_v2/firmware.h"
#include <std_msgs/UInt8.h>
#include <serialmessage.h>
#include "logger.h"
#include <math.h>
#define SENSOR_UNDEFINED 0
#define SENSOR_POSEYAWRATE 1
struct Command
{
	std::string type;
	std::string topic;
	std::string name;
	int index;
	double min_value;
	double max_value;
};
struct Sensor
{
	std::string type;
	uint8_t name;
};
struct Gain
{
	std::string type;
	double P;
	double I;
	double D;
};
struct ControlGroup
{
    std::string name;
	Command command;
	Sensor sensor;
	icarus_rover_v2::pin output;
	Gain gain;
};
class AutoDriveNodeProcess
{
public:


	AutoDriveNodeProcess();
	~AutoDriveNodeProcess();
	icarus_rover_v2::diagnostic init(icarus_rover_v2::diagnostic indiag,std::string hostname);
	icarus_rover_v2::diagnostic update(double dt);
	bool read_ControlGroupFile();
    
private:
	icarus_rover_v2::diagnostic diagnostic;
	std::vector<ControlGroup> controlgroups;
	uint8_t get_sensor(std::string v);
	bool initialized;
	
};
#endif
