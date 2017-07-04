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
#include "icarus_rover_v2/controlgroup.h"
#include <std_msgs/UInt8.h>
#include <serialmessage.h>
#include "logger.h"
#include <math.h>
#include <tinyxml.h>
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
	double input;
};
struct InputSensor
{
	std::string type;
	uint8_t name;
	double input;
};
struct Gain
{
	std::string type;
	double P;
	double I;
	double D;
};
struct Output
{
	icarus_rover_v2::pin pin;
	std::string name;
	std::string topic;
	bool pin_info_received;
};
struct ControlGroup
{
    std::string name;
	Command command;
	InputSensor sensor;
	Output output;
	Gain gain;
	double current_error;
	double cum_error;
};
class AutoDriveNodeProcess
{
public:


	AutoDriveNodeProcess();
	~AutoDriveNodeProcess();
	icarus_rover_v2::diagnostic init(icarus_rover_v2::diagnostic indiag,std::string hostname);
	icarus_rover_v2::diagnostic update(double dt);
	icarus_rover_v2::diagnostic new_devicemsg(icarus_rover_v2::device devicemsg);
	std::vector<ControlGroup> get_controlgroups() { return controlgroups; }
	icarus_rover_v2::diagnostic new_tunecontrolgroupmsg(icarus_rover_v2::controlgroup msg);
	icarus_rover_v2::diagnostic new_sensormsg(std::string cg_name, double v);
	icarus_rover_v2::diagnostic new_commandmsg(std::string cg_name, double v);
	icarus_rover_v2::diagnostic new_armedstatemsg(uint8_t msg);
	icarus_rover_v2::diagnostic get_diagnostic() { return diagnostic; }
	std::vector<icarus_rover_v2::pin> get_controlgroup_pins(std::string topic);
	uint8_t get_armedstate() { return armed_state; }
	bool is_initialized() { return initialized; }
	bool read_ControlGroupFile();
	uint8_t get_sensor(std::string v);
	std::string get_sensorname(uint8_t v);
    
private:
	icarus_rover_v2::diagnostic diagnostic;
	std::vector<ControlGroup> controlgroups;
	uint8_t armed_state;
	bool initialized;
	
};
#endif
