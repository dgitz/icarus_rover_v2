#ifndef TRUTHNODEPROCESS_H
#define TRUTHNODEPROCESS_H

#include "Definitions.h"
#include <sys/time.h>
#include <stdio.h>
#include <iostream>
#include <ctime>
#include <fstream>
#include <string>
#include <boost/algorithm/string.hpp>
#include <boost/circular_buffer.hpp>
#include <boost/lexical_cast.hpp>
#include "icarus_rover_v2/diagnostic.h"
#include "icarus_rover_v2/device.h"
#include "icarus_rover_v2/command.h"
#include "icarus_rover_v2/pin.h"
#include "icarus_rover_v2/firmware.h"
#include "icarus_rover_v2/pose.h"
#include <std_msgs/UInt8.h>
#include <serialmessage.h>
#include "logger.h"
#include <math.h>
#include <tinyxml.h>
#include <Eigen/Dense>
#define G  9.80665
using namespace Eigen;
class TruthNodeProcess
{

public:
	struct Sensor
	{
		std::string pn;
		bool ready;
		bool new_dataavailable;
		int id;
		int seq;
		double tov;
		double last_tov;
		double last_msgtime;
		std::string name;
		double yawrate;
		double pitchrate;
		double rollrate;
		double yaw;
		double pitch;
		double roll;
		double east;
		double north;
		double elev;
		double wheelspeed;
		double groundspeed;
	};

	TruthNodeProcess();
	~TruthNodeProcess();
	icarus_rover_v2::diagnostic init(icarus_rover_v2::diagnostic indiag,std::string hostname);
	icarus_rover_v2::diagnostic update(double dt);
	icarus_rover_v2::diagnostic new_devicemsg(icarus_rover_v2::device device);
	Sensor get_sensor() { return sensor; }
	void set_mydevice(icarus_rover_v2::device v) { myDevice = v; }
	icarus_rover_v2::device get_mydevice() { return myDevice; }
	bool new_message(std::string msg);
	bool new_serialmessage(unsigned char* message,int length); //Return true: valid, false: invalid
	bool get_truthdata(icarus_rover_v2::pose *pose);
	bool get_initialized() { return initialized; }
	void set_sensorname(std::string pn,std::string name,int id)
	{
		sensor.pn = pn;
		sensor.id = id;
		sensor.name = name;
	}
	void set_verbositylevel(std::string v) { verbosity_level = v; }
    std::string get_verbositylevel() { return verbosity_level; }
    bool load_sensorinfo();
    
private:
    std::string hostname;
    std::string verbosity_level;
	SerialMessageHandler *serialmessagehandler;
	icarus_rover_v2::device myDevice;
	icarus_rover_v2::diagnostic diagnostic;
	Sensor sensor;
	bool running;
	double run_time;
	bool message_ready;
	bool initialized;

};
#endif
