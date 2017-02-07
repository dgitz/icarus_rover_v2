#ifndef POWERMONITORNODEPROCESS_H
#define POWERMONITORNODEPROCESS_H

#include "ros/ros.h"
#include "ros/time.h"
#include "Definitions.h"
#include "ros/time.h"
#include <sys/time.h>
#include <stdio.h>
#include <iostream>
#include <ctime>
#include <fstream>
#include <string>
#include <boost/algorithm/string.hpp>
#include "icarus_rover_v2/diagnostic.h"
#include "icarus_rover_v2/device.h"
#include "icarus_rover_v2/command.h"
#include "icarus_rover_v2/pin.h"
#include <boost/algorithm/string.hpp>
#include "logger.h"
#include <serialmessage.h>
#include <math.h>
#include <tinyxml.h>
using std::string;
using namespace std;
struct Battery
{
    std::string name;
    int id;
    std::string chemistry;
    double voltage;
    double rated_capacity_Ah;
    double actual_capacity_Ah;
    double capacity_level_perc;
    bool active;
};
class PowerMonitorNodeProcess
{
public:


	PowerMonitorNodeProcess();
	~PowerMonitorNodeProcess();

	icarus_rover_v2::diagnostic init(icarus_rover_v2::diagnostic indiag,Logger *log,std::string hostname);
	icarus_rover_v2::diagnostic update(double dt);
	icarus_rover_v2::diagnostic new_devicemsg(icarus_rover_v2::device devicemsg);
	icarus_rover_v2::diagnostic new_commandmsg(icarus_rover_v2::command commandmsg);
	icarus_rover_v2::diagnostic new_pinmsg(icarus_rover_v2::pin pinmsg);
	icarus_rover_v2::device get_mydevice() { return mydevice; }
	
	bool is_finished_initializing(){ return all_device_info_received; }
    icarus_rover_v2::diagnostic parse_systemfile(icarus_rover_v2::diagnostic indiag,TiXmlDocument doc);	
    
    int get_batterycount() { return battery_count; }
    std::string get_connectionmethod() { return connection_method; }
    std::vector<Battery> get_batteries() { return batteries; }
protected:
private:

	std::string myhostname;
	bool all_sensor_info_received;
	bool all_device_info_received;
	icarus_rover_v2::device mydevice;
	icarus_rover_v2::diagnostic diagnostic;
	Logger *mylogger;
	long ms_timer;
	long timeout_value_ms;
    double run_time;
	bool timer_timeout;
	double time_diff(struct timeval timea, struct timeval timeb);
    
    int battery_count;
    std::string connection_method;
    std::vector<Battery> batteries;
};

#endif