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
#define BATTERYLEVEL_TO_SWITCH 20.0f
#define BATTERYLEVEL_CHARGED 75.0f
#define BATTERYLEVEL_RECHARGE 50.0f
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
    bool recharging;
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
    uint8_t get_powerstate() { return power_state; }

    icarus_rover_v2::diagnostic new_batterymsg(Battery battery);
    Battery get_activebattery() { return active_battery; }
    double get_defined_batterylevel_toswitch() { return BATTERYLEVEL_TO_SWITCH; }
    double get_defined_batterylevel_charged() { return BATTERYLEVEL_CHARGED; }
    double get_defined_batterylevel_recharge() { return BATTERYLEVEL_RECHARGE; }
    void print_batteryinfo();
    Battery get_bestbattery();
    std::string map_PowerState_ToString(int v);
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
    Battery active_battery;
    uint8_t power_state;
};

#endif
