#ifndef TFBROADCASTERNODEPROCESS_H
#define TFBROADCASTERNODEPROCESS_H

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
#include <tf/transform_broadcaster.h>
#include <math.h>
#include <tinyxml.h>
using std::string;
using namespace std;
struct LeverArm
{
    std::string name;
    std::string reference;
    double x_m;
    double y_m; 
    double z_m;
    double roll_deg;
    double pitch_deg;
    double yaw_deg;
};
class TfBroadcasterNodeProcess
{
public:


	TfBroadcasterNodeProcess();
	~TfBroadcasterNodeProcess();

	icarus_rover_v2::diagnostic init(icarus_rover_v2::diagnostic indiag,Logger *log,std::string hostname);
	icarus_rover_v2::diagnostic update(double dt);
	icarus_rover_v2::diagnostic new_devicemsg(icarus_rover_v2::device devicemsg);
	icarus_rover_v2::diagnostic new_commandmsg(icarus_rover_v2::command commandmsg);
	icarus_rover_v2::diagnostic new_pinmsg(icarus_rover_v2::pin pinmsg);
	icarus_rover_v2::device get_mydevice() { return mydevice; }
	
	bool is_finished_initializing(){ return all_device_info_received; }
    std::vector<icarus_rover_v2::diagnostic> parse_systemfile(icarus_rover_v2::diagnostic indiag,TiXmlDocument doc);

    std::vector<LeverArm> get_leverarms() { return LeverArms; }
     
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
    
    std::vector<LeverArm> LeverArms;
    
};

#endif
