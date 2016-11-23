#ifndef COMMANDNODEPROCESS_H
#define COMMANDNODEPROCESS_H

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
using std::string;
using namespace std;
struct ReadyToArm
{
    std::string Device;
    std::string topic;
    bool ready_to_arm;
};
class CommandNodeProcess
{
public:


	CommandNodeProcess();
	~CommandNodeProcess();

	icarus_rover_v2::diagnostic init(icarus_rover_v2::diagnostic indiag,Logger *log,std::string hostname);
	icarus_rover_v2::diagnostic update(long dt);
	icarus_rover_v2::diagnostic new_devicemsg(icarus_rover_v2::device devicemsg);
	icarus_rover_v2::device get_mydevice() { return mydevice; }
	bool is_finished_initializing() { return all_device_info_received; }
	int get_armeddisarmed_state() { return armeddisarmed_state; 
    icarus_rover_v2::diagnostic new_readytoarmmsg(std::string topic, bool value);
    icarus_rover_v2::diagnostic init_readytoarm_list(std::vector<std::string> topics);
    icarus_rover_v2::diagnostic new_armcommandmsg(uint8_t value);
	//std::vector<icarus_rover_v2::device> get_myboards() { return myboards;

	bool set_timeout_ms(long timeout){ timeout_value_ms = timeout; return true; }
	int get_timeout_ms() { return timeout_value_ms; }
	long get_timer_ms() { return ms_timer; }
	bool reset_timer() { ms_timer = 0; timer_timeout = false; return true;}
protected:

private:

	std::string myhostname;
	bool all_device_info_received;
	icarus_rover_v2::device mydevice;
	icarus_rover_v2::diagnostic diagnostic;
	Logger *mylogger;
	/*
	Port_Info DIO_PortA;
	Port_Info DIO_PortB;
	Port_Info ANA_PortA;
	Port_Info ANA_PortB;
	*/
	long ms_timer;
	long timeout_value_ms;
    ros::Time init_time;
	bool timer_timeout;
	double time_diff(ros::Time timer_a, ros::Time timer_b);
    std::vector<ReadyToArm> ReadyToArmList;
    bool readytoarm;
	int armeddisarmed_state;
};
#endif
