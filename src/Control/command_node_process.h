#ifndef COMMANDNODEPROCESS_H
#define COMMANDNODEPROCESS_H

#include "ros/ros.h"
#include "ros/time.h"
#include "Definitions.h"
//#include "ros/time.h"
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
#define BATTERYLEVEL_TO_RECHARGE 30.0f
#define BATTERYLEVEL_RECHARGED 95.0f
using namespace std;
struct ReadyToArm
{
    std::string Device;
    std::string topic;
    bool ready_to_arm;
};
struct PeriodicCommand
{
	icarus_rover_v2::command command;
	double rate_hz;
	double lasttime_ran;
};
class CommandNodeProcess
{
public:


	CommandNodeProcess();
	~CommandNodeProcess();

	icarus_rover_v2::diagnostic init(icarus_rover_v2::diagnostic indiag,Logger *log,std::string hostname);
	icarus_rover_v2::diagnostic update(double dt);
	icarus_rover_v2::diagnostic new_devicemsg(icarus_rover_v2::device devicemsg);
	icarus_rover_v2::device get_mydevice() { return mydevice; }
	bool is_finished_initializing() { return all_device_info_received; }
	int get_armeddisarmed_state() { return armeddisarmed_state; }
    icarus_rover_v2::diagnostic new_readytoarmmsg(std::string topic, bool value);
    icarus_rover_v2::diagnostic init_readytoarm_list(std::vector<std::string> topics);
    icarus_rover_v2::diagnostic init_PeriodicCommands(std::vector<PeriodicCommand> commands);
    icarus_rover_v2::diagnostic new_user_armcommandmsg(uint8_t value);
    icarus_rover_v2::diagnostic new_targetmsg(std::string target);
  //  int get_armcommand() { return armedcommand; }
	//std::vector<icarus_rover_v2::device> get_myboards() { return myboards;

	bool set_timeout_ms(long timeout){ timeout_value_ms = timeout; return true; }
	int get_timeout_ms() { return timeout_value_ms; }
	long get_timer_ms() { return ms_timer; }
	bool reset_timer() { ms_timer = 0; timer_timeout = false; return true;}
	icarus_rover_v2::command get_currentcommand() { return current_command; }
	int get_currentstate() { return node_state; }
	void set_batterylevel_perc(double v) { batterylevel_perc = v; }
	double get_batterylevel_perc() { return batterylevel_perc; }
	std::string map_RoverCommand_ToString(int v);
	std::vector<ReadyToArm> get_ReadyToArmList() { return ReadyToArmList; }
	std::vector<PeriodicCommand> get_PeriodicCommands() { return periodic_commands; }

protected:

private:

	std::string myhostname;
	bool all_device_info_received;
	icarus_rover_v2::device mydevice;
	icarus_rover_v2::diagnostic diagnostic;
	Logger *mylogger;
	long ms_timer;
	long timeout_value_ms;
   // ros::Time init_time;
	struct timeval init_time;
	double run_time;
	bool timer_timeout;
	//double time_diff(ros::Time timer_a, ros::Time timer_b);
	double time_diff(struct timeval timea,struct timeval timeb);
    std::vector<ReadyToArm> ReadyToArmList;
    bool readytoarm;
	int armeddisarmed_state;
	//int armedcommand;
	int node_state;
	icarus_rover_v2::command current_command;
	icarus_rover_v2::command last_command;
	std::vector<icarus_rover_v2::command> command_history;
	double batterylevel_perc;
	std::vector<PeriodicCommand> periodic_commands;
};
#endif
