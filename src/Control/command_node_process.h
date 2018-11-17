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
    double time_since_lastrx;
};
struct PeriodicCommand
{
	icarus_rover_v2::command command;
	double rate_hz;
	double lasttime_ran;
	bool send_me;
};
class CommandNodeProcess
{
public:


	CommandNodeProcess(std::string _base_node_name,std::string _node_name);
std::string get_basenodename() { return base_node_name; }
	std::string get_nodename() { return node_name; }
	~CommandNodeProcess();

	icarus_rover_v2::diagnostic init(icarus_rover_v2::diagnostic indiag,std::string hostname);
	icarus_rover_v2::diagnostic update(double dt);
	void set_diagnostic(icarus_rover_v2::diagnostic v) { diagnostic = v; }
	icarus_rover_v2::diagnostic get_diagnostic() { return diagnostic; }
	double get_runtime() { return run_time; }
	icarus_rover_v2::device get_mydevice() { return mydevice; }
	icarus_rover_v2::diagnostic new_devicemsg(icarus_rover_v2::device device);
	void set_mydevice(icarus_rover_v2::device device) { mydevice = device; initialized = true; }
	bool get_initialized() { return initialized; }
    bool get_ready() { return ready; }
	std::vector<icarus_rover_v2::diagnostic> new_commandmsg(icarus_rover_v2::command cmd);

	int get_armeddisarmed_state() { return armeddisarmed_state; }
    void new_readytoarmmsg(std::string topic, bool value);
    icarus_rover_v2::diagnostic init_readytoarm_list(std::vector<std::string> topics);

    icarus_rover_v2::diagnostic new_user_commandmsg(icarus_rover_v2::command msg);
    icarus_rover_v2::diagnostic new_targetmsg(std::string target);
  //  int get_armcommand() { return armedcommand; }
	//std::vector<icarus_rover_v2::device> get_myboards() { return myboards;

	bool set_timeout_ms(long timeout){ timeout_value_ms = timeout; return true; }
	int get_timeout_ms() { return timeout_value_ms; }
	long get_timer_ms() { return ms_timer; }
	bool reset_timer() { ms_timer = 0; timer_timeout = false; return true;}
	icarus_rover_v2::command get_currentcommand()
	{
		icarus_rover_v2::command c = current_command;
		current_command.Command = ROVERCOMMAND_NONE;
		return c;
	}
	int get_currentstate() { return node_state; }
	void set_batterylevel_perc(double v) { batterylevel_perc = v; }
	double get_batterylevel_perc() { return batterylevel_perc; }
	std::string map_RoverCommand_ToString(int v);
	std::vector<ReadyToArm> get_ReadyToArmList() { return ReadyToArmList; }
	std::vector<icarus_rover_v2::command> get_PeriodicCommands();
	icarus_rover_v2::diagnostic get_disarmedreason();

protected:

private:
	std::string base_node_name;
	std::string node_name;
	bool unittest_running;
	std::vector<icarus_rover_v2::diagnostic> check_program_variables();
 std::vector<icarus_rover_v2::diagnostic> run_unittest();

	double run_time;
	icarus_rover_v2::diagnostic diagnostic;
	icarus_rover_v2::device mydevice;
	std::string myhostname;
	bool initialized;
    bool ready;

    icarus_rover_v2::diagnostic init_PeriodicCommands();
	long ms_timer;
	long timeout_value_ms;
	struct timeval init_time;
	bool timer_timeout;
	double time_diff(struct timeval timea,struct timeval timeb);
    std::vector<ReadyToArm> ReadyToArmList;
    bool readytoarm;
	int armeddisarmed_state;
	int node_state;
	icarus_rover_v2::command current_command;
	icarus_rover_v2::command last_command;
	std::vector<icarus_rover_v2::command> command_history;
	double batterylevel_perc;
	std::vector<PeriodicCommand> periodic_commands;
	std::string disarmed_reason;
};
#endif
