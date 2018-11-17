#ifndef SAFETYNODEPROCESS_H
#define SAFETYNODEPROCESS_H

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
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8.h>
#include "logger.h"
#include <math.h>
#include <sys/time.h>
#include "ros/time.h"

using std::string;
using namespace std;
class SafetyNodeProcess
{
public:
	SafetyNodeProcess(std::string _base_node_name,std::string _node_name);
	std::string get_basenodename() { return base_node_name; }
	std::string get_nodename() { return node_name; }
	~SafetyNodeProcess();    
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


	bool get_ready_to_arm() { return ready_to_arm; }
	icarus_rover_v2::diagnostic new_armswitchmsg(std_msgs::Bool v);
	bool hat_present(icarus_rover_v2::device device);

	//Generic Hat Functions
	icarus_rover_v2::diagnostic set_hat_running(std::string devicetype,uint16_t id);
	bool is_hat_running(std::string devicetype,uint16_t id);

	//Terminal Hat Functions
	icarus_rover_v2::diagnostic set_terminalhat_initialized();
	std::vector<icarus_rover_v2::pin> get_terminalhatpins(std::string Function);
	int get_pinnumber(std::string name);
	bool set_pinvalue(std::string name,int v);

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
	bool ready_to_arm;
	bool arm_switch;
	std::vector<icarus_rover_v2::device> hats;
	std::vector<bool> hats_running;
};
#endif
