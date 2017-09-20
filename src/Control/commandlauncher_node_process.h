#ifndef COMMANDLAUNCHERNODEPROCESS_H
#define COMMANDLAUNCHERNODEPROCESS_H

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
#include "logger.h"
#include <math.h>
#include <tinyxml.h>
#define MAX_PROCESS_RESTARTS 5
struct ProcessCommand
{
	std::string name;
	bool initialized;
	bool running;
	std::string param_string_1;
	uint32_t param_uint32_1;
	string command_text;
	string process_name;
	uint32_t pid;
	uint32_t restart_counter;
};
struct IPMap
{
	std::string hostname;
	std::string IPAddress;
};
struct PortMap
{
	std::string name;
	uint32_t port;
};

class CommandLauncherNodeProcess
{
public:


	CommandLauncherNodeProcess();
	~CommandLauncherNodeProcess();
	icarus_rover_v2::diagnostic init(icarus_rover_v2::diagnostic indiag,std::string hostname);
	icarus_rover_v2::diagnostic update(double dt);
	double measure_timediff(struct timeval a, struct timeval b);
	bool is_ready() { return ready; }
    icarus_rover_v2::diagnostic new_devicemsg(icarus_rover_v2::device newdevice);
	icarus_rover_v2::device get_mydevice() { return mydevice; }
	bool set_camerastream(std::string address);
	std::string lookup_deviceIP(std::string hostname); //Returns "" on failure
	uint32_t lookup_port(std::string portname); //Returns 0 on failure
	std::vector<IPMap> get_ipmap() { return ipmap; }
	std::vector<PortMap> get_portmap() { return portmap; }
	std::vector<ProcessCommand> get_processlist() { return processlist; }
	bool set_processrunning(std::string name,bool running);
	bool set_processpid(std::string name,uint32_t pid);
	bool set_process_restarted(std::string name);
	std::string get_processinfo();
	
	

private:
	void init_processlist();
	bool load_configfiles();
	std::string myhostname;
	icarus_rover_v2::diagnostic diagnostic;
	double run_time;
	icarus_rover_v2::device mydevice;
    bool initialized;
    bool ready;
	std::vector<ProcessCommand> processlist;
	std::vector<IPMap> ipmap;
	std::vector<PortMap> portmap;
};
#endif
