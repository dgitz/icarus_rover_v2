#ifndef WEBSERVERNODEPROCESS_H
#define WEBSERVERNODEPROCESS_H

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
#include <jsonmessage.h>
#include "logger.h"
#include <math.h>
class WebServerNodeProcess
{
public:

	struct Master
	{
		std::string name;
		bool registered;
		bool info_received;
	};
	WebServerNodeProcess();
	~WebServerNodeProcess();
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
	icarus_rover_v2::diagnostic update_systemdevicelist(std::vector<icarus_rover_v2::device> list);
	std::vector<icarus_rover_v2::device> get_systemdevicelist() { return system_devicelist; }
	void new_armedstatus(uint8_t v);
	

    
private:
	std::vector<icarus_rover_v2::device> system_devicelist;
    std::vector<icarus_rover_v2::diagnostic> check_program_variables();
    
	double run_time;
	icarus_rover_v2::diagnostic diagnostic;
	icarus_rover_v2::device mydevice;
	std::string myhostname;
	bool initialized;
    bool ready;
    uint8_t armed_status;
};
#endif
