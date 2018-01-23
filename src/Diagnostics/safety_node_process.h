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
#include "icarus_rover_v2/estop.h"
#include "icarus_rover_v2/firmware.h"
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8.h>
#include "logger.h"
#include <math.h>
#include <sys/time.h>
#include "ros/time.h"

#define PIN_ESTOP 11 //ControlModule 40-pin Connector
using std::string;
using namespace std;
class SafetyNodeProcess
{
public:
	SafetyNodeProcess();
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


    icarus_rover_v2::estop get_estop() { return estop; }
    icarus_rover_v2::diagnostic new_pinmsg(icarus_rover_v2::pin msg);
    icarus_rover_v2::diagnostic new_pinvalue(int v);
    bool get_ready_to_arm() { return ready_to_arm; }
    int get_estop_pinnumber() { return PIN_ESTOP; }
    
protected:
private:
	std::vector<icarus_rover_v2::diagnostic> check_program_variables();

	double run_time;
	icarus_rover_v2::diagnostic diagnostic;
	icarus_rover_v2::device mydevice;
	std::string myhostname;
	bool initialized;
    bool ready;

    icarus_rover_v2::estop estop;
    bool ready_to_arm;
};
#endif
