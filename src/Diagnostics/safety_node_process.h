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

#define PIN_ESTOP 3 //ControlModule 40-pin Connector
using std::string;
using namespace std;
class SafetyNodeProcess
{
public:
	SafetyNodeProcess();
	~SafetyNodeProcess();    
    void init(std::string name,icarus_rover_v2::diagnostic diag);
    icarus_rover_v2::diagnostic new_devicemsg(icarus_rover_v2::device newdevice);
    bool is_initialized() { return initialized; }
    icarus_rover_v2::estop get_estop() { return estop; }
    icarus_rover_v2::diagnostic new_pinmsg(icarus_rover_v2::pin msg);
    bool get_ready_to_arm() { return ready_to_arm; }
    
protected:
private:
    icarus_rover_v2::diagnostic diagnostic;
    icarus_rover_v2::estop estop;
    bool initialized;
    std::string hostname;
    icarus_rover_v2::device mydevice;
};
#endif
