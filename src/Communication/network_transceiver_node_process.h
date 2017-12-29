#ifndef NETWORKTRANSCEIVERNODEPROCESS_H
#define NETWORKTRANSCEIVERNODEPROCESS_H

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
using namespace std;
struct Message
{
    uint16_t id;
    std::string name;
    uint32_t sent_counter;
    uint32_t recv_counter;
    double sent_rate;
    double recv_rate;
};
class NetworkTransceiverNodeProcess
{
public:


	NetworkTransceiverNodeProcess();
	~NetworkTransceiverNodeProcess();

	icarus_rover_v2::diagnostic init(icarus_rover_v2::diagnostic indiag,std::string hostname);
	icarus_rover_v2::diagnostic update(double dt);
	void set_diagnostic(icarus_rover_v2::diagnostic v) { diagnostic = v; }
	icarus_rover_v2::diagnostic get_diagnostic() { return diagnostic; }
	double get_runtime() { return run_time; }
	icarus_rover_v2::device get_mydevice() { return mydevice; }
	icarus_rover_v2::diagnostic new_devicemsg(icarus_rover_v2::device device);
	void set_mydevice(icarus_rover_v2::device device) { mydevice = device; }
	bool get_initialized() { return initialized; }
	std::vector<icarus_rover_v2::diagnostic> new_commandmsg(icarus_rover_v2::command cmd);
	std::vector<icarus_rover_v2::diagnostic> check_program_variables();

    icarus_rover_v2::diagnostic new_message_sent(uint16_t id);
    icarus_rover_v2::diagnostic new_message_recv(uint16_t id);
    std::string get_messageinfo(bool v);
	

protected:

private:
	double run_time;
    void init_messages();
	icarus_rover_v2::diagnostic diagnostic;
	icarus_rover_v2::device mydevice;
	std::string myhostname;
	bool initialized;
    std::vector<Message> messages;
};
#endif
