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

	icarus_rover_v2::diagnostic init(icarus_rover_v2::diagnostic diag,std::string hostname);
	icarus_rover_v2::diagnostic update(double dt);
    icarus_rover_v2::diagnostic new_message_sent(uint16_t id);
    icarus_rover_v2::diagnostic new_message_recv(uint16_t id);
    std::string get_messageinfo(bool v);

protected:

private:
    void init_messages();
	std::string myhostname;
	icarus_rover_v2::diagnostic diagnostic;
    double run_time;
    std::vector<Message> messages;
};
#endif
