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
#define PRIORITYLEVEL_HIGH 1
#define PRIORITYLEVEL_MEDIUM 2
#define PRIORITYLEVEL_LOW 3
class NetworkTransceiverNodeProcess
{
public:
	struct Message
	{
		uint16_t id;
		std::string name;
		uint32_t sent_counter;
		uint32_t recv_counter;
		double sent_rate;
		double recv_rate;
		double target_sendrate;
		uint8_t priority_level;
	};
	struct QueueElement
	{
		uint16_t id;
		std::string item;
	};
	NetworkTransceiverNodeProcess();
	~NetworkTransceiverNodeProcess();

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

	icarus_rover_v2::diagnostic new_message_sent(uint16_t id);
	icarus_rover_v2::diagnostic new_message_recv(uint16_t id);
	std::string get_messageinfo(bool v);
	void set_initialized(bool v) { initialized = v; }
	std::vector<Message> get_messages() { return messages; };
	bool push_sendqueue(uint16_t id,std::string msg);
	std::vector<QueueElement> get_sendqueue(uint8_t level);



protected:

private:
	std::vector<icarus_rover_v2::diagnostic> check_program_variables();
	Message get_messagebyid(uint16_t id)
	{
		for(std::size_t i = 0; i < messages.size(); i++)
		{
			if(messages.at(i).id == id)
			{
				return messages.at(i);
			}
		}
		Message empty;
		return empty;
	}
	bool push_sendhighqueue(uint16_t id,std::string msg)
	{
		QueueElement elem;
		elem.id = id;
		elem.item = msg;
		Message m = get_messagebyid(id);
		if(m.sent_rate <= (1.5*m.target_sendrate))
		{
			sendqueue_highpriority.push_back(elem);
		}
		return true;
	}
	bool push_sendmediumqueue(uint16_t id,std::string msg)
	{
		QueueElement elem;
		elem.id = id;
		elem.item = msg;
		Message m = get_messagebyid(id);
		if(m.sent_rate <= (1.5*m.target_sendrate))
		{
			sendqueue_mediumpriority.push_back(elem);
		}

		return true;
	}
	bool push_sendlowqueue(uint16_t id,std::string msg)
	{
		QueueElement elem;
		elem.id = id;
		elem.item = msg;
		Message m = get_messagebyid(id);
		if(m.sent_rate <= (1.5*m.target_sendrate))
		{
			sendqueue_lowpriority.push_back(elem);
		}
		return true;
	}
	double run_time;
	icarus_rover_v2::diagnostic diagnostic;
	icarus_rover_v2::device mydevice;
	std::string myhostname;
	bool initialized;
	bool ready;

	void init_messages();
	std::vector<Message> messages;
	std::vector<QueueElement> sendqueue_highpriority;
	std::vector<QueueElement> sendqueue_mediumpriority;
	std::vector<QueueElement> sendqueue_lowpriority;
};
#endif
