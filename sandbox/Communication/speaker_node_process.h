#ifndef SPEAKERNODEPROCESS_H
#define SPEAKERNODEPROCESS_H

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
#include "icarus_rover_v2/usermessage.h"
#include "icarus_rover_v2/pin.h"
#include "icarus_rover_v2/firmware.h"
#include <std_msgs/UInt8.h>
#include <serialmessage.h>
#include <math.h>
#define SPEECH_RATE 12 //Characters per Second
#define PAUSE_BETWEEN 0.25f
/*
struct state_ack
{
	std::string name;
	bool state;
	bool trigger;
	bool retrying;
	struct timeval orig_send_time;
	struct timeval retry_send_time;
	uint16_t retries;
	uint16_t timeout_counter;
	bool retry_mode;
	bool failed;
	int flag1;  //Various Purposes
	double stream_rate;
};
*/
class SpeakerNodeProcess
{
public:


	SpeakerNodeProcess();
	~SpeakerNodeProcess();
	icarus_rover_v2::diagnostic init(icarus_rover_v2::diagnostic indiag,std::string hostname);
	icarus_rover_v2::diagnostic update(double dt);
	icarus_rover_v2::diagnostic new_devicemsg(icarus_rover_v2::device devicemsg);
	icarus_rover_v2::diagnostic new_commandmsg(icarus_rover_v2::command commandmsg);
	icarus_rover_v2::diagnostic new_usermessage(icarus_rover_v2::usermessage usermsg);
	bool readytospeak() { return ready_to_speek; }
	bool isspeaking() { return is_speaking; }
	bool ispaused() { return is_paused; }
	std::string get_speechoutput() { return speech_output; }
	double get_timeleft_tofinishspeaking() { return time_left_to_finish; }
	void initialize_stateack_messages();
	icarus_rover_v2::device get_mydevice() { return mydevice; }
	bool is_finished_initializing(){ return all_device_info_received; }
	bool checkTriggers(std::vector<std::vector<unsigned char > > &tx_buffers);
	icarus_rover_v2::diagnostic get_diagnostic() { return diagnostic; }
    //state_ack get_stateack(std::string name);
	//bool set_stateack(state_ack stateack);
	double get_runtime() { return run_time; }
	std::vector<icarus_rover_v2::usermessage> get_usermessages(uint8_t level);
protected:
	//state_ack send_speechmessage;

private:

	int id;
	std::string myhostname;
	bool all_device_info_received;
	icarus_rover_v2::device mydevice;
	icarus_rover_v2::diagnostic diagnostic;
	long ms_timer;
	long timeout_value_ms;
	struct timeval init_time;
	bool timer_timeout;
    double time_diff(struct timeval timea,struct timeval timeb);
    double compute_timetospeak(std::string s);
	double run_time;
	std::vector<icarus_rover_v2::usermessage> level1_messages;
	std::vector<icarus_rover_v2::usermessage> level2_messages;
	std::vector<icarus_rover_v2::usermessage> level3_messages;
	std::vector<icarus_rover_v2::usermessage> level4_messages;

	bool ready_to_speek;
	bool is_speaking;
	bool is_paused;
	std::string speech_output;
	double time_left_to_finish;
	double pause_timer;
};
#endif
