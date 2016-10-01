#ifndef GPIONODEPROCESS_H
#define GPIONODEPROCESS_H

#include "ros/ros.h"
#include "Definitions.h"
#include "ros/time.h"
#include <sys/time.h>
#include <stdio.h>
#include <iostream>
#include <ctime>
#include <fstream>
#include "icarus_rover_v2/diagnostic.h"
#include "icarus_rover_v2/device.h"
#include "icarus_rover_v2/command.h"
#include "icarus_rover_v2/pin.h"
#include "logger.h"
#include <serialmessage.h>
#define INITIAL_TIMEOUT_VALUE_MS 1000
using std::string;
using namespace std;
struct Port_Info{
		std::string PortName;
		bool Available[8];
		int Value[8];
		int Mode[8];
		int Number[8];
	};
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
};
class GPIONodeProcess
{
public:


	GPIONodeProcess();
	~GPIONodeProcess();

	icarus_rover_v2::diagnostic init(icarus_rover_v2::diagnostic indiag,Logger *log,std::string hostname);
	icarus_rover_v2::diagnostic update(long dt);
	icarus_rover_v2::diagnostic new_devicemsg(icarus_rover_v2::device devicemsg);
	icarus_rover_v2::diagnostic new_commandmsg(icarus_rover_v2::command commandmsg);
	icarus_rover_v2::diagnostic new_pinmsg(icarus_rover_v2::pin pinmsg);
	icarus_rover_v2::device get_mydevice() { return mydevice; }
	std::vector<icarus_rover_v2::device> get_myboards() { return myboards; }
	bool is_finished_initializing(){ return all_device_info_received; }
	bool initialize_Ports();
	Port_Info get_PortInfo(std::string BoardName,std::string PortName);
	std::string map_PinFunction_ToString(int function);
	int map_PinFunction_ToInt(std::string Function);
	int get_nodestate() { return node_state; }
	void set_nodestate(int mode) { node_state = mode; }
	int get_boardstate() { return board_state; }
	bool set_timeout_ms(long timeout){ timeout_value_ms = timeout; return true; }
	int get_timeout_ms() { return timeout_value_ms; }
	long get_timer_ms() { return ms_timer; }
	bool reset_timer() { ms_timer = 0; timer_timeout = false; return true;}
	bool checkTriggers(std::vector<std::vector<unsigned char > > &tx_buffers);
	icarus_rover_v2::diagnostic new_serialmessage_TestMessageCounter(int packet_type,unsigned char* inpacket);
	icarus_rover_v2::diagnostic new_serialmessage_FirmwareVersion(int packet_type,unsigned char* inpacket);
	icarus_rover_v2::diagnostic new_serialmessage_Diagnostic(int packet_type,unsigned char* inpacket);
	icarus_rover_v2::diagnostic new_serialmessage_Get_ANA_PortA(int packet_type,unsigned char* inpacket);
	icarus_rover_v2::diagnostic new_serialmessage_Get_ANA_PortB(int packet_type,unsigned char* inpacket);
	icarus_rover_v2::diagnostic new_serialmessage_Get_DIO_PortA(int packet_type,unsigned char* inpacket);
	icarus_rover_v2::diagnostic new_serialmessage_Get_DIO_PortB(int packet_type,unsigned char* inpacket);
	icarus_rover_v2::diagnostic new_serialmessage_Get_Mode(int packet_type,unsigned char* inpacket);
	state_ack get_stateack(std::string name);
	bool set_stateack(state_ack stateack);
protected:
	state_ack send_configure_DIO_PortA;
	state_ack send_configure_DIO_PortB;
	state_ack send_testmessage_command;
	state_ack send_nodemode;
	state_ack send_set_DIO_PortA;
	state_ack send_set_DIO_PortB;
private:

	int board_state;
	int node_state;
	int prev_node_state;
	SerialMessageHandler *serialmessagehandler;
	bool configure_pin(std::string BoardName,std::string Port, uint8_t Number, std::string Function);
	void initialize_stateack_messages();
	std::string myhostname;
	bool all_device_info_received;
	icarus_rover_v2::device mydevice;
	icarus_rover_v2::diagnostic diagnostic;
	std::vector<icarus_rover_v2::device> myboards;
	Logger *mylogger;
	Port_Info DIO_PortA;
	Port_Info DIO_PortB;
	Port_Info ANA_PortA;
	Port_Info ANA_PortB;
	long ms_timer;
	long timeout_value_ms;
	bool timer_timeout;
};
#endif
