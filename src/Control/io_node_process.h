#ifndef IONODEPROCESS_H
#define IONODEPROCESS_H

#include "ros/ros.h"
#include "ros/time.h"
#include "Definitions.h"
#include "ros/time.h"
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
#define INITIAL_TIMEOUT_VALUE_MS 1000
using std::string;
using namespace std;
struct Sensor {
	std::string name;
	std::string type;
	int adc_resolution;
	double voltage_reference;
	double Rm_ohms;
	std::vector<double> input_vector;
	std::vector<double> output_vector;
	std::string spec_path;
	std::string spec_relationship;
	double slope;
	double intercept;
};
struct Port_Info{
		std::string PortName;
		bool Available[32];
		int Value[32];
		int Mode[32];
		int Number[32];
		std::vector<std::string> ConnectingDevice;
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
struct message_info
{
	int id;
	std::string protocol;
	ros::Time last_time_received;
	ros::Time last_time_transmitted;
	long received_counter;
	long sent_counter;
	double received_rate;
	double transmitted_rate;

};
class IONodeProcess
{
public:


	IONodeProcess();
	~IONodeProcess();

	icarus_rover_v2::diagnostic init(icarus_rover_v2::diagnostic indiag,Logger *log,std::string hostname);
	icarus_rover_v2::diagnostic update(long dt);
	icarus_rover_v2::diagnostic new_devicemsg(icarus_rover_v2::device devicemsg);
	icarus_rover_v2::diagnostic new_commandmsg(icarus_rover_v2::command commandmsg);
	icarus_rover_v2::diagnostic new_pinmsg(icarus_rover_v2::pin pinmsg);
	icarus_rover_v2::device get_mydevice() { return mydevice; }
	std::vector<icarus_rover_v2::pin> get_mypins() { return mypins; }
	//std::vector<icarus_rover_v2::device> get_myboards() { return myboards; }
	bool is_finished_initializing(){ return all_device_info_received; }
	bool initialize_Ports();
	Port_Info get_PortInfo(std::string PortName);
	std::string map_PinFunction_ToString(int function);
	int map_PinFunction_ToInt(std::string Function);
	bool set_timeout_ms(long timeout){ timeout_value_ms = timeout; return true; }
	int get_timeout_ms() { return timeout_value_ms; }
	long get_timer_ms() { return ms_timer; }
	bool reset_timer() { ms_timer = 0; timer_timeout = false; return true;}
	bool checkTriggers(std::vector<std::vector<unsigned char > > &tx_buffers);
	state_ack get_stateack(std::string name);
	bool set_stateack(state_ack stateack);
protected:
	/*
	state_ack send_configure_DIO_PortA;
	state_ack send_configure_DIO_PortB;
	state_ack send_testmessage_command;
	state_ack send_nodemode;
	state_ack send_set_DIO_PortA;
	state_ack send_set_DIO_PortB;
	*/
private:

	bool configure_pin(std::string Port, uint8_t Number, std::string Function,std::string ConnectedDevice);
	void initialize_stateack_messages();
	std::string myhostname;
	bool all_sensor_info_received;
	bool all_device_info_received;
	icarus_rover_v2::device mydevice;
	icarus_rover_v2::diagnostic diagnostic;
	std::vector<icarus_rover_v2::device> mysensors;
	std::vector<Sensor> SensorSpecs;
	std::vector<icarus_rover_v2::pin> mypins;
	Logger *mylogger;
	Port_Info GPIO_Port;
	/*
	Port_Info DIO_PortA;
	Port_Info DIO_PortB;
	Port_Info ANA_PortA;
	Port_Info ANA_PortB;
	*/
	long ms_timer;
	long timeout_value_ms;
    ros::Time init_time;
	bool timer_timeout;
	double time_diff(ros::Time timer_a, ros::Time timer_b);
};
#endif
