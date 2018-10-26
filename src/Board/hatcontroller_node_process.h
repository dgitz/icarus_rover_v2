#ifndef HATCONTROLLERNODEPROCESS_H
#define HATCONTROLLERNODEPROCESS_H

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
#include "icarus_rover_v2/iopins.h"
#include "icarus_rover_v2/firmware.h"
#include "icarus_rover_v2/signal.h"
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8.h>
#include "logger.h"
#include <math.h>
#include <sys/time.h>
#include "ros/time.h"
#include <tinyxml.h>
#define TIMING_BUFFER_LENGTH 100
using std::string;
using namespace std;
class HatControllerNodeProcess
{
public:
	struct Sensor
	{
		double tov;
		uint8_t status;
		bool initialized;
		std::string type;
		std::string name;
		std::string remapped_topicname;
		icarus_rover_v2::device connected_hat;
		icarus_rover_v2::pin connected_pin;
		bool convert;
		double value;
		std::string units;
		std::string output_datatype;
		double min_inputvalue;
		double max_inputvalue;
		double min_outputvalue;
		double max_outputvalue;
	};

	HatControllerNodeProcess();
	~HatControllerNodeProcess();    
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

	uint8_t get_armedstate() { return armed_state; }
	bool get_ready_to_arm() { return ready_to_arm; }
	bool is_ready() { return ready; }
	void set_analyzetiming(bool v) { analyze_timing = v; }
	bool get_analyzetiming() { return analyze_timing; }
	double get_timedelay();

	icarus_rover_v2::diagnostic new_armedstatemsg(uint8_t msg);
	icarus_rover_v2::diagnostic new_pinsmsg(icarus_rover_v2::iopins msg);
	icarus_rover_v2::diagnostic new_pinmsg(icarus_rover_v2::pin msg);
	icarus_rover_v2::diagnostic new_ppsmsg(std_msgs::Bool msg);

	Sensor find_sensor(std::string name);
	bool update_sensorinfo(Sensor sensor);
	std::vector<Sensor> get_sensordata() { return sensors; }

	//Generic Hat Functions
	icarus_rover_v2::diagnostic set_hat_running(std::string devicetype,uint16_t id);
	bool is_hat_running(std::string devicetype,uint16_t id);

	//Servo Hat Functions
	std::vector<icarus_rover_v2::pin> get_servohatpins(uint16_t id);
	std::vector<uint16_t> get_servohataddresses();

	//Terminal Hat Functions
	icarus_rover_v2::diagnostic set_terminalhat_initialized();
	std::vector<icarus_rover_v2::pin> get_terminalhatpins(std::string Function,bool match_exact);


	//GPIO Hat Functions
	bool is_gpiohat_running(uint16_t id);
	std::vector<icarus_rover_v2::pin> get_gpiohatpins(uint16_t id);
	std::vector<uint16_t> get_gpiohataddresses();
	icarus_rover_v2::diagnostic new_message_GetDIOPort1(uint8_t hatid,double tov,uint16_t v1,uint16_t v2,uint16_t v3,uint16_t v4);

	ros::Time convert_time(struct timeval t);

protected:
private:
	std::vector<icarus_rover_v2::diagnostic> check_program_variables();
	void init_messages();
	std::string map_PinFunction_ToString(int function);
	double map_input_to_output(double input_value,double min_input,double max_input,double min_output,double max_output);
	int map_PinFunction_ToInt(std::string Function);
	bool sensors_initialized();
	bool update_sensor(icarus_rover_v2::device,icarus_rover_v2::pin,double tov,double value);
	bool load_sensorinfo(std::string name);
	bool parse_sensorfile(TiXmlDocument doc,std::string name);
	icarus_rover_v2::device find_hat(uint8_t hatid);
	icarus_rover_v2::pin find_pin(icarus_rover_v2::device hatid,std::string pinfunction,uint8_t pinnumber);
	double run_time;
	icarus_rover_v2::diagnostic diagnostic;
	icarus_rover_v2::device mydevice;
	std::string myhostname;
	bool initialized;
	bool ready;

	bool hat_present(icarus_rover_v2::device device);
	double measure_time_diff(struct timeval start, struct timeval end);
	double measure_time_diff(ros::Time start, struct timeval end);
	double measure_time_diff(double start, struct timeval end);
	double measure_time_diff(double start, double end);
	double measure_time_diff(struct timeval start, double end);

	std::vector<Sensor> sensors;

	uint8_t armed_state;
	bool ready_to_arm;
	std::vector<icarus_rover_v2::device> hats;
	std::vector<bool> hats_running;
	uint64_t pps_counter;
	double time_sincelast_pps;
	bool analyze_timing;
	std::vector<double> timing_diff;
};
#endif
