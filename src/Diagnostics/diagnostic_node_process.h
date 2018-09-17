#ifndef DIAGNOSTICNODEPROCESS_H
#define DIAGNOSTICNODEPROCESS_H

#include "Definitions.h"
#include "ros/ros.h"
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
#include "icarus_rover_v2/resource.h"
#include <icarus_rover_v2/estop.h>
#include <std_msgs/UInt8.h>
#include <serialmessage.h>
#include "logger.h"
#include <math.h>
#define WORSTDIAG_TIMELIMIT 5.0
//#include "../../../eROS/include/DiagnosticClass.h"
class DiagnosticNodeProcess
{
public:

	struct Task
	{
		std::string Task_Name;
		double last_diagnostic_received;
		double last_resource_received;
		double last_heartbeat_received;
		int16_t PID;
		int16_t CPU_Perc;
		int64_t RAM_MB;
		uint8_t last_diagnostic_level;
		std::string resource_topic;
		std::string diagnostic_topic;
		std::string heartbeat_topic;
		//ros::Subscriber resource_sub;
		//ros::Subscriber diagnostic_sub;
		//ros::Subscriber heartbeat_sub;
	};

	struct DeviceResourceAvailable
	{
		std::string Device_Name;
		int16_t CPU_Perc_Available;
		int64_t RAM_Mb_Available;
	};
	struct DiagLevel
	{
		uint8_t Level;
		double last_time;
		icarus_rover_v2::diagnostic diag;
	};

	DiagnosticNodeProcess();
	~DiagnosticNodeProcess();
	icarus_rover_v2::diagnostic init(icarus_rover_v2::diagnostic indiag,std::string hostname);
    void no_connectedlcd()
    {
        lcd_available = false;
    }
    bool get_lcdavailable() { return lcd_available; }
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

	void set_resourcethresholds(int RAM_usage_threshold_MB_,int CPU_usage_threshold_percent_)
	{
		RAM_usage_threshold_MB = RAM_usage_threshold_MB_;
		CPU_usage_threshold_percent = CPU_usage_threshold_percent_;
	}
	std::vector<Task> get_TaskList() { return TaskList; }
	std::vector<DeviceResourceAvailable> get_DeviceResourceAvailableList() { return DeviceResourceAvailableList; }
	void add_Task(Task v);
	void new_heartbeatmsg(std::string topicname);
	void new_resourcemsg(std::string topicname,icarus_rover_v2::resource resource);
	void new_diagnosticmsg(std::string topicname,icarus_rover_v2::diagnostic diagnostic);
	bool get_readytoarm() { return ready_to_arm; }
	void set_nodename(std::string v) { node_name = v; }
	icarus_rover_v2::diagnostic new_1ppsmsg();
	icarus_rover_v2::diagnostic new_01ppsmsg();
    uint8_t get_lcdwidth(){ return lcd_width; }
    uint8_t get_lcdheight() { return lcd_height; }
    void set_batterylevel(double v) { battery_level = v;}
    void set_batteryvoltage(double v) { voltage_received = true; battery_voltage = v;}
    void new_armedstatemsg(uint8_t v) { armed_state = v; }
    std::string build_lcdmessage();
    
private:
	//DiagnosticClass diagclass;
	std::vector<icarus_rover_v2::diagnostic> check_program_variables();

    std::string get_batterylevelstr(double v);
    unsigned char get_lcdclockchar(int v);
    std::string get_batteryvoltagestr();
    std::string get_armedstatestr(uint8_t v);
    std::string get_diagstr();
    std::string get_lcdcommandstr();
    void init_diaglevels();
	double run_time;
	icarus_rover_v2::diagnostic diagnostic;
	icarus_rover_v2::device mydevice;
	std::string myhostname;
	bool initialized;
    bool ready;

	std::vector<icarus_rover_v2::diagnostic> check_tasks();

	std::vector<Task> TaskList;
	std::vector<DeviceResourceAvailable> DeviceResourceAvailableList;
	int RAM_usage_threshold_MB;
	int CPU_usage_threshold_percent;
	bool ready_to_arm;
	std::string node_name;

	double last_1pps_timer;
	double last_01pps_timer;
	double last_cmddiagnostic_timer;
	double last_cmd_timer;
    double battery_level;
    uint8_t lcd_width;
    uint8_t lcd_height;
    std::string lcd_partnumber;
    int lcd_clock;
    bool voltage_received;
    double battery_voltage;
    uint8_t armed_state;
    std::vector<DiagLevel> diaglevels;
    bool bad_diagnostic_received;
    bool any_diagnostic_received;
    bool lcd_available;
    icarus_rover_v2::command current_command;
    bool command_received;
    double lcdclock_timer;

    
};
#endif
