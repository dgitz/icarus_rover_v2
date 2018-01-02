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
#include <std_msgs/UInt8.h>
#include <serialmessage.h>
#include "logger.h"
#include <math.h>
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

	DiagnosticNodeProcess();
	~DiagnosticNodeProcess();
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
    
private:
	std::vector<icarus_rover_v2::diagnostic> check_program_variables();
	
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
};
#endif
