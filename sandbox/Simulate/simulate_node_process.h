#ifndef SIMULATENODEPROCESS_H
#define SIMULATENODEPROCESS_H

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
#include "icarus_rover_v2/firmware.h"
#include <std_msgs/UInt8.h>
#include <std_msgs/Float32.h>
#include "icarus_rover_v2/encoder.h"
#include <serialmessage.h>
#include "logger.h"
#include <math.h>
#include <tinyxml.h>
#include "icarus_rover_v2/pose.h"
#include <std_msgs/Float32.h>
#include <iostream>
#include <fstream>
struct ScriptAction
{
	std::string Command;
	double Option1;
	double Option2;
	double Option3;
	double Option4;
	double Option5;
	std::string Comment;
};
class SimulateNodeProcess
{
	
public:
	enum VehicleModel
	{
		UNDEFINED,
		TANK,
		ACKERMAN = 2
	};
	enum SimulationMode
	{
		REMOTECONTROL =1,
		SCRIPT = 2
	};
	
	struct VehicleParameters
	{
		double tirediameter_m;
		double vehiclelength_m;
		double wheelbase_m;
		double maxspeed_mps;
	};
	

	SimulateNodeProcess();
	~SimulateNodeProcess();
	icarus_rover_v2::diagnostic init(icarus_rover_v2::diagnostic indiag,std::string hostname);
	icarus_rover_v2::diagnostic update(double dt);
	void set_mydevice(icarus_rover_v2::device v) { mydevice = v; }
	icarus_rover_v2::diagnostic new_devicemsg(icarus_rover_v2::device v);
	bool get_initialized() { return initialized; }
	bool load_scriptfile(std::string path);
	icarus_rover_v2::pose get_pose() { return pose; }
	void set_throttlecommand(double v) { throttle_command = v; }
    void set_steercommand(double v) { steer_command = v; }
	bool is_poseready() { return pose_ready; }
	std::vector<ScriptAction> get_scriptactions() { return actions; }
	bool set_vehiclemodel(uint8_t model)
	{
		switch(model)
		{
			case TANK: vehicle_model = model; break;
			case ACKERMAN: vehicle_model = model; break;
			default: vehicle_model = UNDEFINED; break;
		}
		if(vehicle_model == UNDEFINED) { return false; }
		else { return true; }
	}
	bool set_simulationmode(uint8_t mode)
	{
		switch(mode)
		{
			case REMOTECONTROL: simulation_mode = mode; break;
			case SCRIPT: simulation_mode = mode; break;
			default: simulation_mode = UNDEFINED; break;
		}
		if(simulation_mode == UNDEFINED) { return false; }
		else { return true; }
	}
	uint8_t map_simulationmode_toint(std::string v);
	double get_left_encoder() { return left_encoder; }
	double get_right_encoder() { return right_encoder; }
	double get_left_wheelspeed_command() { return left_wheelspeed_command; }
	double get_right_wheelspeed_command() { return right_wheelspeed_command; }
	double get_yawrate() { return yaw_rate; }
	std::string get_currentcommand() { return current_action.Command; }
    
private:
	icarus_rover_v2::device mydevice;
	bool initialized;
	bool load_configfiles();
	double get_rand();
	bool pose_ready;
	icarus_rover_v2::diagnostic diagnostic;
	VehicleParameters vehicle_params;
	uint8_t vehicle_model;
	uint8_t simulation_mode;
	icarus_rover_v2::pose pose;
	double throttle_command;
	double steer_command;
	double left_wheelspeed_command;
	double right_wheelspeed_command;
	double left_encoder;
	double right_encoder;
	double yaw_rate;
	std::vector<ScriptAction> actions;
	ScriptAction current_action;
	uint8_t action_index;
	double time_in_action;
	bool script_running;
};
#endif
