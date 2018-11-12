#include <gtest/gtest.h>
#include "ros/ros.h"
#include "ros/time.h"
#include <stdlib.h>
#include "icarus_rover_v2/command.h"
#include "icarus_rover_v2/device.h"
#include "icarus_rover_v2/diagnostic.h"
#include "../webserver_node_process.h"
#include "../../../include/jsonmessage.h"

std::string Node_Name = "/unittest_webserver_node_process";
std::string Host_Name = "unittest";
std::string ros_DeviceName = Host_Name;

bool check_json(std::string str,bool print = true)
{
	if(print) {	printf("\r\nValidating:\r\n%s\n",str.c_str()); }

	char tempstr[str.size()+10];
	sprintf(tempstr,"echo '%s' | jsonlint >> /dev/null",str.c_str());
	int v = system(tempstr);
	if(v == 0)
	{
		return true;
	}
	else
	{
		return false;
	}
}
WebServerNodeProcess* initializeprocess()
{
	icarus_rover_v2::diagnostic diagnostic;
	diagnostic.DeviceName = ros_DeviceName;
	diagnostic.Node_Name = Node_Name;
	diagnostic.System = ROVER;
	diagnostic.SubSystem = ROBOT_CONTROLLER;
	diagnostic.Component = COMMUNICATION_NODE;

	diagnostic.Diagnostic_Type = NOERROR;
	diagnostic.Level = INFO;
	diagnostic.Diagnostic_Message = INITIALIZING;
	diagnostic.Description = "Node Initializing";

	icarus_rover_v2::device device;
	device.DeviceName = diagnostic.DeviceName;
	device.BoardCount = 0;
	device.SensorCount = 0;
	device.DeviceParent = "None";
	device.Architecture = "x86_64";

	WebServerNodeProcess *process;
	process = new WebServerNodeProcess;
	diagnostic = process->init(diagnostic,std::string(Host_Name));
	EXPECT_TRUE(diagnostic.Level <= NOTICE);
	EXPECT_TRUE(process->get_initialized() == false);
	process->set_mydevice(device);
	EXPECT_TRUE(process->get_initialized() == true);
	EXPECT_TRUE(process->get_mydevice().DeviceName == device.DeviceName);
	return process;
}
WebServerNodeProcess* readyprocess(WebServerNodeProcess* process)
{
	icarus_rover_v2::diagnostic diag = process->update(0);
	EXPECT_TRUE(diag.Level <= NOTICE);
	EXPECT_TRUE(process->get_ready() == true);
	return process;
}
TEST(Template,Process_Initialization)
{
	WebServerNodeProcess* process = initializeprocess();
}
TEST(DeviceList,Retreive_DeviceListInfo)
{
	icarus_rover_v2::diagnostic diagnostic;
	WebServerNodeProcess* process = initializeprocess();
	process = readyprocess(process);
	std::vector<std::string> master_list;
	master_list.push_back("ControlModule1");
	master_list.push_back("ControlModule2");
	master_list.push_back("ControlModule3");
	master_list.push_back("ControlModule4");
	master_list.push_back("ControlModule5");
	master_list.push_back("ControlModule6");
	int count = 0;
	for(std::size_t i = 0; i < master_list.size(); i++)
	{
		std::vector<icarus_rover_v2::device> dev_list;
		for(int j = 0; j < 2*(i+1);j++)
		{
			count++;
			icarus_rover_v2::device dev;
			char tempstr[512];
			sprintf(tempstr,"%s_%d",master_list.at(i).c_str(),j);
			dev.DeviceName = std::string(tempstr);
			dev_list.push_back(dev);
		}

		diagnostic = process->update_systemdevicelist(dev_list);
		EXPECT_TRUE(diagnostic.Level <= NOTICE);
		EXPECT_TRUE(process->get_systemdevicelist().size() == count);
	}
	JSONMessageHandler *handler;
	std::string result = handler->encode_DeviceJSON(process->get_systemdevicelist());
	EXPECT_TRUE(check_json(result,false));
}
TEST(Template,Process_Command)
{
	WebServerNodeProcess* process = initializeprocess();
	process = readyprocess(process);
	double time_to_run = 20.0;
	double dt = 0.001;
	double current_time = 0.0;
	bool fastrate_fire = false; //10 Hz
	bool mediumrate_fire = false; //1 Hz
	bool slowrate_fire = false; //0.1 Hz
	while(current_time <= time_to_run)
	{
		icarus_rover_v2::diagnostic diag = process->update(dt);
		EXPECT_TRUE(diag.Level <= NOTICE);
		int current_time_ms = (int)(current_time*1000.0);
		if((current_time_ms % 100) == 0)
		{
			fastrate_fire = true;
		}
		else { fastrate_fire = false; }
		if((current_time_ms % 1000) == 0)
		{
			mediumrate_fire = true;
		}
		else { mediumrate_fire = false; }
		if((current_time_ms % 10000) == 0)
		{
			slowrate_fire = true;
		}
		else { slowrate_fire = false; }
		icarus_rover_v2::command cmd;
		cmd.Command = ROVERCOMMAND_RUNDIAGNOSTIC;
		if(fastrate_fire == true)
		{
			cmd.Option1 = LEVEL3;
			std::vector<icarus_rover_v2::diagnostic> diaglist = process->new_commandmsg(cmd);
			for(std::size_t i = 0; i < diaglist.size(); i++)
			{
				EXPECT_TRUE(diaglist.at(i).Level <= NOTICE);
			}
			EXPECT_TRUE(diaglist.size() > 0);

		}
		if(mediumrate_fire == true)
		{
			cmd.Option1 = LEVEL2;
			std::vector<icarus_rover_v2::diagnostic> diaglist = process->new_commandmsg(cmd);
			for(std::size_t i = 0; i < diaglist.size(); i++)
			{
				EXPECT_TRUE(diaglist.at(i).Level <= NOTICE);
			}
			EXPECT_TRUE(diaglist.size() > 0);

		}
		if(slowrate_fire == true)
		{
			cmd.Option1 = LEVEL1;
			std::vector<icarus_rover_v2::diagnostic> diaglist = process->new_commandmsg(cmd);
			for(std::size_t i = 0; i < diaglist.size(); i++)
			{
				EXPECT_TRUE(diaglist.at(i).Level <= NOTICE);
			}
			EXPECT_TRUE(diaglist.size() > 0);
		}
		current_time += dt;
	}
	EXPECT_TRUE(process->get_runtime() >= time_to_run);
}
TEST(Communicat,JSON_Encoding)
{
	WebServerNodeProcess* process = initializeprocess();
	process = readyprocess(process);
	icarus_rover_v2::diagnostic diag = process->update(0);
	EXPECT_TRUE(diag.Level <= NOTICE);
	JSONMessageHandler *handler;
	{
		std::string result = handler->encode_DiagnosticJSON(diag);
		EXPECT_TRUE(check_json(result));
	}


	std::vector<icarus_rover_v2::device> devicelist;

	{
		icarus_rover_v2::device dev;
		dev.DeviceParent = "None";
		dev.PartNumber = "100003";
		dev.DeviceName = "ControlModule1";
		dev.DeviceType = "ControlModule";
		dev.PrimaryIP = "10.0.0.174";
		dev.Architecture = "armv7l";
		dev.ID = 128;
		dev.Capabilities.push_back("ROS");
		dev.BoardCount = 0;
		dev.HatCount = 2;
		dev.SensorCount = 0;
		dev.ShieldCount = 0;
		devicelist.push_back(dev);
		std::string result = handler->encode_DeviceJSON(devicelist);
		EXPECT_TRUE(check_json(result));
	}


	{
		icarus_rover_v2::device dev;
		dev.DeviceParent = "ControlModule1";
		dev.PartNumber = "UNKNOWN";
		dev.DeviceName = "TerminalHat1";
		dev.DeviceType = "TerminalHat";
		dev.ID = 0;
		dev.Capabilities.push_back("GPIO");
		dev.BoardCount = 0;
		dev.HatCount = 0;
		dev.SensorCount = 0;
		dev.ShieldCount = 0;
		devicelist.push_back(dev);
		std::string result = handler->encode_DeviceJSON(devicelist);
		EXPECT_TRUE(check_json(result));
	}


	{
		icarus_rover_v2::device dev;
		dev.DeviceParent = "ControlModule1";
		dev.PartNumber = "625004";
		dev.DeviceName = "PWMHat1";
		dev.DeviceType = "ServoHat";
		dev.ID = 64;
		dev.Capabilities.push_back("GPIO");
		dev.BoardCount = 0;
		dev.HatCount = 0;
		dev.SensorCount = 0;
		dev.ShieldCount = 0;
		devicelist.push_back(dev);
		std::string result = handler->encode_DeviceJSON(devicelist);
		EXPECT_TRUE(check_json(result));
	}


	{
		icarus_rover_v2::device dev;
		dev.DeviceParent = "ControlModule1";
		dev.PartNumber = "160002";
		dev.DeviceName = "LeftMicrophone";
		dev.DeviceType = "Microphone";
		dev.ID = 0;
		dev.Capabilities.push_back("mono");
		dev.BoardCount = 0;
		dev.HatCount = 0;
		dev.SensorCount = 0;
		dev.ShieldCount = 0;
		devicelist.push_back(dev);
		std::string result = handler->encode_DeviceJSON(devicelist);
		EXPECT_TRUE(check_json(result));
	}
	{
		uint8_t armed_status = ARMEDSTATUS_ARMED;
		std::string result = handler->encode_Arm_StatusJSON(armed_status);
		EXPECT_TRUE(check_json(result));
	}



}
int main(int argc, char **argv){
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}

