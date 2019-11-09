#include <gtest/gtest.h>

#include "../NetworkTransceiverNodeProcess.h"
#include "ros/ros.h"
#include "ros/time.h"

std::string Node_Name = "/unittest_networktransceiver_node_process";
std::string Host_Name = "unittest";
std::string ros_DeviceName = Host_Name;
#define DIAGNOSTIC_TYPE_COUNT 5

void print_diagnostic(uint8_t level,eros::diagnostic diagnostic)
{
	if(diagnostic.Level >= level)
	{
		printf("Type: %d Message: %d Level: %d Device: %s Desc: %s\n",diagnostic.Diagnostic_Type,diagnostic.Diagnostic_Message,
			  		diagnostic.Level,diagnostic.DeviceName.c_str(),diagnostic.Description.c_str());
	}
}
NetworkTransceiverNodeProcess* initializeprocess()
{

	eros::device device;
	device.DeviceName = ros_DeviceName;
	device.BoardCount = 0;
	device.SensorCount = 0;
	device.DeviceParent = "None";
	device.Architecture = "x86_64";

	NetworkTransceiverNodeProcess *process;
	process = new NetworkTransceiverNodeProcess;
	process->initialize("networktransceiver_node",Node_Name,Host_Name,ROVER,ROBOT_CONTROLLER,COMMUNICATION_NODE);
	std::vector<uint8_t> diagnostic_types;
	diagnostic_types.push_back(SOFTWARE);
	diagnostic_types.push_back(DATA_STORAGE);
	diagnostic_types.push_back(SYSTEM_RESOURCE);
	diagnostic_types.push_back(COMMUNICATIONS);
	diagnostic_types.push_back(REMOTE_CONTROL);
	process->enable_diagnostics(diagnostic_types);
	process->finish_initialization();
	EXPECT_TRUE(process->is_initialized() == false);
	process->set_mydevice(device);
	EXPECT_TRUE(process->is_initialized() == true);
	EXPECT_TRUE(process->get_mydevice().DeviceName == device.DeviceName);
	return process;
}
NetworkTransceiverNodeProcess* readyprocess(NetworkTransceiverNodeProcess* process)
{
	eros::diagnostic diag = process->update(0.0,0.0);
	EXPECT_TRUE(diag.Level <= NOTICE);
	EXPECT_TRUE(process->is_ready() == true);
	EXPECT_TRUE(process->is_ready() == true);
	{
		process->update_diagnostic(SYSTEM_RESOURCE,INFO,NOERROR,"Not Checking System Resource during Unit Test.");
		process->update_diagnostic(REMOTE_CONTROL,INFO,NOERROR,"Not Checking Remote Control during Unit Test.");
		std::vector<eros::diagnostic> diagnostics = process->get_diagnostics();
		EXPECT_TRUE(diagnostics.size() == DIAGNOSTIC_TYPE_COUNT);
		for(std::size_t i = 0; i < diagnostics.size(); ++i)
		{
			if ((diagnostics.at(i).Diagnostic_Type != COMMUNICATIONS)) //No heartbeats processed yet, so this should be in a warn state
			{
				EXPECT_TRUE(diagnostics.at(i).Level <= NOTICE);
			}
		}
	}
	return process;
}
TEST(Template,Process_Initialization)
{
	NetworkTransceiverNodeProcess* process = initializeprocess();
}
TEST(StressTest,RemoteHeartbeat)
{
	NetworkTransceiverNodeProcess* process = initializeprocess();
	process = readyprocess(process);
	return;
	double time_to_run = 20.0;
	double dt = 0.001;
	double current_time = 0.0;
	bool fastrate_fire = false; //10 Hz
	bool mediumrate_fire = false; //1 Hz
	bool slowrate_fire = false; //0.1 Hz
	std::vector<NetworkTransceiverNodeProcess::Message> messages = process->get_messages();
	for(std::size_t i = 0; i < messages.size(); i++)
	{
		process->new_message_sent(messages.at(i).id);
	}
	while(current_time <= time_to_run)
	{
		eros::diagnostic diag = process->update(dt,current_time);
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
		eros::command cmd;
		cmd.Command = ROVERCOMMAND_RUNDIAGNOSTIC;
		if(fastrate_fire == true)
		{
			process->new_remoteheartbeatmsg(current_time,"Remote1",current_time,current_time+.01);
			cmd.Option1 = LEVEL1;
			eros::command::ConstPtr cmd_ptr(new eros::command(cmd));
			std::vector<eros::diagnostic> diaglist = process->new_commandmsg(cmd_ptr);
			for(std::size_t i = 0; i < diaglist.size(); i++)
			{
				EXPECT_TRUE(diaglist.at(i).Level <= NOTICE);
			}
			EXPECT_TRUE(diaglist.size() > 0);

		}
		if(mediumrate_fire == true)
		{
			cmd.Option1 = LEVEL2;
			eros::command::ConstPtr cmd_ptr(new eros::command(cmd));
			std::vector<eros::diagnostic> diaglist = process->new_commandmsg(cmd_ptr);
			for(std::size_t i = 0; i < diaglist.size(); i++)
			{
				EXPECT_TRUE(diaglist.at(i).Level <= NOTICE);
			}
			EXPECT_TRUE(diaglist.size() > 0);

		}
		if(slowrate_fire == true)
		{
			std::vector<eros::diagnostic> diagnostics = process->get_diagnostics();
			EXPECT_TRUE(diagnostics.size() >= DIAGNOSTIC_TYPE_COUNT);
			for (std::size_t i = 0; i < diagnostics.size(); ++i)
			{
				EXPECT_TRUE(diagnostics.at(i).Level <= NOTICE);
			}
		}
		current_time += dt;
	}
	EXPECT_TRUE(process->get_runtime() >= time_to_run);
}
TEST(Template,Process_Command)
{
	return;
	NetworkTransceiverNodeProcess* process = initializeprocess();
	process = readyprocess(process);
	double time_to_run = 20.0;
	double dt = 0.001;
	double current_time = 0.0;
	bool fastrate_fire = false; //10 Hz
	bool mediumrate_fire = false; //1 Hz
	bool slowrate_fire = false; //0.1 Hz
	std::vector<NetworkTransceiverNodeProcess::Message> messages = process->get_messages();
	for(std::size_t i = 0; i < messages.size(); i++)
	{
		process->new_message_sent(messages.at(i).id);
	}
	while(current_time <= time_to_run)
	{
		eros::diagnostic diag = process->update(dt,current_time);
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
		eros::command cmd;
		cmd.Command = ROVERCOMMAND_RUNDIAGNOSTIC;
		if(fastrate_fire == true)
		{
			process->new_remoteheartbeatmsg(current_time,"Remote1",current_time,current_time+.01);
			cmd.Option1 = LEVEL1;
			eros::command::ConstPtr cmd_ptr(new eros::command(cmd));
			std::vector<eros::diagnostic> diaglist = process->new_commandmsg(cmd_ptr);
			for(std::size_t i = 0; i < diaglist.size(); i++)
			{
				EXPECT_TRUE(diaglist.at(i).Level <= NOTICE);
			}
			EXPECT_TRUE(diaglist.size() > 0);

		}
		if(mediumrate_fire == true)
		{
			cmd.Option1 = LEVEL2;
			eros::command::ConstPtr cmd_ptr(new eros::command(cmd));
			std::vector<eros::diagnostic> diaglist = process->new_commandmsg(cmd_ptr);
			for(std::size_t i = 0; i < diaglist.size(); i++)
			{
				EXPECT_TRUE(diaglist.at(i).Level <= NOTICE);
			}
			EXPECT_TRUE(diaglist.size() > 0);
		}
		if(slowrate_fire == true)
		{
			std::vector<eros::diagnostic> diagnostics = process->get_diagnostics();
			EXPECT_TRUE(diagnostics.size() >= DIAGNOSTIC_TYPE_COUNT);
			for (std::size_t i = 0; i < diagnostics.size(); ++i)
			{
				EXPECT_TRUE(diagnostics.at(i).Level <= NOTICE);
			}
		}
		current_time += dt;
	}
	EXPECT_TRUE(process->get_runtime() >= time_to_run);
}
int main(int argc, char **argv){
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}

