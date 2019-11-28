#include <gtest/gtest.h>
#include "ros/ros.h"
#include "ros/time.h"
#include "../CommandLauncherNodeProcess.h"

std::string Node_Name = "/unittest_commandlauncher_node_process";
std::string Host_Name = "unittest";
std::string ros_DeviceName = Host_Name;
std::string ros_DeviceType = "ControlModule";
#define DIAGNOSTIC_TYPE_COUNT 3


void print_ipmap(std::vector<CommandLauncherNodeProcess::IPMap> ipmap);
void print_portmap(std::vector<CommandLauncherNodeProcess::PortMap> ipmap);
CommandLauncherNodeProcess* initializeprocess()
{
	eros::device device;
	device.DeviceName = ros_DeviceName;
	device.BoardCount = 0;
	device.SensorCount = 0;
	device.DeviceParent = "None";
	device.Architecture = "x86_64";

	CommandLauncherNodeProcess *process;
	process = new CommandLauncherNodeProcess;
	process->initialize("commandlauncher_node", Node_Name, Host_Name, ROVER, ROBOT_CONTROLLER, CONTROLLER_NODE);
	std::vector<uint8_t> diagnostic_types;
	diagnostic_types.push_back(SOFTWARE);
	diagnostic_types.push_back(DATA_STORAGE);
	diagnostic_types.push_back(SYSTEM_RESOURCE);
	process->enable_diagnostics(diagnostic_types);
	
	process->finish_initialization();
	EXPECT_TRUE(process->is_initialized() == false);
	process->set_mydevice(device);
	EXPECT_TRUE(process->is_initialized() == true);
	EXPECT_TRUE(process->get_mydevice().DeviceName == device.DeviceName);

	std::vector<CommandLauncherNodeProcess::ProcessCommand> processlist = process->get_processlist();
	EXPECT_TRUE(processlist.size() == 0); //NO Processes Defined Yet.
	for(std::size_t i = 0; i < processlist.size(); i++)
	{
		EXPECT_TRUE(processlist.at(i).initialized == true); 
		EXPECT_TRUE(processlist.at(i).running == false); 
		printf("[%d] Process: %s Command: '%s'\n",(int)i,processlist.at(i).name.c_str(),processlist.at(i).command_text.c_str());
	}

	for(std::size_t i = 0; i < processlist.size(); i++)
	{
		EXPECT_TRUE(processlist.at(i).initialized == true); 
		EXPECT_TRUE(process->set_processrunning(processlist.at(i).name,true));
		processlist = process->get_processlist();
		EXPECT_TRUE(processlist.at(i).running == true); 
	}
	return process;
}
CommandLauncherNodeProcess* readyprocess(CommandLauncherNodeProcess* process)
{
	eros::diagnostic diag = process->update(0.0,0.0);
	EXPECT_TRUE(diag.Level <= NOTICE);
	EXPECT_TRUE(process->is_ready() == true);
	{
		std::vector<eros::diagnostic> diagnostics = process->get_diagnostics();
		EXPECT_TRUE(diagnostics.size() == DIAGNOSTIC_TYPE_COUNT);
		for (std::size_t i = 0; i < diagnostics.size(); ++i)
		{
			EXPECT_TRUE(diagnostics.at(i).Level <= NOTICE);
		}
	}
	return process;
}
TEST(Template,Process_Initialization)
{
	CommandLauncherNodeProcess* process = initializeprocess();
	std::vector<CommandLauncherNodeProcess::IPMap> ipmap = process->get_ipmap();
	print_ipmap(ipmap);
	std::vector<CommandLauncherNodeProcess::PortMap> portmap = process->get_portmap();
	print_portmap(portmap);
	eros::diagnostic diagnostic = process->update(0.0,0.0);
	EXPECT_TRUE(diagnostic.Level <= NOTICE);
	printf("%s\n",process->get_processinfo().c_str());
}

TEST(Template, Process_Command)
{
	CommandLauncherNodeProcess *process = initializeprocess();
	process = readyprocess(process);
	double time_to_run = 20.0;
	double dt = 0.001;
	double current_time = 0.0;
	bool fastrate_fire = false;   //10 Hz
	bool mediumrate_fire = false; //1 Hz
	bool slowrate_fire = false;   //0.1 Hz
	while (current_time <= time_to_run)
	{
		eros::diagnostic diag = process->update(dt, current_time);
		EXPECT_TRUE(diag.Level <= NOTICE);

		int current_time_ms = (int)(current_time * 1000.0);
		if ((current_time_ms % 100) == 0)
		{
			fastrate_fire = true;
		}
		else
		{
			fastrate_fire = false;
		}
		if ((current_time_ms % 1000) == 0)
		{
			mediumrate_fire = true;
		}
		else
		{
			mediumrate_fire = false;
		}
		if ((current_time_ms % 10000) == 0)
		{
			slowrate_fire = true;
		}
		else
		{
			slowrate_fire = false;
		}

		eros::command cmd;
		cmd.Command = ROVERCOMMAND_RUNDIAGNOSTIC;

		if (fastrate_fire == true) //Nothing to do here
		{
			cmd.Option1 = LEVEL1;
			eros::command::ConstPtr cmd_ptr(new eros::command(cmd));
			std::vector<eros::diagnostic> diaglist = process->new_commandmsg(cmd_ptr);
			for (std::size_t i = 0; i < diaglist.size(); i++)
			{
				EXPECT_TRUE(diaglist.at(i).Level <= NOTICE);
			}
			EXPECT_TRUE(diaglist.size() > 0);
		}
		if (mediumrate_fire == true)
		{
			cmd.Option1 = LEVEL2;
			eros::command::ConstPtr cmd_ptr(new eros::command(cmd));
			std::vector<eros::diagnostic> diaglist = process->new_commandmsg(cmd_ptr);
			for (std::size_t i = 0; i < diaglist.size(); i++)
			{
				EXPECT_TRUE(diaglist.at(i).Level <= NOTICE);
			}
			EXPECT_TRUE(diaglist.size() > 0);
		}
		if (slowrate_fire == true)
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

void print_ipmap(std::vector<CommandLauncherNodeProcess::IPMap> ipmap)
{
	printf("-----IP Map-----\n");
	for(std::size_t i = 0; i < ipmap.size(); i++)
	{
		printf("[%d] Host: %s IP Address: %s\n",
				(int)i,ipmap.at(i).hostname.c_str(),ipmap.at(i).IPAddress.c_str());
	}
	printf("----------------\n");
}
void print_portmap(std::vector<CommandLauncherNodeProcess::PortMap> portmap)
{
	printf("-----Port Map-----\n");
	for(std::size_t i = 0; i < portmap.size(); i++)
	{
		printf("[%d] Name: %s Number: %d\n",
				(int)i,portmap.at(i).name.c_str(),portmap.at(i).port);
	}
	printf("------------------\n");
}
