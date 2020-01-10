#include <gtest/gtest.h>
#include "ros/ros.h"
#include "ros/time.h"
#include "../DataLoggerNodeProcess.h"

std::string Node_Name = "/unittest_datalogger_node_process";
std::string Host_Name = "unittest";
std::string ros_DeviceName = Host_Name;
#define DIAGNOSTIC_TYPE_COUNT 3
void print_diagnostic(uint8_t level,eros::diagnostic diagnostic)
{
	DiagnosticClass diag_helper;
	if(diagnostic.Level >= level)
	{
		printf("Type: %s Message: %s Level: %s Device: %s Desc: %s\n",
			diag_helper.get_DiagTypeString(diagnostic.Diagnostic_Type).c_str(),
			diag_helper.get_DiagMessageString(diagnostic.Diagnostic_Message).c_str(),
			diag_helper.get_DiagLevelString(diagnostic.Level).c_str(),
			diagnostic.DeviceName.c_str(),diagnostic.Description.c_str());
	}
}
DataLoggerNodeProcess *initializeprocess()
{

	eros::device device;
	device.DeviceName = ros_DeviceName;
	device.BoardCount = 0;
	device.SensorCount = 0;
	device.DeviceParent = "None";
	device.Architecture = "x86_64";

	DataLoggerNodeProcess *process;
	process = new DataLoggerNodeProcess;
	process->initialize("datalogger_node", Node_Name, Host_Name, ROVER, ROBOT_CONTROLLER, DIAGNOSTIC_NODE);
	std::vector<uint8_t> diagnostic_types;
	diagnostic_types.push_back(SOFTWARE);
	diagnostic_types.push_back(DATA_STORAGE);
	diagnostic_types.push_back(SYSTEM_RESOURCE);
	process->enable_diagnostics(diagnostic_types);
	process->finish_initialization();
	EXPECT_TRUE(process->get_taskstate() == TASKSTATE_INITIALIZING);
	process->set_mydevice(device);
	EXPECT_TRUE(process->get_taskstate() == TASKSTATE_INITIALIZED);
	EXPECT_TRUE(process->get_mydevice().DeviceName == device.DeviceName);
	return process;
}
DataLoggerNodeProcess *readyprocess(DataLoggerNodeProcess *process)
{
	eros::diagnostic diag = process->update(0.0, 0.0);
	EXPECT_TRUE(diag.Level <= NOTICE);
	EXPECT_TRUE(process->get_taskstate() == TASKSTATE_RUNNING);
	{
		std::vector<eros::diagnostic> diagnostics = process->get_diagnostics();
		EXPECT_TRUE(diagnostics.size() >= DIAGNOSTIC_TYPE_COUNT);
		for (std::size_t i = 0; i < diagnostics.size(); ++i)
		{
			EXPECT_TRUE(diagnostics.at(i).Level <= NOTICE);
		}
	}
	return process;
}
TEST(Template, Process_Initialization)
{
	initializeprocess();
}

TEST(Template, Process_Command)
{
	DataLoggerNodeProcess *process = initializeprocess();
	process = readyprocess(process);
	double time_to_run = 20.0;
	double dt = 0.001;
	double current_time = 0.0;
	bool fastrate_fire = false;   //10 Hz
	bool mediumrate_fire = false; //1 Hz
	bool slowrate_fire = false;   //0.1 Hz
	bool pause_resume_ran = false;
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
			if(pause_resume_ran == false)
			{
				{
					EXPECT_TRUE(process->get_taskstate() == TASKSTATE_RUNNING);
					eros::command cmd_taskcontrol;
					cmd_taskcontrol.Command = ROVERCOMMAND_TASKCONTROL;
					cmd_taskcontrol.Option1 = SUBSYSTEM_UNKNOWN;
					cmd_taskcontrol.Option2 = TASKSTATE_PAUSE;
					cmd_taskcontrol.CommandText = Node_Name;
					eros::command::ConstPtr cmd_ptr(new eros::command(cmd_taskcontrol));
					std::vector<eros::diagnostic> diaglist = process->new_commandmsg(cmd_ptr);
					for (std::size_t i = 0; i < diaglist.size(); i++)
					{
						EXPECT_TRUE(diaglist.at(i).Level <= NOTICE);
					}
					EXPECT_TRUE(diaglist.size() > 0);
					EXPECT_TRUE(process->get_taskstate() == TASKSTATE_PAUSE);
				}
				{
					EXPECT_TRUE(process->get_taskstate() == TASKSTATE_PAUSE);
					eros::command cmd_taskcontrol;
					cmd_taskcontrol.Command = ROVERCOMMAND_TASKCONTROL;
					cmd_taskcontrol.Option1 = SUBSYSTEM_UNKNOWN;
					cmd_taskcontrol.Option2 = TASKSTATE_RUNNING;
					cmd_taskcontrol.CommandText = Node_Name;
					eros::command::ConstPtr cmd_ptr(new eros::command(cmd_taskcontrol));

					std::vector<eros::diagnostic> diaglist = process->new_commandmsg(cmd_ptr);
					for (std::size_t i = 0; i < diaglist.size(); i++)
					{
						EXPECT_TRUE(diaglist.at(i).Level <= NOTICE);
					}
					EXPECT_TRUE(diaglist.size() > 0);
					EXPECT_TRUE(process->get_taskstate() == TASKSTATE_RUNNING);
				}
				{
					EXPECT_TRUE(process->get_taskstate() == TASKSTATE_RUNNING);
					eros::command cmd_taskcontrol;
					cmd_taskcontrol.Command = ROVERCOMMAND_TASKCONTROL;
					cmd_taskcontrol.Option1 = SUBSYSTEM_UNKNOWN;
					cmd_taskcontrol.Option2 = TASKSTATE_RESET;
					cmd_taskcontrol.CommandText = Node_Name;
					eros::command::ConstPtr cmd_ptr(new eros::command(cmd_taskcontrol));

					std::vector<eros::diagnostic> diaglist = process->new_commandmsg(cmd_ptr);
					for (std::size_t i = 0; i < diaglist.size(); i++)
					{
						EXPECT_TRUE(diaglist.at(i).Level <= NOTICE);
					}
					EXPECT_TRUE(diaglist.size() > 0);
					EXPECT_TRUE(process->get_taskstate() == TASKSTATE_RESET);
					diag = process->update(0.0, 0.0);
					EXPECT_TRUE(diag.Level <= NOTICE);
					EXPECT_TRUE(process->get_taskstate() == TASKSTATE_RUNNING);
				}
				pause_resume_ran = true;
			}
		}
		current_time += dt;
	}
	EXPECT_TRUE(pause_resume_ran == true);
	EXPECT_TRUE(process->get_runtime() >= time_to_run);
}
int main(int argc, char **argv)
{
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
