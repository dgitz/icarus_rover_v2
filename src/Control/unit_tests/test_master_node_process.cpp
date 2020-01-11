#include <gtest/gtest.h>
#include "ros/ros.h"
#include "ros/time.h"
#include "../MasterNodeProcess.h"

std::string Node_Name = "/unittest_master_node_process";
std::string Host_Name = "dgitzrosmaster";
std::string ros_DeviceName = Host_Name;
std::string ros_ParentDevice = "";
std::string ros_DeviceType = "ComputeModule";
double SLOW_RATE = 0.1f;
double MEDIUM_RATE = 1.0f;
double FAST_RATE = 10.0f;
int DeviceID = 123;
#define DIAGNOSTIC_TYPE_COUNT 5
void print_diagnostic(uint8_t level, eros::diagnostic diagnostic)
{
	DiagnosticClass diag_helper;
	if (diagnostic.Level >= level)
	{
		printf("Type: %s Message: %s Level: %s Device: %s Desc: %s\n",
			   diag_helper.get_DiagTypeString(diagnostic.Diagnostic_Type).c_str(),
			   diag_helper.get_DiagMessageString(diagnostic.Diagnostic_Message).c_str(),
			   diag_helper.get_DiagLevelString(diagnostic.Level).c_str(),
			   diagnostic.DeviceName.c_str(), diagnostic.Description.c_str());
	}
}
MasterNodeProcess *initializeprocess(std::string devicepath, std::string systempath)
{

	MasterNodeProcess *process;
	process = new MasterNodeProcess;
	process->initialize("master_node", Node_Name, Host_Name, ROVER, ROBOT_CONTROLLER, CONTROLLER_NODE);
	process->set_filepaths(systempath, devicepath);
	std::vector<uint8_t> diagnostic_types;
	diagnostic_types.push_back(SOFTWARE);
	diagnostic_types.push_back(DATA_STORAGE);
	diagnostic_types.push_back(SYSTEM_RESOURCE);
	diagnostic_types.push_back(COMMUNICATIONS);
	diagnostic_types.push_back(SENSORS);
	process->enable_diagnostics(diagnostic_types);
	eros::diagnostic diag = process->finish_initialization();
	EXPECT_TRUE(process->get_taskstate() == TASKSTATE_RUNNING);
	EXPECT_TRUE(diag.Level <= NOTICE);
	std::vector<eros::device> child_devices = process->get_childdevices();
	EXPECT_TRUE(child_devices.size() > 0);
	printf("-----CHILD DEVICES-----\n");
	process->print_device(child_devices);
	std::vector<eros::leverarm> leverarms = process->get_allleverarms();
	printf("-----LEVER ARMS-----\n");
	process->print_leverarm(leverarms);
	return process;
}
MasterNodeProcess *readyprocess(MasterNodeProcess *process)
{
	eros::diagnostic diag = process->update(0.0, 0.0);
	EXPECT_TRUE(diag.Level <= NOTICE);
	EXPECT_TRUE(process->get_taskstate() == TASKSTATE_RUNNING);
	{
		process->update_diagnostic(SYSTEM_RESOURCE, INFO, NOERROR, "Not Checking System Resources during Unit Test.");
		process->update_diagnostic(COMMUNICATIONS, INFO, NOERROR, "Not checking Serial Ports during Unit Test");
		process->update_diagnostic(SENSORS, INFO, NOERROR, "Not Checking Device Temperature during Unit Test.");
		std::vector<eros::diagnostic> diagnostics = process->get_diagnostics();
		EXPECT_TRUE(diagnostics.size() == DIAGNOSTIC_TYPE_COUNT);
		for (std::size_t i = 0; i < diagnostics.size(); ++i)
		{
			EXPECT_TRUE(diagnostics.at(i).Level <= NOTICE);
		}
	}
	return process;
}
TEST(Template, Process_Initialization)
{
	initializeprocess("/home/robot/catkin_ws/src/icarus_rover_v2/src/Control/unit_tests/UnitTestDeviceFile.xml",
					  "/home/robot/catkin_ws/src/icarus_rover_v2/src/Control/unit_tests/UnitTestSystemFile.xml");
}
TEST(Template, Process_Command)
{
	std::vector<std::string> devicepathlist;
	devicepathlist.push_back("/home/robot/catkin_ws/src/icarus_rover_v2/src/Control/unit_tests/UnitTestDeviceFile.xml");
	devicepathlist.push_back("/home/robot/catkin_ws/src/icarus_rover_v2/src/Control/unit_tests/DeviceFile.xml");
	for (std::size_t i = 0; i < devicepathlist.size(); i++)
	{
		std::string devicepath = devicepathlist.at(i);
		std::string systempath = "/home/robot/catkin_ws/src/icarus_rover_v2/src/Control/unit_tests/UnitTestSystemFile.xml";
		//printf("Loading: %s\n",path.c_str());
		MasterNodeProcess *process = initializeprocess(devicepath, systempath);
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
			if (fastrate_fire == true)
			{
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
				{
					std::vector<eros::diagnostic> diagnostics = process->get_diagnostics();
					EXPECT_TRUE(diagnostics.size() >= DIAGNOSTIC_TYPE_COUNT);
					for (std::size_t i = 0; i < diagnostics.size(); ++i)
					{
						EXPECT_TRUE(diagnostics.at(i).Level <= NOTICE);
					}
				}
				if (pause_resume_ran == false)
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
							print_diagnostic(DEBUG, diaglist.at(i));
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
							print_diagnostic(DEBUG, diaglist.at(i));
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
		EXPECT_TRUE(process->get_runtime() >= time_to_run);
		EXPECT_TRUE(pause_resume_ran == true);
	}
}
TEST(ProcessInitialization, NormalOperation)
{
	std::vector<std::string> devicepathlist;
	devicepathlist.push_back("/home/robot/catkin_ws/src/icarus_rover_v2/src/Control/unit_tests/UnitTestDeviceFile.xml");
	devicepathlist.push_back("/home/robot/catkin_ws/src/icarus_rover_v2/src/Control/unit_tests/DeviceFile.xml");
	for (std::size_t i = 0; i < devicepathlist.size(); i++)
	{
		std::string devicepath = devicepathlist.at(i);
		std::string systempath = "/home/robot/catkin_ws/src/icarus_rover_v2/src/Control/unit_tests/UnitTestSystemFile.xml";
		//printf("Loading: %s\n",path.c_str());
		MasterNodeProcess *process = initializeprocess(devicepath, systempath);
		std::vector<std::string> serialportlist;
		serialportlist.push_back("/dev/ttyUSB0");
		serialportlist.push_back("/dev/ttyACM0");
		//serialportlist.push_back("/dev/ttyS0");

		eros::diagnostic diag = process->set_serialportlist(serialportlist);
		EXPECT_TRUE(diag.Level <= NOTICE);
		if (i == 0)
		{
			EXPECT_TRUE(process->get_allserialbaudrates().size() == 1);
		}
		else if (i == 1)
		{
			EXPECT_TRUE(process->get_allserialbaudrates().size() == 1);
		}
		eros::leverarm la_IMU1;
		EXPECT_TRUE(process->get_leverarm(&la_IMU1, "IMU1"));

		process->print_leverarm("IMU1", "BodyOrigin", la_IMU1);
	}
}
int main(int argc, char **argv)
{
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
