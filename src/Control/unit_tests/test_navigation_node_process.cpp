#include <gtest/gtest.h>
#include "ros/ros.h"
#include "ros/time.h"
#include "../NavigationNodeProcess.h"

std::string Node_Name = "/unittest_navigation_node_process";
std::string Host_Name = "unittest";
std::string ros_DeviceName = Host_Name;
#define DIAGNOSTIC_TYPE_COUNT 4

#define LEFTDRIVE_MAX 1900
#define LEFTDRIVE_DEFAULT 1450
#define LEFTDRIVE_MIN 1100
#define RIGHTDRIVE_MAX 1050
#define RIGHTDRIVE_DEFAULT 1525
#define RIGHTDRIVE_MIN 1950

bool isequal(double a, double b)
{
	double v = a - b;
	if (fabs(v) < .0000001)
	{
		return true;
	}
	else
	{
		return false;
	}
}
void print_diagnostic(uint8_t level,eros::diagnostic diagnostic)
{
	if(diagnostic.Level >= level)
	{
		printf("Type: %d Message: %d Level: %d Device: %s Desc: %s\n",diagnostic.Diagnostic_Type,diagnostic.Diagnostic_Message,
			  		diagnostic.Level,diagnostic.DeviceName.c_str(),diagnostic.Description.c_str());
	}
}
NavigationNodeProcess *initializeprocess(std::string controlgroup_filepath)
{
	eros::device device;
	device.DeviceName = ros_DeviceName;
	device.BoardCount = 0;
	device.SensorCount = 0;
	device.DeviceParent = "None";
	device.Architecture = "x86_64";

	NavigationNodeProcess *process;
	process = new NavigationNodeProcess;
	process->initialize("navigation_node", Node_Name, Host_Name, ROVER, ROBOT_CONTROLLER, NAVIGATION_NODE);
	std::vector<uint8_t> diagnostic_types;
	diagnostic_types.push_back(SOFTWARE);
	diagnostic_types.push_back(DATA_STORAGE);
	diagnostic_types.push_back(SYSTEM_RESOURCE);
	diagnostic_types.push_back(REMOTE_CONTROL);
	process->enable_diagnostics(diagnostic_types);
	process->finish_initialization();
	process->set_filepaths(controlgroup_filepath);
	EXPECT_TRUE(process->is_initialized() == false);
	process->set_mydevice(device);

	EXPECT_TRUE(process->is_initialized() == true);
	EXPECT_TRUE(process->get_mydevice().DeviceName == device.DeviceName);

	eros::device servohat1_device;
	servohat1_device.DeviceName = "ServoHat1";
	servohat1_device.DeviceParent = ros_DeviceName;
	servohat1_device.DeviceType = "ServoHat";
	servohat1_device.BoardCount = 0;
	servohat1_device.ID = 64;

	{ //PIN1
		eros::pin newpin;
		newpin.ConnectedDevice = "LeftMotorController";
		newpin.DefaultValue = LEFTDRIVE_DEFAULT;
		newpin.MaxValue = LEFTDRIVE_MAX;
		newpin.MinValue = LEFTDRIVE_MIN;
		newpin.Function = "PWMOutput";
		servohat1_device.pins.push_back(newpin);
	}

	{ //PIN2
		eros::pin newpin;
		newpin.ConnectedDevice = "RightMotorController";
		newpin.DefaultValue = RIGHTDRIVE_DEFAULT;
		newpin.MaxValue = RIGHTDRIVE_MAX;
		newpin.MinValue = RIGHTDRIVE_MIN;
		newpin.Function = "PWMOutput-NonActuator";
		servohat1_device.pins.push_back(newpin);
	}

	EXPECT_TRUE(process->is_initialized() == true);

	EXPECT_TRUE(process->is_ready() == false);

	eros::device::ConstPtr dev_ptr(new eros::device(servohat1_device));
	eros::diagnostic diag = process->new_devicemsg(dev_ptr);
	EXPECT_TRUE(diag.Level <= NOTICE);
	diag = process->fetch_complete();
	EXPECT_TRUE(diag.Level <= NOTICE);
	diag = process->load_controlgroupfile();
	print_diagnostic(DEBUG,diag);
	EXPECT_TRUE(diag.Level <= NOTICE);

	return process;
}
NavigationNodeProcess *readyprocess(NavigationNodeProcess *process)
{
	eros::diagnostic diag = process->update(0.0, 0.0);
	EXPECT_TRUE(diag.Level <= NOTICE);
	EXPECT_TRUE(process->is_ready() == true);
	process->print_controlgroups();
	std::vector<NavigationNodeProcess::ControlGroup> controlgroups = process->get_controlgroups();
	EXPECT_TRUE(controlgroups.size() > 0);
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
TEST(Template, Process_Initialization)
{
	initializeprocess("/home/robot/catkin_ws/src/icarus_rover_v2/src/Control/unit_tests/UnitTestControlGroup.xml");
}
TEST(Template, Process_Command)
{
	NavigationNodeProcess *process = initializeprocess("/home/robot/catkin_ws/src/icarus_rover_v2/src/Control/unit_tests/UnitTestControlGroup.xml");
	process = readyprocess(process);
	{
		std::vector<eros::pin> pins = process->get_pins();
		bool found1 = false;
		bool found2 = false;
		for (std::size_t i = 0; i < pins.size(); ++i)
		{
			if (pins.at(i).ConnectedDevice == "LeftMotorController")
			{
				found1 = true;
				EXPECT_TRUE(pins.at(i).Value == pins.at(i).DefaultValue);
				EXPECT_TRUE(pins.at(i).MinValue == LEFTDRIVE_MIN);
				EXPECT_TRUE(pins.at(i).Value == LEFTDRIVE_DEFAULT);
				EXPECT_TRUE(pins.at(i).MaxValue == LEFTDRIVE_MAX);
			}
			if (pins.at(i).ConnectedDevice == "RightMotorController")
			{
				found2 = true;
				EXPECT_TRUE(pins.at(i).Value == pins.at(i).DefaultValue);
				EXPECT_TRUE(pins.at(i).MinValue == RIGHTDRIVE_MIN);
				EXPECT_TRUE(pins.at(i).Value == RIGHTDRIVE_DEFAULT);
				EXPECT_TRUE(pins.at(i).MaxValue == RIGHTDRIVE_MAX);
			}
		}
		EXPECT_TRUE(found1);
		EXPECT_TRUE(found2);
	}
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

			{ //Drive Full Forwards
				eros::command drive_cmd;
				drive_cmd.Command = ROVERCOMMAND_DRIVECOMMAND;
				json obj;
				obj["ControlType"] = "OpenLoop";
				obj["ControlGroup"] = "ArcadeDrive";
				obj["ForwardVelocityPerc"] = 100.0;
				obj["RotateZAxisPerc"] = 0.0;
				drive_cmd.CommandText = obj.dump();
				drive_cmd.Description = "Drive Forwards";
				eros::command::ConstPtr cmd_ptr(new eros::command(drive_cmd));
				std::vector<eros::diagnostic> diaglist = process->new_commandmsg(cmd_ptr);
				for (std::size_t i = 0; i < diaglist.size(); i++)
				{
					EXPECT_TRUE(diaglist.at(i).Level <= NOTICE);
					EXPECT_TRUE(diaglist.at(i).Diagnostic_Message != DEVICE_NOT_AVAILABLE);
				}
				EXPECT_TRUE(diaglist.size() > 0);
				std::vector<eros::pin> pins = process->get_pins();
				bool found1 = false;
				bool found2 = false;
				for (std::size_t i = 0; i < pins.size(); ++i)
				{
					if (pins.at(i).ConnectedDevice == "LeftMotorController")
					{
						found1 = true;
						EXPECT_TRUE(pins.at(i).Value == LEFTDRIVE_MAX);
					}
					if (pins.at(i).ConnectedDevice == "RightMotorController")
					{
						found2 = true;
						EXPECT_TRUE(pins.at(i).Value == RIGHTDRIVE_MAX);
					}
				}
				EXPECT_TRUE(found1);
				EXPECT_TRUE(found2);
			}
			{ //Drive Full Reverse
				eros::command drive_cmd;
				drive_cmd.Command = ROVERCOMMAND_DRIVECOMMAND;
				json obj;
				obj["ControlType"] = "OpenLoop";
				obj["ControlGroup"] = "ArcadeDrive";
				obj["ForwardVelocityPerc"] = -100.0;
				obj["RotateZAxisPerc"] = 0.0;
				drive_cmd.CommandText = obj.dump();
				drive_cmd.Description = "Drive Backwards";
				eros::command::ConstPtr cmd_ptr(new eros::command(drive_cmd));
				std::vector<eros::diagnostic> diaglist = process->new_commandmsg(cmd_ptr);
				for (std::size_t i = 0; i < diaglist.size(); i++)
				{
					EXPECT_TRUE(diaglist.at(i).Level <= NOTICE);
					EXPECT_TRUE(diaglist.at(i).Diagnostic_Message != DEVICE_NOT_AVAILABLE);
				}
				EXPECT_TRUE(diaglist.size() > 0);
				std::vector<eros::pin> pins = process->get_pins();
				bool found1 = false;
				bool found2 = false;
				for (std::size_t i = 0; i < pins.size(); ++i)
				{
					if (pins.at(i).ConnectedDevice == "LeftMotorController")
					{
						found1 = true;

						EXPECT_TRUE(pins.at(i).Value == LEFTDRIVE_MIN);
					}
					if (pins.at(i).ConnectedDevice == "RightMotorController")
					{
						found2 = true;
						EXPECT_TRUE(pins.at(i).Value == RIGHTDRIVE_MIN);
					}
				}
				EXPECT_TRUE(found1);
				EXPECT_TRUE(found2);
			}
			{ //Drive Full Left
				eros::command drive_cmd;
				drive_cmd.Command = ROVERCOMMAND_DRIVECOMMAND;
				json obj;
				obj["ControlType"] = "OpenLoop";
				obj["ControlGroup"] = "ArcadeDrive";
				obj["ForwardVelocityPerc"] = 0.0;
				obj["RotateZAxisPerc"] = -100.0;
				drive_cmd.CommandText = obj.dump();
				drive_cmd.Description = "Drive Turn Left";
				eros::command::ConstPtr cmd_ptr(new eros::command(drive_cmd));
				std::vector<eros::diagnostic> diaglist = process->new_commandmsg(cmd_ptr);
				for (std::size_t i = 0; i < diaglist.size(); i++)
				{
					EXPECT_TRUE(diaglist.at(i).Level <= NOTICE);
					EXPECT_TRUE(diaglist.at(i).Diagnostic_Message != DEVICE_NOT_AVAILABLE);
				}
				EXPECT_TRUE(diaglist.size() > 0);
				std::vector<eros::pin> pins = process->get_pins();
				bool found1 = false;
				bool found2 = false;
				for (std::size_t i = 0; i < pins.size(); ++i)
				{
					if (pins.at(i).ConnectedDevice == "LeftMotorController")
					{
						found1 = true;
						EXPECT_TRUE(pins.at(i).Value == LEFTDRIVE_MIN);
					}
					if (pins.at(i).ConnectedDevice == "RightMotorController")
					{
						found2 = true;
						EXPECT_TRUE(pins.at(i).Value == RIGHTDRIVE_MAX);
					}
				}
				EXPECT_TRUE(found1);
				EXPECT_TRUE(found2);
			}
			{ //Drive Full Right
				eros::command drive_cmd;
				drive_cmd.Command = ROVERCOMMAND_DRIVECOMMAND;
				json obj;
				obj["ControlType"] = "OpenLoop";
				obj["ControlGroup"] = "ArcadeDrive";
				obj["ForwardVelocityPerc"] = 0.0;
				obj["RotateZAxisPerc"] = 100.0;
				drive_cmd.CommandText = obj.dump();
				drive_cmd.Description = "Drive Turn Right";
				eros::command::ConstPtr cmd_ptr(new eros::command(drive_cmd));
				std::vector<eros::diagnostic> diaglist = process->new_commandmsg(cmd_ptr);
				for (std::size_t i = 0; i < diaglist.size(); i++)
				{
					EXPECT_TRUE(diaglist.at(i).Level <= NOTICE);
					EXPECT_TRUE(diaglist.at(i).Diagnostic_Message != DEVICE_NOT_AVAILABLE);
				}
				EXPECT_TRUE(diaglist.size() > 0);
				std::vector<eros::pin> pins = process->get_pins();
				bool found1 = false;
				bool found2 = false;
				for (std::size_t i = 0; i < pins.size(); ++i)
				{
					if (pins.at(i).ConnectedDevice == "LeftMotorController")
					{
						found1 = true;
						EXPECT_TRUE(pins.at(i).Value == LEFTDRIVE_MAX);
					}
					if (pins.at(i).ConnectedDevice == "RightMotorController")
					{
						found2 = true;
						EXPECT_TRUE(pins.at(i).Value == RIGHTDRIVE_MIN);
					}
				}
				EXPECT_TRUE(found1);
				EXPECT_TRUE(found2);
			}
			{ //Drive half throttle half right
				eros::command drive_cmd;
				drive_cmd.Command = ROVERCOMMAND_DRIVECOMMAND;
				json obj;
				obj["ControlType"] = "OpenLoop";
				obj["ControlGroup"] = "ArcadeDrive";
				obj["ForwardVelocityPerc"] = 75.0;
				obj["RotateZAxisPerc"] = 75.0;
				drive_cmd.CommandText = obj.dump();
				drive_cmd.Description = "Drive Turn Right slow";
				eros::command::ConstPtr cmd_ptr(new eros::command(drive_cmd));
				std::vector<eros::diagnostic> diaglist = process->new_commandmsg(cmd_ptr);
				for (std::size_t i = 0; i < diaglist.size(); i++)
				{
					EXPECT_TRUE(diaglist.at(i).Level <= NOTICE);
					EXPECT_TRUE(diaglist.at(i).Diagnostic_Message != DEVICE_NOT_AVAILABLE);
				}
				EXPECT_TRUE(diaglist.size() > 0);
				std::vector<eros::pin> pins = process->get_pins();
				bool found1 = false;
				bool found2 = false;
				for (std::size_t i = 0; i < pins.size(); ++i)
				{
					if (pins.at(i).ConnectedDevice == "LeftMotorController")
					{
						found1 = true;
						//EXPECT_TRUE(pins.at(i).Value == LEFTDRIVE_MAX);
					}
					if (pins.at(i).ConnectedDevice == "RightMotorController")
					{
						found2 = true;
						//EXPECT_TRUE(pins.at(i).Value == RIGHTDRIVE_MIN);
					}
				}
				EXPECT_TRUE(found1);
				EXPECT_TRUE(found2);
			}
		}
		if (slowrate_fire == true)
		{
			if (slowrate_fire == true)
			{
				std::vector<eros::diagnostic> diagnostics = process->get_diagnostics();
				EXPECT_TRUE(diagnostics.size() >= DIAGNOSTIC_TYPE_COUNT);
				for (std::size_t i = 0; i < diagnostics.size(); ++i)
				{
					EXPECT_TRUE(diagnostics.at(i).Level <= NOTICE);
				}
			}
		}
		current_time += dt;
	}
	EXPECT_TRUE(process->get_runtime() >= time_to_run);
}

TEST(SupportFunctions, TestSupportFunctions)
{
	NavigationNodeProcess *process = initializeprocess("/home/robot/catkin_ws/src/icarus_rover_v2/src/Control/unit_tests/UnitTestControlGroup.xml");
	process = readyprocess(process);
	{
		NavigationNodeProcess::DrivePerc d = process->arcade_mix(0.0, 0.0);
		EXPECT_TRUE(isequal(d.left, 0.0));
		EXPECT_TRUE(isequal(d.right, 0.0));
	}
	{
		NavigationNodeProcess::DrivePerc d = process->arcade_mix(100.0, 0.0);
		EXPECT_TRUE(isequal(d.left, 100.0));
		EXPECT_TRUE(isequal(d.right, 100.0));
	}
	{
		NavigationNodeProcess::DrivePerc d = process->arcade_mix(-100.0, 0.0);
		EXPECT_TRUE(isequal(d.left, -100.0));
		EXPECT_TRUE(isequal(d.right, -100.0));
	}
}
int main(int argc, char **argv)
{
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
