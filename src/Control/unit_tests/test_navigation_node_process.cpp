#include <gtest/gtest.h>
#include "ros/ros.h"
#include "ros/time.h"
#include "../NavigationNodeProcess.h"

std::string Node_Name = "/unittest_navigation_node_process";
std::string Host_Name = "unittest";
std::string ros_DeviceName = Host_Name;

#define LEFTDRIVE_MAX 1900
#define LEFTDRIVE_DEFAULT 1450
#define LEFTDRIVE_MIN 1100
#define RIGHTDRIVE_MAX 1050
#define RIGHTDRIVE_DEFAULT 1525
#define RIGHTDRIVE_MIN 1950

bool isequal(double a,double b)
{
	double v = a-b;
	if(fabs(v) < .0000001)
	{
		return true;
	}
	else
	{
		return false;
	}
}
NavigationNodeProcess* initializeprocess(std::string controlgroup_filepath)
{
	eros::diagnostic diagnostic;
	diagnostic.DeviceName = ros_DeviceName;
	diagnostic.Node_Name = Node_Name;
	diagnostic.System = ROVER;
	diagnostic.SubSystem = ROBOT_CONTROLLER;
	diagnostic.Component = COMMUNICATION_NODE;

	diagnostic.Diagnostic_Type = NOERROR;
	diagnostic.Level = INFO;
	diagnostic.Diagnostic_Message = INITIALIZING;
	diagnostic.Description = "Node Initializing";

	eros::device device;
	device.DeviceName = diagnostic.DeviceName;
	device.BoardCount = 0;
	device.SensorCount = 0;
	device.DeviceParent = "None";
	device.Architecture = "x86_64";

	NavigationNodeProcess *process;
	process = new NavigationNodeProcess;
	process->initialize("navigation_node",Node_Name,Host_Name);
	process->set_diagnostic(diagnostic);
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

	{	//PIN1
		eros::pin newpin;
		newpin.ConnectedDevice = "LeftMotorController";
		newpin.Number = 0;
		newpin.DefaultValue=LEFTDRIVE_DEFAULT;
		newpin.MaxValue=LEFTDRIVE_MAX;
		newpin.MinValue=LEFTDRIVE_MIN;
		newpin.Function = "PWMOutput";
		servohat1_device.pins.push_back(newpin);
	}

	{	//PIN2
		eros::pin newpin;
		newpin.ConnectedDevice = "RightMotorController";
		newpin.Number = 1;
		newpin.DefaultValue=RIGHTDRIVE_DEFAULT;
		newpin.MaxValue=RIGHTDRIVE_MAX;
		newpin.MinValue=RIGHTDRIVE_MIN;
		newpin.Function = "PWMOutput-NonActuator";
		servohat1_device.pins.push_back(newpin);
	}

	EXPECT_TRUE(process->is_initialized() == true);

	EXPECT_TRUE(process->is_ready() == false);

	eros::device::ConstPtr dev_ptr(new eros::device(servohat1_device));
	diagnostic = process->new_devicemsg(dev_ptr);
	EXPECT_TRUE(diagnostic.Level <= NOTICE);
	diagnostic = process->fetch_complete();
	diagnostic = process->load_controlgroupfile();
	EXPECT_TRUE(diagnostic.Level <= NOTICE);
	EXPECT_TRUE(diagnostic.Level <= NOTICE);

	return process;
}
NavigationNodeProcess* readyprocess(NavigationNodeProcess* process)
{
	eros::diagnostic diag = process->update(0.0,0.0);
	EXPECT_TRUE(diag.Level <= NOTICE);
	EXPECT_TRUE(process->is_ready() == true);
	process->print_controlgroups();
	std::vector<NavigationNodeProcess::ControlGroup> controlgroups = process->get_controlgroups();
	EXPECT_TRUE(controlgroups.size() > 0);
	EXPECT_TRUE(controlgroups.at(0).outputs.at(0).pin.Number == 0); //PIN1
	EXPECT_TRUE(controlgroups.at(0).outputs.at(1).pin.Number == 1); //PIN2
	return process;
}
TEST(Template,Process_Initialization)
{
	NavigationNodeProcess* process = initializeprocess("/home/robot/catkin_ws/src/icarus_rover_v2/src/Control/unit_tests/UnitTestControlGroup.xml");
}
TEST(Template,Process_Command)
{
	NavigationNodeProcess* process = initializeprocess("/home/robot/catkin_ws/src/icarus_rover_v2/src/Control/unit_tests/UnitTestControlGroup.xml");
	process = readyprocess(process);
	{
		std::vector<eros::pin> pins = process->get_pins();
		bool found1 = false;
		bool found2 = false;
		for(std::size_t i = 0; i < pins.size(); ++i)
		{
			if(pins.at(i).ConnectedDevice=="LeftMotorController")
			{
				found1 = true;
				EXPECT_TRUE(pins.at(i).Value == pins.at(i).DefaultValue);
				EXPECT_TRUE(pins.at(i).MinValue == LEFTDRIVE_MIN);
				EXPECT_TRUE(pins.at(i).Value == LEFTDRIVE_DEFAULT);
				EXPECT_TRUE(pins.at(i).MaxValue == LEFTDRIVE_MAX);
			}
			if(pins.at(i).ConnectedDevice=="RightMotorController")
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
	bool fastrate_fire = false; //10 Hz
	bool mediumrate_fire = false; //1 Hz
	bool slowrate_fire = false; //0.1 Hz
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

		if(fastrate_fire == true) //Nothing to do here
		{
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

			{	//Drive Full Forwards
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
				for(std::size_t i = 0; i < diaglist.size(); i++)
				{
					EXPECT_TRUE(diaglist.at(i).Level <= NOTICE);
					EXPECT_TRUE(diaglist.at(i).Diagnostic_Message != DEVICE_NOT_AVAILABLE);
				}
				EXPECT_TRUE(diaglist.size() > 0);
				std::vector<eros::pin> pins = process->get_pins();
				bool found1 = false;
				bool found2 = false;
				for(std::size_t i = 0; i < pins.size(); ++i)
				{
					if(pins.at(i).ConnectedDevice=="LeftMotorController")
					{
						found1 = true;
						EXPECT_TRUE(pins.at(i).Value == LEFTDRIVE_MAX);
					}
					if(pins.at(i).ConnectedDevice=="RightMotorController")
					{
						found2 = true;
						EXPECT_TRUE(pins.at(i).Value == RIGHTDRIVE_MAX);
					}
				}
				EXPECT_TRUE(found1);
				EXPECT_TRUE(found2);
			}
			{	//Drive Full Reverse
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
				for(std::size_t i = 0; i < diaglist.size(); i++)
				{
					EXPECT_TRUE(diaglist.at(i).Level <= NOTICE);
					EXPECT_TRUE(diaglist.at(i).Diagnostic_Message != DEVICE_NOT_AVAILABLE);
				}
				EXPECT_TRUE(diaglist.size() > 0);
				std::vector<eros::pin> pins = process->get_pins();
				bool found1 = false;
				bool found2 = false;
				for(std::size_t i = 0; i < pins.size(); ++i)
				{
					if(pins.at(i).ConnectedDevice=="LeftMotorController")
					{
						found1 = true;
						
						EXPECT_TRUE(pins.at(i).Value == LEFTDRIVE_MIN);
					}
					if(pins.at(i).ConnectedDevice=="RightMotorController")
					{
						found2 = true;
						EXPECT_TRUE(pins.at(i).Value == RIGHTDRIVE_MIN);
					}
				}
				EXPECT_TRUE(found1);
				EXPECT_TRUE(found2);
			}
			{	//Drive Full Left
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
				for(std::size_t i = 0; i < diaglist.size(); i++)
				{
					EXPECT_TRUE(diaglist.at(i).Level <= NOTICE);
					EXPECT_TRUE(diaglist.at(i).Diagnostic_Message != DEVICE_NOT_AVAILABLE);
				}
				EXPECT_TRUE(diaglist.size() > 0);
				std::vector<eros::pin> pins = process->get_pins();
				bool found1 = false;
				bool found2 = false;
				for(std::size_t i = 0; i < pins.size(); ++i)
				{
					if(pins.at(i).ConnectedDevice=="LeftMotorController")
					{
						found1 = true;
						EXPECT_TRUE(pins.at(i).Value == LEFTDRIVE_MIN);
					}
					if(pins.at(i).ConnectedDevice=="RightMotorController")
					{
						found2 = true;
						EXPECT_TRUE(pins.at(i).Value == RIGHTDRIVE_MAX);
					}
				}
				EXPECT_TRUE(found1);
				EXPECT_TRUE(found2);
			}
			{	//Drive Full Right
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
				for(std::size_t i = 0; i < diaglist.size(); i++)
				{
					EXPECT_TRUE(diaglist.at(i).Level <= NOTICE);
					EXPECT_TRUE(diaglist.at(i).Diagnostic_Message != DEVICE_NOT_AVAILABLE);
				}
				EXPECT_TRUE(diaglist.size() > 0);
				std::vector<eros::pin> pins = process->get_pins();
				bool found1 = false;
				bool found2 = false;
				for(std::size_t i = 0; i < pins.size(); ++i)
				{
					if(pins.at(i).ConnectedDevice=="LeftMotorController")
					{
						found1 = true;
						EXPECT_TRUE(pins.at(i).Value == LEFTDRIVE_MAX);
					}
					if(pins.at(i).ConnectedDevice=="RightMotorController")
					{
						found2 = true;
						EXPECT_TRUE(pins.at(i).Value == RIGHTDRIVE_MIN);
					}
				}
				EXPECT_TRUE(found1);
				EXPECT_TRUE(found2);
			}
			{	//Drive half throttle half right
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
				for(std::size_t i = 0; i < diaglist.size(); i++)
				{
					EXPECT_TRUE(diaglist.at(i).Level <= NOTICE);
					EXPECT_TRUE(diaglist.at(i).Diagnostic_Message != DEVICE_NOT_AVAILABLE);
				}
				EXPECT_TRUE(diaglist.size() > 0);
				std::vector<eros::pin> pins = process->get_pins();
				bool found1 = false;
				bool found2 = false;
				for(std::size_t i = 0; i < pins.size(); ++i)
				{
					if(pins.at(i).ConnectedDevice=="LeftMotorController")
					{
						found1 = true;
						//EXPECT_TRUE(pins.at(i).Value == LEFTDRIVE_MAX);
					}
					if(pins.at(i).ConnectedDevice=="RightMotorController")
					{
						found2 = true;
						//EXPECT_TRUE(pins.at(i).Value == RIGHTDRIVE_MIN);
					}
				}
				EXPECT_TRUE(found1);
				EXPECT_TRUE(found2);
			}
		}
		if(slowrate_fire == true)
		{

			//Don't run LEVEL3 Test, as this will be called circularly and is only responsible for running this test anyways.
		}
		current_time += dt;
	}
	EXPECT_TRUE(process->get_runtime() >= time_to_run);
}

TEST(SupportFunctions,TestSupportFunctions)
{
	NavigationNodeProcess* process = initializeprocess("/home/robot/catkin_ws/src/icarus_rover_v2/src/Control/unit_tests/UnitTestControlGroup.xml");
	process = readyprocess(process);
	{
		NavigationNodeProcess::DrivePerc d = process->arcade_mix(0.0,0.0);
		EXPECT_TRUE(isequal(d.left,0.0));
		EXPECT_TRUE(isequal(d.right,0.0));
	}
	{
		NavigationNodeProcess::DrivePerc d = process->arcade_mix(100.0,0.0);
		EXPECT_TRUE(isequal(d.left,100.0));
		EXPECT_TRUE(isequal(d.right,100.0));
	}
	{
		NavigationNodeProcess::DrivePerc d = process->arcade_mix(-100.0,0.0);
		EXPECT_TRUE(isequal(d.left,-100.0));
		EXPECT_TRUE(isequal(d.right,-100.0));
	}
}
int main(int argc, char **argv){
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}

