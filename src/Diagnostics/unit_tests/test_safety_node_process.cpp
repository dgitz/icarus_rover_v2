#include <gtest/gtest.h>
#include "ros/ros.h"
#include "ros/time.h"
#include "std_msgs/Bool.h"
#include "icarus_rover_v2/device.h"
#include "icarus_rover_v2/diagnostic.h"
#include "icarus_rover_v2/command.h"
#include "../safety_node_process.h"

std::string Node_Name = "/unittest_safety_node_process";
std::string Host_Name = "ControlModule1";
std::string ros_DeviceName = Host_Name;
std::string ros_ParentDevice = "";
std::string ros_DeviceType = "ControlModule";

SafetyNodeProcess* initializeprocess()
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
	device.HatCount = 1;
	device.SensorCount = 0;
	device.DeviceParent = "None";

	SafetyNodeProcess *process;
	process = new SafetyNodeProcess("safety_node",Node_Name);
	diagnostic = process->init(diagnostic,std::string(Host_Name));
	EXPECT_TRUE(diagnostic.Level <= NOTICE);
	EXPECT_TRUE(process->get_initialized() == false);
	process->set_mydevice(device);
	EXPECT_TRUE(process->get_initialized() == true);
	EXPECT_TRUE(process->get_ready() == false);
	EXPECT_TRUE(process->get_mydevice().DeviceName == device.DeviceName);

	icarus_rover_v2::device hat1;
	hat1.DeviceName = "TerminalHat1";
	hat1.DeviceType = "TerminalHat";
	hat1.DeviceParent = ros_DeviceName;
	hat1.ID = 0;
	hat1.Architecture = "None";
	hat1.ShieldCount = 0;
	hat1.SensorCount = 0;
	hat1.pins.clear();
	icarus_rover_v2::pin newpin;
	newpin.Number = 16;
	newpin.Function = "DigitalInput-Safety";
	newpin.Name = "ArmSwitch";
	newpin.DefaultValue = 0;
	newpin.Value = 0;
	newpin.ParentDevice = hat1.DeviceName;
	hat1.pins.push_back(newpin);


	diagnostic = process->update(0.02);
	EXPECT_TRUE(diagnostic.Level <= NOTICE);

	diagnostic = process->new_devicemsg(hat1);
	EXPECT_TRUE(diagnostic.Level <= NOTICE);


	EXPECT_TRUE(process->get_ready() == false);
	EXPECT_FALSE(process->is_hat_running(hat1.DeviceType,hat1.ID));
	diagnostic = process->set_terminalhat_initialized();
	EXPECT_TRUE(diagnostic.Level <= NOTICE);
	EXPECT_TRUE(process->get_ready() == true);




	EXPECT_TRUE(diagnostic.Level <= NOTICE);
	EXPECT_TRUE(process->is_hat_running(hat1.DeviceType,hat1.ID));

	diagnostic = process->update(0.02);
	EXPECT_TRUE(diagnostic.Level <= NOTICE);
	EXPECT_TRUE(process->get_ready_to_arm() == false);
	EXPECT_TRUE(process->get_pinnumber(newpin.Name) == newpin.Number);
	EXPECT_TRUE(process->set_pinvalue(newpin.Name,0));
	diagnostic = process->update(0.02);
	EXPECT_TRUE(diagnostic.Level <= NOTICE);
	EXPECT_TRUE(process->get_ready_to_arm() == false);

	EXPECT_TRUE(process->set_pinvalue(newpin.Name,1));
	diagnostic = process->update(0.02);
	EXPECT_TRUE(diagnostic.Level <= NOTICE);
	EXPECT_TRUE(process->get_ready_to_arm() == true);

	EXPECT_TRUE(process->set_pinvalue(newpin.Name,0));
	diagnostic = process->update(0.02);
	EXPECT_TRUE(diagnostic.Level <= NOTICE);
	EXPECT_TRUE(process->get_ready_to_arm() == false);

	EXPECT_TRUE(process->set_pinvalue(newpin.Name,1));
	diagnostic = process->update(0.02);
	EXPECT_TRUE(diagnostic.Level <= NOTICE);
	EXPECT_TRUE(process->get_ready_to_arm() == true);

	return process;
}
SafetyNodeProcess* readyprocess(SafetyNodeProcess* process)
{
	icarus_rover_v2::diagnostic diag = process->update(0);
	EXPECT_TRUE(diag.Level <= NOTICE);
	EXPECT_TRUE(process->get_ready() == true);
	return process;
}
TEST(Template,Process_Initialization)
{
	SafetyNodeProcess* process = initializeprocess();
}

TEST(Template,Process_Command)
{
	SafetyNodeProcess* process = initializeprocess();
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
			cmd.Option1 = LEVEL1;
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
		}
		current_time += dt;
	}
	EXPECT_TRUE(process->get_runtime() >= time_to_run);
}

TEST(DeviceOperation,DeviceRunning)
{
	SafetyNodeProcess* process = initializeprocess();
	icarus_rover_v2::diagnostic diagnostic = process->update(0.0);
	EXPECT_TRUE(diagnostic.Level <= NOTICE);
	EXPECT_TRUE(process->update(0.0).Level <= NOTICE);


}
int main(int argc, char **argv){
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
