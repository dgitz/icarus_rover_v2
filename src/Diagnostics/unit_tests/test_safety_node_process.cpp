#include <gtest/gtest.h>
#include "ros/ros.h"
#include "ros/time.h"
#include "std_msgs/Bool.h"
#include "../SafetyNodeProcess.h"

std::string Node_Name = "/unittest_safety_node_process";
std::string Host_Name = "ControlModule1";
std::string ros_DeviceName = Host_Name;
std::string ros_ParentDevice = "";
std::string ros_DeviceType = DEVICETYPE_CONTROLMODULE;
#define DIAGNOSTIC_TYPE_COUNT 4

SafetyNodeProcess* initializeprocess()
{

	eros::device device;
	device.DeviceName = ros_DeviceName;
	device.HatCount = 1;
	device.SensorCount = 0;
	device.DeviceParent = "None";

	SafetyNodeProcess *process;
	process = new SafetyNodeProcess;
    process->initialize("safety_node",Node_Name,Host_Name,ROVER,ROBOT_CONTROLLER,DIAGNOSTIC_NODE);
	std::vector<uint8_t> diagnostic_types;
	diagnostic_types.push_back(SOFTWARE);
	diagnostic_types.push_back(DATA_STORAGE);
	diagnostic_types.push_back(SYSTEM_RESOURCE);
	diagnostic_types.push_back(REMOTE_CONTROL);
	process->enable_diagnostics(diagnostic_types);
	process->finish_initialization();
	EXPECT_TRUE(process->is_initialized() == false);
	process->set_mydevice(device);
	EXPECT_TRUE(process->is_initialized() == true);
	EXPECT_TRUE(process->get_mydevice().DeviceName == device.DeviceName);

	eros::device hat1;
	hat1.DeviceName = "TerminalHat1";
	hat1.DeviceType = DEVICETYPE_TERMINALHAT;
	hat1.DeviceParent = ros_DeviceName;
	hat1.ID = 0;
	hat1.Architecture = "None";
	hat1.ShieldCount = 0;
	hat1.SensorCount = 0;
	hat1.pins.clear();
	eros::pin newpin;
	newpin.Function = "DigitalInput-Safety";
	newpin.Name = "ArmSwitch";
	newpin.DefaultValue = 0;
	newpin.Value = 0;
	newpin.ParentDevice = hat1.DeviceName;
	hat1.pins.push_back(newpin);


	eros::diagnostic diagnostic = process->update(0.02,0.02);
	EXPECT_TRUE(diagnostic.Level <= NOTICE);

    eros::device::ConstPtr hat_ptr(new eros::device(hat1));
	diagnostic = process->new_devicemsg(hat_ptr);
	EXPECT_TRUE(diagnostic.Level <= NOTICE);


	EXPECT_TRUE(process->is_ready() == false);
	EXPECT_FALSE(process->is_hat_running(hat1.DeviceType,hat1.ID));
	diagnostic = process->set_terminalhat_initialized();
	EXPECT_TRUE(diagnostic.Level <= NOTICE);
	EXPECT_TRUE(process->is_ready() == true);

	EXPECT_TRUE(diagnostic.Level <= NOTICE);
	EXPECT_TRUE(process->is_hat_running(hat1.DeviceType,hat1.ID));

	diagnostic = process->update(0.02,0.04);
	EXPECT_TRUE(diagnostic.Level <= NOTICE);
	EXPECT_TRUE(process->get_ready_to_arm() == false);
	EXPECT_TRUE(process->set_pinvalue(newpin.Name,0));
	diagnostic = process->update(0.02,0.06);
	EXPECT_TRUE(diagnostic.Level <= NOTICE);
	EXPECT_TRUE(process->get_ready_to_arm() == false);

	EXPECT_TRUE(process->set_pinvalue(newpin.Name,1));
	diagnostic = process->update(0.02,0.08);
	EXPECT_TRUE(diagnostic.Level <= NOTICE);
	EXPECT_TRUE(process->get_ready_to_arm() == true);

	EXPECT_TRUE(process->set_pinvalue(newpin.Name,0));
	diagnostic = process->update(0.02,0.10);
	EXPECT_TRUE(diagnostic.Level <= NOTICE);
	EXPECT_TRUE(process->get_ready_to_arm() == false);

	EXPECT_TRUE(process->set_pinvalue(newpin.Name,1));
	diagnostic = process->update(0.02,0.12);
	EXPECT_TRUE(diagnostic.Level <= NOTICE);
	EXPECT_TRUE(process->get_ready_to_arm() == true);

	return process;
}
SafetyNodeProcess* readyprocess(SafetyNodeProcess* process)
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
	initializeprocess();
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
			{
				std::vector<eros::diagnostic> diagnostics = process->get_diagnostics();
				EXPECT_TRUE(diagnostics.size() == DIAGNOSTIC_TYPE_COUNT);
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

TEST(DeviceOperation,DeviceRunning)
{
	SafetyNodeProcess* process = initializeprocess();
	eros::diagnostic diagnostic = process->update(0.0,0.0);
	EXPECT_TRUE(diagnostic.Level <= NOTICE);
	EXPECT_TRUE(process->update(0.0,0.0).Level <= NOTICE);


}
int main(int argc, char **argv){
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
