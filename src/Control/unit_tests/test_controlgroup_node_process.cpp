#include <gtest/gtest.h>
#include "ros/ros.h"
#include "ros/time.h"
#include <iostream>
#include <fstream>
#include<boost/algorithm/string/split.hpp>
#include<boost/algorithm/string.hpp>
#include "../ControlGroupNodeProcess.h"

std::string Node_Name = "/unittest_implement_node_process";
std::string Host_Name = "unittest";
std::string ros_DeviceName = Host_Name;
#define DIAGNOSTIC_TYPE_COUNT 4
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
bool isequal(double a, double b, double precision)
{
	double v = fabs(a-b);
	if(v < precision)
	{
		return true;
	}
	else
	{
		return false;
	}
	
}
ControlGroupNodeProcess *initializeprocess(std::string controlgroupconfig_filepath)
{
	eros::device device;
	device.DeviceName = ros_DeviceName;
	device.BoardCount = 0;
	device.SensorCount = 0;
	device.DeviceParent = "None";
	device.Architecture = "x86_64";

	ControlGroupNodeProcess *process;
	process = new ControlGroupNodeProcess;
	process->initialize("controlgroup_node", Node_Name, Host_Name, ROVER, ROBOT_CONTROLLER, CONTROLLER_NODE);
	process->set_config_filepaths(controlgroupconfig_filepath);
	std::vector<uint8_t> diagnostic_types;
	diagnostic_types.push_back(SOFTWARE);
	diagnostic_types.push_back(DATA_STORAGE);
	diagnostic_types.push_back(SYSTEM_RESOURCE);
	diagnostic_types.push_back(ACTUATORS);
	process->enable_diagnostics(diagnostic_types);
	eros::diagnostic diag = process->finish_initialization();
	EXPECT_TRUE(diag.Level <= NOTICE);
	EXPECT_TRUE(process->get_taskstate() == TASKSTATE_INITIALIZING);
	process->set_mydevice(device);
	EXPECT_TRUE(process->get_taskstate() == TASKSTATE_INITIALIZED);
	EXPECT_TRUE(process->get_mydevice().DeviceName == device.DeviceName);
	return process;
}
ControlGroupNodeProcess *readyprocess(ControlGroupNodeProcess *process)
{
	eros::diagnostic diag = process->update(0.0, 0.0);
	//Simulate get pin message for Control Group Output
	{
		eros::pin output_pin;
		output_pin.ConnectedDevice = "ImplementCylinderCommand";
		output_pin.MaxValue = 2000;
		output_pin.MinValue = 1000;
		output_pin.DefaultValue = 0;
		output_pin.Function = "PWMOutput";
		diag = process->set_pinproperties(output_pin);
		EXPECT_TRUE(diag.Level <= NOTICE);
	}

	diag = process->update(0.1, 0.1);
	EXPECT_TRUE(diag.Level <= NOTICE);
	diag = process->update(0.1, 0.2);
	EXPECT_TRUE(diag.Level <= NOTICE);
	EXPECT_TRUE(process->get_taskstate() == TASKSTATE_RUNNING);
	{
		std::vector<eros::diagnostic> diagnostics = process->get_diagnostics();
		//EXPECT_TRUE((int)diagnostics.size() == DIAGNOSTIC_TYPE_COUNT+(2*(int)process->get_controlgroups().size()));
		for (std::size_t i = 0; i < diagnostics.size(); ++i)
		{
			print_diagnostic(WARN,diagnostics.at(i));
			EXPECT_TRUE(diagnostics.at(i).Level <= NOTICE);
		}
	}
	return process;
}
TEST(Operation,CheckPIDComputationFromSpreadsheet)
{
	uint16_t INDEX_COLUMN = 0;
	uint16_t TIMEDELTA_COLUMN = 1;
	uint16_t CURRENT_TIME_COLUMN = 2;
	uint16_t COMMANDVALUE_COLUMN = 3;
	uint16_t INPUTVALUE_COLUMN = 4;
	uint16_t OUTPUTVALUE_COLUMN = 14;
	ControlGroupNodeProcess *process = initializeprocess("/home/robot/catkin_ws/src/icarus_rover_v2/src/Control/unit_tests/UnitTestControlGroup_PID.xml");
	process = readyprocess(process);
	eros::diagnostic diag = process->update(0.0,0.0);
	std::vector<ControlGroup> controlgroups = process->get_controlgroups();
	EXPECT_TRUE(controlgroups.size() == 1);
	eros::signal sense_signal = controlgroups.at(0).get_inputsignal("CurrentPosition");
	EXPECT_TRUE(sense_signal.name == "CurrentPosition");
	eros::signal command_signal = controlgroups.at(0).get_inputsignal("CommandPosition");
	EXPECT_TRUE(command_signal.name == "CommandPosition");

	std::string line;
	std::ifstream myfile ("/home/robot/catkin_ws/src/icarus_rover_v2/src/Control/unit_tests/ControlGroupTest.csv");
	double gain_P = 0.0;
	double gain_I = 0.0;
	double gain_D = 0.0;
	bool process_maindata = false;
	EXPECT_TRUE(myfile.is_open() == true);
	if (myfile.is_open())
	{
		while ( getline (myfile,line) )
		{
			std::vector<std::string> elements;
			boost::split(elements,line,boost::is_any_of(","));
			if(process_maindata == true)
			{
				uint64_t index = std::atoi(elements.at(INDEX_COLUMN).c_str());
				sense_signal.value = std::atof(elements.at(INPUTVALUE_COLUMN).c_str());
				command_signal.value = std::atof(elements.at(COMMANDVALUE_COLUMN).c_str());
				double dt = std::atof(elements.at(TIMEDELTA_COLUMN).c_str());
				double cur_time = std::atof(elements.at(CURRENT_TIME_COLUMN).c_str());
				double expected_output =  std::atof(elements.at(OUTPUTVALUE_COLUMN).c_str());
				{
					eros::signal::ConstPtr signal_ptr(new eros::signal(sense_signal));
					diag = process->new_inputsignalmsg(signal_ptr);
					EXPECT_TRUE(diag.Level <= NOTICE);
				}
				{
					eros::signal::ConstPtr signal_ptr(new eros::signal(command_signal));
					diag = process->new_inputsignalmsg(signal_ptr);
					EXPECT_TRUE(diag.Level <= NOTICE);
				}
				diag = process->update(dt,cur_time);
				EXPECT_TRUE(diag.Level <= NOTICE);

				// Check output signals
				std::vector<eros::signal> output_signals = process->get_outputsignals();
				EXPECT_TRUE(output_signals.size() == 1);
				EXPECT_TRUE(output_signals.at(0).status == SIGNALSTATE_UPDATED);
				EXPECT_TRUE(isequal(output_signals.at(0).value,expected_output,.0001) == true);

				// Check output pins
				std::vector<eros::pin> output_pins = process->get_outputpins();
				EXPECT_TRUE(output_pins.size() == 1);
				EXPECT_TRUE(isequal(output_pins.at(0).Value,(int32_t)expected_output,2.0) == true); //integer comparison
			}
			else
			{
				if(elements.size() > 0)
				{
					if(elements.at(0).compare("P_Gain") == 0)
					{
						gain_P = std::atof(elements.at(1).c_str());
					}
					if(elements.at(0).compare("I_Gain") == 0)
					{
						gain_I = std::atof(elements.at(1).c_str());
					}
					if(elements.at(0).compare("D_Gain") == 0)
					{
						gain_D = std::atof(elements.at(1).c_str());
					}
					if(elements.at(0).compare("Index") == 0)
					{
						diag = process->set_PIDGains(controlgroups.at(0).get_name(),gain_P,gain_I,gain_D);
						EXPECT_TRUE(diag.Level <= NOTICE);
						process_maindata = true;
					}
				}
			}
			
		}
		myfile.close();
	}
}
TEST(Template, Process_Initialization)
{
	{
		ControlGroupNodeProcess *process = initializeprocess("/home/robot/catkin_ws/src/icarus_rover_v2/src/Control/unit_tests/UnitTestControlGroup_PID.xml");
		EXPECT_TRUE(process->get_taskstate() == TASKSTATE_INITIALIZED);
	}
	{
		ControlGroupNodeProcess *process = initializeprocess("/home/robot/catkin_ws/src/icarus_rover_v2/src/Control/unit_tests/UnitTestControlGroup_Direct.xml");
		EXPECT_TRUE(process->get_taskstate() == TASKSTATE_INITIALIZED);
	}
}
TEST(Operation,TimingTest)
{
	//Baseline Checks
	double simple_operation_run = 0.0;
	double single_controlgroup_runtime = 0.0;
	uint64_t loop_count = 1000000;
	{
		ControlGroupNodeProcess *process = initializeprocess("/home/robot/catkin_ws/src/icarus_rover_v2/src/Control/unit_tests/UnitTestControlGroup_PID.xml");
		process = readyprocess(process);
		process->clear_controlgroups();
		eros::diagnostic diag = process->update(0.0,0.5);
		EXPECT_TRUE(diag.Level <= NOTICE);
		struct timeval start;
		struct timeval stop;
		gettimeofday(&start,NULL);
		uint64_t counter = 0;
		while(counter < loop_count)
		{
			diag = process->update(0.1,0.1*counter);
			counter++;
		}
		gettimeofday(&stop,NULL);
		ros::Time ros_start = process->convert_time(start);
		ros::Time ros_stop = process->convert_time(stop);
		simple_operation_run = ros_stop.toSec()-ros_start.toSec();
	}
	{
		ControlGroupNodeProcess *process = initializeprocess("/home/robot/catkin_ws/src/icarus_rover_v2/src/Control/unit_tests/UnitTestControlGroup_PID.xml");
		process = readyprocess(process);
		std::vector<ControlGroup> controlgroups = process->get_controlgroups();
		EXPECT_TRUE(controlgroups.size() == 1);
		eros::signal sense_signal = controlgroups.at(0).get_inputsignal("CurrentPosition");
		EXPECT_TRUE(sense_signal.name == "CurrentPosition");
		sense_signal.value = 0.0;
		eros::signal command_signal = controlgroups.at(0).get_inputsignal("CommandPosition");
		EXPECT_TRUE(command_signal.name == "CommandPosition");
		command_signal.value = 0.0;
		eros::diagnostic diag = process->update(0.0,0.5);
		EXPECT_TRUE(diag.Level <= NOTICE);
		struct timeval start;
		struct timeval stop;
		gettimeofday(&start,NULL);
		uint64_t counter = 0;
		while(counter < loop_count)
		{
			{
				eros::signal::ConstPtr signal_ptr(new eros::signal(sense_signal));
				diag = process->new_inputsignalmsg(signal_ptr);
			}
			{
				eros::signal::ConstPtr signal_ptr(new eros::signal(command_signal));
				process->new_inputsignalmsg(signal_ptr);
			}
			diag = process->update(0.1,0.1*counter);
			std::vector<eros::pin> pins = process->get_outputpins();
			counter++;
		}
		gettimeofday(&stop,NULL);
		ros::Time ros_start = process->convert_time(start);
		ros::Time ros_stop = process->convert_time(stop);
		single_controlgroup_runtime = ros_stop.toSec()-ros_start.toSec();
	}
	printf("Simple (No Control Groups) Run Time: %4.4f (sec) Time Per Loop: %4.4f (nS)\n",simple_operation_run,1000000.0*simple_operation_run/(double)(loop_count));
	printf("Single ControlGroup Run Time: %4.4f (sec) Time Per Loop: %4.4f (nS)\n",single_controlgroup_runtime,1000000.0*single_controlgroup_runtime/(double)(loop_count));
	printf("Speed Reduction Factor: %4.4f\n",single_controlgroup_runtime/simple_operation_run);

}
TEST(Operation,InputReceive_PIDControlGroup)
{
	ControlGroupNodeProcess *process = initializeprocess("/home/robot/catkin_ws/src/icarus_rover_v2/src/Control/unit_tests/UnitTestControlGroup_PID.xml");
	process = readyprocess(process);

	std::vector<ControlGroup> controlgroups = process->get_controlgroups();
	EXPECT_TRUE(controlgroups.size() == 1);
	eros::signal sense_signal = controlgroups.at(0).get_inputsignal("CurrentPosition");
	EXPECT_TRUE(sense_signal.name == "CurrentPosition");
	sense_signal.value = 0.0;
	eros::signal command_signal = controlgroups.at(0).get_inputsignal("CommandPosition");
	EXPECT_TRUE(command_signal.name == "CommandPosition");
	command_signal.value = 0.0;
	std::vector<eros::signal> output_signals = process->get_outputsignals();
	EXPECT_TRUE(output_signals.size() == 1);
	eros::diagnostic diag;
	EXPECT_TRUE(output_signals.at(0).status == SIGNALSTATE_INITIALIZING);
	{
		eros::signal::ConstPtr signal_ptr(new eros::signal(sense_signal));
		diag = process->new_inputsignalmsg(signal_ptr);
	}
	EXPECT_TRUE(diag.Level <= NOTICE);
	output_signals = process->get_outputsignals();
	EXPECT_TRUE(output_signals.at(0).status == SIGNALSTATE_INITIALIZING);
	diag = process->update(0.0,0.0);
	EXPECT_TRUE(diag.Level <= NOTICE);
	output_signals = process->get_outputsignals();
	EXPECT_TRUE(output_signals.at(0).status == SIGNALSTATE_INITIALIZING);
	{
		eros::signal::ConstPtr signal_ptr(new eros::signal(command_signal));
		process->new_inputsignalmsg(signal_ptr);
	}
	EXPECT_TRUE(diag.Level <= NOTICE);
	output_signals = process->get_outputsignals();
	EXPECT_TRUE(output_signals.at(0).status == SIGNALSTATE_INITIALIZING);
	diag = process->update(0.0,0.0);
	EXPECT_TRUE(diag.Level <= NOTICE);
	output_signals = process->get_outputsignals();
	EXPECT_TRUE(output_signals.at(0).status == SIGNALSTATE_UPDATED);
}

TEST(Operation,InputReceive_DirectControlGroup)
{
	ControlGroupNodeProcess *process = initializeprocess("/home/robot/catkin_ws/src/icarus_rover_v2/src/Control/unit_tests/UnitTestControlGroup_Direct.xml");
	process = readyprocess(process);

	std::vector<ControlGroup> controlgroups = process->get_controlgroups();
	EXPECT_TRUE(controlgroups.size() == 1);
	eros::signal input_signal = controlgroups.at(0).get_inputsignal("InputSignal");
	EXPECT_TRUE(input_signal.name == "InputSignal");
	input_signal.value = 0.0;
	std::vector<eros::signal> output_signals = process->get_outputsignals();
	EXPECT_TRUE(output_signals.size() == 1);
	eros::diagnostic diag;
	EXPECT_TRUE(output_signals.at(0).status == SIGNALSTATE_INITIALIZING);
	{
		eros::signal::ConstPtr signal_ptr(new eros::signal(input_signal));
		diag = process->new_inputsignalmsg(signal_ptr);
	}
	EXPECT_TRUE(diag.Level <= NOTICE);
	output_signals = process->get_outputsignals();
	diag = process->update(0.0,0.0);
	EXPECT_TRUE(diag.Level <= NOTICE);
	output_signals = process->get_outputsignals();
	EXPECT_TRUE(output_signals.at(0).status == SIGNALSTATE_UPDATED);
	{
		eros::signal::ConstPtr signal_ptr(new eros::signal(input_signal));
		process->new_inputsignalmsg(signal_ptr);
	}
	EXPECT_TRUE(diag.Level <= NOTICE);
	output_signals = process->get_outputsignals();
	EXPECT_TRUE(output_signals.at(0).status == SIGNALSTATE_UPDATED);
	diag = process->update(0.0,0.0);
	EXPECT_TRUE(diag.Level <= NOTICE);
	output_signals = process->get_outputsignals();
	EXPECT_TRUE(output_signals.at(0).status == SIGNALSTATE_UPDATED);

	{//Set Input to Max and check Output is Max
		input_signal.value = 1.0;
		eros::signal::ConstPtr signal_ptr(new eros::signal(input_signal));
		diag = process->new_inputsignalmsg(signal_ptr);
		diag = process->update(0.0,0.0);
		EXPECT_TRUE(diag.Level <= NOTICE);
		std::vector<eros::pin> output_pins = process->get_outputpins();
		EXPECT_TRUE(output_pins.size() == 1);
		EXPECT_TRUE(isequal(output_pins.at(0).Value,(int32_t)2000.0,2.0) == true); //integer comparison
	}
	{//Set Input to Default and check Output is Default
		input_signal.value = 0.0;
		eros::signal::ConstPtr signal_ptr(new eros::signal(input_signal));
		diag = process->new_inputsignalmsg(signal_ptr);
		diag = process->update(0.0,0.0);
		EXPECT_TRUE(diag.Level <= NOTICE);
		std::vector<eros::pin> output_pins = process->get_outputpins();
		EXPECT_TRUE(output_pins.size() == 1);
		EXPECT_TRUE(isequal(output_pins.at(0).Value,(int32_t)1500.0,2.0) == true); //integer comparison

	}
	{//Set Input to Min and check Output is Min
		input_signal.value = -1.0;
		eros::signal::ConstPtr signal_ptr(new eros::signal(input_signal));
		diag = process->new_inputsignalmsg(signal_ptr);
		diag = process->update(0.0,0.0);
		EXPECT_TRUE(diag.Level <= NOTICE);
		std::vector<eros::pin> output_pins = process->get_outputpins();
		EXPECT_TRUE(output_pins.size() == 1);
		EXPECT_TRUE(isequal(output_pins.at(0).Value,(int32_t)1000.0,2.0) == true); //integer comparison
	}
	{//Set Input higher than expected
		input_signal.value = 2.0;
		eros::signal::ConstPtr signal_ptr(new eros::signal(input_signal));
		diag = process->new_inputsignalmsg(signal_ptr);
		diag = process->update(0.0,0.0);
		EXPECT_TRUE(diag.Level <= NOTICE);
		std::vector<eros::pin> output_pins = process->get_outputpins();
		EXPECT_TRUE(output_pins.size() == 1);
		EXPECT_TRUE(isequal(output_pins.at(0).Value,(int32_t)2000.0,2.0) == true); //integer comparison
	}
}
TEST(Template, Process_Command)
{
	ControlGroupNodeProcess *process = initializeprocess("/home/robot/catkin_ws/src/icarus_rover_v2/src/Control/unit_tests/UnitTestControlGroup_PID.xml");
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
