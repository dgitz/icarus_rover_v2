#include <gtest/gtest.h>
#include "ros/ros.h"
#include "ros/time.h"
#include <math.h>
#include "../CommandNodeProcess.h"

std::string Node_Name = "/unittest_command_node_process";
std::string Host_Name = "unittest";
std::string ros_DeviceName = Host_Name;
#define DIAGNOSTIC_TYPE_COUNT 5
std::string ros_ParentDevice = "";
std::string ros_DeviceType = "ComputeModule";
double SLOW_RATE = 0.1f;
double MEDIUM_RATE = 1.0f;
double FAST_RATE = 10.0f;
int DeviceID = 123;
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

CommandNodeProcess* initializeprocess()
{
	eros::diagnostic diagnostic;
	eros::device device;
	device.DeviceName = ros_DeviceName;
	device.BoardCount = 0;
	device.SensorCount = 0;
	device.DeviceParent = "None";
	device.Architecture = "x86_64";

	CommandNodeProcess* process;
	process = new CommandNodeProcess;
	process->initialize("command_node",Node_Name,Host_Name,ROVER,ROBOT_CONTROLLER,CONTROLLER_NODE);
	std::vector<uint8_t> diagnostic_types;
	diagnostic_types.push_back(SOFTWARE);
	diagnostic_types.push_back(DATA_STORAGE);
	diagnostic_types.push_back(SYSTEM_RESOURCE);
	diagnostic_types.push_back(REMOTE_CONTROL);
	diagnostic_types.push_back(TARGET_ACQUISITION);
	process->enable_diagnostics(diagnostic_types);
	process->finish_initialization();
	EXPECT_TRUE(diagnostic.Level <= NOTICE);
	EXPECT_TRUE(process->is_initialized() == false);
	process->set_mydevice(device);
	EXPECT_TRUE(process->is_initialized() == true);
	EXPECT_TRUE(process->get_mydevice().DeviceName == device.DeviceName);
	EXPECT_EQ(process->get_currentcommand().Command,ROVERCOMMAND_BOOT);
	EXPECT_EQ(process->get_currentstate(),NODESTATE_BOOTING);
	EXPECT_TRUE(diagnostic.Level <= NOTICE);
	process->set_batterylevel_perc(100.0);
	return process;
}
CommandNodeProcess* readyprocess(CommandNodeProcess* process)
{
	eros::diagnostic diag = process->update(0.0,0.0);
	EXPECT_TRUE(diag.Level <= NOTICE);
	EXPECT_TRUE(process->is_ready() == true);
	EXPECT_EQ(process->get_currentstate(),NODESTATE_RUNNING);
	EXPECT_EQ(process->get_currentcommand().Command,ROVERCOMMAND_NONE);
	{
		process->update_diagnostic(SYSTEM_RESOURCE, INFO, NOERROR, "Not Checking System Resources during Unit Test.");
		process->update_diagnostic(TARGET_ACQUISITION, INFO, NOERROR, "Not checking Target Acquisition during Unit Test");
		
		std::vector<eros::diagnostic> diagnostics = process->get_diagnostics();
		EXPECT_TRUE(diagnostics.size() == DIAGNOSTIC_TYPE_COUNT);
		for (std::size_t i = 0; i < diagnostics.size(); ++i)
		{	
			if(diagnostics.at(i).Diagnostic_Type == REMOTE_CONTROL) // This should not be ok yet, waiting on a user command message
			{

			}
			else
			{
				EXPECT_TRUE(diagnostics.at(i).Level <= NOTICE);	
			}		
		}
	}
	return process;
}
TEST(Template,Process_Initialization)
{
	CommandNodeProcess* process = initializeprocess();
	//Check a few Command Maps
	{
		EXPECT_TRUE(process->map_RoverCommand_ToString(ROVERCOMMAND_BOOT) == "BOOT") ;
		EXPECT_TRUE(process->map_RoverCommand_ToInt("BOOT") == ROVERCOMMAND_BOOT);
		EXPECT_TRUE(process->map_RoverCommand_ToInt("ARM ROBOT") == ROVERCOMMAND_ARM);
		EXPECT_TRUE(process->map_RoverCommand_ToInt("STOP MOVEMENT") == ROVERCOMMAND_STOPMOVEMENT);
		EXPECT_TRUE(process->map_RoverCommand_ToInt("A Command that will never exist.") == ROVERCOMMAND_UNDEFINED);
	}
}
TEST(PeriodicCommands,TestA)
{
	/*
	 DIAGNOSTIC LEVEL1 @ FAST_RATE
	 DIAGNOSTIC LEVEL2 @ MEDIUM RATE
	 DIAGNOSTIC LEVEL3 @ SLOW RATE
	 */
	eros::diagnostic diagnostic;
	CommandNodeProcess* process = initializeprocess();
	process = readyprocess(process);

	double time_to_run = 20.0;
	double dt = 0.001;
	double current_time = 0.0;
	int level1_counter = 0;
	int level2_counter = 0;
	int level3_counter = 0;
	while(current_time <= time_to_run)
	{
		eros::diagnostic diag = process->update(dt,current_time);
		EXPECT_TRUE(diag.Level <= NOTICE);
		std::vector<eros::command> p_commands = process->get_PeriodicCommands();
		for(std::size_t i = 0; i < p_commands.size(); i++)
		{
			EXPECT_TRUE(p_commands.at(i).Command == ROVERCOMMAND_RUNDIAGNOSTIC);
			if(p_commands.at(i).Option1 		== LEVEL1){ level1_counter++;}
			else if(p_commands.at(i).Option1	== LEVEL2)
			{ 
				level2_counter++;
				{
					std::vector<eros::diagnostic> diagnostics = process->get_diagnostics();
					EXPECT_TRUE(diagnostics.size() >= DIAGNOSTIC_TYPE_COUNT);
					for (std::size_t i = 0; i < diagnostics.size(); ++i)
					{
						if(diagnostics.at(i).Diagnostic_Type == REMOTE_CONTROL) // This should not be ok yet, waiting on a user command message
						{

						}
						else
						{
							EXPECT_TRUE(diagnostics.at(i).Level <= NOTICE);
						}
					}
				}
			}
			else if(p_commands.at(i).Option1 	== LEVEL3){ level3_counter++;}
		}

		current_time += dt;
	}
	EXPECT_TRUE(process->get_runtime() >= time_to_run);
	EXPECT_TRUE(level1_counter == ((int)time_to_run*10)-1);
	EXPECT_TRUE(level2_counter == ((int)time_to_run));
}

TEST(ArmDisarm,TestA)
{
	eros::diagnostic diagnostic;
	CommandNodeProcess* process = initializeprocess();
	process = readyprocess(process);

	diagnostic = process->update(1.0/(FAST_RATE*1000.0),(double)(0)*1.0/(FAST_RATE*1000.0));
	EXPECT_TRUE(diagnostic.Level <= NOTICE);
	EXPECT_EQ(process->get_currentstate(),NODESTATE_RUNNING);
	EXPECT_EQ(process->get_currentcommand().Command,ROVERCOMMAND_NONE);
	for(int i = 0; i < 10; i++)
	{
		diagnostic = process->update(1.0/(FAST_RATE*1000.0),(double)(i)*1.0/(FAST_RATE*1000.0));
		EXPECT_TRUE(diagnostic.Level <= NOTICE);
		EXPECT_EQ(process->get_armeddisarmed_state(),ARMEDSTATUS_DISARMED_CANNOTARM);
	}
	std::vector<std::string> ready_to_arm_topics;
	for(int i = 0; i < 7; i++)
	{
		std::string topic = "topic" + boost::lexical_cast<std::string>(i);
		ready_to_arm_topics.push_back(topic);
	}
	diagnostic = process->init_readytoarm_list(ready_to_arm_topics);
	EXPECT_TRUE(diagnostic.Level <= NOTICE);
	EXPECT_EQ(process->get_armeddisarmed_state(),ARMEDSTATUS_DISARMED_CANNOTARM);
	EXPECT_EQ(process->get_ReadyToArmList().size(),ready_to_arm_topics.size());
	EXPECT_TRUE(process->get_ReadyToArmList().size() > 0);
	for(std::size_t i = 0; i < ready_to_arm_topics.size(); i++)
	{
		process->new_readytoarmmsg(ready_to_arm_topics.at(i),false);
		diagnostic = process->update(1.0/(FAST_RATE*1000.0),(double)(i)*1.0/(FAST_RATE*1000.0));
		EXPECT_TRUE(diagnostic.Level <= NOTICE);
		EXPECT_EQ(process->get_armeddisarmed_state(),ARMEDSTATUS_DISARMED_CANNOTARM);
	}
	for(std::size_t i = 0; i < (ready_to_arm_topics.size()-1); i++)
	{
		process->new_readytoarmmsg(ready_to_arm_topics.at(i),true);
		diagnostic = process->update(1.0/(FAST_RATE*1000.0),(double)(i)*1.0/(FAST_RATE*1000.0));
		EXPECT_TRUE(diagnostic.Level <= NOTICE);
		EXPECT_EQ(process->get_armeddisarmed_state(),ARMEDSTATUS_DISARMED_CANNOTARM);
	}
	process->new_readytoarmmsg(ready_to_arm_topics.at(ready_to_arm_topics.size()-1),true);
	diagnostic = process->update(1.0/(FAST_RATE*1000.0),(double)(0)*1.0/(FAST_RATE*1000.0));
	EXPECT_EQ(process->get_armeddisarmed_state(),ARMEDSTATUS_DISARMED);
	process->new_readytoarmmsg(ready_to_arm_topics.at(0),false);
	diagnostic = process->update(1.0/(FAST_RATE*1000.0),(double)(0)*1.0/(FAST_RATE*1000.0));
	EXPECT_TRUE(diagnostic.Level <= NOTICE);
	EXPECT_EQ(process->get_armeddisarmed_state(),ARMEDSTATUS_DISARMED_CANNOTARM);


	process->new_readytoarmmsg(ready_to_arm_topics.at(0),true);
	diagnostic = process->update(1.0/(FAST_RATE*1000.0),(double)(0)*1.0/(FAST_RATE*1000.0));
	EXPECT_TRUE(diagnostic.Level <= NOTICE);
	EXPECT_EQ(process->get_armeddisarmed_state(),ARMEDSTATUS_DISARMED);
	eros::command arm_command;
	arm_command.Command = ROVERCOMMAND_ARM;
	eros::command::ConstPtr cmd_ptr(new eros::command(arm_command));
	diagnostic = process->new_user_commandmsg(cmd_ptr);
	EXPECT_TRUE(diagnostic.Level <= NOTICE);
	std::vector<eros::diagnostic> diagnostics = process->get_diagnostics();
	for (std::size_t i = 0; i < diagnostics.size(); ++i)
	{
		EXPECT_TRUE(diagnostics.at(i).Level <= NOTICE);
	}
	EXPECT_EQ(process->get_currentcommand().Command,ROVERCOMMAND_ARM);
	for(int i = 0; i < 10; i++)
	{
		diagnostic = process->update(1.0/(FAST_RATE*1000.0),(double)(i)*1.0/(FAST_RATE*1000.0));
		EXPECT_TRUE(diagnostic.Level <= NOTICE);
		EXPECT_EQ(process->get_armeddisarmed_state(),ARMEDSTATUS_ARMED);
	}
	diagnostics = process->get_diagnostics();
	for (std::size_t i = 0; i < diagnostics.size(); ++i)
	{
		EXPECT_TRUE(diagnostics.at(i).Level <= NOTICE);
	}
					

}

TEST(Scripting,ScriptExecutor)
{
	{
		CommandNodeProcess* process = initializeprocess();
		//This file should always have exactly 1 command being published
		eros::diagnostic diagnostic = process->load_loadscriptingfiles("/home/robot/catkin_ws/src/icarus_rover_v2/src/Control/unit_tests/scriptfiles/Test1/");

		process->print_scriptcommand_list();
		
		EXPECT_TRUE(diagnostic.Level <= NOTICE);
		EXPECT_TRUE(process->get_scriptexecutiontime() > 0.0);
		double script_execution_time = process->get_scriptexecutiontime();
		double dt = 0.01;
		double current_time = 0.0;
		while(current_time < (process->get_scriptexecutiontime()-dt))
		{
			std::vector<eros::command> prev_buffer = process->get_command_buffer();
			diagnostic = process->update(dt,current_time);
			EXPECT_TRUE(diagnostic.Level <= NOTICE);
			std::vector<eros::command> command_buffer = process->get_command_buffer();
			EXPECT_TRUE(command_buffer.size() == 1);
			if(process->get_runtime() < (3.0-dt/1000.0))
			{
				EXPECT_TRUE(command_buffer.at(0).Command == ROVERCOMMAND_STOPMOVEMENT);
			}
			else if(process->get_runtime() < (4.5-dt/1000.0))
			{
				EXPECT_TRUE(command_buffer.at(0).Command == ROVERCOMMAND_DRIVECOMMAND);
			}
			else if(process->get_runtime() < (6.0-dt/1000.0))
			{
				EXPECT_TRUE(command_buffer.at(0).Command == ROVERCOMMAND_DRIVECOMMAND);
			}
			else if(process->get_runtime() < (6.5-dt/1000.0))
			{
				EXPECT_TRUE(command_buffer.at(0).Command == ROVERCOMMAND_DRIVECOMMAND);
			}
			else if(process->get_runtime() < (7.0-dt/1000.0))
			{
				EXPECT_TRUE(command_buffer.at(0).Command == ROVERCOMMAND_DRIVECOMMAND);
			}
			else if(process->get_runtime() < (15.0-dt/1000.0))
			{
				EXPECT_TRUE(command_buffer.at(0).Command == ROVERCOMMAND_STOPMOVEMENT);
			}

			current_time += dt;
			
		}
		
		//Should not be anything left in the script queue now
		while(process->get_runtime() < (2.0*script_execution_time))
		{
			diagnostic = process->update(dt,current_time);
			EXPECT_TRUE(diagnostic.Level <= NOTICE);
			std::vector<eros::command> command_buffer = process->get_command_buffer();
			EXPECT_TRUE(command_buffer.size() == 0);
			current_time += dt;
		}

	}
	{
		CommandNodeProcess* process = initializeprocess();
		//This file will have multiple commands being published at times
		eros::diagnostic diagnostic = process->load_loadscriptingfiles("/home/robot/catkin_ws/src/icarus_rover_v2/src/Control/unit_tests/scriptfiles/Test2/");

		process->print_scriptcommand_list();
		EXPECT_TRUE(diagnostic.Level <= NOTICE);
		EXPECT_TRUE(process->get_scriptexecutiontime() > 0.0);
		double script_execution_time = process->get_scriptexecutiontime();
		double dt = 0.01;
		double current_time = 0.0;
		uint8_t arm_count = 0;
		while(process->get_runtime() < (process->get_scriptexecutiontime()-dt))
		{
			diagnostic = process->update(dt,current_time);
			EXPECT_TRUE(diagnostic.Level <= NOTICE);
			std::vector<eros::command> command_buffer = process->get_command_buffer();
			if(process->get_runtime() < (1.0-dt/1000.0))
			{
				EXPECT_TRUE(command_buffer.size() == 1);
				EXPECT_TRUE(command_buffer.at(0).Command == ROVERCOMMAND_STOPMOVEMENT);
			}
			else if(process->get_runtime() < (2.0-dt/1000.0))
			{
				EXPECT_TRUE(command_buffer.size() <= 1);
				if(command_buffer.size() > 0)
				{
					if(command_buffer.at(0).Command == ROVERCOMMAND_ARM)
					{
						arm_count++;
					}
				}
			}
			else if(process->get_runtime() < (7.0-dt/1000.0))
			{
				EXPECT_TRUE(command_buffer.at(0).Command == ROVERCOMMAND_DRIVECOMMAND);
			}
			else if(process->get_runtime() < (15.0-dt/1000.0))
			{
				EXPECT_TRUE(command_buffer.at(0).Command == ROVERCOMMAND_STOPMOVEMENT);
			}

			current_time += dt;
		}
		EXPECT_TRUE(arm_count == 5);
		//Should not be anything left in the script queue now

		while(process->get_runtime() < (2.0*script_execution_time))
		{
			diagnostic = process->update(dt,current_time);
			EXPECT_TRUE(diagnostic.Level <= NOTICE);
			std::vector<eros::command> command_buffer = process->get_command_buffer();
			EXPECT_TRUE(command_buffer.size() == 0);
			current_time += dt;
		}


	}


}

int main(int argc, char **argv){
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}

