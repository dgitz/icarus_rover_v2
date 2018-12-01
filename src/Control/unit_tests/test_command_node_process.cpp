#include <gtest/gtest.h>
#include "ros/ros.h"
#include "ros/time.h"
#include "icarus_rover_v2/device.h"
#include "icarus_rover_v2/diagnostic.h"
#include "../CommandNodeProcess.h"

std::string Node_Name = "/unittest_command_node_process";
std::string Host_Name = "unittest";
std::string ros_DeviceName = Host_Name;
std::string ros_ParentDevice = "";
std::string ros_DeviceType = "ComputeModule";
double SLOW_RATE = 0.1f;
double MEDIUM_RATE = 1.0f;
double FAST_RATE = 10.0f;
int DeviceID = 123;


CommandNodeProcess* initializeprocess()
{
	icarus_rover_v2::diagnostic diagnostic;
	diagnostic.DeviceName = ros_DeviceName;
	diagnostic.Node_Name = Node_Name;
	diagnostic.System = ROVER;
	diagnostic.SubSystem = ROBOT_CONTROLLER;
	diagnostic.Component = CONTROLLER_NODE;

	diagnostic.Diagnostic_Type = NOERROR;
	diagnostic.Level = INFO;
	diagnostic.Diagnostic_Message = INITIALIZING;
	diagnostic.Description = "Node Initializing";

	icarus_rover_v2::device device;
	device.DeviceName = diagnostic.DeviceName;
	device.BoardCount = 0;
	device.SensorCount = 0;
	device.DeviceParent = "None";
	device.Architecture = "x86_64";

	CommandNodeProcess* process;
	process = new CommandNodeProcess;
	process->set_diagnostic(diagnostic);
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
	icarus_rover_v2::diagnostic diag = process->update(0.0,0.0);
	EXPECT_TRUE(diag.Level <= NOTICE);
	EXPECT_TRUE(process->is_ready() == true);
	EXPECT_EQ(process->get_currentstate(),NODESTATE_RUNNING);
	EXPECT_EQ(process->get_currentcommand().Command,ROVERCOMMAND_NONE);
	return process;
}
TEST(Template,Process_Initialization)
{
	CommandNodeProcess* process = initializeprocess();
}
TEST(PeriodicCommands,TestA)
{
	/*
	 DIAGNOSTIC LEVEL1 @ FAST_RATE
	 DIAGNOSTIC LEVEL2 @ MEDIUM RATE
	 DIAGNOSTIC LEVEL3 @ SLOW RATE
	 */
	icarus_rover_v2::diagnostic diagnostic;
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
		icarus_rover_v2::diagnostic diag = process->update(dt,current_time);
		EXPECT_TRUE(diag.Level <= NOTICE);
		std::vector<icarus_rover_v2::command> p_commands = process->get_PeriodicCommands();
		for(std::size_t i = 0; i < p_commands.size(); i++)
		{
			EXPECT_TRUE(p_commands.at(i).Command == ROVERCOMMAND_RUNDIAGNOSTIC);
			if(p_commands.at(i).Option1 		== LEVEL1){ level1_counter++;}
			else if(p_commands.at(i).Option1	== LEVEL2){ level2_counter++;}
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

	icarus_rover_v2::diagnostic diagnostic;
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
	for(int i = 0; i < ready_to_arm_topics.size(); i++)
	{
		process->new_readytoarmmsg(ready_to_arm_topics.at(i),false);
		diagnostic = process->update(1.0/(FAST_RATE*1000.0),(double)(i)*1.0/(FAST_RATE*1000.0));
		EXPECT_TRUE(diagnostic.Level <= NOTICE);
		EXPECT_EQ(process->get_armeddisarmed_state(),ARMEDSTATUS_DISARMED_CANNOTARM);
	}
	for(int i = 0; i < (ready_to_arm_topics.size()-1); i++)
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
	icarus_rover_v2::command arm_command;
	arm_command.Command = ROVERCOMMAND_ARM;
	icarus_rover_v2::command::ConstPtr cmd_ptr(new icarus_rover_v2::command(arm_command));
	diagnostic = process->new_user_commandmsg(cmd_ptr);
	EXPECT_TRUE(diagnostic.Level <= NOTICE);
	EXPECT_EQ(process->get_currentcommand().Command,ROVERCOMMAND_ARM);
	for(int i = 0; i < 10; i++)
	{
		diagnostic = process->update(1.0/(FAST_RATE*1000.0),(double)(i)*1.0/(FAST_RATE*1000.0));
		EXPECT_TRUE(diagnostic.Level <= NOTICE);
		EXPECT_EQ(process->get_armeddisarmed_state(),ARMEDSTATUS_ARMED);
	}

}
/*
TEST(AutoRecharge,TestA)
{
	icarus_rover_v2::diagnostic diagnostic;
	CommandNodeProcess* process = initializeprocess();
	process = readyprocess(process);

	diagnostic = process->update(1.0/(FAST_RATE*1000.0));
	EXPECT_TRUE(diagnostic.Level <= NOTICE);
	EXPECT_EQ(process->get_currentstate(),NODESTATE_RUNNING);
	EXPECT_EQ(process->get_currentcommand().Command,ROVERCOMMAND_NONE);
	diagnostic = process->update(1.0/(FAST_RATE*1000.0));
	EXPECT_TRUE(diagnostic.Level <= NOTICE);
	process->set_batterylevel_perc(20.0);

	diagnostic = process->update(1.0/(FAST_RATE*1000.0));
	EXPECT_TRUE(diagnostic.Level <= NOTICE);
	EXPECT_EQ(process->get_currentcommand().Command,ROVERCOMMAND_SEARCHFOR_RECHARGE_FACILITY);
	EXPECT_EQ(process->get_currentstate(),NODESTATE_SEARCHING_FOR_RECHARGE_FACILITY);
	diagnostic = process->update(1.0/(FAST_RATE*1000.0));
	EXPECT_TRUE(diagnostic.Level <= NOTICE);
	diagnostic = process->new_targetmsg("unknownA");  //Should result in a WARN
	EXPECT_TRUE(diagnostic.Level == WARN);
	diagnostic = process->new_targetmsg("outlet");
	EXPECT_TRUE(diagnostic.Level <= NOTICE);
	EXPECT_EQ(process->get_currentcommand().Command,ROVERCOMMAND_STOPSEARCHFOR_RECHARGE_FACILITY);
	EXPECT_EQ(process->get_currentstate(),NODESTATE_ACQUIRING_TARGET);
	diagnostic = process->update(1.0/(FAST_RATE*1000.0));
	EXPECT_TRUE(diagnostic.Level <= NOTICE);
	EXPECT_EQ(process->get_currentcommand().Command,ROVERCOMMAND_ACQUIRE_TARGET);
	EXPECT_EQ(process->get_currentstate(),NODESTATE_ACQUIRING_TARGET);
}
 */
int main(int argc, char **argv){
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}

