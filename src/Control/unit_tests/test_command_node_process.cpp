#include <gtest/gtest.h>
#include "ros/ros.h"
#include "ros/time.h"
#include "icarus_rover_v2/device.h"
#include "icarus_rover_v2/diagnostic.h"
#include "../command_node_process.h"

std::string Node_Name = "/unittest_command_node_process";
std::string Host_Name = "unittest";
std::string ros_DeviceName = Host_Name;
std::string ros_ParentDevice = "";
std::string ros_DeviceType = "ComputeModule";
double SLOW_RATE = 0.1f;
double MEDIUM_RATE = 1.0f;
double FAST_RATE = 10.0f;
int DeviceID = 123;


CommandNodeProcess initialize_process();
TEST(PeriodicCommands,TestA)
{
	/*
	 DIAGNOSTIC LEVEL1 @ FAST_RATE
	 DIAGNOSTIC LEVEL2 @ MEDIUM RATE
	 DIAGNOSTIC LEVEL3 @ SLOW RATE
	 */
	icarus_rover_v2::diagnostic diagnostic_status;
	CommandNodeProcess process = initialize_process();

	std::vector<PeriodicCommand> p_commands = process.get_PeriodicCommands();
	int level1_counter = 0;
	int level2_counter = 0;
	int level3_counter = 0;
	double run_time = 0.0;
	for(int i = 0; i < 10600; i++)
	{
		double dt = 1.0/(10.0*FAST_RATE);
		run_time += dt;
		diagnostic_status = process.update(dt);
		EXPECT_TRUE(diagnostic_status.Level <= NOTICE);
		if(process.get_currentcommand().Command == ROVERCOMMAND_RUNDIAGNOSTIC)
		{
			if(process.get_currentcommand().Option1 		== LEVEL1){ level1_counter++;}
			else if(process.get_currentcommand().Option1	== LEVEL2){ level2_counter++;}
			else if(process.get_currentcommand().Option1 	== LEVEL3){ level3_counter++;}
		}
	}
	EXPECT_TRUE(level1_counter == 1009);
	EXPECT_TRUE(level2_counter == 105);
	EXPECT_TRUE(level3_counter == 10);
}

TEST(ArmDisarm,TestA)
{

	icarus_rover_v2::diagnostic diagnostic_status;
	CommandNodeProcess process = initialize_process();

	diagnostic_status = process.update(1.0/(FAST_RATE*1000.0));
	EXPECT_TRUE(diagnostic_status.Level <= NOTICE);
	EXPECT_EQ(process.get_currentstate(),NODESTATE_RUNNING);
	EXPECT_EQ(process.get_currentcommand().Command,ROVERCOMMAND_NONE);
	for(int i = 0; i < 10; i++)
	{
		diagnostic_status = process.update(1.0/(FAST_RATE*1000.0));
		EXPECT_TRUE(diagnostic_status.Level <= NOTICE);
		EXPECT_EQ(process.get_armeddisarmed_state(),ARMEDSTATUS_DISARMED_CANNOTARM);
	}
	std::vector<std::string> ready_to_arm_topics;
	for(int i = 0; i < 7; i++)
	{
		std::string topic = "topic" + boost::lexical_cast<std::string>(i);
		ready_to_arm_topics.push_back(topic);
	}
	diagnostic_status = process.init_readytoarm_list(ready_to_arm_topics);
	EXPECT_TRUE(diagnostic_status.Level <= NOTICE);
	EXPECT_EQ(process.get_armeddisarmed_state(),ARMEDSTATUS_DISARMED_CANNOTARM);
	EXPECT_EQ(process.get_ReadyToArmList().size(),ready_to_arm_topics.size());
	EXPECT_TRUE(process.get_ReadyToArmList().size() > 0);
	for(int i = 0; i < ready_to_arm_topics.size(); i++)
	{
		diagnostic_status = process.new_readytoarmmsg(ready_to_arm_topics.at(i),false);
		EXPECT_TRUE(diagnostic_status.Level <= NOTICE);
		diagnostic_status = process.update(1.0/(FAST_RATE*1000.0));
		EXPECT_TRUE(diagnostic_status.Level <= NOTICE);
		EXPECT_EQ(process.get_armeddisarmed_state(),ARMEDSTATUS_DISARMED_CANNOTARM);
	}
	for(int i = 0; i < (ready_to_arm_topics.size()-1); i++)
	{
		diagnostic_status = process.new_readytoarmmsg(ready_to_arm_topics.at(i),true);
		EXPECT_TRUE(diagnostic_status.Level <= NOTICE);
		diagnostic_status =process.update(1.0/(FAST_RATE*1000.0));
		EXPECT_TRUE(diagnostic_status.Level <= NOTICE);
		EXPECT_EQ(process.get_armeddisarmed_state(),ARMEDSTATUS_DISARMED_CANNOTARM);
	}
	diagnostic_status = process.new_readytoarmmsg(ready_to_arm_topics.at(ready_to_arm_topics.size()-1),true);
	EXPECT_TRUE(diagnostic_status.Level <= NOTICE);
	EXPECT_TRUE(diagnostic_status.Level <= NOTICE);
	diagnostic_status = process.update(1.0/(FAST_RATE*1000.0));
	EXPECT_EQ(process.get_armeddisarmed_state(),ARMEDSTATUS_DISARMED);
	diagnostic_status = process.new_readytoarmmsg(ready_to_arm_topics.at(0),false);
	EXPECT_TRUE(diagnostic_status.Level <= NOTICE);
	EXPECT_TRUE(diagnostic_status.Level <= NOTICE);
	diagnostic_status = process.update(1.0/(FAST_RATE*1000.0));
	EXPECT_EQ(process.get_armeddisarmed_state(),ARMEDSTATUS_DISARMED_CANNOTARM);
	diagnostic_status = process.new_readytoarmmsg(ready_to_arm_topics.at(0),true);
	EXPECT_TRUE(diagnostic_status.Level <= NOTICE);
	diagnostic_status = process.update(1.0/(FAST_RATE*1000.0));
	EXPECT_TRUE(diagnostic_status.Level <= NOTICE);
	EXPECT_EQ(process.get_armeddisarmed_state(),ARMEDSTATUS_DISARMED);
	diagnostic_status = process.new_user_armcommandmsg(ROVERCOMMAND_ARM);
	EXPECT_TRUE(diagnostic_status.Level <= NOTICE);
	EXPECT_EQ(process.get_currentcommand().Command,ROVERCOMMAND_ARM);
	for(int i = 0; i < 10; i++)
	{
		diagnostic_status = process.update(1.0/(FAST_RATE*1000.0));
		EXPECT_TRUE(diagnostic_status.Level <= NOTICE);
		EXPECT_EQ(process.get_armeddisarmed_state(),ARMEDSTATUS_ARMED);
	}

}

TEST(AutoRecharge,TestA)
{
	icarus_rover_v2::diagnostic diagnostic_status;
	CommandNodeProcess process = initialize_process();
	diagnostic_status = process.update(1.0/(FAST_RATE*1000.0));
	EXPECT_TRUE(diagnostic_status.Level <= NOTICE);
	EXPECT_EQ(process.get_currentstate(),NODESTATE_RUNNING);
	EXPECT_EQ(process.get_currentcommand().Command,ROVERCOMMAND_NONE);
	diagnostic_status = process.update(1.0/(FAST_RATE*1000.0));
	EXPECT_TRUE(diagnostic_status.Level <= NOTICE);
	process.set_batterylevel_perc(20.0);

	diagnostic_status = process.update(1.0/(FAST_RATE*1000.0));
	EXPECT_TRUE(diagnostic_status.Level <= NOTICE);
	EXPECT_EQ(process.get_currentcommand().Command,ROVERCOMMAND_SEARCHFOR_RECHARGE_FACILITY);
	EXPECT_EQ(process.get_currentstate(),NODESTATE_SEARCHING_FOR_RECHARGE_FACILITY);
	diagnostic_status = process.update(1.0/(FAST_RATE*1000.0));
	EXPECT_TRUE(diagnostic_status.Level <= NOTICE);
	diagnostic_status = process.new_targetmsg("unknownA");  //Should result in a WARN
	EXPECT_TRUE(diagnostic_status.Level == WARN);
	diagnostic_status = process.new_targetmsg("outlet");
	EXPECT_TRUE(diagnostic_status.Level <= NOTICE);
	EXPECT_EQ(process.get_currentcommand().Command,ROVERCOMMAND_STOPSEARCHFOR_RECHARGE_FACILITY);
	EXPECT_EQ(process.get_currentstate(),NODESTATE_ACQUIRING_TARGET);
	diagnostic_status = process.update(1.0/(FAST_RATE*1000.0));
	EXPECT_TRUE(diagnostic_status.Level <= NOTICE);
	EXPECT_EQ(process.get_currentcommand().Command,ROVERCOMMAND_ACQUIRE_TARGET);
	EXPECT_EQ(process.get_currentstate(),NODESTATE_ACQUIRING_TARGET);
}

int main(int argc, char **argv){
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
CommandNodeProcess initialize_process()
{
	icarus_rover_v2::diagnostic diagnostic_status;
	Logger *logger;
	std::string log_output = Node_Name + boost::lexical_cast<std::string>(1);
	logger = new Logger("DEBUG","UNIT_TESTS",log_output);
	diagnostic_status.DeviceName = Host_Name;
	diagnostic_status.Node_Name = Node_Name;
	diagnostic_status.System = ROVER;
	diagnostic_status.SubSystem = ROBOT_CONTROLLER;
	diagnostic_status.Component = CONTROLLER_NODE;

	diagnostic_status.Diagnostic_Type = NOERROR;
	diagnostic_status.Level = INFO;
	diagnostic_status.Diagnostic_Message = INITIALIZING;
	diagnostic_status.Description = "Node Initializing";

	CommandNodeProcess process;
	diagnostic_status = process.init(diagnostic_status,logger,std::string(Host_Name));
	EXPECT_TRUE(diagnostic_status.Level <= NOTICE);
	EXPECT_EQ(process.get_currentcommand().Command,ROVERCOMMAND_BOOT);
	EXPECT_EQ(process.get_currentstate(),NODESTATE_BOOTING);
	icarus_rover_v2::device newdevicemsg;
	newdevicemsg.DeviceName = ros_DeviceName;
	newdevicemsg.BoardCount = 0;
	newdevicemsg.Architecture = "x86_64";
	newdevicemsg.DeviceParent = ros_ParentDevice;
	newdevicemsg.DeviceType = ros_DeviceType;
	newdevicemsg.ID = DeviceID;
	newdevicemsg.SensorCount = 0;
	newdevicemsg.ShieldCount = 0;
	diagnostic_status = process.new_devicemsg(newdevicemsg);
	EXPECT_TRUE(diagnostic_status.Level <= NOTICE);
	EXPECT_EQ(process.is_finished_initializing(),true);
	EXPECT_EQ(process.get_currentstate(),NODESTATE_RUNNING);
	EXPECT_EQ(process.get_currentcommand().Command,ROVERCOMMAND_NONE);
	process.set_batterylevel_perc(100.0);

	std::vector<PeriodicCommand> pcommands;
	icarus_rover_v2::command command1;
	command1.Command = ROVERCOMMAND_RUNDIAGNOSTIC;
	command1.Option1 = LEVEL1;
	command1.Description = "Low-Level Diagnostics";
	PeriodicCommand pcommand1;
	pcommand1.command = command1;
	pcommand1.rate_hz = FAST_RATE;
	pcommands.push_back(pcommand1);

	icarus_rover_v2::command command2;
	command2.Command = ROVERCOMMAND_RUNDIAGNOSTIC;
	command2.Option1 = LEVEL2;
	command2.Description = "Mid-Level Diagnostics";
	PeriodicCommand pcommand2;
	pcommand2.command = command2;
	pcommand2.rate_hz = MEDIUM_RATE;
	pcommands.push_back(pcommand2);

	icarus_rover_v2::command command3;
	command3.Command = ROVERCOMMAND_RUNDIAGNOSTIC;
	command3.Option1 = LEVEL3;
	command3.Description = "High-Level Diagnostics";
	PeriodicCommand pcommand3;
	pcommand3.command = command3;
	pcommand3.rate_hz = SLOW_RATE;
	pcommands.push_back(pcommand3);
	diagnostic_status = process.init_PeriodicCommands(pcommands);
	EXPECT_TRUE(diagnostic_status.Level <= NOTICE);




	return process;
}
