#include <gtest/gtest.h>
#include "ros/ros.h"
#include "ros/time.h"
#include "icarus_rover_v2/device.h"
#include "icarus_rover_v2/diagnostic.h"
#include "../simulate_node_process.h"

std::string Node_Name = "/unittest_simple_node_process";
std::string Host_Name = "unittest";
std::string ros_DeviceName = Host_Name;


SimulateNodeProcess *initialized_process;


bool check_if_initialized(SimulateNodeProcess process);
bool isequal(double a,double b);
void print_scriptactions(std::vector<ScriptAction> actions);
TEST(Template,ProcessInitialization)
{
    icarus_rover_v2::diagnostic diagnostic;
    diagnostic.DeviceName = ros_DeviceName;
    diagnostic.Node_Name = Node_Name;
    diagnostic.System = ROVER;
    diagnostic.SubSystem = ROBOT_CONTROLLER;
    diagnostic.Component = POSE_NODE;

    diagnostic.Diagnostic_Type = NOERROR;
    diagnostic.Level = INFO;
    diagnostic.Diagnostic_Message = INITIALIZING;
    diagnostic.Description = "Node Initializing";

    SimulateNodeProcess *process;
    process = new SimulateNodeProcess;
	diagnostic = process->init(diagnostic,std::string(Host_Name));
    EXPECT_TRUE(diagnostic.Level <= NOTICE);
	EXPECT_TRUE(process->set_vehiclemodel(SimulateNodeProcess::TANK));	
}
TEST(Tank,Wheelspeed)
{
	icarus_rover_v2::diagnostic diagnostic;
    diagnostic.DeviceName = ros_DeviceName;
    diagnostic.Node_Name = Node_Name;
    diagnostic.System = ROVER;
    diagnostic.SubSystem = ROBOT_CONTROLLER;
    diagnostic.Component = POSE_NODE;

    diagnostic.Diagnostic_Type = NOERROR;
    diagnostic.Level = INFO;
    diagnostic.Diagnostic_Message = INITIALIZING;
    diagnostic.Description = "Node Initializing";

    SimulateNodeProcess *process;
    process = new SimulateNodeProcess;
	diagnostic = process->init(diagnostic,std::string(Host_Name));
    EXPECT_TRUE(diagnostic.Level <= NOTICE);
	EXPECT_TRUE(process->set_vehiclemodel(SimulateNodeProcess::TANK));	
	{ //Test Stationary
		process->set_throttlecommand(0.0);
		process->set_steercommand(0.0);
		process->update(.01);
		EXPECT_TRUE(isequal(process->get_left_wheelspeed_command(),0.0));
		EXPECT_TRUE(isequal(process->get_right_wheelspeed_command(),0.0));
	}
	
	{ //Test Full Forward
		process->set_throttlecommand(1.0);
		process->set_steercommand(0.0);
		process->update(.01);
		EXPECT_TRUE(isequal(process->get_left_wheelspeed_command(),1.0));
		EXPECT_TRUE(isequal(process->get_right_wheelspeed_command(),1.0));
	}
	
	{ //Test Full Left
		process->set_throttlecommand(0.0);
		process->set_steercommand(-1.0);
		process->update(.01);
		EXPECT_TRUE(isequal(process->get_left_wheelspeed_command(),-1.0));
		EXPECT_TRUE(isequal(process->get_right_wheelspeed_command(),1.0));
	}
	
	{ //Test Full Back Right
		process->set_throttlecommand(-1.0);
		process->set_steercommand(1.0);
		process->update(.01);
		EXPECT_TRUE(isequal(process->get_left_wheelspeed_command(),0.0));
		EXPECT_TRUE(isequal(process->get_right_wheelspeed_command(),-1.0));
	}
	
	{ //Test Half Left
		process->set_throttlecommand(0.0);
		process->set_steercommand(-0.5);
		process->update(.01);
		EXPECT_TRUE(isequal(process->get_left_wheelspeed_command(),-0.5));
		EXPECT_TRUE(isequal(process->get_right_wheelspeed_command(),0.5));
	}
	
}
TEST(Script,ReadAndExecute)
{
	icarus_rover_v2::diagnostic diagnostic;
    diagnostic.DeviceName = ros_DeviceName;
    diagnostic.Node_Name = Node_Name;
    diagnostic.System = ROVER;
    diagnostic.SubSystem = ROBOT_CONTROLLER;
    diagnostic.Component = POSE_NODE;

    diagnostic.Diagnostic_Type = NOERROR;
    diagnostic.Level = INFO;
    diagnostic.Diagnostic_Message = INITIALIZING;
    diagnostic.Description = "Node Initializing";

    SimulateNodeProcess *process;
    process = new SimulateNodeProcess;
	diagnostic = process->init(diagnostic,std::string(Host_Name));
    EXPECT_TRUE(diagnostic.Level <= NOTICE);
	EXPECT_TRUE(process->set_vehiclemodel(SimulateNodeProcess::TANK));	
	EXPECT_TRUE(process->set_simulationmode(process->map_simulationmode_toint("Script")));
	
	EXPECT_TRUE(process->load_scriptfile("/home/robot/config/sim_scripts/unittestA.csv"));
	
	std::vector<ScriptAction> actions = process->get_scriptactions();
	print_scriptactions(actions);
	double time_to_run = 0.0;
	for(std::size_t i = 0; i < actions.size(); i++)
	{
		if(	(actions.at(i).Command == "Drive") or
			(actions.at(i).Command == "Wait") or
			(actions.at(i).Command == "Stop"))
		{
			time_to_run += actions.at(i).Option1;
		}
	}
	double timer = 0.0;
	double dt = 0.1;
	while(timer < (time_to_run+1.0))
	{
		process->update(dt);
		timer+=dt;
	}
	EXPECT_TRUE(process->get_currentcommand() == "Stop");
}
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
bool isequal(double a,double b)
{
	if((a-b)<.0000001) { return true; } else { return false; }
}
void print_scriptactions(std::vector<ScriptAction> actions)
{
	printf("----- Script Actions -----\n");
	for(std::size_t i = 0; i < actions.size(); i++)
	{
		printf("[%d] %s: op1: %f op2: %f op3: %f op4: %f op5: %f comments: %s\n",
			i,
			actions.at(i).Command.c_str(),
			actions.at(i).Option1,
			actions.at(i).Option2,
			actions.at(i).Option3,
			actions.at(i).Option4,
			actions.at(i).Option5,
			actions.at(i).Comment.c_str());
	}
}

