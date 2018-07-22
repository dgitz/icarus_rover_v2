#include <gtest/gtest.h>
#include "ros/ros.h"
#include "ros/time.h"
#include "icarus_rover_v2/command.h"
#include "icarus_rover_v2/device.h"
#include "icarus_rover_v2/diagnostic.h"
#include "../diagnostic_node_process.h"

std::string Node_Name = "/unittest_diagnostic_node_process";
std::string Host_Name = "unittest";
std::string ros_DeviceName = Host_Name;


DiagnosticNodeProcess* initializeprocess()
{
    icarus_rover_v2::diagnostic diagnostic;
    diagnostic.DeviceName = ros_DeviceName;
    diagnostic.Node_Name = Node_Name;
    diagnostic.System = ROVER;
    diagnostic.SubSystem = ROBOT_CONTROLLER;
    diagnostic.Component = DIAGNOSTIC_NODE;

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

    DiagnosticNodeProcess *process;
    process = new DiagnosticNodeProcess;
	diagnostic = process->init(diagnostic,std::string(Host_Name));
    EXPECT_TRUE(diagnostic.Level <= NOTICE);
    EXPECT_TRUE(process->get_initialized() == false);
    process->set_mydevice(device);
    EXPECT_TRUE(process->get_initialized() == true);
    EXPECT_TRUE(process->get_mydevice().DeviceName == device.DeviceName);
    return process;
}
DiagnosticNodeProcess* readyprocess(DiagnosticNodeProcess* process)
{
    icarus_rover_v2::diagnostic diag = process->update(0);
    EXPECT_TRUE(diag.Level <= NOTICE);
    EXPECT_TRUE(process->get_ready() == true);
    std::vector<std::string> topics_to_add;
    topics_to_add.push_back("/Task0/diagnostic");
    topics_to_add.push_back("/Task1/diagnostic");
    topics_to_add.push_back("/Task2/diagnostic");
    topics_to_add.push_back("/Task3/diagnostic");
    topics_to_add.push_back("/Task4/diagnostic");
    for(int i = 0; i < topics_to_add.size(); i++)
    {
    	std::string taskname = topics_to_add.at(i).substr(1,topics_to_add.at(i).find("/diagnostic")-1);;
    	DiagnosticNodeProcess::Task newTask;
    	newTask.Task_Name = taskname;
    	newTask.diagnostic_topic = topics_to_add.at(i);
    	newTask.heartbeat_topic = "/" + taskname + "/heartbeat";
    	newTask.resource_topic = "/" + taskname + "/resource";
    	process->add_Task(newTask);
    }
    EXPECT_TRUE(process->get_TaskList().size()==topics_to_add.size());
    return process;
}
TEST(Template,Process_Initialization)
{
	DiagnosticNodeProcess* process = initializeprocess();
}

TEST(Template,Process_Command)
{
	DiagnosticNodeProcess* process = initializeprocess();
    process = readyprocess(process);
    double time_to_run = 50.0;
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
        	std::vector<DiagnosticNodeProcess::Task> tasklist = process->get_TaskList();
        	for(std::size_t i = 0; i < tasklist.size(); i++)
        	{
        		process->new_heartbeatmsg(tasklist.at(i).heartbeat_topic);
        	}
        	cmd.Option1 = LEVEL3;
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
            process->new_1ppsmsg();
            
        }
        if(slowrate_fire == true)
        {
            cmd.Option1 = LEVEL1;
            std::vector<icarus_rover_v2::diagnostic> diaglist = process->new_commandmsg(cmd);
            for(std::size_t i = 0; i < diaglist.size(); i++)
            {
                EXPECT_TRUE(diaglist.at(i).Level <= NOTICE);
            }
            EXPECT_TRUE(diaglist.size() > 0);
            process->new_01ppsmsg();
        }
        current_time += dt;   
    }
    EXPECT_TRUE(process->get_runtime() >= time_to_run);
}
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
