#include <gtest/gtest.h>
#include "ros/ros.h"
#include "ros/time.h"
#include "icarus_rover_v2/device.h"
#include "icarus_rover_v2/diagnostic.h"
#include "../commandlauncher_node_process.h"

std::string Node_Name = "/unittest_boardcontroller_node_process";
std::string Host_Name = "unittest";
std::string ros_DeviceName = Host_Name;
std::string ros_DeviceType = "ControlModule";

CommandLauncherNodeProcess *initialized_process;


bool check_if_initialized(CommandLauncherNodeProcess process);
void print_ipmap(std::vector<IPMap> ipmap);
void print_portmap(std::vector<PortMap> ipmap);
TEST(Template,ProcessInitialization)
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

    CommandLauncherNodeProcess *process;
    process = new CommandLauncherNodeProcess;
	diagnostic = process->init(diagnostic,std::string(Host_Name));
	std::vector<IPMap> ipmap = process->get_ipmap();
	print_ipmap(ipmap);
	std::vector<PortMap> portmap = process->get_portmap();
	print_portmap(portmap);
    EXPECT_TRUE(diagnostic.Level <= NOTICE);
	
	EXPECT_TRUE(process->set_camerastream("DriverStation:CameraPort"));
	
	std::vector<ProcessCommand> processlist = process->get_processlist();
	EXPECT_TRUE(processlist.size() > 0);
	for(std::size_t i = 0; i < processlist.size(); i++)
	{
		EXPECT_TRUE(processlist.at(i).initialized == true); 
		EXPECT_TRUE(processlist.at(i).running == false); 
		printf("[%d] Process:%s Command: '%s'\n",i,processlist.at(i).name.c_str(),processlist.at(i).command_text.c_str());
	}
	
	for(std::size_t i = 0; i < processlist.size(); i++)
	{
		EXPECT_TRUE(processlist.at(i).initialized == true); 
		EXPECT_TRUE(process->set_processrunning(processlist.at(i).name,true));
		processlist = process->get_processlist();
		EXPECT_TRUE(processlist.at(i).running == true); 
	}
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

void print_ipmap(std::vector<IPMap> ipmap)
{
	printf("-----IP Map-----\n");
	for(std::size_t i = 0; i < ipmap.size(); i++)
	{
		printf("[%d] Host: %s IP Address: %s\n",
			i,ipmap.at(i).hostname.c_str(),ipmap.at(i).IPAddress.c_str());
	}
	printf("----------------\n");
}
void print_portmap(std::vector<PortMap> portmap)
{
	printf("-----Port Map-----\n");
	for(std::size_t i = 0; i < portmap.size(); i++)
	{
		printf("[%d] Name: %s Number: %d\n",
			i,portmap.at(i).name.c_str(),portmap.at(i).port);
	}
	printf("------------------\n");
}
