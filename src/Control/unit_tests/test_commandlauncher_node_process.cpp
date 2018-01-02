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



void print_ipmap(std::vector<IPMap> ipmap);
void print_portmap(std::vector<PortMap> ipmap);
CommandLauncherNodeProcess* initializeprocess()
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
    device.BoardCount = 0;
    device.SensorCount = 0;
    device.DeviceParent = "None";
    device.Architecture = "x86_64";

    CommandLauncherNodeProcess *process;
    process = new CommandLauncherNodeProcess;
	diagnostic = process->init(diagnostic,std::string(Host_Name));
    EXPECT_TRUE(diagnostic.Level <= NOTICE);
    EXPECT_TRUE(process->get_initialized() == false);
    process->set_mydevice(device);
    EXPECT_TRUE(process->get_initialized() == true);
    EXPECT_TRUE(process->get_mydevice().DeviceName == device.DeviceName);

	EXPECT_TRUE(process->set_camerastream("CameraPort"));
	
	std::vector<ProcessCommand> processlist = process->get_processlist();
	EXPECT_TRUE(processlist.size() > 0);
	for(std::size_t i = 0; i < processlist.size(); i++)
	{
		EXPECT_TRUE(processlist.at(i).initialized == true); 
		EXPECT_TRUE(processlist.at(i).running == false); 
		printf("[%d] Process: %s Command: '%s'\n",(int)i,processlist.at(i).name.c_str(),processlist.at(i).command_text.c_str());
	}
	
	for(std::size_t i = 0; i < processlist.size(); i++)
	{
		EXPECT_TRUE(processlist.at(i).initialized == true); 
		EXPECT_TRUE(process->set_processrunning(processlist.at(i).name,true));
		processlist = process->get_processlist();
		EXPECT_TRUE(processlist.at(i).running == true); 
	}
    return process;
}
CommandLauncherNodeProcess* readyprocess(CommandLauncherNodeProcess* process)
{
    icarus_rover_v2::diagnostic diag = process->update(0);
    EXPECT_TRUE(diag.Level <= NOTICE);
    EXPECT_TRUE(process->get_ready() == true);
    return process;
}
TEST(Template,Process_Initialization)
{
	CommandLauncherNodeProcess* process = initializeprocess();
	std::vector<IPMap> ipmap = process->get_ipmap();
	print_ipmap(ipmap);
	std::vector<PortMap> portmap = process->get_portmap();
	print_portmap(portmap);
	icarus_rover_v2::diagnostic diagnostic = process->update(0);
    EXPECT_TRUE(diagnostic.Level <= NOTICE);
    printf("%s\n",process->get_processinfo().c_str());
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
			(int)i,ipmap.at(i).hostname.c_str(),ipmap.at(i).IPAddress.c_str());
	}
	printf("----------------\n");
}
void print_portmap(std::vector<PortMap> portmap)
{
	printf("-----Port Map-----\n");
	for(std::size_t i = 0; i < portmap.size(); i++)
	{
		printf("[%d] Name: %s Number: %d\n",
			(int)i,portmap.at(i).name.c_str(),portmap.at(i).port);
	}
	printf("------------------\n");
}
