#include <gtest/gtest.h>
#include "ros/ros.h"
#include "ros/time.h"
#include "icarus_rover_v2/device.h"
#include "icarus_rover_v2/diagnostic.h"
#include "../powermonitor_node_process.h"

std::string Node_Name = "/unittest_powermonitor_node_process";
std::string Host_Name = "unittest";
std::string ros_DeviceName = Host_Name;
std::string ros_ParentDevice = "";
std::string ros_DeviceType = "ComputeModule";
double SLOW_RATE = 0.1f;
double MEDIUM_RATE = 1.0f;
double FAST_RATE = 10.0f;
int DeviceID = 123;


PowerMonitorNodeProcess initialize_process();
TEST(DeviceInitialization,Boot)
{
	icarus_rover_v2::diagnostic diagnostic_status;
	PowerMonitorNodeProcess process = initialize_process();
    
}
TEST(DeviceInitialization,ReadPowerInfo)
{
    icarus_rover_v2::diagnostic diagnostic_status;
	PowerMonitorNodeProcess process = initialize_process();
    int battery_count = process.get_batterycount();
    for(int i = 0; i < battery_count; i++)
    {
        icarus_rover_v2::device battery;
        battery.DeviceName = "Battery" + boost::lexical_cast<std::string>(i+1);
        battery.DeviceType = "Battery";
        diagnostic_status = process.new_devicemsg(battery);
        EXPECT_TRUE(diagnostic_status.Level <= NOTICE);
    }
}
int main(int argc, char **argv){
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
PowerMonitorNodeProcess initialize_process()
{
	icarus_rover_v2::diagnostic diagnostic_status;
	Logger *logger;
	std::string log_output = Node_Name + boost::lexical_cast<std::string>(1);
	logger = new Logger("DEBUG","UNIT_TESTS",log_output);
	diagnostic_status.DeviceName = Host_Name;
	diagnostic_status.Node_Name = Node_Name;
	diagnostic_status.System = ROVER;
	diagnostic_status.SubSystem = ROBOT_CONTROLLER;
	diagnostic_status.Component = POWER_NODE;

	diagnostic_status.Diagnostic_Type = NOERROR;
	diagnostic_status.Level = INFO;
	diagnostic_status.Diagnostic_Message = INITIALIZING;
	diagnostic_status.Description = "Node Initializing";

	PowerMonitorNodeProcess process;
	diagnostic_status = process.init(diagnostic_status,logger,std::string(Host_Name));
    if(diagnostic_status.Level > NOTICE)
    {
        printf("Initialize Error: %s\n",diagnostic_status.Description.c_str());
    }
    EXPECT_TRUE(diagnostic_status.Level <= NOTICE);
    EXPECT_TRUE(process.get_batterycount() > 0);
    EXPECT_EQ(process.get_batterycount(),process.get_batteries().size());
    
	return process;
}
