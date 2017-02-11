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
    std::vector<Battery> batteries = process.get_batteries();

    for(int i = 0; i < batteries.size(); i++)
    {
    	batteries.at(i).capacity_level_perc = 100.0;
        diagnostic_status = process.new_batterymsg(batteries.at(i));
        EXPECT_TRUE(diagnostic_status.Level <= NOTICE);
    }

    diagnostic_status = process.update(0.01);
    EXPECT_TRUE(diagnostic_status.Level <= NOTICE);
    EXPECT_EQ(process.get_powerstate(),POWERSTATE_NORMAL);
    EXPECT_TRUE(process.get_activebattery().name != "");
    EXPECT_TRUE(process.get_activebattery().active == true);
}
TEST(StateMachineLogic,TestA)
{
    icarus_rover_v2::diagnostic diagnostic_status;
	PowerMonitorNodeProcess process = initialize_process();
    int battery_count = process.get_batterycount();
    std::vector<Battery> batteries = process.get_batteries();

    for(int i = 0; i < batteries.size(); i++)
    {
    	batteries.at(i).capacity_level_perc = 100.0;
        diagnostic_status = process.new_batterymsg(batteries.at(i));
        EXPECT_TRUE(diagnostic_status.Level <= NOTICE);
    }


    diagnostic_status = process.update(0.01);
    EXPECT_TRUE(diagnostic_status.Level <= NOTICE);
    EXPECT_EQ(process.get_powerstate(),POWERSTATE_NORMAL);
    EXPECT_TRUE(process.get_activebattery().name != "");
    EXPECT_TRUE(process.get_activebattery().active == true);
    double capacity;
    Battery last_battery;
    Battery current_battery;
    Battery active_battery;
    for(int i = 0; i < batteries.size(); i++)
	{
    	printf("Draining: %s\n",process.get_activebattery().name.c_str());
    	diagnostic_status = process.update(0.01);
    	EXPECT_TRUE(diagnostic_status.Level <= NOTICE);
    	capacity = process.get_activebattery().capacity_level_perc;
    	EXPECT_EQ(process.get_powerstate(),POWERSTATE_NORMAL);
    	last_battery = process.get_activebattery();
    	while(capacity > process.get_defined_batterylevel_toswitch())
    	{
    		capacity -= 5.0;
    		active_battery= process.get_activebattery();
    		active_battery.capacity_level_perc = capacity;
    		diagnostic_status = process.new_batterymsg(active_battery);
    		EXPECT_TRUE(diagnostic_status.Level <= NOTICE);
    		diagnostic_status = process.update(0.01);
    		EXPECT_TRUE(diagnostic_status.Level <= NOTICE);
    		printf("Power State: %s\n",process.map_PowerState_ToString(process.get_powerstate()).c_str());
    		if(i < (batteries.size()-1)) //Nothing to do here
    		{

    		}
    		else
    		{
    			if(capacity < process.get_defined_batterylevel_recharge())
    			{
    				break;
    			}
    		}
    	}
    	if(i < (batteries.size()-1))
    	{
    		EXPECT_EQ(process.get_powerstate(),POWERSTATE_CHANGINGACTIVEBATTERY);
    		current_battery = process.get_activebattery();
    		printf("Changed Batteries: From: %s to %s\n",last_battery.name.c_str(),current_battery.name.c_str());
    		EXPECT_NE(current_battery.name,last_battery.name);
    		EXPECT_TRUE(current_battery.active == true);
    	}
    	else
    	{
    		EXPECT_EQ(process.get_powerstate(),POWERSTATE_REQUIRERECHARGE);
    	}
	}
    active_battery= process.get_activebattery();
    active_battery.capacity_level_perc = process.get_defined_batterylevel_toswitch()-1.0;
    diagnostic_status = process.new_batterymsg(active_battery);
    EXPECT_TRUE(diagnostic_status.Level <= NOTICE);
    diagnostic_status = process.update(0.01);
    EXPECT_TRUE(diagnostic_status.Level <= NOTICE);
    printf("Power State: %s\n",process.map_PowerState_ToString(process.get_powerstate()).c_str());
    EXPECT_EQ(process.get_powerstate(),POWERSTATE_EMERGENCY);
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
    EXPECT_TRUE(diagnostic_status.Level <= NOTICE);
    EXPECT_TRUE(process.get_batterycount() > 0);
    EXPECT_EQ(process.get_batterycount(),process.get_batteries().size());
    EXPECT_EQ(process.get_powerstate(), POWERSTATE_NORMAL);

    diagnostic_status = process.update(0.01);
    EXPECT_TRUE(diagnostic_status.Level <= NOTICE);
    EXPECT_EQ(process.get_powerstate(),POWERSTATE_NORMAL);
	return process;
}
