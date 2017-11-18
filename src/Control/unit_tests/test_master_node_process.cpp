#include <gtest/gtest.h>
#include "ros/ros.h"
#include "ros/time.h"
#include "icarus_rover_v2/device.h"
#include "icarus_rover_v2/diagnostic.h"
#include "../master_node_process.h"

std::string Node_Name = "/unittest_master_node_process";
std::string Host_Name = "dgitzrosmaster";
std::string ros_DeviceName = Host_Name;
std::string ros_ParentDevice = "";
std::string ros_DeviceType = "ComputeModule";
double SLOW_RATE = 0.1f;
double MEDIUM_RATE = 1.0f;
double FAST_RATE = 10.0f;
int DeviceID = 123;


MasterNodeProcess initialize_process(std::string path);
TEST(ProcessInitialization,NormalOperation)
{
    std::vector<std::string> devicepathlist;
    devicepathlist.push_back("/home/robot/catkin_ws/src/icarus_rover_v2/src/Control/unit_tests/UnitTestDeviceFile.xml");
    devicepathlist.push_back("/home/robot/catkin_ws/src/icarus_rover_v2/src/Control/unit_tests/DeviceFile.xml");
    for(std::size_t i = 0; i < devicepathlist.size(); i++)
    {
        std::string path = devicepathlist.at(i);
        printf("Loading: %s\n",path.c_str());
        MasterNodeProcess process = initialize_process(path);
        std::vector<std::string> serialportlist;
        serialportlist.push_back("/dev/ttyUSB0");
        serialportlist.push_back("/dev/ttyACM0");
        serialportlist.push_back("/dev/ttyS0");
    
        icarus_rover_v2::diagnostic diag = process.set_serialportlist(serialportlist);
        EXPECT_TRUE(diag.Level <= NOTICE);
        if(i == 0)
        {
            EXPECT_TRUE(process.get_allserialbaudrates().size() == 1);
        }
        else if(i == 1)
        {
            EXPECT_TRUE(process.get_allserialbaudrates().size() == 1);
        }
    }
 
    
   
    
}
int main(int argc, char **argv){
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
MasterNodeProcess initialize_process(std::string devicefilepath)
{
	icarus_rover_v2::diagnostic diagnostic_status;
	diagnostic_status.DeviceName = Host_Name;
	diagnostic_status.Node_Name = Node_Name;
	diagnostic_status.System = ROVER;
	diagnostic_status.SubSystem = ROBOT_CONTROLLER;
	diagnostic_status.Component = CONTROLLER_NODE;

	diagnostic_status.Diagnostic_Type = NOERROR;
	diagnostic_status.Level = INFO;
	diagnostic_status.Diagnostic_Message = INITIALIZING;
	diagnostic_status.Description = "Node Initializing";

	MasterNodeProcess process;
	diagnostic_status = process.init(diagnostic_status,std::string(Host_Name));
	EXPECT_TRUE(diagnostic_status.Level <= NOTICE);
    diagnostic_status = process.load_devicefile(devicefilepath);
    //printf("diag: %s\n",diagnostic_status.Description.c_str());
    EXPECT_TRUE(diagnostic_status.Level <= NOTICE);
    std::vector<icarus_rover_v2::device> child_devices = process.get_childdevices();
    EXPECT_TRUE(child_devices.size() > 0);
    printf("-----CHILD DEVICES-----\n");
    process.print_device(child_devices);


	return process;
}
