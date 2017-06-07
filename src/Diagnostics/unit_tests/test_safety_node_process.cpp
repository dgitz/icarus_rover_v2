#include <gtest/gtest.h>
#include "ros/ros.h"
#include "ros/time.h"
#include "std_msgs/Bool.h"
#include "icarus_rover_v2/device.h"
#include "icarus_rover_v2/diagnostic.h"
#include "icarus_rover_v2/command.h"
#include "icarus_rover_v2/estop.h"
#include "../safety_node_process.h"

std::string Node_Name = "/unittest_safety_node_process";
std::string Host_Name = "ControlModule1";
std::string ros_DeviceName = Host_Name;
std::string ros_ParentDevice = "";
std::string ros_DeviceType = "ControlModule";

SafetyNodeProcess initialized_process;
TEST(DeviceOperation,DeviceInitialization)
{
    SafetyNodeProcess process;

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
    process.init(Host_Name,diagnostic);
    
    icarus_rover_v2::device ros_device;
    ros_device.DeviceName = ros_DeviceName;
    ros_device.DeviceParent = ros_ParentDevice;
    ros_device.DeviceType = ros_DeviceType;
    ros_device.BoardCount = 0;
    ros_device.ID = 122;

    diagnostic = process.new_devicemsg(ros_device);
    EXPECT_TRUE(diagnostic.Level <= NOTICE);
    
    EXPECT_TRUE(process.get_ready_to_arm() == false);
    EXPECT_TRUE(process.get_estop().name == ros_device.DeviceName);
    EXPECT_TRUE(process.get_estop().state == ESTOP_UNDEFINED);
    initialized_process = process;    
}
TEST(DeviceOperation,DeviceRunning)
{
    SafetyNodeProcess process = initialized_process;
    icarus_rover_v2::diagnostic diagnostic = process.update();
    EXPECT_TRUE(diagnostic.Level <= NOTICE);
    
    {
        icarus_rover_v2::pin estop_pin;
        estop_pin.ParentDevice = ros_DeviceName;
        estop_pin.Number = PIN_ESTOP;
        estop_pin.Value = 0;
        diagnostic = process.new_pinmsg(estop_pin);
        EXPECT_TRUE(diagnostic.Level <= NOTICE);        
    }
    
    diagnostic = process.update();
    EXPECT_TRUE(diagnostic.Level <= NOTICE);
    EXPECT_TRUE(process.get_ready_to_arm() == true);
    EXPECT_TRUE(process.get_estop().state == ESTOP_DISACTIVATED);
    
    {
        icarus_rover_v2::pin estop_pin;
        estop_pin.ParentDevice = ros_DeviceName;
        estop_pin.Number = PIN_ESTOP;
        estop_pin.Value = 1;
        diagnostic = process.new_pinmsg(estop_pin);
        EXPECT_TRUE(diagnostic.Level <= NOTICE);        
    }
    
    diagnostic = process.update();
    EXPECT_TRUE(diagnostic.Level <= NOTICE);
    EXPECT_TRUE(process.get_ready_to_arm() == false);
    EXPECT_TRUE(process.get_estop().state == ESTOP_ACTIVATED);
    
    {
        icarus_rover_v2::pin estop_pin;
        estop_pin.ParentDevice = ros_DeviceName;
        estop_pin.Number = PIN_ESTOP;
        estop_pin.Value = 0;
        diagnostic = process.new_pinmsg(estop_pin);
        EXPECT_TRUE(diagnostic.Level <= NOTICE);        
    }
    
    diagnostic = process.update();
    EXPECT_TRUE(diagnostic.Level <= NOTICE);
    EXPECT_TRUE(process.get_ready_to_arm() == true);
    EXPECT_TRUE(process.get_estop().state == ESTOP_DISACTIVATED);
    
}
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
