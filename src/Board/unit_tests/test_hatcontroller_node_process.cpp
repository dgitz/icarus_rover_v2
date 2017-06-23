#include <gtest/gtest.h>
#include "ros/ros.h"
#include "ros/time.h"
#include "std_msgs/Bool.h"
#include "icarus_rover_v2/device.h"
#include "icarus_rover_v2/diagnostic.h"
#include "icarus_rover_v2/command.h"
#include "../hatcontroller_node_process.h"

std::string Node_Name = "/unittest_hatcontroller_node_process";
std::string Host_Name = "ControlModule1";
std::string ros_DeviceName = Host_Name;
std::string ros_ParentDevice = "";
std::string ros_DeviceType = "ControlModule";

HatControllerNodeProcess initialized_process;
HatControllerNodeProcess configured_process;
/*
TEST(DeviceInitialization,DeviceInitialization_TerminalHat)
{
    HatControllerNodeProcess process;
    icarus_rover_v2::device ros_device;
    ros_device.DeviceName = ros_DeviceName;
    ros_device.DeviceParent = ros_ParentDevice;
    ros_device.DeviceType = ros_DeviceType;
    ros_device.BoardCount = 1;
    ros_device.ID = 123;


    icarus_rover_v2::diagnostic diagnostic;
    diagnostic.DeviceName = ros_DeviceName;
    diagnostic.Node_Name = Node_Name;
    diagnostic.System = ROVER;
    diagnostic.SubSystem = ROBOT_CONTROLLER;
    diagnostic.Component = GPIO_NODE;

    diagnostic.Diagnostic_Type = NOERROR;
    diagnostic.Level = INFO;
    diagnostic.Diagnostic_Message = INITIALIZING;
    diagnostic.Description = "Node Initializing";
    Logger *logger;
    std::string log_output = Node_Name + boost::lexical_cast<std::string>(1);
    logger = new Logger("DEBUG","UNIT_TESTS",log_output);
    process.init(Host_Name,diagnostic);
    process.set_analyzetiming(true);
    EXPECT_EQ(process.get_armedstate(),ARMEDSTATUS_DISARMED_CANNOTARM);
    EXPECT_EQ(process.get_ready_to_arm(),false);
    
    diagnostic = process.new_devicemsg(ros_device);
    EXPECT_TRUE(diagnostic.Level <= NOTICE);
    EXPECT_TRUE(process.is_initialized() == true);
    EXPECT_TRUE(process.is_ready() == false);
    
    std_msgs::Bool pps;
    pps.data = true;
    diagnostic = process.new_ppsmsg(pps);
    EXPECT_TRUE(diagnostic.Level <= NOTICE);
    
    icarus_rover_v2::device hat1;
    hat1.DeviceName = "TerminalHat1";
    hat1.DeviceType = "TerminalHat";
    hat1.DeviceParent = ros_DeviceName;
    hat1.ID = 64;
    hat1.Architecture = "None";
    hat1.ShieldCount = 0;
    hat1.SensorCount = 0;
    hat1.pins.clear();
    std::vector<icarus_rover_v2::pin> gpio_input_pins;
    std::vector<icarus_rover_v2::pin> gpio_output_actuator_pins;
    std::vector<icarus_rover_v2::pin> gpio_output_nonactuator_pins;
    for(int i = 0; i < 24;i++)
    {
        icarus_rover_v2::pin newpin;
        newpin.Number = i;
        if(i < 8) {  newpin.Function = "DigitalOutput"; }
        else if(i < 16) { newpin.Function = "DigitalInput"; }
        else { newpin.Function = "DigitalOutput-NonActuator"; }
        newpin.DefaultValue = 0;
        newpin.Value = 1;
        newpin.ParentDevice = hat1.DeviceName;
        if(i < 8) { gpio_output_actuator_pins.push_back(newpin); }
        else if(i < 16) { gpio_input_pins.push_back(newpin); }
        else { gpio_output_nonactuator_pins.push_back(newpin); }
        hat1.pins.push_back(newpin);
   	}
    
    diagnostic = process.update(0.02);
    EXPECT_TRUE(diagnostic.Level <= NOTICE);
    
    diagnostic = process.new_devicemsg(hat1);
    EXPECT_TRUE(diagnostic.Level <= NOTICE);
    
    
    EXPECT_TRUE(process.is_ready() == true);
    diagnostic = process.set_terminalhat_initialized();
    EXPECT_TRUE(diagnostic.Level <= NOTICE);
    
    diagnostic = process.update(0.02);
    EXPECT_TRUE(diagnostic.Level <= NOTICE);
    
    EXPECT_EQ(process.get_ready_to_arm(),true);
    EXPECT_TRUE(process.get_armedstate() == ARMEDSTATUS_DISARMED_CANNOTARM);
    
    std::vector<icarus_rover_v2::pin> input_pins = process.get_terminalhatpins("DigitalInput");
    std::vector<icarus_rover_v2::pin> output_nonactuator_pins = process.get_terminalhatpins("DigitalOutput-NonActuator");
    std::vector<icarus_rover_v2::pin> output_actuator_pins = process.get_terminalhatpins("DigitalOutput");
    EXPECT_TRUE(input_pins.size() == gpio_input_pins.size());
    EXPECT_TRUE(output_nonactuator_pins.size() == gpio_output_nonactuator_pins.size());
    EXPECT_TRUE(output_actuator_pins.size() == gpio_output_actuator_pins.size());
    EXPECT_TRUE((input_pins.size() + output_nonactuator_pins.size() + output_actuator_pins.size())  == hat1.pins.size());
    for(std::size_t i = 0; i < output_actuator_pins.size(); i++)
    {
        EXPECT_TRUE(output_actuator_pins.at(i).DefaultValue == output_actuator_pins.at(i).Value);
        EXPECT_TRUE(output_actuator_pins.at(i).Value == gpio_output_actuator_pins.at(i).DefaultValue);
    }   
    
    
    diagnostic = process.new_armedstatemsg(ARMEDSTATUS_ARMED);
    EXPECT_TRUE(diagnostic.Level <= NOTICE);
    EXPECT_TRUE(process.get_armedstate() == ARMEDSTATUS_ARMED);
    
    output_actuator_pins.clear();
    output_actuator_pins = process.get_terminalhatpins("DigitalOutput");
    
    for(std::size_t i = 0; i < output_actuator_pins.size(); i++)
    {
        EXPECT_TRUE(output_actuator_pins.at(i).Value == gpio_output_actuator_pins.at(i).Value);
    }
    
    EXPECT_TRUE(diagnostic.Level <= NOTICE);
    diagnostic = process.update(0.02);
    EXPECT_TRUE(diagnostic.Level <= NOTICE);
    EXPECT_TRUE(process.get_armedstate() == ARMEDSTATUS_DISARMED);
    
    output_actuator_pins.clear();
    output_actuator_pins = process.get_terminalhatpins("DigitalOutput");
    
    for(std::size_t i = 0; i < output_actuator_pins.size(); i++)
    {
        EXPECT_TRUE(output_actuator_pins.at(i).Value == gpio_output_actuator_pins.at(i).DefaultValue);
    }
    
    
    int value = 0;
    //icarus_rover_v2::iopins p;
    for(std::size_t i = 0; i < input_pins.size(); i++)
    {
        icarus_rover_v2::pin pin = input_pins.at(i);
        pin.Value = value;
        value = !value;
        gpio_input_pins.at(i) = pin;
        struct timeval now;
        gettimeofday(&now,NULL);
        pin.stamp = process.convert_time(now);
        usleep(10000);
        diagnostic = process.new_pinmsg(pin);
        EXPECT_TRUE(diagnostic.Level <= NOTICE);
        //p.pins.push_back(pin);
       
    }
    

    input_pins.clear();
    input_pins = process.get_terminalhatpins("DigitalInput");
    EXPECT_TRUE(input_pins.size() == gpio_input_pins.size());
    for(std::size_t i = 0; i < input_pins.size(); i++)
    {
        EXPECT_TRUE(input_pins.at(i).Value == gpio_input_pins.at(i).Value);
    }
}
*/
TEST(DeviceInitialization,DeviceInitialization_ServoHat)
{
    HatControllerNodeProcess process;
    icarus_rover_v2::device ros_device;
    ros_device.DeviceName = ros_DeviceName;
    ros_device.DeviceParent = ros_ParentDevice;
    ros_device.DeviceType = ros_DeviceType;
    ros_device.BoardCount = 1;
    ros_device.ID = 123;


    icarus_rover_v2::diagnostic diagnostic;
    diagnostic.DeviceName = ros_DeviceName;
    diagnostic.Node_Name = Node_Name;
    diagnostic.System = ROVER;
    diagnostic.SubSystem = ROBOT_CONTROLLER;
    diagnostic.Component = GPIO_NODE;

    diagnostic.Diagnostic_Type = NOERROR;
    diagnostic.Level = INFO;
    diagnostic.Diagnostic_Message = INITIALIZING;
    diagnostic.Description = "Node Initializing";
    Logger *logger;
    std::string log_output = Node_Name + boost::lexical_cast<std::string>(1);
    logger = new Logger("DEBUG","UNIT_TESTS",log_output);
    process.init(Host_Name,diagnostic);
    process.set_analyzetiming(true);
    EXPECT_EQ(process.get_armedstate(),ARMEDSTATUS_DISARMED_CANNOTARM);
    EXPECT_EQ(process.get_ready_to_arm(),false);
    
    diagnostic = process.new_devicemsg(ros_device);
    EXPECT_TRUE(diagnostic.Level <= NOTICE);
    EXPECT_TRUE(process.is_initialized() == true);
    EXPECT_TRUE(process.is_ready() == false);
    
    std_msgs::Bool pps;
    pps.data = true;
    diagnostic = process.new_ppsmsg(pps);
    EXPECT_TRUE(diagnostic.Level <= NOTICE);
    
    icarus_rover_v2::device hat1;
    hat1.DeviceName = "ServoHat1";
    hat1.DeviceType = "ServoHat";
    hat1.DeviceParent = ros_DeviceName;
    hat1.ID = 64;
    hat1.Architecture = "None";
    hat1.ShieldCount = 0;
    hat1.SensorCount = 0;
    hat1.pins.clear();
    for(int i = 0; i < 16;i++)
    {
        icarus_rover_v2::pin newpin;
        newpin.Number = i;
        if(i < 8) {  newpin.Function = "PWMOutput"; }
        else { newpin.Function = "PWMOutput-NonActuator"; }
        newpin.DefaultValue = 1600+i*3;
        newpin.Value = 1400-(i*3);
        newpin.ParentDevice = hat1.DeviceName;
        hat1.pins.push_back(newpin);
   	}
    diagnostic = process.update(0.02);
    EXPECT_TRUE(diagnostic.Level <= NOTICE);
    
    diagnostic = process.new_devicemsg(hat1);
    EXPECT_TRUE(diagnostic.Level <= NOTICE);
    EXPECT_TRUE(process.is_ready() == true);
    diagnostic = process.set_servohat_initialized(0);
    EXPECT_TRUE(diagnostic.Level > NOTICE);
    
    diagnostic = process.set_servohat_initialized(hat1.ID);
    EXPECT_TRUE(diagnostic.Level <= NOTICE);
    diagnostic = process.update(0.02);
    EXPECT_TRUE(diagnostic.Level <= NOTICE);
    EXPECT_EQ(process.get_ready_to_arm(),true);
    std::vector<icarus_rover_v2::pin> pins = process.get_servohatpins(hat1.ID);
    EXPECT_TRUE(pins.size() == hat1.pins.size());
    for(std::size_t i = 0; i < hat1.pins.size(); i++)
    {
        EXPECT_TRUE(hat1.pins.at(i).Number == pins.at(i).Number);
        if(hat1.pins.at(i).Function == "PWMOutput")
        {
            EXPECT_TRUE(hat1.pins.at(i).DefaultValue == pins.at(i).Value);
        }
        else if(hat1.pins.at(i).Function == "PWMOutput-NonActuator")
        {
            EXPECT_TRUE(hat1.pins.at(i).Value == pins.at(i).Value);
        }
        else
        {
            EXPECT_TRUE(1 == 0); //Shouldn't get here
        }
    }
    diagnostic = process.new_armedstatemsg((uint8_t)(ARMEDSTATUS_ARMED));
    EXPECT_TRUE(diagnostic.Level <= NOTICE);
    
    pins.clear();
    std::vector<uint16_t> servohats = process.get_servohataddresses();
    EXPECT_TRUE(servohats.size() == 1);
    pins = process.get_servohatpins(servohats.at(0));
    EXPECT_TRUE(pins.size() == hat1.pins.size());
    
    for(std::size_t i = 0; i < hat1.pins.size(); i++)
    {
        EXPECT_TRUE(hat1.pins.at(i).Number == pins.at(i).Number);
        EXPECT_TRUE(hat1.pins.at(i).Value == pins.at(i).Value);
    }
    
    diagnostic = process.new_armedstatemsg(ARMEDSTATUS_DISARMED);
    EXPECT_TRUE(diagnostic.Level <= NOTICE);
    diagnostic = process.update(0.02);
    EXPECT_TRUE(diagnostic.Level <= NOTICE);
    EXPECT_TRUE(process.get_armedstate() == ARMEDSTATUS_DISARMED);

    pins.clear();
    pins = process.get_servohatpins(servohats.at(0));
    EXPECT_TRUE(pins.size() == hat1.pins.size());
    
    for(std::size_t i = 0; i < hat1.pins.size(); i++)
    {
        EXPECT_TRUE(hat1.pins.at(i).Number == pins.at(i).Number);
        if(hat1.pins.at(i).Function == "PWMOutput")
        {
            EXPECT_TRUE(hat1.pins.at(i).DefaultValue == pins.at(i).Value);
        }
        else if(hat1.pins.at(i).Function == "PWMOutput-NonActuator")
        {
            EXPECT_TRUE(hat1.pins.at(i).Value == pins.at(i).Value);
        }
        else
        {
            EXPECT_TRUE(1 == 0); //Shouldn't get here
        }
    }
    //icarus_rover_v2::iopins p;
    for(std::size_t i = 0; i < hat1.pins.size(); i++)
    {
        icarus_rover_v2::pin pin = hat1.pins.at(i);
        pin.Value = 1000 + i*5;
        hat1.pins.at(i).Value = pin.Value;
        struct timeval now;
        gettimeofday(&now,NULL);
        pin.stamp = process.convert_time(now);
        usleep(10000);
        diagnostic = process.new_pinmsg(pin);
        EXPECT_TRUE(diagnostic.Level <= NOTICE);
       
    }
    
    
    pins.clear();
    pins = process.get_servohatpins(servohats.at(0));
    EXPECT_TRUE(pins.size() == hat1.pins.size());
    for(std::size_t i = 0; i < hat1.pins.size(); i++)
    {
        EXPECT_TRUE(hat1.pins.at(i).Number == pins.at(i).Number);
        if(hat1.pins.at(i).Function == "PWMOutput")
        {
            EXPECT_TRUE(hat1.pins.at(i).DefaultValue == pins.at(i).Value);
        }
        else if(hat1.pins.at(i).Function == "PWMOutput-NonActuator")
        {
            EXPECT_TRUE(hat1.pins.at(i).Value == pins.at(i).Value);
        }
        else
        {
            EXPECT_TRUE(1 == 0); //Shouldn't get here
        }
    }
    
    diagnostic = process.new_armedstatemsg(ARMEDSTATUS_ARMED);
    EXPECT_TRUE(diagnostic.Level <= NOTICE);
    diagnostic = process.update(0.02);
    EXPECT_TRUE(diagnostic.Level <= NOTICE);
    EXPECT_TRUE(process.get_armedstate() == ARMEDSTATUS_ARMED);
    pins.clear();
    pins = process.get_servohatpins(servohats.at(0));
    EXPECT_TRUE(pins.size() == hat1.pins.size());
    
    for(std::size_t i = 0; i < hat1.pins.size(); i++)
    {
        EXPECT_TRUE(hat1.pins.at(i).Number == pins.at(i).Number);
        EXPECT_TRUE(hat1.pins.at(i).Value == pins.at(i).Value);
    }
}
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
