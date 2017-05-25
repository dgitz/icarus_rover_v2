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
    EXPECT_EQ(process.get_armedstate(),ARMEDSTATUS_DISARMED_CANNOTARM);
    EXPECT_EQ(process.get_ready_to_arm(),false);
    
    diagnostic = process.new_devicemsg(ros_device);
    EXPECT_TRUE(diagnostic.Level <= NOTICE);
    EXPECT_TRUE(process.is_initialized() == true);
    EXPECT_TRUE(process.is_ready() == false);
    
        
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
        hat1.pins.push_back(newpin);
   	}
    
    diagnostic = process.update(0.02);
    EXPECT_TRUE(diagnostic.Level <= NOTICE);
    
    diagnostic = process.new_devicemsg(hat1);
    EXPECT_TRUE(diagnostic.Level <= NOTICE);
    EXPECT_TRUE(process.is_ready() == true);
    
    diagnostic = process.set_hat_initialized(0);
    EXPECT_TRUE(diagnostic.Level > NOTICE);
    
    diagnostic = process.set_hat_initialized(hat1.ID);
    EXPECT_TRUE(diagnostic.Level <= NOTICE);
    
    diagnostic = process.update(0.02);
    EXPECT_TRUE(diagnostic.Level <= NOTICE);
    
    EXPECT_EQ(process.get_ready_to_arm(),true);
    std::vector<ChannelValue> cv = process.get_servohatvalues(hat1.ID);
    EXPECT_TRUE(cv.size() == hat1.pins.size());
    
    for(std::size_t i = 0; i < hat1.pins.size(); i++)
    {
        EXPECT_TRUE(hat1.pins.at(i).Number == cv.at(i).channel);
        if(hat1.pins.at(i).Function == "PWMOutput")
        {
            EXPECT_TRUE(hat1.pins.at(i).DefaultValue == cv.at(i).value);
        }
        else if(hat1.pins.at(i).Function == "PWMOutput-NonActuator")
        {
            EXPECT_TRUE(hat1.pins.at(i).Value == cv.at(i).value);
        }
        else
        {
            EXPECT_TRUE(1 == 0); //Shouldn't get here
        }
    }
    
    icarus_rover_v2::command armdisarm_command;
    armdisarm_command.Command = ROVERCOMMAND_ARM;
    diagnostic = process.new_commandmsg(armdisarm_command);
    EXPECT_TRUE(diagnostic.Level <= NOTICE);
    
    cv.clear();
    cv = process.get_servohatvalues(hat1.ID);
    EXPECT_TRUE(cv.size() == hat1.pins.size());
    
    for(std::size_t i = 0; i < hat1.pins.size(); i++)
    {
        EXPECT_TRUE(hat1.pins.at(i).Number == cv.at(i).channel);
        EXPECT_TRUE(hat1.pins.at(i).Value == cv.at(i).value);
    }
    
    armdisarm_command.Command = ROVERCOMMAND_DISARM;
    diagnostic = process.new_commandmsg(armdisarm_command);
    EXPECT_TRUE(diagnostic.Level <= NOTICE);
    
    cv.clear();
    cv = process.get_servohatvalues(hat1.ID);
    EXPECT_TRUE(cv.size() == hat1.pins.size());
    
    for(std::size_t i = 0; i < hat1.pins.size(); i++)
    {
        EXPECT_TRUE(hat1.pins.at(i).Number == cv.at(i).channel);
        if(hat1.pins.at(i).Function == "PWMOutput")
        {
            EXPECT_TRUE(hat1.pins.at(i).DefaultValue == cv.at(i).value);
        }
        else if(hat1.pins.at(i).Function == "PWMOutput-NonActuator")
        {
            EXPECT_TRUE(hat1.pins.at(i).Value == cv.at(i).value);
        }
        else
        {
            EXPECT_TRUE(1 == 0); //Shouldn't get here
        }
    }
    
    /*

    diagnostic = processes.at(processindex).new_devicemsg(ros_device);
    EXPECT_TRUE(diagnostic.Level <= NOTICE);
    diagnostic = processes.at(processindex).update(0.02);
    EXPECT_TRUE(diagnostic.Level <= NOTICE);
    diagnostic = processes.at(processindex).new_devicemsg(board1);
    EXPECT_TRUE(diagnostic.Level <= NOTICE);
    diagnostic = processes.at(processindex).update(0.02);
    EXPECT_TRUE(diagnostic.Level <= NOTICE);
    processes.at(processindex).set_boardstate(BOARDMODE_BOOT);
    EXPECT_FALSE(processes.at(processindex).is_finished_initializing());
    EXPECT_TRUE(processes.at(processindex).get_nodestate() == BOARDMODE_BOOT);
    for(int s = 0; s < (board1.ShieldCount);s++)
    {

    	icarus_rover_v2::device shield;
    	shield.DeviceName = "ServoShield" + boost::lexical_cast<std::string>(ShieldID++);
    	shield.DeviceType = "ServoShield";
    	shield.DeviceParent = board1.DeviceName;
    	shield.ID = s+1;
    	shield.pins.clear();
    	for(int i = 0; i < 10;i++)
    	{
    		icarus_rover_v2::pin newpin;
    		newpin.Number = i*2;
    		newpin.Function = "PWMOutput";
    		newpin.DefaultValue = 127;
    		shield.pins.push_back(newpin);
    	}
    	EXPECT_EQ(processes.at(processindex).get_nodestate(),BOARDMODE_BOOT);
    	diagnostic = processes.at(processindex).new_devicemsg(shield);
    	EXPECT_TRUE(diagnostic.Level <= NOTICE);
    	diagnostic = processes.at(processindex).update(0.02);

    	EXPECT_TRUE(diagnostic.Level <= NOTICE);
    }
    EXPECT_TRUE(processes.at(processindex).get_nodestate() == BOARDMODE_INITIALIZING);
    EXPECT_TRUE(check_if_initialized(processes));
    for(int s = 0; s < board1.ShieldCount;s++)
    {
    	EXPECT_EQ(processes.at(processindex).get_portlist(s+1).size(), 3);
    	std::vector<int> portlist = processes.at(processindex).get_portlist(s+1);
    	int availablecount = 0;
    	for(int p = 0; p < portlist.size();p++)
    	{
    		Port_Info port = processes.at(processindex).get_PortInfo(s+1,p+1);
    		EXPECT_TRUE(port.ShieldID > 0);
    		EXPECT_TRUE(port.PortID > 0);

    		for(int i = 0; i < port.Number.size();i++)
    		{
    			if(port.Available.at(i) > 0) {availablecount++;}
    		}
    	}
    	EXPECT_EQ(availablecount,10);
    }

    processindex++;
    icarus_rover_v2::device board2;
    board2.DeviceName = "ArduinoBoard2";
    board2.DeviceType = "ArduinoBoard";
    board2.ID = 2;
    board2.Architecture = "ArduinoMega";
    board2.ShieldCount = 3;
    board2.SensorCount = 0;
    BoardControllerNodeProcess boardprocess2("/dev/dummy2",board2.ID);
    EXPECT_EQ(boardprocess2.get_armedstate(),ARMEDSTATUS_DISARMED_CANNOTARM);
    EXPECT_EQ(boardprocess2.get_ready_to_arm(),false);
    processes.push_back(boardprocess2);
    diagnostic = processes.at(processindex).init(diagnostic,Host_Name,1);
    EXPECT_TRUE(processes.at(processindex).get_nodestate() == BOARDMODE_BOOT);
    EXPECT_TRUE(diagnostic.Level <= NOTICE);
    diagnostic = processes.at(processindex).new_devicemsg(ros_device);
    EXPECT_TRUE(diagnostic.Level <= NOTICE);
    diagnostic = processes.at(processindex).update(0.02);
    EXPECT_TRUE(diagnostic.Level <= NOTICE);
    diagnostic = processes.at(processindex).new_devicemsg(board2);
    EXPECT_TRUE(diagnostic.Level <= NOTICE);
    diagnostic = processes.at(processindex).update(0.02);
    EXPECT_TRUE(diagnostic.Level <= NOTICE);

    processes.at(processindex).set_boardstate(BOARDMODE_BOOT);
    EXPECT_FALSE(processes.at(processindex).is_finished_initializing());
    EXPECT_TRUE(processes.at(processindex).get_nodestate() == BOARDMODE_BOOT);
    for(int s = 0; s < board2.ShieldCount;s++)
    {
    	icarus_rover_v2::device shield;
    	shield.DeviceName = "ServoShield" + boost::lexical_cast<std::string>(ShieldID++);
    	shield.DeviceType = "ServoShield";
    	shield.DeviceParent = board2.DeviceName;
    	shield.ID = s+1;
    	shield.pins.clear();
    	for(int i = 0; i < 10;i++)
    	{
    		icarus_rover_v2::pin newpin;
    		newpin.Number = i*2;
    		newpin.Function = "PWMOutput";
    		newpin.DefaultValue = 127;
    		shield.pins.push_back(newpin);
    	}
    	EXPECT_TRUE(processes.at(processindex).get_nodestate() == BOARDMODE_BOOT);
    	diagnostic = processes.at(processindex).new_devicemsg(shield);
    	EXPECT_TRUE(diagnostic.Level <= NOTICE);
    	diagnostic = processes.at(processindex).update(0.02);
    	EXPECT_TRUE(diagnostic.Level <= NOTICE);
    }
    EXPECT_TRUE(processes.at(processindex).get_nodestate() == BOARDMODE_INITIALIZING);
    EXPECT_TRUE(check_if_initialized(processes));

    for(int s = 0; s < board2.ShieldCount;s++)
    {
    	EXPECT_EQ(processes.at(processindex).get_portlist(s+1).size(), 3);
    	std::vector<int> portlist = processes.at(processindex).get_portlist(s+1);
    	int availablecount = 0;
    	for(int p = 0; p < portlist.size();p++)
    	{
    		Port_Info port = processes.at(processindex).get_PortInfo(s+1,p+1);
    		EXPECT_TRUE(port.ShieldID > 0);
    		EXPECT_TRUE(port.PortID > 0);

    		for(int i = 0; i < port.Number.size();i++)
    		{
    			if(port.Available.at(i) > 0) {availablecount++;}
    		}
    	}
    	EXPECT_EQ(availablecount,10);
    }
    
    processindex++;
    icarus_rover_v2::device board3;
    board3.DeviceName = "ArduinoBoard3";
    board3.DeviceType = "ArduinoBoard";
    board3.ID = 3;
    board3.Architecture = "ArduinoUno";
    board3.ShieldCount = 1;
    board3.SensorCount = 0;
    BoardControllerNodeProcess boardprocess3("/dev/dummy3",board3.ID);
    EXPECT_EQ(boardprocess3.get_armedstate(),ARMEDSTATUS_DISARMED_CANNOTARM);
    EXPECT_EQ(boardprocess3.get_ready_to_arm(),false);
    processes.push_back(boardprocess3);
    diagnostic = processes.at(processindex).init(diagnostic,Host_Name,1);
    EXPECT_TRUE(processes.at(processindex).get_nodestate() == BOARDMODE_BOOT);
    EXPECT_TRUE(diagnostic.Level <= NOTICE);
    diagnostic = processes.at(processindex).new_devicemsg(ros_device);
    EXPECT_TRUE(diagnostic.Level <= NOTICE);
    diagnostic = processes.at(processindex).update(0.02);
    EXPECT_TRUE(diagnostic.Level <= NOTICE);
    diagnostic = processes.at(processindex).new_devicemsg(board3);
    EXPECT_TRUE(diagnostic.Level <= NOTICE);
    diagnostic = processes.at(processindex).update(0.02);
    EXPECT_TRUE(diagnostic.Level <= NOTICE);

    processes.at(processindex).set_boardstate(BOARDMODE_BOOT);
    EXPECT_FALSE(processes.at(processindex).is_finished_initializing());
    EXPECT_TRUE(processes.at(processindex).get_nodestate() == BOARDMODE_BOOT);
    for(int s = 0; s < board3.ShieldCount;s++)
    {
    	icarus_rover_v2::device shield;
    	shield.DeviceName = "LCDShield" + boost::lexical_cast<std::string>(ShieldID++);
    	shield.DeviceType = "LCDShield";
    	shield.DeviceParent = board3.DeviceName;
    	shield.ID = s+1;
    	shield.pins.clear();
        
        for(int i = 0; i < 4;i++)
    	{
    		icarus_rover_v2::pin newpin;
    		newpin.Number = i;
    		newpin.Function = "DigitalInput";
    		newpin.DefaultValue = 0;
    		shield.pins.push_back(newpin);
    	}

    	
    	EXPECT_TRUE(processes.at(processindex).get_nodestate() == BOARDMODE_BOOT);
    	diagnostic = processes.at(processindex).new_devicemsg(shield);
    	EXPECT_TRUE(diagnostic.Level <= NOTICE);
    	diagnostic = processes.at(processindex).update(0.02);
    	EXPECT_TRUE(diagnostic.Level <= NOTICE);
        
    }
    EXPECT_TRUE(processes.at(processindex).get_nodestate() == BOARDMODE_INITIALIZING);
    EXPECT_TRUE(check_if_initialized(processes));
    
    initialized_processes = processes;
    */
}
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}