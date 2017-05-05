#include <gtest/gtest.h>
#include "ros/ros.h"
#include "ros/time.h"
#include <unistd.h>
#include "std_msgs/Bool.h"
#include "icarus_rover_v2/device.h"
#include "icarus_rover_v2/diagnostic.h"

#include "../boardcontroller_node_process.h"

std::string Node_Name = "/unittest_boardcontroller_node_process";
std::string Host_Name = "unittest";
std::string ros_DeviceName = Host_Name;
std::string ros_ParentDevice = "";
std::string ros_DeviceType = "ComputeModule";

std::vector<BoardControllerNodeProcess> initialized_processes;
std::vector<BoardControllerNodeProcess> configured_processes;

bool check_if_initialized(std::vector<BoardControllerNodeProcess> processes);
void print_processinfo(std::vector<BoardControllerNodeProcess> processes);
TEST(ProcessBootInitialization,ProcessBootInitialization_1Board_1Shield_0Pins)
{
    std::vector<BoardControllerNodeProcess> processes;
    icarus_rover_v2::device ros_device;
    ros_device.DeviceName = ros_DeviceName;
	ros_device.DeviceParent = ros_ParentDevice;
	ros_device.DeviceType = ros_DeviceType;
    ros_device.BoardCount = 1;
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
    for(int j = 0; j < ros_device.BoardCount;j++)
    {
        std::string boardname = "ArduinoBoard" + boost::lexical_cast<std::string>(j+1);
        icarus_rover_v2::device board;
        board.DeviceParent = ros_device.DeviceName;
        board.DeviceName = boardname;
        board.DeviceType = "ArduinoBoard";
        board.ID = j+1;
        board.Architecture = "ArduinoUno";
        board.ShieldCount = 1;
        board.SensorCount = 0;
        icarus_rover_v2::device shield;
        shield.DeviceName = "TerminalShield1";
        shield.ID = 0;
        shield.DeviceParent = board.DeviceName;
        shield.DeviceType = "TerminalShield";
        shield.pins.clear();
        BoardControllerNodeProcess boardprocess("/dev/dummy",j+1);
        diagnostic = boardprocess.init(diagnostic,Host_Name,j+1);
        EXPECT_TRUE(diagnostic.Level <= NOTICE);
        diagnostic = boardprocess.new_devicemsg(ros_device);
        EXPECT_TRUE(diagnostic.Level <= NOTICE);
        diagnostic = boardprocess.new_devicemsg(board);
        EXPECT_TRUE(diagnostic.Level <= NOTICE);
        EXPECT_TRUE(boardprocess.get_boardname()==boardname);
        diagnostic = boardprocess.update(20.0);
        EXPECT_TRUE(diagnostic.Level <= NOTICE);
        diagnostic = boardprocess.new_devicemsg(shield);
        EXPECT_TRUE(diagnostic.Level <= NOTICE);
        diagnostic = boardprocess.update(.01);
        EXPECT_TRUE(diagnostic.Level <= NOTICE);
        processes.push_back(boardprocess);
    }
    EXPECT_TRUE(processes.size() == ros_device.BoardCount);
    EXPECT_TRUE(check_if_initialized(processes));
}
TEST(ProcessBootInitialization,ProcessBootInitialization_1Board_2Shields_FullPins)
{
    std::vector<BoardControllerNodeProcess> processes;
    icarus_rover_v2::device ros_device;
    ros_device.DeviceName = ros_DeviceName;
	ros_device.DeviceParent = ros_ParentDevice;
	ros_device.DeviceType = ros_DeviceType;
    ros_device.BoardCount = 1;
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
    for(int j = 0; j < ros_device.BoardCount;j++)
    {
        std::string boardname = "ArduinoBoard" + boost::lexical_cast<std::string>(j+1);
        icarus_rover_v2::device board;
        board.DeviceParent = ros_device.DeviceName;
        board.DeviceName = boardname;
        board.DeviceType = "ArduinoBoard";
        board.ID = j+1;
        board.Architecture = "ArduinoMega";
        board.ShieldCount = 2;
        board.SensorCount = 0;
        icarus_rover_v2::device shield1;
        shield1.DeviceName = "TerminalShield1";
        shield1.ID = 0;
        shield1.DeviceParent = board.DeviceName;
        shield1.DeviceType = "TerminalShield";
        shield1.pins.clear();
        for(int i = 0; i < 24;i++)
    	{
    		icarus_rover_v2::pin newpin;
    		newpin.Number = i;
    		newpin.Function = "DigitalOutput";
    		newpin.DefaultValue = 0;
    		shield1.pins.push_back(newpin);
    	}
        for(int i = 24; i < 48;i++)
    	{
    		icarus_rover_v2::pin newpin;
    		newpin.Number = i;
    		newpin.Function = "DigitalInput";
    		newpin.DefaultValue = 0;
    		shield1.pins.push_back(newpin);
    	}
        for(int i = 0; i < 8;i++)
    	{
    		icarus_rover_v2::pin newpin;
    		newpin.Number = i;
    		newpin.Function = "AnalogInput";
    		newpin.DefaultValue = 0;
            newpin.ADCResolution = 10;
            newpin.VoltageReference = 5.0;
            newpin.ScaleFactor = 1.0;
    		shield1.pins.push_back(newpin);
    	}
        icarus_rover_v2::device shield2;
        shield2.DeviceName = "ServoShield1";
        shield2.ID = 40;
        shield2.DeviceParent = board.DeviceName;
        shield2.DeviceType = "ServoShield";
        shield2.pins.clear();
        for(int i = 0; i < 16;i++)
    	{
    		icarus_rover_v2::pin newpin;
    		newpin.Number = i;
    		newpin.Function = "PWMOutput";
    		newpin.DefaultValue = 127;
    		shield2.pins.push_back(newpin);
    	}
        BoardControllerNodeProcess boardprocess("/dev/dummy",j+1);
        diagnostic = boardprocess.init(diagnostic,Host_Name,j+1);
        EXPECT_TRUE(diagnostic.Level <= NOTICE);
        diagnostic = boardprocess.new_devicemsg(ros_device);
        EXPECT_TRUE(diagnostic.Level <= NOTICE);
        diagnostic = boardprocess.new_devicemsg(board);
        EXPECT_TRUE(diagnostic.Level <= NOTICE);
        EXPECT_TRUE(boardprocess.get_boardname()==boardname);
        diagnostic = boardprocess.update(20.0);
        EXPECT_TRUE(diagnostic.Level <= NOTICE);
        diagnostic = boardprocess.new_devicemsg(shield1);
        EXPECT_TRUE(diagnostic.Level <= NOTICE);
        diagnostic = boardprocess.new_devicemsg(shield2);
        EXPECT_TRUE(diagnostic.Level <= NOTICE);
        diagnostic = boardprocess.update(.01);
        EXPECT_TRUE(diagnostic.Level <= NOTICE);
        EXPECT_FALSE(boardprocess.get_ready_to_arm());
        EXPECT_TRUE(boardprocess.get_armedstate() == ARMEDSTATUS_DISARMED_CANNOTARM);
        processes.push_back(boardprocess);
        
    }
    //print_processinfo(processes);
    EXPECT_TRUE(processes.size() == ros_device.BoardCount);
    EXPECT_TRUE(check_if_initialized(processes));
    initialized_processes = processes;
}
TEST(Operation,NormalOperation_1Board_2Shields_FullPins)
{
    SerialMessageHandler *serialmessagehandler;
    serialmessagehandler = new SerialMessageHandler();
    icarus_rover_v2::device ros_device;
    ros_device.DeviceName = ros_DeviceName;
	ros_device.DeviceParent = ros_ParentDevice;
	ros_device.DeviceType = ros_DeviceType;
    ros_device.BoardCount = 1;
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
    std::vector<BoardControllerNodeProcess> processes = initialized_processes;
    EXPECT_TRUE(processes.size() == ros_device.BoardCount);
    EXPECT_TRUE(check_if_initialized(processes));
    for(std::size_t i = 0; i < processes.size(); i++)
    {
        std::vector<std::vector<unsigned char> > tx_buffers;
        EXPECT_TRUE(processes.at(i).get_nodestate() == BOARDMODE_INITIALIZING);
        {
            unsigned char packet[12];
            int length;
            int computed_checksum;
            packet[0] = 0;
            packet[1] = 0;
            packet[2] = BOARDMODE_INITIALIZING;
            diagnostic = processes.at(i).new_serialmessage_Get_Mode(SERIAL_Mode_ID,packet);    
            EXPECT_TRUE(diagnostic.Level <= NOTICE);
        }
        diagnostic = processes.at(i).update(.1);
        EXPECT_TRUE(diagnostic.Level <= NOTICE);
        EXPECT_TRUE(processes.at(i).get_nodestate() == BOARDMODE_CONFIGURING);
        EXPECT_FALSE(processes.at(i).get_ready_to_arm());
        EXPECT_TRUE(processes.at(i).get_armedstate() == ARMEDSTATUS_DISARMED_CANNOTARM);
        tx_buffers.clear();
		EXPECT_TRUE(processes.at(i).checkTriggers(tx_buffers));
        bool found_send_config_command = false;
        for(std::size_t j = 0; j < tx_buffers.size(); j++)
        {
            unsigned char tx_buffer[16];
            std::vector<unsigned char> tempstr = tx_buffers.at(j);
            if((tempstr[1] == SERIAL_Command_ID) and (tempstr[3] == ROVERCOMMAND_CONFIGURE))
            {
                found_send_config_command = true;
            }
        }
        EXPECT_TRUE(found_send_config_command);
        {
            unsigned char packet[12];
            int length;
            int computed_checksum;
            packet[0] = 0;
            packet[1] = 0;
            packet[2] = BOARDMODE_CONFIGURING;
            diagnostic = processes.at(i).new_serialmessage_Get_Mode(SERIAL_Mode_ID,packet);    
            EXPECT_TRUE(diagnostic.Level <= NOTICE);
        }
        diagnostic = processes.at(i).update(.1);
        EXPECT_TRUE(diagnostic.Level <= NOTICE);
        for(int k = 0; k < 5; k++)
        {
            tx_buffers.clear();
            EXPECT_TRUE(processes.at(i).checkTriggers(tx_buffers));
            //Check Configure DIO Messages
            {
                std::vector<Port_Info> dio_ports = processes.at(i).get_alldioports();
                int dio_port_messages = 0;
                for(int j = 0; j < tx_buffers.size();j++)
                {
                    unsigned char tx_buffer[16];
                    std::vector<unsigned char> tempstr = tx_buffers.at(j);
                    if(tempstr[1] == SERIAL_Configure_DIO_Port_ID)
                    {
                        dio_port_messages++;
                    }
                }
                EXPECT_TRUE(dio_port_messages > 0);
                EXPECT_TRUE(dio_ports.size() == dio_port_messages);    
            }
            diagnostic = processes.at(i).update(.1);
            EXPECT_TRUE(diagnostic.Level <= NOTICE);
            
            //Check DIO Default Messages
            {
                std::vector<Port_Info> dio_ports = processes.at(i).get_alldioports();
                int dio_default_messages = 0;
                for(int j = 0; j < tx_buffers.size();j++)
                {
                    unsigned char tx_buffer[16];
                    std::vector<unsigned char> tempstr = tx_buffers.at(j);
                    if(tempstr[1] == SERIAL_Set_DIO_Port_DefaultValue_ID)
                    {
                        dio_default_messages++;
                    }
                }
                EXPECT_TRUE(dio_default_messages > 0);
                EXPECT_TRUE(dio_ports.size() == dio_default_messages);    
                diagnostic = processes.at(i).update(.1);
                EXPECT_TRUE(diagnostic.Level <= NOTICE);
            }
            EXPECT_FALSE(processes.at(i).get_ready_to_arm());
            EXPECT_TRUE(processes.at(i).get_armedstate() == ARMEDSTATUS_DISARMED_CANNOTARM);
            //Check Configure ANA Messages
            {
                std::vector<Port_Info> ana_ports = processes.at(i).get_allanaports();
                int ana_port_messages = 0;
                for(int j = 0; j < tx_buffers.size();j++)
                {
                    unsigned char tx_buffer[16];
                    std::vector<unsigned char> tempstr = tx_buffers.at(j);
                    if(tempstr[1] == SERIAL_Configure_ANA_Port_ID)
                    {
                        ana_port_messages++;
                    }
                }
                EXPECT_TRUE(ana_port_messages > 0);
                EXPECT_TRUE(ana_ports.size() == ana_port_messages); 
            }
            diagnostic = processes.at(i).update(.1);
            EXPECT_TRUE(diagnostic.Level <= NOTICE);
        }        
        EXPECT_TRUE(processes.at(i).get_nodestate() == BOARDMODE_CONFIGURED);
        {
            unsigned char packet[12];
            int length;
            int computed_checksum;
            packet[0] = 0;
            packet[1] = 0;
            packet[2] = BOARDMODE_CONFIGURED;
            diagnostic = processes.at(i).new_serialmessage_Get_Mode(SERIAL_Mode_ID,packet);    
            EXPECT_TRUE(diagnostic.Level <= NOTICE);
        }
        diagnostic = processes.at(i).update(.1);
        EXPECT_TRUE(diagnostic.Level <= NOTICE);
        EXPECT_TRUE(processes.at(i).get_nodestate() == BOARDMODE_RUNNING);
        EXPECT_FALSE(processes.at(i).get_ready_to_arm());
        EXPECT_TRUE(processes.at(i).get_armedstate() == ARMEDSTATUS_DISARMED_CANNOTARM);
        {
            unsigned char packet[12];
            int length;
            int computed_checksum;
            packet[0] = 0;
            packet[1] = 0;
            packet[2] = BOARDMODE_RUNNING;
            diagnostic = processes.at(i).new_serialmessage_Get_Mode(SERIAL_Mode_ID,packet);    
            EXPECT_TRUE(diagnostic.Level <= NOTICE);
            diagnostic = processes.at(i).update(.1);
            EXPECT_TRUE(diagnostic.Level <= NOTICE);
            EXPECT_TRUE(processes.at(i).get_boardstate() == BOARDMODE_RUNNING);
            EXPECT_TRUE(processes.at(i).get_nodestate() == BOARDMODE_RUNNING);
        }
        diagnostic = processes.at(i).update(.1);
        EXPECT_TRUE(diagnostic.Level <= NOTICE);
        EXPECT_TRUE(processes.at(i).get_ready_to_arm());
        EXPECT_TRUE(processes.at(i).get_armedstate() == ARMEDSTATUS_DISARMED);
    }
    configured_processes = processes;
}

TEST(Operation,Running_1Board_2Shields_FullPins)
{
    SerialMessageHandler *serialmessagehandler;
    serialmessagehandler = new SerialMessageHandler();
    icarus_rover_v2::device ros_device;
    ros_device.DeviceName = ros_DeviceName;
	ros_device.DeviceParent = ros_ParentDevice;
	ros_device.DeviceType = ros_DeviceType;
    ros_device.BoardCount = 1;
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
    std::vector<BoardControllerNodeProcess> processes = configured_processes;
    for(std::size_t i = 0; i < processes.size(); i++)
    {
        std::vector<std::vector<unsigned char> > tx_buffers;
        EXPECT_TRUE(processes.at(i).get_nodestate() == BOARDMODE_RUNNING);
        EXPECT_TRUE(processes.at(i).get_boardstate() == BOARDMODE_RUNNING);
        EXPECT_TRUE(processes.at(i).get_ready_to_arm());
        EXPECT_TRUE(processes.at(i).get_armedstate() == ARMEDSTATUS_DISARMED);
        diagnostic = processes.at(i).update(.1);
        EXPECT_TRUE(diagnostic.Level <= NOTICE);
        tx_buffers.clear();
		EXPECT_TRUE(processes.at(i).checkTriggers(tx_buffers));
        //Check Send DIO Set Messages
        {
            std::vector<Port_Info> dio_ports = processes.at(i).get_alldioports();
            int set_dioport_messages = 0;
            for(int j = 0; j < tx_buffers.size();j++)
            {
                unsigned char tx_buffer[16];
                std::vector<unsigned char> tempstr = tx_buffers.at(j);
                if(tempstr[1] == SERIAL_Set_DIO_Port_ID)
                {
                    set_dioport_messages++;
                }
            }
            EXPECT_TRUE(set_dioport_messages > 0);
            EXPECT_TRUE(dio_ports.size() == set_dioport_messages); 
        }
        diagnostic = processes.at(i).update(.1);
        EXPECT_TRUE(diagnostic.Level <= NOTICE);
        
        //Check Analog Inputs
        std::vector<Port_Info> ana_ports = processes.at(i).get_allanaports();
        int available_analog_pins = 0;
        for(std::size_t j = 0; j < ana_ports.size(); j++)
        {
            for(int k = 0; k < ANA_PORT_SIZE; k++)
            {
                if(ana_ports.at(j).Available[k] == 1)
                {
                   available_analog_pins++; 
                }
            }
        }
        int step_size = 1024/(available_analog_pins-1);
        int current_value = 1;
        for(std::size_t j = 0; j < ana_ports.size(); j++)
        {
            std::vector<int> port_value;
            for(int k = 0; k < ANA_PORT_SIZE; k++)
            {
                port_value.push_back(current_value-1);
                
                current_value+= step_size;
            }
            char buffer[16];
			int length;
			int computed_checksum;
            int tx_status = serialmessagehandler->encode_Get_ANA_PortSerial(buffer,&length,ana_ports.at(j).ShieldID,ana_ports.at(j).PortID,
                port_value[0],port_value[1],port_value[2],port_value[3]);
            unsigned char buffer2[12];
            for(int k = 0; k < 12; k++)
            {
                buffer2[k] = buffer[k+3];
            }
            diagnostic = processes.at(i).new_serialmessage_Get_ANA_Port(SERIAL_Get_ANA_Port_ID,buffer2);
            EXPECT_TRUE(diagnostic.Level <= NOTICE);
            Port_Info port = processes.at(i).get_anaPortInfo(ana_ports.at(j).ShieldID,ana_ports.at(j).PortID);
            for(int k = 0; k < ANA_PORT_SIZE; k++)
            {
                float expected_value = port_value[k]*5.0/1024.0;
                float error = fabs(expected_value-port.Value[k]);
                EXPECT_TRUE(error < .0000001);
                //EXPECT_TRUE(port_value.at(k)==port.Value[k]);
            }
        }
        

        //Check PPS Transmit & Receive
        for(int j = 0; j < 10; j++)
        {
            diagnostic = processes.at(i).new_pps_transmit();
            EXPECT_TRUE(diagnostic.Level <= NOTICE);
            tx_buffers.clear();
            EXPECT_TRUE(processes.at(i).checkTriggers(tx_buffers));
            bool found_send_pps = false;
            for(std::size_t j = 0; j < tx_buffers.size(); j++)
            {
                unsigned char tx_buffer[16];
                std::vector<unsigned char> tempstr = tx_buffers.at(j);
                if((tempstr[1] == SERIAL_PPS_ID))
                {
                    found_send_pps = true;
                }
            }
            EXPECT_TRUE(found_send_pps);
            {
                unsigned char packet[12];
                int length;
                int computed_checksum;
                packet[0] = j+1;
                diagnostic = processes.at(i).new_serialmessage_PPS(SERIAL_PPS_ID,packet);
                EXPECT_TRUE(diagnostic.Level <= NOTICE);
            }
        }
        
        //Check Diagnostic Transmit
        for(int j = 0; j < 10; j++)
        {
            diagnostic.Diagnostic_Type = SOFTWARE;
            diagnostic.Level = WARN;
            diagnostic.Diagnostic_Message = DROPPING_PACKETS;
            diagnostic.Description = "Test Diagnostic Message";
            diagnostic = processes.at(i).new_diagnosticmsg(diagnostic);
            EXPECT_TRUE(diagnostic.Level <= NOTICE);
        }
        tx_buffers.clear();
		EXPECT_TRUE(processes.at(i).checkTriggers(tx_buffers));
        int found_send_diagnostic = 0;
        for(std::size_t j = 0; j < tx_buffers.size(); j++)
        {
            unsigned char tx_buffer[16];
            std::vector<unsigned char> tempstr = tx_buffers.at(j);
            if((tempstr[1] == SERIAL_Diagnostic_ID))
            {
                found_send_diagnostic++;
            }
        }
        EXPECT_TRUE(found_send_diagnostic==10);
        EXPECT_TRUE(processes.at(i).get_diagnostics_to_send().size() == 0);
        
        //Check Arm/Disarm Command Transmit
        icarus_rover_v2::command arm_command;
        arm_command.Command = ROVERCOMMAND_DISARM;
        diagnostic = processes.at(i).new_commandmsg(arm_command);
        if( processes.at(i).get_stateack("Send Arm Command").stream_rate > 0)
        {
            usleep(1000+(1000000*(1.0/processes.at(i).get_stateack("Send Arm Command").stream_rate)));
        }
        EXPECT_TRUE(diagnostic.Level <= NOTICE);
        {
            tx_buffers.clear();
            EXPECT_TRUE(processes.at(i).checkTriggers(tx_buffers));
            bool found_send_disarmcommand = false;
            for(std::size_t j = 0; j < tx_buffers.size(); j++)
            {
                unsigned char tx_buffer[16];
                std::vector<unsigned char> tempstr = tx_buffers.at(j);
                if((tempstr[1] == SERIAL_Command_ID) && (tempstr[3] == ROVERCOMMAND_DISARM))
                {
                    found_send_disarmcommand = true;
                }
            }
            EXPECT_TRUE(found_send_disarmcommand);
        }
        diagnostic = processes.at(i).update(.01);
        EXPECT_TRUE(diagnostic.Level <= NOTICE);
        EXPECT_TRUE(processes.at(i).get_armedstate() == ARMEDSTATUS_DISARMED);
        arm_command.Command = ROVERCOMMAND_ARM;
        diagnostic = processes.at(i).new_commandmsg(arm_command);
        EXPECT_TRUE(diagnostic.Level <= NOTICE);
        diagnostic = processes.at(i).update(.01);
        EXPECT_TRUE(diagnostic.Level <= NOTICE);
        EXPECT_TRUE(processes.at(i).get_armedstate() == ARMEDSTATUS_ARMED);
        if( processes.at(i).get_stateack("Send Arm Command").stream_rate > 0)
        {
            usleep(1000+(1000000*(1.0/processes.at(i).get_stateack("Send Arm Command").stream_rate)));
        }
        EXPECT_TRUE(diagnostic.Level <= NOTICE);
        {
            tx_buffers.clear();
            EXPECT_TRUE(processes.at(i).checkTriggers(tx_buffers));
            bool found_send_armcommand = false;
            for(std::size_t j = 0; j < tx_buffers.size(); j++)
            {
                unsigned char tx_buffer[16];
                std::vector<unsigned char> tempstr = tx_buffers.at(j);
                if((tempstr[1] == SERIAL_Command_ID) && (tempstr[3] == ROVERCOMMAND_ARM))
                {
                    found_send_armcommand = true;
                }
            }
            EXPECT_TRUE(found_send_armcommand);
        }
        arm_command.Command = ROVERCOMMAND_DISARM;
        diagnostic = processes.at(i).new_commandmsg(arm_command);
        EXPECT_TRUE(diagnostic.Level <= NOTICE);
        diagnostic = processes.at(i).update(.01);
        EXPECT_TRUE(diagnostic.Level <= NOTICE);
        EXPECT_TRUE(processes.at(i).get_armedstate() == ARMEDSTATUS_DISARMED);
        if( processes.at(i).get_stateack("Send Arm Command").stream_rate > 0)
        {
            usleep(1000+(1000000*(1.0/processes.at(i).get_stateack("Send Arm Command").stream_rate)));
        }
        EXPECT_TRUE(diagnostic.Level <= NOTICE);
        {
            tx_buffers.clear();
            EXPECT_TRUE(processes.at(i).checkTriggers(tx_buffers));
            bool found_send_disarmcommand = false;
            for(std::size_t j = 0; j < tx_buffers.size(); j++)
            {
                unsigned char tx_buffer[16];
                std::vector<unsigned char> tempstr = tx_buffers.at(j);
                if((tempstr[1] == SERIAL_Command_ID) && (tempstr[3] == ROVERCOMMAND_DISARM))
                {
                    found_send_disarmcommand = true;
                }
            }
            EXPECT_TRUE(found_send_disarmcommand);
        }
        
        
    }
}
/*
TEST(Operation,ErrorChecking_1Board_2Shields_FullPins)
{
    SerialMessageHandler *serialmessagehandler;
    serialmessagehandler = new SerialMessageHandler();
    icarus_rover_v2::device ros_device;
    ros_device.DeviceName = ros_DeviceName;
	ros_device.DeviceParent = ros_ParentDevice;
	ros_device.DeviceType = ros_DeviceType;
    ros_device.BoardCount = 1;
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
    std::vector<BoardControllerNodeProcess> processes = initialized_processes;
    EXPECT_TRUE(processes.size() == ros_device.BoardCount);
    EXPECT_TRUE(check_if_initialized(processes));
    for(std::size_t i = 0; i < processes.size(); i++)
    {
        std::vector<std::vector<unsigned char> > tx_buffers;
        EXPECT_TRUE(processes.at(i).get_nodestate() == BOARDMODE_INITIALIZING);
        {
            unsigned char packet[12];
            int length;
            int computed_checksum;
            packet[0] = 0;
            packet[1] = 0;
            packet[2] = BOARDMODE_INITIALIZING;
            diagnostic = processes.at(i).new_serialmessage_Get_Mode(SERIAL_Mode_ID,packet);    
            EXPECT_TRUE(diagnostic.Level <= NOTICE);
        }
        diagnostic = processes.at(i).update(.1);
        EXPECT_TRUE(diagnostic.Level <= NOTICE);
        EXPECT_TRUE(processes.at(i).get_nodestate() == BOARDMODE_CONFIGURING);
        EXPECT_FALSE(processes.at(i).get_ready_to_arm());
        EXPECT_TRUE(processes.at(i).get_armedstate() == ARMEDSTATUS_DISARMED_CANNOTARM);
        tx_buffers.clear();
		EXPECT_TRUE(processes.at(i).checkTriggers(tx_buffers));
        bool found_send_config_command = false;
        for(std::size_t j = 0; j < tx_buffers.size(); j++)
        {
            unsigned char tx_buffer[16];
            std::vector<unsigned char> tempstr = tx_buffers.at(j);
            if((tempstr[1] == SERIAL_Command_ID) and (tempstr[3] == ROVERCOMMAND_CONFIGURE))
            {
                found_send_config_command = true;
            }
        }
        EXPECT_TRUE(found_send_config_command);
        {
            unsigned char packet[12];
            int length;
            int computed_checksum;
            packet[0] = 0;
            packet[1] = 0;
            packet[2] = BOARDMODE_CONFIGURING;
            diagnostic = processes.at(i).new_serialmessage_Get_Mode(SERIAL_Mode_ID,packet);    
            EXPECT_TRUE(diagnostic.Level <= NOTICE);
        }
        diagnostic = processes.at(i).update(.1);
        EXPECT_TRUE(diagnostic.Level <= NOTICE);
        for(int k = 0; k < 5; k++)
        {
            tx_buffers.clear();
            EXPECT_TRUE(processes.at(i).checkTriggers(tx_buffers));
            //Check Configure DIO Messages
            {
                std::vector<Port_Info> dio_ports = processes.at(i).get_alldioports();
                int dio_port_messages = 0;
                for(int j = 0; j < tx_buffers.size();j++)
                {
                    unsigned char tx_buffer[16];
                    std::vector<unsigned char> tempstr = tx_buffers.at(j);
                    if(tempstr[1] == SERIAL_Configure_DIO_Port_ID)
                    {
                        dio_port_messages++;
                    }
                }
                EXPECT_TRUE(dio_port_messages > 0);
                EXPECT_TRUE(dio_ports.size() == dio_port_messages);    
            }
            diagnostic = processes.at(i).update(.1);
            EXPECT_TRUE(diagnostic.Level <= NOTICE);
            
            //Check DIO Default Messages
            {
                std::vector<Port_Info> dio_ports = processes.at(i).get_alldioports();
                int dio_default_messages = 0;
                for(int j = 0; j < tx_buffers.size();j++)
                {
                    unsigned char tx_buffer[16];
                    std::vector<unsigned char> tempstr = tx_buffers.at(j);
                    if(tempstr[1] == SERIAL_Set_DIO_Port_DefaultValue_ID)
                    {
                        dio_default_messages++;
                    }
                }
                EXPECT_TRUE(dio_default_messages > 0);
                EXPECT_TRUE(dio_ports.size() == dio_default_messages);    
                diagnostic = processes.at(i).update(.1);
                EXPECT_TRUE(diagnostic.Level <= NOTICE);
            }
            EXPECT_FALSE(processes.at(i).get_ready_to_arm());
            EXPECT_TRUE(processes.at(i).get_armedstate() == ARMEDSTATUS_DISARMED_CANNOTARM);
            //Check Configure ANA Messages
            {
                std::vector<Port_Info> ana_ports = processes.at(i).get_allanaports();
                int ana_port_messages = 0;
                for(int j = 0; j < tx_buffers.size();j++)
                {
                    unsigned char tx_buffer[16];
                    std::vector<unsigned char> tempstr = tx_buffers.at(j);
                    if(tempstr[1] == SERIAL_Configure_ANA_Port_ID)
                    {
                        ana_port_messages++;
                    }
                }
                EXPECT_TRUE(ana_port_messages > 0);
                EXPECT_TRUE(ana_ports.size() == ana_port_messages); 
            }
            diagnostic = processes.at(i).update(.1);
            EXPECT_TRUE(diagnostic.Level <= NOTICE);
        }        
        EXPECT_TRUE(processes.at(i).get_nodestate() == BOARDMODE_CONFIGURED);
        {
            unsigned char packet[12];
            int length;
            int computed_checksum;
            packet[0] = 0;
            packet[1] = 0;
            packet[2] = BOARDMODE_CONFIGURED;
            diagnostic = processes.at(i).new_serialmessage_Get_Mode(SERIAL_Mode_ID,packet);    
            EXPECT_TRUE(diagnostic.Level <= NOTICE);
        }
        diagnostic = processes.at(i).update(.1);
        EXPECT_TRUE(diagnostic.Level <= NOTICE);
        EXPECT_TRUE(processes.at(i).get_nodestate() == BOARDMODE_RUNNING);
        EXPECT_FALSE(processes.at(i).get_ready_to_arm());
        EXPECT_TRUE(processes.at(i).get_armedstate() == ARMEDSTATUS_DISARMED_CANNOTARM);
        {
            unsigned char packet[12];
            int length;
            int computed_checksum;
            packet[0] = 0;
            packet[1] = 0;
            packet[2] = BOARDMODE_RUNNING;
            diagnostic = processes.at(i).new_serialmessage_Get_Mode(SERIAL_Mode_ID,packet);    
            EXPECT_TRUE(diagnostic.Level <= NOTICE);
            diagnostic = processes.at(i).update(.1);
            EXPECT_TRUE(diagnostic.Level <= NOTICE);
            EXPECT_TRUE(processes.at(i).get_boardstate() == BOARDMODE_RUNNING);
            EXPECT_TRUE(processes.at(i).get_nodestate() == BOARDMODE_RUNNING);
        }
        diagnostic = processes.at(i).update(.1);
        EXPECT_TRUE(diagnostic.Level <= NOTICE);
        EXPECT_TRUE(processes.at(i).get_ready_to_arm());
        EXPECT_TRUE(processes.at(i).get_armedstate() == ARMEDSTATUS_DISARMED);
    }
}
*/

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
bool check_if_initialized(std::vector<BoardControllerNodeProcess> processes)
{
	bool status = true;
    if(processes.size() == 0)
    {
        return false;
    }
	for(int i = 0; i < processes.size();i++)
	{
		bool temp = processes.at(i).is_finished_initializing();
		EXPECT_TRUE(temp);
		status = status and temp;
        EXPECT_EQ(processes.at(i).get_nodestate(),BOARDMODE_INITIALIZING);
	}
	return status;
}

void print_processinfo(std::vector<BoardControllerNodeProcess> processes)
{
	for(int i = 0; i < processes.size(); i++)
	{
		//printf("Process Board: %s:%d Shield Count: %d\n",processes.at(i).get_boardname().c_str(),processes.at(i).get_boardid(),processes.at(i).get_shields().size());
		std::vector<icarus_rover_v2::device> shields = processes.at(i).get_shields();

		for(int s = 0; s < shields.size();s++)
		{
			printf("Shield %s:%ld\n",shields.at(s).DeviceName.c_str(),shields.at(s).ID);
			std::vector<Port_Info> dio_portlist = processes.at(i).get_alldioports();
			for(std::size_t j = 0; j < dio_portlist.size(); j++)
			{
				if(dio_portlist.at(j).ShieldID == shields.at(s).ID)
				{
					for(std::size_t p = 0; p < dio_portlist.at(j).Number.size(); p++)
					{
						printf("DIO Port ID: %d Pin: Number: %d Mode: %s Available: %d\n",
                        dio_portlist.at(j).PortID,
                        dio_portlist.at(j).Number[p],
                        processes.at(i).map_PinFunction_ToString(dio_portlist.at(j).Mode[p]).c_str(),
                        dio_portlist.at(j).Available[p]
                        );
					}
				}
			}
            std::vector<Port_Info> ana_portlist = processes.at(i).get_allanaports();
			for(std::size_t j = 0; j < ana_portlist.size(); j++)
			{
				if(ana_portlist.at(j).ShieldID == shields.at(s).ID)
				{
					for(std::size_t p = 0; p < ana_portlist.at(j).Number.size(); p++)
					{
						printf("ANA Port ID: %d Pin: Number: %d Mode: %s Available: %d\n",
                        ana_portlist.at(j).PortID,
                        ana_portlist.at(j).Number[p],
                        processes.at(i).map_PinFunction_ToString(ana_portlist.at(j).Mode[p]).c_str(),
                        ana_portlist.at(j).Available[p]
                        );
					}
				}
			}
			//printf("Pin Count: %d\n",shields.at(s))
		}

	}
}
