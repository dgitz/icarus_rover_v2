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
        }
        diagnostic = processes.at(i).update(.1);
        EXPECT_TRUE(diagnostic.Level <= NOTICE);
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
        int step_size = 65536/(available_analog_pins-1);
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
                EXPECT_TRUE(port_value.at(k)==port.Value[k]);
            }
        }
        

        //Check PPS Transmit
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
TEST(DeviceInitialization,DeviceInitialization_0_4Boards_0Shields)
{
	int MaxBoardCount = 4;
	for(int i = 0; i <= MaxBoardCount; i++)
	{
		std::vector<BoardControllerNodeProcess> processes;
		icarus_rover_v2::device ros_device;
		ros_device.DeviceName = ros_DeviceName;
		ros_device.DeviceParent = ros_ParentDevice;
		ros_device.DeviceType = ros_DeviceType;
		ros_device.BoardCount = i;


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
		std::string log_output = Node_Name + boost::lexical_cast<std::string>(i);
		logger = new Logger("DEBUG",log_output);
		for(int j = 0; j < ros_device.BoardCount;j++)
		{
			std::string boardname = "ArduinoBoard" + boost::lexical_cast<std::string>(j+1);
			icarus_rover_v2::device board;
			board.DeviceName = boardname;
			board.DeviceType = "ArduinoBoard";
			board.ID = j+1;
			board.Architecture = "ArduinoUno";
			board.ShieldCount = 0;
			board.SensorCount = 0;
			BoardControllerNodeProcess boardprocess("/dev/dummy",j+1);
			diagnostic = boardprocess.init(diagnostic,logger,Host_Name,j+1);
			EXPECT_TRUE(diagnostic.Level <= NOTICE);
			diagnostic = boardprocess.new_devicemsg(ros_device);
			EXPECT_TRUE(diagnostic.Level <= NOTICE);
			diagnostic = boardprocess.new_devicemsg(board);
			EXPECT_TRUE(diagnostic.Level <= NOTICE);
			printf("Board Name: %s\n",boardprocess.get_boardname().c_str());
			EXPECT_TRUE(boardprocess.get_boardname()==boardname);
			diagnostic = boardprocess.update(20.0);
			EXPECT_TRUE(diagnostic.Level <= NOTICE);
			processes.push_back(boardprocess);
		}

		EXPECT_TRUE(processes.size() == ros_device.BoardCount);
		EXPECT_TRUE(check_if_initialized(processes));
	}
}
*/
/*
TEST(DeviceInitialization,DeviceInitialization_2Boards_3Shields)
{
    int processindex = 0;
	int ShieldID = 1;
    std::vector<BoardControllerNodeProcess> processes;
    icarus_rover_v2::device ros_device;
    ros_device.DeviceName = ros_DeviceName;
    ros_device.DeviceParent = ros_ParentDevice;
    ros_device.DeviceType = ros_DeviceType;
    ros_device.BoardCount = 3;


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
    Logger *logger;
    std::string log_output = Node_Name + boost::lexical_cast<std::string>(1);
    logger = new Logger("DEBUG","UNIT_TESTS",log_output);

    icarus_rover_v2::device board1;
    board1.DeviceName = "ArduinoBoard1";
    board1.DeviceType = "ArduinoBoard";
    board1.ID = 1;
    board1.Architecture = "ArduinoUno";
    board1.ShieldCount = 3;
    board1.SensorCount = 0;
    BoardControllerNodeProcess boardprocess1("/dev/dummy1",board1.ID);
    EXPECT_EQ(boardprocess1.get_armedstate(),ARMEDSTATUS_DISARMED_CANNOTARM);
    EXPECT_EQ(boardprocess1.get_ready_to_arm(),false);
    processes.push_back(boardprocess1);
    diagnostic = processes.at(processindex).init(diagnostic,Host_Name,1);
    EXPECT_TRUE(processes.at(processindex).get_nodestate() == BOARDMODE_BOOT);
    EXPECT_TRUE(diagnostic.Level <= NOTICE);
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
    {
    	icarus_rover_v2::device shield;
    	shield.DeviceName = "RelayShield1";
    	shield.Device
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

    for(int s = 0; s < board3.ShieldCount;s++)
    {
        /*
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
    
    initialized_processes = processes;
}
*/
/*
TEST(DeviceConfiguration,ConfigureAllShields)
{
	std::vector<std::vector<unsigned char> > tx_buffers;
	bool send;
	std::vector<BoardControllerNodeProcess> processes = initialized_processes;
	icarus_rover_v2::diagnostic diagnostic;
	for(int i = 0; i < processes.size();i++) //No Board Mode Yet
	{
		diagnostic = processes.at(i).update(0.02);
		EXPECT_TRUE(diagnostic.Level <= NOTICE);
		state_ack configure_shield = processes.at(i).get_stateack("Send Configure Shields");
		EXPECT_TRUE(configure_shield.name == "Send Configure Shields");
		EXPECT_FALSE(configure_shield.trigger == true);
	}
	for(int i = 0; i < processes.size();i++)
	{
        EXPECT_EQ(processes.at(i).get_armedstate(),ARMEDSTATUS_DISARMED_CANNOTARM);
        EXPECT_EQ(processes.at(i).get_ready_to_arm(),false);
		processes.at(i).set_boardstate(BOARDMODE_INITIALIZING);
		diagnostic = processes.at(i).update(0.02);
		EXPECT_TRUE(diagnostic.Level <= NOTICE);
		state_ack configure_shield = processes.at(i).get_stateack("Send Configure Shields");

		EXPECT_TRUE(configure_shield.name == "Send Configure Shields");
		EXPECT_TRUE(configure_shield.trigger == true);
		diagnostic = processes.at(i).update(0.02);
		std::vector<icarus_rover_v2::device> shields = processes.at(i).get_myshields();
		for(int j = 0; j < shields.size();j++)
		{
            
			EXPECT_EQ(processes.at(i).get_portlist(shields.at(j).ID).size(),
                      processes.at(i).get_portcount(shields.at(j).ID));
		}

		tx_buffers.clear();
		send = processes.at(i).checkTriggers(tx_buffers);
		EXPECT_TRUE(send);
		int shield_configuration_messages = 0;
		if(send == true)
		{
			for(int j = 0; j < tx_buffers.size();j++)
			{
				std::vector<unsigned char> tempstr = tx_buffers.at(j);
				if((tempstr.at(0) == 0xAB) &&
				   (tempstr.at(1) == SERIAL_Configure_Shield_ID))
				{
                    
					shield_configuration_messages++;
					printf("Sending from Board: %s:%d Shield Config (0xAB%0x) \n",
							processes.at(i).get_boardname().c_str(),
							processes.at(i).get_boardid(),
							SERIAL_Configure_Shield_ID);
                    
					for(int k = 0; k < tempstr.size();k++)
					{
						printf(" %0x ",tempstr.at(k));
					}
					printf("\n");
                    
				}
			}
		}
		EXPECT_EQ(shield_configuration_messages,processes.at(i).get_myshields().size());
		EXPECT_TRUE(diagnostic.Level <= NOTICE);
		processes.at(i).set_boardstate(BOARDMODE_SHIELDS_CONFIGURED);
		diagnostic = processes.at(i).update(0.2);
		EXPECT_TRUE(diagnostic.Level <= NOTICE);
		EXPECT_TRUE(processes.at(i).get_nodestate() == BOARDMODE_SHIELDS_CONFIGURED);
        EXPECT_EQ(processes.at(i).get_armedstate(),ARMEDSTATUS_DISARMED_CANNOTARM);
        EXPECT_EQ(processes.at(i).get_ready_to_arm(),false);

		state_ack send_dio_configure = processes.at(i).get_stateack("Send Configure DIO Ports");
		EXPECT_TRUE(send_dio_configure.name == "Send Configure DIO Ports");
		EXPECT_TRUE(send_dio_configure.trigger == true);

		tx_buffers.clear();
		send = processes.at(i).checkTriggers(tx_buffers);
		EXPECT_TRUE(send);
		int dio_configuration_messages = 0;
		if(send == true)
		{
			for(int j = 0; j < tx_buffers.size();j++)
			{
				std::vector<unsigned char> tempstr = tx_buffers.at(j);
				if((tempstr.at(0) == 0xAB) &&
						(tempstr.at(1) == SERIAL_Configure_DIO_Port_ID))
				{
					dio_configuration_messages++;
					/*printf("Sending from Board: %s:%d DIO Config (0xAB%0x): ",
							processes.at(i).get_boardname().c_str(),
							processes.at(i).get_boardid(),
							SERIAL_Configure_DIO_Port_ID);
                    
					/*for(int k = 0; k < tempstr.size();k++)
					{
						printf(" %0x ",tempstr.at(k));
					}
					printf("\n");
                    
				}
			}
		}
        int portcount = 0;
        std::vector<icarus_rover_v2::device> myshields = processes.at(i).get_myshields();
        for(int j = 0; j < myshields.size();j++)
        {
            portcount += processes.at(i).get_portcount(myshields.at(j).ID);
        }
		EXPECT_EQ(dio_configuration_messages,portcount);
		EXPECT_TRUE(diagnostic.Level <= NOTICE);
		processes.at(i).set_boardstate(BOARDMODE_INITIALIZED);
		diagnostic = processes.at(i).update(0.2);
		EXPECT_TRUE(diagnostic.Level <= NOTICE);
		EXPECT_TRUE(processes.at(i).get_nodestate() == BOARDMODE_INITIALIZED);
        EXPECT_EQ(processes.at(i).get_armedstate(),ARMEDSTATUS_DISARMED_CANNOTARM);
        EXPECT_EQ(processes.at(i).get_ready_to_arm(),false);
        state_ack send_defaultvalue_DIO_Port = processes.at(i).get_stateack("Send DefaultValue DIO Port");
		EXPECT_TRUE(send_defaultvalue_DIO_Port.name == "Send DefaultValue DIO Port");
		EXPECT_TRUE(send_defaultvalue_DIO_Port.trigger == true);
        tx_buffers.clear();
		send = processes.at(i).checkTriggers(tx_buffers);
		EXPECT_TRUE(send);
		int dio_defaultvalue_messages = 0;
		if(send == true)
		{
			for(int j = 0; j < tx_buffers.size();j++)
			{
				std::vector<unsigned char> tempstr = tx_buffers.at(j);
				if((tempstr.at(0) == 0xAB) &&
						(tempstr.at(1) == SERIAL_Set_DIO_Port_DefaultValue_ID))
				{
                    
					dio_defaultvalue_messages++;
				}
			}
		}
        EXPECT_EQ(dio_defaultvalue_messages,portcount); //# Shields w/ 3 Ports Each
                
		processes.at(i).set_boardstate(BOARDMODE_RUNNING);
		diagnostic = processes.at(i).update(0.2);
		EXPECT_TRUE(diagnostic.Level <= NOTICE);
		EXPECT_TRUE(processes.at(i).get_nodestate() == BOARDMODE_RUNNING);
        EXPECT_EQ(processes.at(i).get_armedstate(),ARMEDSTATUS_DISARMED);
        EXPECT_EQ(processes.at(i).get_ready_to_arm(),true);


	}

	configured_processes = processes;
}

TEST(Timing,Test1)
{
    std::vector<std::vector<unsigned char> > tx_buffers;
	std::vector<BoardControllerNodeProcess> processes = configured_processes;
	icarus_rover_v2::diagnostic diagnostic;
    for(int i = 0; i < processes.size(); i++)
    {
        EXPECT_TRUE(processes.at(i).get_nodestate() == BOARDMODE_RUNNING);
        EXPECT_EQ(processes.at(i).get_armedstate(),ARMEDSTATUS_DISARMED);
        EXPECT_EQ(processes.at(i).get_ready_to_arm(),true);
        std_msgs::Bool pps;
        diagnostic = processes.at(i).update(0.2);
		EXPECT_TRUE(diagnostic.Level <= NOTICE);
        EXPECT_TRUE(processes.at(i).get_nodestate() == BOARDMODE_RUNNING);
        EXPECT_EQ(processes.at(i).get_armedstate(),ARMEDSTATUS_DISARMED);
    }
}
TEST(Arming,ArmDisarm)
{
    std::vector<std::vector<unsigned char> > tx_buffers;
	bool send;
	std::vector<BoardControllerNodeProcess> processes = configured_processes;
	//print_processinfo(processes);
	icarus_rover_v2::diagnostic diagnostic;
    for(int i = 0; i < processes.size(); i++)
    {
        EXPECT_TRUE(processes.at(i).get_nodestate() == BOARDMODE_RUNNING);
        EXPECT_EQ(processes.at(i).get_armedstate(),ARMEDSTATUS_DISARMED);
        EXPECT_EQ(processes.at(i).get_ready_to_arm(),true);
        icarus_rover_v2::command newcommand;
        newcommand.Command = ARM_COMMAND_ID;
        newcommand.Option1 = ROVERCOMMAND_DISARM;

        diagnostic = processes.at(i).new_commandmsg(newcommand);
        diagnostic = processes.at(i).update(0.2);
		EXPECT_TRUE(diagnostic.Level <= NOTICE);
        EXPECT_TRUE(processes.at(i).get_nodestate() == BOARDMODE_RUNNING);
        EXPECT_EQ(processes.at(i).get_armedstate(),ARMEDSTATUS_DISARMED);
        EXPECT_EQ(processes.at(i).get_ready_to_arm(),true);
        
        state_ack send_armed_command = processes.at(i).get_stateack("Send Arm Command");
		EXPECT_TRUE(send_armed_command.name == "Send Arm Command");
		EXPECT_TRUE(send_armed_command.trigger == true);

        tx_buffers.clear();
		send = processes.at(i).checkTriggers(tx_buffers);
		EXPECT_TRUE(send);
		int send_armcommand = 0;
		if(send == true)
		{
			for(int j = 0; j < tx_buffers.size();j++)
			{
				std::vector<unsigned char> tempstr = tx_buffers.at(j);
				if((tempstr.at(0) == 0xAB) &&
						(tempstr.at(1) == SERIAL_Arm_Command_ID))
				{
                    
					send_armcommand++;
				}
			}
		}
		printf("Arm count: %d\n",send_armcommand);
        //EXPECT_EQ(dio_defaultvalue_messages,9); //# Shields w/ 3 Ports Each
        

    }
}
TEST(DeviceUsage,Diagnostics)
{
    
}
TEST(DeviceUsage,PWMOutput)
{
	std::vector<std::vector<unsigned char> > tx_buffers;
	bool send;
	std::vector<BoardControllerNodeProcess> processes = configured_processes;
	icarus_rover_v2::diagnostic diagnostic;
    for(int i = 0; i < processes.size(); i++)
    {
        //processes.at(i).new_commandmsg(ARM_COMMAND_ID,)
        for(int p = 0; p < 9; p++)
        {
            icarus_rover_v2::pin pinmsg;
            pinmsg.BoardID = 1;
            pinmsg.ShieldID = 1;
            pinmsg.Number = p;
            pinmsg.Function = "PWMOutput";
            pinmsg.Value = 200;
            if(processes.at(i).get_boardid() == pinmsg.BoardID)
            {
                bool found = true;
                diagnostic = processes.at(i).new_pinmsg(pinmsg);
                if(p % 2 == 0)
                {
                    EXPECT_TRUE(diagnostic.Level <= NOTICE);
                    diagnostic = processes.at(i).update(0.2);
                    EXPECT_TRUE(diagnostic.Level <= NOTICE);
                    tx_buffers.clear();
                    send = processes.at(i).checkTriggers(tx_buffers);
                    bool sending_set_dio_message = false;
                    EXPECT_TRUE(send);
                }
                else
                {
                    EXPECT_FALSE(diagnostic.Level <= NOTICE);
                }
            }
        }
        //printf("Board ID: %d ")
    }
}
*/
/*
TEST(DeviceConfiguration,CheckReboot)
{
	std::vector<std::vector<unsigned char> > tx_buffers;
	bool send;
	std::vector<BoardControllerNodeProcess> processes = configured_processes;
	icarus_rover_v2::diagnostic diagnostic;
	for(int i = 0; i < processes.size();i++)
	{
		processes.at(i).set_boardstate(BOARDMODE_BOOT);
		diagnostic = processes.at(i).update(0.2);
		EXPECT_TRUE(diagnostic.Level <= NOTICE);
		EXPECT_EQ(processes.at(i).get_nodestate(),BOARDMODE_INITIALIZING);
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
