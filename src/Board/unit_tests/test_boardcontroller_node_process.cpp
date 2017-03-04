#include <gtest/gtest.h>
#include "ros/ros.h"
#include "ros/time.h"
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
TEST(DeviceInitialization,DeviceInitialization_2Boards_3Shields)
{
	int ShieldID = 1;
    std::vector<BoardControllerNodeProcess> processes;
    icarus_rover_v2::device ros_device;
    ros_device.DeviceName = ros_DeviceName;
    ros_device.DeviceParent = ros_ParentDevice;
    ros_device.DeviceType = ros_DeviceType;
    ros_device.BoardCount = 2;


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
    BoardControllerNodeProcess boardprocess1("/dev/dummy1",1);
    EXPECT_EQ(boardprocess1.get_armedstate(),ARMEDSTATUS_DISARMED_CANNOTARM);
    EXPECT_EQ(boardprocess1.get_ready_to_arm(),false);
    processes.push_back(boardprocess1);
    diagnostic = processes.at(0).init(diagnostic,Host_Name,1);
    EXPECT_TRUE(processes.at(0).get_nodestate() == BOARDMODE_BOOT);
    EXPECT_TRUE(diagnostic.Level <= NOTICE);
    diagnostic = processes.at(0).new_devicemsg(ros_device);
    EXPECT_TRUE(diagnostic.Level <= NOTICE);
    diagnostic = processes.at(0).update(0.02);
    EXPECT_TRUE(diagnostic.Level <= NOTICE);
    diagnostic = processes.at(0).new_devicemsg(board1);
    EXPECT_TRUE(diagnostic.Level <= NOTICE);
    diagnostic = processes.at(0).update(0.02);
    EXPECT_TRUE(diagnostic.Level <= NOTICE);
    processes.at(0).set_boardstate(BOARDMODE_BOOT);
    EXPECT_FALSE(processes.at(0).is_finished_initializing());
    EXPECT_TRUE(processes.at(0).get_nodestate() == BOARDMODE_BOOT);
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
    	EXPECT_EQ(processes.at(0).get_nodestate(),BOARDMODE_BOOT);
    	diagnostic = processes.at(0).new_devicemsg(shield);
    	EXPECT_TRUE(diagnostic.Level <= NOTICE);
    	diagnostic = processes.at(0).update(0.02);

    	EXPECT_TRUE(diagnostic.Level <= NOTICE);
    }
    /*
    {
    	icarus_rover_v2::device shield;
    	shield.DeviceName = "RelayShield1";
    	shield.Device
    }
    */
    EXPECT_TRUE(processes.at(0).get_nodestate() == BOARDMODE_INITIALIZING);
    EXPECT_TRUE(check_if_initialized(processes));
    for(int s = 0; s < board1.ShieldCount;s++)
    {
    	EXPECT_EQ(processes.at(0).get_portlist(s+1).size(), 3);
    	std::vector<int> portlist = processes.at(0).get_portlist(s+1);
    	int availablecount = 0;
    	for(int p = 0; p < portlist.size();p++)
    	{
    		Port_Info port = processes.at(0).get_PortInfo(s+1,p+1);
    		EXPECT_TRUE(port.ShieldID > 0);
    		EXPECT_TRUE(port.PortID > 0);

    		for(int i = 0; i < port.Number.size();i++)
    		{
    			if(port.Available.at(i) > 0) {availablecount++;}
    		}
    	}
    	EXPECT_EQ(availablecount,10);
    }

    icarus_rover_v2::device board2;
    board2.DeviceName = "ArduinoBoard2";
    board2.DeviceType = "ArduinoBoard";
    board2.ID = 2;
    board2.Architecture = "ArduinoMega";
    board2.ShieldCount = 3;
    board2.SensorCount = 0;
    BoardControllerNodeProcess boardprocess2("/dev/dummy2",2);
    EXPECT_EQ(boardprocess2.get_armedstate(),ARMEDSTATUS_DISARMED_CANNOTARM);
    EXPECT_EQ(boardprocess2.get_ready_to_arm(),false);
    processes.push_back(boardprocess2);
    diagnostic = processes.at(1).init(diagnostic,Host_Name,1);
    EXPECT_TRUE(processes.at(1).get_nodestate() == BOARDMODE_BOOT);
    EXPECT_TRUE(diagnostic.Level <= NOTICE);
    diagnostic = processes.at(1).new_devicemsg(ros_device);
    EXPECT_TRUE(diagnostic.Level <= NOTICE);
    diagnostic = processes.at(1).update(0.02);
    EXPECT_TRUE(diagnostic.Level <= NOTICE);
    diagnostic = processes.at(1).new_devicemsg(board2);
    EXPECT_TRUE(diagnostic.Level <= NOTICE);
    diagnostic = processes.at(1).update(0.02);
    EXPECT_TRUE(diagnostic.Level <= NOTICE);

    processes.at(1).set_boardstate(BOARDMODE_BOOT);
    EXPECT_FALSE(processes.at(1).is_finished_initializing());
    EXPECT_TRUE(processes.at(1).get_nodestate() == BOARDMODE_BOOT);
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
    	EXPECT_TRUE(processes.at(1).get_nodestate() == BOARDMODE_BOOT);
    	diagnostic = processes.at(1).new_devicemsg(shield);
    	EXPECT_TRUE(diagnostic.Level <= NOTICE);
    	diagnostic = processes.at(1).update(0.02);
    	EXPECT_TRUE(diagnostic.Level <= NOTICE);
    }
    EXPECT_TRUE(processes.at(1).get_nodestate() == BOARDMODE_INITIALIZING);
    EXPECT_TRUE(check_if_initialized(processes));

    for(int s = 0; s < board2.ShieldCount;s++)
    {
    	EXPECT_EQ(processes.at(1).get_portlist(s+1).size(), 3);
    	std::vector<int> portlist = processes.at(1).get_portlist(s+1);
    	int availablecount = 0;
    	for(int p = 0; p < portlist.size();p++)
    	{
    		Port_Info port = processes.at(1).get_PortInfo(s+1,p+1);
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
			EXPECT_EQ(processes.at(i).get_portlist(shields.at(j).ID).size(),3);
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
					/*printf("Sending from Board: %s:%d Shield Config (0xAB%0x): ",
							processes.at(i).get_boardname().c_str(),
							processes.at(i).get_boardid(),
							SERIAL_Configure_Shield_ID);
					for(int k = 0; k < tempstr.size();k++)
					{
						printf(" %0x ",tempstr.at(k));
					}
					printf("\n");
                    */
				}
			}
		}
		EXPECT_EQ(shield_configuration_messages,3);//3 Shields
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
                    */
					/*for(int k = 0; k < tempstr.size();k++)
					{
						printf(" %0x ",tempstr.at(k));
					}
					printf("\n");
                    */
				}
			}
		}

		EXPECT_EQ(dio_configuration_messages,9);//3 Shields w/ 3 Ports each
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
        EXPECT_EQ(dio_defaultvalue_messages,9); //# Shields w/ 3 Ports Each
                
		processes.at(i).set_boardstate(BOARDMODE_RUNNING);
		diagnostic = processes.at(i).update(0.2);
		EXPECT_TRUE(diagnostic.Level <= NOTICE);
		EXPECT_TRUE(processes.at(i).get_nodestate() == BOARDMODE_RUNNING);
        EXPECT_EQ(processes.at(i).get_armedstate(),ARMEDSTATUS_DISARMED);
        EXPECT_EQ(processes.at(i).get_ready_to_arm(),true);


	}

	configured_processes = processes;
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
	}
	return status;
}
void print_processinfo(std::vector<BoardControllerNodeProcess> processes)
{
	for(int i = 0; i < processes.size(); i++)
	{
		printf("Process Board: %s:%d Shield Count: %d\n",processes.at(i).get_boardname().c_str(),processes.at(i).get_boardid(),processes.at(i).get_myshields().size());
		std::vector<icarus_rover_v2::device> shields = processes.at(i).get_myshields();

		for(int s = 0; s < shields.size();s++)
		{
			printf("Shield %s:%d\n",shields.at(s).DeviceName.c_str(),shields.at(s).ID);
			std::vector<int> portlist = processes.at(i).get_portlist(shields.at(s).ID);
			for(int p = 0; p < portlist.size();p++)
			{
				Port_Info port = processes.at(i).get_PortInfo(shields.at(s).ID,portlist.at(p));
				printf("Port %d\n",port.PortID);
				for(int j = 0; j < 8; j++)
				{
					printf("Pin: Number: %d Mode: %s\n",port.Number[j],processes.at(i).map_PinFunction_ToString(port.Mode[j]).c_str());
				}
			}
			//printf("Pin Count: %d\n",shields.at(s))
		}

	}
}
