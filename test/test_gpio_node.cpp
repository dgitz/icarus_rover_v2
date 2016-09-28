#include "ros/ros.h"
#include "ros/time.h"
#include <gtest/gtest.h>
#include <tinyxml.h>
#include "icarus_rover_v2/device.h"
#include "icarus_rover_v2/diagnostic.h"
#include "icarus_rover_v2/pin.h"
#include "logger.h"
#include "time.h"
#include "/home/robot/catkin_ws/src/icarus_rover_v2/src/Board/gpio_node_process.h"

icarus_rover_v2::device myDevice;
std::vector<icarus_rover_v2::device> myBoards;
std::vector<icarus_rover_v2::device> otherDevices;
icarus_rover_v2::diagnostic diagnostic_status;
Logger *logger;
GPIONodeProcess *process;
char hostname[1024];

bool parse_devicefile(TiXmlDocument doc);
bool setup();
void print_mydevice();
void print_otherdevices();
int mode_states[] = {	GPIO_MODE_UNDEFINED,
						GPIO_MODE_BOOT,
						GPIO_MODE_INITIALIZING,
						GPIO_MODE_INITIALIZED,
						GPIO_MODE_RUNNING,
						GPIO_MODE_STOPPED};
int mode_state_count = 6;

TEST(TestSuite,FSM_Boot)
{
	EXPECT_TRUE(setup()==true);
	EXPECT_TRUE(process->get_boardstate() == GPIO_MODE_UNDEFINED);
	EXPECT_TRUE(process->get_nodestate() == GPIO_MODE_BOOT);
	process->new_devicemsg(myDevice);
	EXPECT_TRUE(myDevice.DeviceName == process->get_mydevice().DeviceName);
	EXPECT_TRUE(process->is_finished_initializing() == false);
	for(int i = 0; i < otherDevices.size();i++)
	{
		//cout << "i: " << i << " Device: " << otherDevices.at(i).DeviceName << endl;
		process->new_devicemsg(otherDevices.at(i));
	}
	EXPECT_TRUE(process->is_finished_initializing() == true);
	EXPECT_TRUE(process->get_boardstate() == GPIO_MODE_UNDEFINED);
	EXPECT_TRUE(process->get_nodestate() == GPIO_MODE_INITIALIZED);
	EXPECT_TRUE(process->get_timeout_ms() == 1000);
	std::vector<std::string> tx_buffers;
	process->checkTriggers(tx_buffers);
	EXPECT_TRUE(tx_buffers.empty() == true); //Should not be sending anything yet
	for(int i = 0; i < 19; i++)
	{
		diagnostic_status = process->update(50);
		EXPECT_TRUE((diagnostic_status.Level == NOERROR) || (diagnostic_status.Level == INFO));
	}
	diagnostic_status = process->update(50);
	EXPECT_TRUE((diagnostic_status.Level == NOERROR) || (diagnostic_status.Level == INFO));
	myBoards = process->get_myboards();
	//print_mydevice();
}
TEST(TestSuite,FSM_Board_Reset)
{
	EXPECT_TRUE(setup()==true);
	EXPECT_TRUE(process->get_boardstate() == GPIO_MODE_UNDEFINED);
	EXPECT_TRUE(process->get_nodestate() == GPIO_MODE_BOOT);
	process->new_devicemsg(myDevice);
	EXPECT_TRUE(myDevice.DeviceName == process->get_mydevice().DeviceName);
	EXPECT_TRUE(process->is_finished_initializing() == false);
	for(int i = 0; i < otherDevices.size();i++)
	{
		//cout << "i: " << i << " Device: " << otherDevices.at(i).DeviceName << endl;
		process->new_devicemsg(otherDevices.at(i));
	}
	EXPECT_TRUE(process->is_finished_initializing() == true);
	EXPECT_TRUE(process->get_boardstate() == GPIO_MODE_UNDEFINED);
	EXPECT_TRUE(process->get_nodestate() == GPIO_MODE_INITIALIZED);
	EXPECT_TRUE(process->get_timeout_ms() == 1000);
	std::vector<std::string> tx_buffers;
	process->checkTriggers(tx_buffers);
	EXPECT_TRUE(tx_buffers.empty() == true); //Should not be sending anything yet
	for(int i = 0; i < 19; i++)
	{
		diagnostic_status = process->update(50);
		EXPECT_TRUE((diagnostic_status.Level == NOERROR) || (diagnostic_status.Level == INFO));
	}
	diagnostic_status = process->update(50);
	EXPECT_TRUE((diagnostic_status.Level == NOERROR) || (diagnostic_status.Level == INFO));
	myBoards = process->get_myboards();
	//print_mydevice();
	{//Test Recv GPIO Board Mode: BOOT
		unsigned char rx_packet[8];
		rx_packet[0] = GPIO_MODE_BOOT;
		process->new_serialmessage_Get_Mode(SERIAL_Mode_ID,rx_packet);
		EXPECT_TRUE(process->get_nodestate() == GPIO_MODE_INITIALIZED);
		diagnostic_status = process->update(50);
		EXPECT_TRUE((diagnostic_status.Level == NOERROR) || (diagnostic_status.Level == INFO));
		tx_buffers.clear();
		process->checkTriggers(tx_buffers);
		bool is_sending_node_mode = false;
		for(int i = 0; i < tx_buffers.size();i++)
		{
			std::string buffer = tx_buffers.at(i);
			if(buffer.at(1) == SERIAL_Mode_ID){ is_sending_node_mode = true; }
		}
		EXPECT_TRUE(is_sending_node_mode == true);
	}
	{//Test Recv GPIO Board Mode: INITIALIZING
		unsigned char rx_packet[8];
		rx_packet[0] = GPIO_MODE_INITIALIZING;
		process->new_serialmessage_Get_Mode(SERIAL_Mode_ID,rx_packet);
		EXPECT_TRUE(process->get_boardstate() == GPIO_MODE_INITIALIZING);


		EXPECT_TRUE(process->get_nodestate() == GPIO_MODE_INITIALIZED);
		EXPECT_TRUE(process->get_boardstate() == GPIO_MODE_INITIALIZING);
		diagnostic_status = process->update(50);
		EXPECT_TRUE((diagnostic_status.Level == NOERROR) || (diagnostic_status.Level == INFO));
		process->checkTriggers(tx_buffers);
		EXPECT_TRUE(tx_buffers.empty() == false); //Should be sending configure Port info
		bool is_sending_Config_DIO_PortA = false;
		bool is_sending_Config_DIO_PortB = false;
		for(int i = 0; i < tx_buffers.size();i++)
		{
			std::string buffer = tx_buffers.at(i);
			if(buffer.at(1) == SERIAL_Configure_DIO_PortA_ID){ is_sending_Config_DIO_PortA = true; }
			if(buffer.at(1) == SERIAL_Configure_DIO_PortB_ID){ is_sending_Config_DIO_PortB = true; }
		}
		EXPECT_TRUE(is_sending_Config_DIO_PortA == true);
		EXPECT_TRUE(is_sending_Config_DIO_PortB == true);
	}
}
TEST(TestSuite,FSM_Configure)
{
	EXPECT_TRUE(setup()==true);
	EXPECT_TRUE(process->get_boardstate() == GPIO_MODE_UNDEFINED);
	EXPECT_TRUE(process->get_nodestate() == GPIO_MODE_BOOT);
	process->new_devicemsg(myDevice);
	EXPECT_TRUE(myDevice.DeviceName == process->get_mydevice().DeviceName);
	EXPECT_TRUE(process->is_finished_initializing() == false);
	for(int i = 0; i < otherDevices.size();i++)
	{
		process->new_devicemsg(otherDevices.at(i));
	}
	EXPECT_TRUE(process->is_finished_initializing() == true);
	EXPECT_TRUE(process->get_boardstate() == GPIO_MODE_UNDEFINED);
	EXPECT_TRUE(process->get_nodestate() == GPIO_MODE_INITIALIZED);
	EXPECT_TRUE(process->get_timeout_ms() == 1000);
	std::vector<std::string> tx_buffers;

	process->checkTriggers(tx_buffers);

	tx_buffers.clear();
	for(int i = 0; i < 19; i++)
	{
		diagnostic_status = process->update(50);
		EXPECT_TRUE((diagnostic_status.Level == NOERROR) || (diagnostic_status.Level == INFO));
		if(i == 10)
		{
			unsigned char rx_packet[8];
			rx_packet[0] = GPIO_MODE_BOOT;
			process->new_serialmessage_Get_Mode(SERIAL_Mode_ID,rx_packet);
			EXPECT_TRUE(process->get_boardstate() == GPIO_MODE_BOOT);
		}
		if(i == 12)
		{
			unsigned char rx_packet[8];
			rx_packet[0] = GPIO_MODE_INITIALIZING;
			process->new_serialmessage_Get_Mode(SERIAL_Mode_ID,rx_packet);
			EXPECT_TRUE(process->get_boardstate() == GPIO_MODE_INITIALIZING);
		}
	}

	EXPECT_TRUE(process->get_nodestate() == GPIO_MODE_INITIALIZED);
	EXPECT_TRUE(process->get_boardstate() == GPIO_MODE_INITIALIZING);
	diagnostic_status = process->update(50);
	EXPECT_TRUE((diagnostic_status.Level == NOERROR) || (diagnostic_status.Level == INFO));
	process->checkTriggers(tx_buffers);
	EXPECT_TRUE(tx_buffers.empty() == false); //Should be sending configure Port info
	bool is_sending_Config_DIO_PortA = false;
	bool is_sending_Config_DIO_PortB = false;
	for(int i = 0; i < tx_buffers.size();i++)
	{
		std::string buffer = tx_buffers.at(i);
		if(buffer.at(1) == SERIAL_Configure_DIO_PortA_ID){ is_sending_Config_DIO_PortA = true; }
		if(buffer.at(1) == SERIAL_Configure_DIO_PortB_ID){ is_sending_Config_DIO_PortB = true; }
	}
	EXPECT_TRUE(is_sending_Config_DIO_PortA == true);
	EXPECT_TRUE(is_sending_Config_DIO_PortB == true);
}

TEST(TestSuite,FSM_Running)
{
	EXPECT_TRUE(setup()==true);
	EXPECT_TRUE(process->get_boardstate() == GPIO_MODE_UNDEFINED);
	EXPECT_TRUE(process->get_nodestate() == GPIO_MODE_BOOT);
	process->new_devicemsg(myDevice);
	EXPECT_TRUE(myDevice.DeviceName == process->get_mydevice().DeviceName);
	EXPECT_TRUE(process->is_finished_initializing() == false);
	for(int i = 0; i < otherDevices.size();i++)
	{
		process->new_devicemsg(otherDevices.at(i));
	}
	EXPECT_TRUE(process->is_finished_initializing() == true);
	EXPECT_TRUE(process->get_boardstate() == GPIO_MODE_UNDEFINED);
	EXPECT_TRUE(process->get_nodestate() == GPIO_MODE_INITIALIZED);
	EXPECT_TRUE(process->get_timeout_ms() == 1000);
	std::vector<std::string> tx_buffers;

	process->checkTriggers(tx_buffers);

	tx_buffers.clear();
	for(int i = 0; i < 19; i++)
	{
		diagnostic_status = process->update(50);
		EXPECT_TRUE((diagnostic_status.Level == NOERROR) || (diagnostic_status.Level == INFO));
		if(i == 10)
		{
			unsigned char rx_packet[8];
			rx_packet[0] = GPIO_MODE_BOOT;
			process->new_serialmessage_Get_Mode(SERIAL_Mode_ID,rx_packet);
			EXPECT_TRUE(process->get_boardstate() == GPIO_MODE_BOOT);
		}
		if(i == 12)
		{
			unsigned char rx_packet[8];
			rx_packet[0] = GPIO_MODE_INITIALIZING;
			process->new_serialmessage_Get_Mode(SERIAL_Mode_ID,rx_packet);
			EXPECT_TRUE(process->get_boardstate() == GPIO_MODE_INITIALIZING);
		}
	}

	EXPECT_TRUE(process->get_nodestate() == GPIO_MODE_INITIALIZED);
	EXPECT_TRUE(process->get_boardstate() == GPIO_MODE_INITIALIZING);
	diagnostic_status = process->update(50);
	EXPECT_TRUE((diagnostic_status.Level == NOERROR) || (diagnostic_status.Level == INFO));
	process->checkTriggers(tx_buffers);
	EXPECT_TRUE(tx_buffers.empty() == false); //Should be sending configure Port info
	bool is_sending_Config_DIO_PortA = false;
	bool is_sending_Config_DIO_PortB = false;
	for(int i = 0; i < tx_buffers.size();i++)
	{
		std::string buffer = tx_buffers.at(i);
		if(buffer.at(1) == SERIAL_Configure_DIO_PortA_ID){ is_sending_Config_DIO_PortA = true; }
		if(buffer.at(1) == SERIAL_Configure_DIO_PortB_ID){ is_sending_Config_DIO_PortB = true; }
	}
	EXPECT_TRUE(is_sending_Config_DIO_PortA == true);
	EXPECT_TRUE(is_sending_Config_DIO_PortB == true);
	unsigned char rx_packet[8];
	rx_packet[0] = GPIO_MODE_RUNNING;
	process->new_serialmessage_Get_Mode(SERIAL_Mode_ID,rx_packet);
	EXPECT_TRUE(process->get_boardstate() == GPIO_MODE_RUNNING);
	diagnostic_status = process->update(50);
	EXPECT_TRUE((diagnostic_status.Level == NOERROR) || (diagnostic_status.Level == INFO));
	EXPECT_TRUE(process->get_nodestate() == GPIO_MODE_RUNNING);

}
TEST(TestSuite,TestMessage)
{
	EXPECT_TRUE(setup()==true);
	EXPECT_TRUE(process->get_boardstate() == GPIO_MODE_UNDEFINED);
	EXPECT_TRUE(process->get_nodestate() == GPIO_MODE_BOOT);
	process->new_devicemsg(myDevice);
	EXPECT_TRUE(myDevice.DeviceName == process->get_mydevice().DeviceName);
	EXPECT_TRUE(process->is_finished_initializing() == false);
	for(int i = 0; i < otherDevices.size();i++)
	{
		process->new_devicemsg(otherDevices.at(i));
	}
	EXPECT_TRUE(process->is_finished_initializing() == true);
	EXPECT_TRUE(process->get_boardstate() == GPIO_MODE_UNDEFINED);
	EXPECT_TRUE(process->get_nodestate() == GPIO_MODE_INITIALIZED);
	EXPECT_TRUE(process->get_timeout_ms() == 1000);
	std::vector<std::string> tx_buffers;

	process->checkTriggers(tx_buffers);

	tx_buffers.clear();
	for(int i = 0; i < 19; i++)
	{
		diagnostic_status = process->update(50);
		EXPECT_TRUE((diagnostic_status.Level == NOERROR) || (diagnostic_status.Level == INFO));
		if(i == 10)
		{
			unsigned char rx_packet[8];
			rx_packet[0] = GPIO_MODE_BOOT;
			process->new_serialmessage_Get_Mode(SERIAL_Mode_ID,rx_packet);
			EXPECT_TRUE(process->get_boardstate() == GPIO_MODE_BOOT);
		}
		if(i == 12)
		{
			unsigned char rx_packet[8];
			rx_packet[0] = GPIO_MODE_INITIALIZING;
			process->new_serialmessage_Get_Mode(SERIAL_Mode_ID,rx_packet);
			EXPECT_TRUE(process->get_boardstate() == GPIO_MODE_INITIALIZING);
		}
	}

	EXPECT_TRUE(process->get_nodestate() == GPIO_MODE_INITIALIZED);
	EXPECT_TRUE(process->get_boardstate() == GPIO_MODE_INITIALIZING);
	{
		bool is_sending_testmessage_command = false;
		process->checkTriggers(tx_buffers);
		for(int i = 0; i < tx_buffers.size();i++)
		{
			std::string buffer = tx_buffers.at(i);
			if(buffer.at(1) == SERIAL_TestMessageCommand_ID){ is_sending_testmessage_command = true; }
		}
		EXPECT_TRUE(is_sending_testmessage_command == false);
	}
	diagnostic_status = process->update(50);
	EXPECT_TRUE((diagnostic_status.Level == NOERROR) || (diagnostic_status.Level == INFO));
	process->checkTriggers(tx_buffers);
	EXPECT_TRUE(tx_buffers.empty() == false); //Should be sending configure Port info
	bool is_sending_Config_DIO_PortA = false;
	bool is_sending_Config_DIO_PortB = false;
	for(int i = 0; i < tx_buffers.size();i++)
	{
		std::string buffer = tx_buffers.at(i);
		if(buffer.at(1) == SERIAL_Configure_DIO_PortA_ID){ is_sending_Config_DIO_PortA = true; }
		if(buffer.at(1) == SERIAL_Configure_DIO_PortB_ID){ is_sending_Config_DIO_PortB = true; }
	}
	EXPECT_TRUE(is_sending_Config_DIO_PortA == true);
	EXPECT_TRUE(is_sending_Config_DIO_PortB == true);
	unsigned char rx_packet[8];
	rx_packet[0] = GPIO_MODE_RUNNING;
	process->new_serialmessage_Get_Mode(SERIAL_Mode_ID,rx_packet);
	EXPECT_TRUE(process->get_boardstate() == GPIO_MODE_RUNNING);
	diagnostic_status = process->update(50);
	EXPECT_TRUE((diagnostic_status.Level == NOERROR) || (diagnostic_status.Level == INFO));
	EXPECT_TRUE(process->get_nodestate() == GPIO_MODE_RUNNING);

	icarus_rover_v2::command command;
	command.Command = DIAGNOSTIC_ID;
	command.Option1 = LEVEL2;
	process->new_commandmsg(command);
	bool is_sending_testmessage_command = false;
	process->checkTriggers(tx_buffers);
	for(int i = 0; i < tx_buffers.size();i++)
	{
		std::string buffer = tx_buffers.at(i);
		if(buffer.at(1) == SERIAL_TestMessageCommand_ID){ is_sending_testmessage_command = true; }
	}
	EXPECT_TRUE(is_sending_testmessage_command == true);

}
TEST(TestSuite,FSM_ReadSerial)
{
	EXPECT_TRUE(setup()==true);
	EXPECT_TRUE(process->get_boardstate() == GPIO_MODE_UNDEFINED);
	EXPECT_TRUE(process->get_nodestate() == GPIO_MODE_BOOT);
	process->new_devicemsg(myDevice);
	EXPECT_TRUE(myDevice.DeviceName == process->get_mydevice().DeviceName);
	EXPECT_TRUE(process->is_finished_initializing() == false);
	for(int i = 0; i < otherDevices.size();i++)
	{
		process->new_devicemsg(otherDevices.at(i));
	}
	EXPECT_TRUE(process->is_finished_initializing() == true);
	EXPECT_TRUE(process->get_boardstate() == GPIO_MODE_UNDEFINED);
	EXPECT_TRUE(process->get_nodestate() == GPIO_MODE_INITIALIZED);
	EXPECT_TRUE(process->get_timeout_ms() == 1000);
	std::vector<std::string> tx_buffers;

	process->checkTriggers(tx_buffers);

	tx_buffers.clear();
	for(int i = 0; i < 19; i++)
	{
		diagnostic_status = process->update(50);
		EXPECT_TRUE((diagnostic_status.Level == NOERROR) || (diagnostic_status.Level == INFO));
		if(i == 10)
		{
			unsigned char rx_packet[8];
			rx_packet[0] = GPIO_MODE_BOOT;
			process->new_serialmessage_Get_Mode(SERIAL_Mode_ID,rx_packet);
			EXPECT_TRUE(process->get_boardstate() == GPIO_MODE_BOOT);
		}
		if(i == 12)
		{
			unsigned char rx_packet[8];
			rx_packet[0] = GPIO_MODE_INITIALIZING;
			process->new_serialmessage_Get_Mode(SERIAL_Mode_ID,rx_packet);
			EXPECT_TRUE(process->get_boardstate() == GPIO_MODE_INITIALIZING);
		}
	}

	EXPECT_TRUE(process->get_nodestate() == GPIO_MODE_INITIALIZED);
	EXPECT_TRUE(process->get_boardstate() == GPIO_MODE_INITIALIZING);
	diagnostic_status = process->update(50);
	EXPECT_TRUE((diagnostic_status.Level == NOERROR) || (diagnostic_status.Level == INFO));
	process->checkTriggers(tx_buffers);
	EXPECT_TRUE(tx_buffers.empty() == false); //Should be sending configure Port info
	bool is_sending_Config_DIO_PortA = false;
	bool is_sending_Config_DIO_PortB = false;
	for(int i = 0; i < tx_buffers.size();i++)
	{
		std::string buffer = tx_buffers.at(i);
		if(buffer.at(1) == SERIAL_Configure_DIO_PortA_ID){ is_sending_Config_DIO_PortA = true; }
		if(buffer.at(1) == SERIAL_Configure_DIO_PortB_ID){ is_sending_Config_DIO_PortB = true; }
	}
	EXPECT_TRUE(is_sending_Config_DIO_PortA == true);
	EXPECT_TRUE(is_sending_Config_DIO_PortB == true);
	unsigned char rx_packet[8];
	rx_packet[0] = GPIO_MODE_RUNNING;
	process->new_serialmessage_Get_Mode(SERIAL_Mode_ID,rx_packet);
	EXPECT_TRUE(process->get_boardstate() == GPIO_MODE_RUNNING);
	diagnostic_status = process->update(50);
	EXPECT_TRUE((diagnostic_status.Level == NOERROR) || (diagnostic_status.Level == INFO));
	EXPECT_TRUE(process->get_nodestate() == GPIO_MODE_RUNNING);

	//Check the following:
	/*
	int decode_DiagnosticSerial
	int decode_ModeSerial
	int decode_Get_ANA_PortASerial
	int decode_Get_ANA_PortBSerial
	int decode_Get_DIO_PortASerial
	int decode_Get_DIO_PortBSerial
	 */
	/*
	{//DiagnosticSerial
		unsigned char rx_packet[8];
		diagnostic_status = process->new_serialmessage_Diagnostic(SERIAL_Diagnostic_ID,rx_packet);
		EXPECT_TRUE((diagnostic_status.Level == NOERROR) || (diagnostic_status.Level == INFO));
	}

	{//decode_ModeSerial
		unsigned char rx_packet[8];
		diagnostic_status = process->new_serialmessage_Get_Mode(SERIAL_Mode_ID,rx_packet);
		EXPECT_TRUE((diagnostic_status.Level == NOERROR) || (diagnostic_status.Level == INFO));
	}

	{//decode_Get_ANA_PortASerial
		unsigned char rx_packet[8];
		diagnostic_status = process->new_serialmessage_Get_ANA_PortA(SERIAL_Get_ANA_PortA_ID,rx_packet);
		EXPECT_TRUE((diagnostic_status.Level == NOERROR) || (diagnostic_status.Level == INFO));
	}
	{//decode_Get_ANA_PortBSerial
		unsigned char rx_packet[8];
		diagnostic_status = process->new_serialmessage_Get_ANA_PortB(SERIAL_Get_ANA_PortB_ID,rx_packet);
		EXPECT_TRUE((diagnostic_status.Level == NOERROR) || (diagnostic_status.Level == INFO));
	}
	{//decode_Get_DIO_PortASerial
		unsigned char rx_packet[8];
		diagnostic_status = process->new_serialmessage_Get_DIO_PortA(SERIAL_Get_DIO_PortA_ID,rx_packet);
		EXPECT_TRUE((diagnostic_status.Level == NOERROR) || (diagnostic_status.Level == INFO));
	}
	{//decode_Get_DIO_PortBSerial
		unsigned char rx_packet[8];
		diagnostic_status = process->new_serialmessage_Get_DIO_PortB(SERIAL_Get_DIO_PortB_ID,rx_packet);
		EXPECT_TRUE((diagnostic_status.Level == NOERROR) || (diagnostic_status.Level == INFO));
	}
	{//decode_TestMessageCounterSerial
		unsigned char rx_packet[8];
		diagnostic_status = process->new_serialmessage_TestMessageCounter(SERIAL_TestMessageCounter_ID,rx_packet);
		EXPECT_TRUE((diagnostic_status.Level == NOERROR) || (diagnostic_status.Level == INFO));
	}
	*/
}
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
void print_mydevice()
{
	cout << "\n-----------------------------\nMY DEVICE:\n-----------------------------" << endl;
	cout << "Parent Device: " << myDevice.DeviceParent << endl;
	cout << "Device Name: " << myDevice.DeviceName << endl;
	cout << "Device Type: " << myDevice.DeviceType << endl;
	cout << "Architecture: " << myDevice.Architecture << endl;
	cout << "---MY BOARDS---" << endl;
	for(int i = 0; i < myBoards.size();i++)
	{
		Port_Info DIO_PortA = process->get_PortInfo(myBoards.at(i).DeviceName,"DIO_PortA");
		Port_Info DIO_PortB = process->get_PortInfo(myBoards.at(i).DeviceName,"DIO_PortB");
		Port_Info ANA_PortA = process->get_PortInfo(myBoards.at(i).DeviceName,"ANA_PortA");
		Port_Info ANA_PortB = process->get_PortInfo(myBoards.at(i).DeviceName,"ANA_PortB");
		cout << "DIO_PORTA:\n-----------------------------" << endl;
		for(int i = 0; i < 8; i++)
		{
			cout << "Pin " << DIO_PortA.Number[i] << " Mode: " << process->map_PinFunction_ToString(DIO_PortA.Mode[i]) <<
					" Available: " << DIO_PortA.Available[i] << endl;
		}
		cout << "-----------------------------\nDIO_PORTB:\n-----------------------------" << endl;
		for(int i = 0; i < 8; i++)
		{
			cout << "Pin " << DIO_PortB.Number[i] << " Mode: " << process->map_PinFunction_ToString(DIO_PortB.Mode[i]) <<
					" Available: " << DIO_PortB.Available[i] << endl;
		}
		cout << "-----------------------------\nANA_PORTA:\n-----------------------------" << endl;
		for(int i = 0; i < 4; i++)
		{
			cout << "Pin " << ANA_PortA.Number[i] << " Mode: " << process->map_PinFunction_ToString(ANA_PortA.Mode[i]) <<
					" Available: " << ANA_PortA.Available[i] << endl;
		}
		cout << "-----------------------------\nANA_PORTB:\n-----------------------------" << endl;
		for(int i = 0; i < 4; i++)
		{
			cout << "Pin " << ANA_PortB.Number[i] << " Mode: " << process->map_PinFunction_ToString(ANA_PortB.Mode[i]) <<
					" Available: " << ANA_PortB.Available[i] << endl;
		}
		printf("-----------------------\n");
	}
}
bool setup()
{
	hostname[1023] = '\0';
	gethostname(hostname,1023);
	logger = new Logger("DEBUG","_unittest_gpio_node");

	TiXmlDocument device_doc("/home/robot/config/DeviceFile.xml");
	myDevice.DeviceName = std::string(hostname);
	bool devicefile_loaded = device_doc.LoadFile();
	EXPECT_TRUE(device_doc.LoadFile());
	EXPECT_TRUE(parse_devicefile(device_doc));
	diagnostic_status.Node_Name = "gpio_node";
	diagnostic_status.System = ROVER;
	diagnostic_status.SubSystem = ROBOT_CONTROLLER;
	diagnostic_status.Component = GPIO_NODE;
	diagnostic_status.Diagnostic_Type = NOERROR;
	diagnostic_status.Level = INFO;
	diagnostic_status.Diagnostic_Message = INITIALIZING;



	process = new GPIONodeProcess;
	diagnostic_status = process->init(diagnostic_status,logger,std::string(hostname));
	EXPECT_TRUE(diagnostic_status.Level == INFO);

	return true;
}

bool parse_devicefile(TiXmlDocument doc)
{
	TiXmlElement *l_pRootElement = doc.RootElement();

	if( NULL != l_pRootElement )
	{
	    TiXmlElement *l_pDeviceList = l_pRootElement->FirstChildElement( "DeviceList" );

	    if ( NULL != l_pDeviceList )
	    {
	        TiXmlElement *l_pDevice = l_pDeviceList->FirstChildElement( "Device" );

	        while( l_pDevice )
	        {
	        	icarus_rover_v2::device newDevice;
	        	std::vector<icarus_rover_v2::pin> pins;
	        	TiXmlElement *l_pDeviceParent = l_pDevice->FirstChildElement( "ParentDevice" );
				if ( NULL != l_pDeviceParent )
				{
					newDevice.DeviceParent = l_pDeviceParent->GetText();
				}

	            TiXmlElement *l_pDeviceName = l_pDevice->FirstChildElement( "DeviceName" );
	            if ( NULL != l_pDeviceName )
	            {
	                newDevice.DeviceName = l_pDeviceName->GetText();
	            }

	            TiXmlElement *l_pDeviceType = l_pDevice->FirstChildElement( "DeviceType" );
	            if ( NULL != l_pDeviceType )
	            {
	                newDevice.DeviceType = l_pDeviceType->GetText();
	            }

	            TiXmlElement *l_pDeviceArchitecture = l_pDevice->FirstChildElement( "Architecture" );
				if ( NULL != l_pDeviceArchitecture )
				{
					newDevice.Architecture = l_pDeviceArchitecture->GetText();
				}

				TiXmlElement *l_pBoardCount = l_pDevice->FirstChildElement( "BoardCount" );
				if ( NULL != l_pBoardCount )
				{
					newDevice.BoardCount = atoi(l_pBoardCount->GetText());
				}

				TiXmlElement *l_pPin = l_pDevice->FirstChildElement( "Pin" );
				while( l_pPin )
				{
					icarus_rover_v2::pin newpin;
					TiXmlElement *l_pPinPort = l_pPin->FirstChildElement( "Port" );
					if ( NULL != l_pPinPort )
					{
						newpin.Port = l_pPinPort->GetText();
					}
					TiXmlElement *l_pPinNumber = l_pPin->FirstChildElement( "Number" );
					if ( NULL != l_pPinNumber )
					{
						newpin.Number = atoi(l_pPinNumber->GetText());
					}

					TiXmlElement *l_pPinFunction = l_pPin->FirstChildElement( "Function" );
					if ( NULL != l_pPinFunction )
					{
						newpin.Function = l_pPinFunction->GetText();
					}
					l_pPin = l_pPin->NextSiblingElement( "Pin" );
					pins.push_back(newpin);

				}
				newDevice.pins = pins;
	            if(newDevice.DeviceName == myDevice.DeviceName)
	            {
	            	myDevice = newDevice;
	            }
	            else
	            {
	            	otherDevices.push_back(newDevice);
	            }
	            l_pDevice = l_pDevice->NextSiblingElement( "Device" );
	        }
	    }
	}
	return true;
}
