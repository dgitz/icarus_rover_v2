#include "gpio_node_process.h"

GPIONodeProcess::GPIONodeProcess()
{
	board_state = GPIO_MODE_UNDEFINED;
	node_state = GPIO_MODE_BOOT;
	all_sensor_info_received = false;
	all_board_info_received = false;
	all_device_info_received = false;
	extrapolate_values = false;
	initialize_Ports();
	ms_timer = 0;
	timeout_value_ms = 0;
    init_time = ros::Time::now();
}
GPIONodeProcess::~GPIONodeProcess()
{

}
bool GPIONodeProcess::initialize_Ports()
{
	DIO_PortA.PortName = "DIO_PortA";
	DIO_PortB.PortName = "DIO_PortB";
	for(int i = 0; i < 8; i++)
	{
		DIO_PortA.Number[i] = i+1;
		DIO_PortA.Mode[i] = PINMODE_UNDEFINED;
		DIO_PortA.Available[i] = false;
		DIO_PortA.Value[i] = 0;
		DIO_PortA.ConnectingDevice.push_back("");
		
		DIO_PortB.Number[i] = i-1;
		DIO_PortB.Mode[i] = PINMODE_UNDEFINED;
		DIO_PortB.Available[i] = false;
		DIO_PortB.Value[i] = 0;
		DIO_PortB.ConnectingDevice.push_back("");
	}
	ANA_PortA.PortName = "ANA_PortA";
	ANA_PortB.PortName = "ANA_PortB";
	for(int i = 0; i < 4; i++)
	{
		ANA_PortA.Number[i] = i-1;
		ANA_PortA.Mode[i] = PINMODE_ANALOG_INPUT;
		ANA_PortA.Available[i] = false;
		ANA_PortA.Value[i] = 0;
		ANA_PortA.ConnectingDevice.push_back("");
		
		ANA_PortB.Number[i] = 4+i+1;
		ANA_PortB.Mode[i] = PINMODE_ANALOG_INPUT;
		ANA_PortB.Available[i] = false;
		ANA_PortB.Value[i] = 0;
		ANA_PortB.ConnectingDevice.push_back("");
	}
}
icarus_rover_v2::diagnostic GPIONodeProcess::init(icarus_rover_v2::diagnostic indiag,
		Logger *log,std::string hostname,std::string sensorspecpath,bool extrapolate)
{
	initialize_stateack_messages();
	initialize_message_info();
	serialmessagehandler = new SerialMessageHandler();
	board_state = GPIO_MODE_UNDEFINED;

	node_state = GPIO_MODE_BOOT;
	prev_node_state = GPIO_MODE_BOOT;
	myhostname = hostname;
	diagnostic = indiag;
	mylogger = log;
	mydevice.DeviceName = hostname;
	timeout_value_ms = INITIAL_TIMEOUT_VALUE_MS;
	sensor_spec_path = sensorspecpath;
	extrapolate_values = extrapolate;
	return diagnostic;
}
icarus_rover_v2::diagnostic GPIONodeProcess::update(long dt)
{
	ms_timer += dt;
	if(ms_timer >= timeout_value_ms) { timer_timeout = true; }
	if(timer_timeout == true)
	{
		timer_timeout = false;
		//printf("Mode: %d,%d\n",node_state,board_state);
		/*if((node_state == GPIO_MODE_INITIALIZING) && (board_state == GPIO_MODE_INITIALIZING))
		{
			//printf("Setting to true.\n");
			send_configure_DIO_PortA.trigger = true;
			send_configure_DIO_PortB.trigger = true;
			prev_node_state = node_state;
			node_state = GPIO_MODE_INITIALIZED;
		}
		*/

	}
	if(prev_node_state != node_state)
	{
		send_nodemode.trigger = true;
		prev_node_state = node_state;
	}
	if((board_state == GPIO_MODE_RUNNING) && (node_state == GPIO_MODE_RUNNING))
	{
		send_set_DIO_PortA.trigger = true;
	}
	//send_nodemode.trigger = true;
	diagnostic.Level = INFO;
	diagnostic.Diagnostic_Message = NOERROR;
	diagnostic.Description = "Node Executing.";
	return diagnostic;
}
state_ack GPIONodeProcess::get_stateack(std::string name)
{

	if(name == send_configure_DIO_PortA.name)
	{
		return send_configure_DIO_PortA;
	}
	else if(name == send_configure_DIO_PortB.name)
	{
		return send_configure_DIO_PortB;
	}
	else if(name == send_testmessage_command.name)
	{
		return send_testmessage_command;
	}
	else if(name == send_nodemode.name)
	{
		return send_nodemode;
	}
	else if(name == send_set_DIO_PortA.name)
	{
		return send_set_DIO_PortA;
	}
	else if(name == send_set_DIO_PortB.name)
	{
		return send_set_DIO_PortB;
	}
	else
	{
		state_ack emptystateack;
		emptystateack.name = "";
		return emptystateack;
	}
}
bool GPIONodeProcess::set_stateack(state_ack stateack)
{
	printf("name: %s\n",stateack.name.c_str());
	if(stateack.name == "Send Configure DIO PortA")
	{
		send_configure_DIO_PortA = stateack;
	}
	else if(stateack.name == "Send Configure DIO PortB")
	{
		send_configure_DIO_PortB = stateack;
	}
	else if(stateack.name == "Send Test Message Command")
	{
		send_testmessage_command = stateack;
	}
	else if(stateack.name == "Send Node Mode")
	{
		send_nodemode = stateack;
	}
	else if(stateack.name == "Send Set DIO PortA")
	{
		send_set_DIO_PortA = stateack;

	}
	else if(stateack.name == "Send Set DIO PortB")
	{
		send_set_DIO_PortB = stateack;
	}
	else
	{
		return false;
	}
	return true;
}
icarus_rover_v2::diagnostic GPIONodeProcess::new_pinmsg(icarus_rover_v2::pin pinmsg)
{
	if((board_state == GPIO_MODE_RUNNING) && (node_state == GPIO_MODE_RUNNING))
	{
		if(pinmsg.Port == DIO_PortA.PortName)
		{
			DIO_PortA.Value[pinmsg.Number-1] = pinmsg.Value;
			diagnostic.Level = NOERROR;
			send_set_DIO_PortA.trigger = true;
		}
		else if(pinmsg.Port == DIO_PortB.PortName)
		{
			DIO_PortB.Value[pinmsg.Number-1] = pinmsg.Value;
			diagnostic.Level = NOERROR;
			send_set_DIO_PortB.trigger = true;
		}
		else
		{
			diagnostic.Level = WARN;
		}
	}
	return diagnostic;
}
bool GPIONodeProcess::checkTriggers(std::vector<std::vector<unsigned char > > &tx_buffers)
{
	bool nothing_triggered = true;
	if(send_configure_DIO_PortA.trigger == true)
	{
		nothing_triggered = false;
		std::string BoardName = myboards.at(send_configure_DIO_PortA.flag1).DeviceName;
		Port_Info Port = get_PortInfo(BoardName,"DIO_PortA");
		if(Port.PortName == "")
		{
			nothing_triggered = false;
			return false;
			printf("SHOULD NOT GET HERE.\n");
			mylogger->log_error("COULDN'T LOOK UP PORT NAME!!!");
		}
		char buffer[12];
		int length;
		int tx_status = serialmessagehandler->encode_Configure_DIO_PortASerial(buffer,&length,
				Port.Mode[0],Port.Mode[1],Port.Mode[2],Port.Mode[3],Port.Mode[4],Port.Mode[5],Port.Mode[6],Port.Mode[7]);
		bool status = gather_message_info(SERIAL_Configure_DIO_PortA_ID, "transmit");
		tx_buffers.push_back(std::vector<unsigned char>(buffer,buffer+sizeof(buffer)/sizeof(buffer[0])));
		send_configure_DIO_PortA.state = true;
		if (send_configure_DIO_PortA.retrying == false)
		{
			gettimeofday(&send_configure_DIO_PortA.orig_send_time,NULL);
			send_configure_DIO_PortA.retries = 0;
		}
		send_configure_DIO_PortA.trigger = false;
	}
	if(send_configure_DIO_PortB.trigger == true)
	{
		nothing_triggered = false;
		//printf("send_configure_DIO_PortB Triggered.\n");
		std::string BoardName = myboards.at(send_configure_DIO_PortB.flag1).DeviceName;
		Port_Info Port = get_PortInfo(BoardName,"DIO_PortB");
		char buffer[12];
		int length;
		int tx_status = serialmessagehandler->encode_Configure_DIO_PortBSerial(buffer,&length,
				Port.Mode[0],Port.Mode[1],Port.Mode[2],Port.Mode[3],Port.Mode[4],Port.Mode[5],Port.Mode[6],Port.Mode[7]);
		bool status = gather_message_info(SERIAL_Configure_DIO_PortB_ID, "transmit");
		tx_buffers.push_back(std::vector<unsigned char>(buffer,buffer+sizeof(buffer)/sizeof(buffer[0])));

		send_configure_DIO_PortB.state = true;
		if (send_configure_DIO_PortB.retrying == false)
		{
			gettimeofday(&send_configure_DIO_PortB.orig_send_time,NULL);
			send_configure_DIO_PortB.retries = 0;
		}
		send_configure_DIO_PortB.trigger = false;
	}
	if(send_testmessage_command.trigger == true)
	{
		nothing_triggered = false;
		//printf("send_testmessage_command Triggered.\n");
		char buffer[12];
		int length;
		int tx_status = serialmessagehandler->encode_TestMessageCommandSerial(buffer,&length,
				0,0,0,0,0,0,0,0);
		bool status = gather_message_info(SERIAL_TestMessageCommand_ID, "transmit");
		tx_buffers.push_back(std::vector<unsigned char>(buffer,buffer+sizeof(buffer)/sizeof(buffer[0])));

		send_testmessage_command.state = true;
		if (send_testmessage_command.retrying == false)
		{
			gettimeofday(&send_testmessage_command.orig_send_time,NULL);
			send_testmessage_command.retries = 0;
		}
		send_testmessage_command.trigger = false;
	}
	if(send_nodemode.trigger == true)
	{
		nothing_triggered = false;
		//printf("send_nodemode Triggered.\n");
		char buffer[12];
		int length;
		int computed_checksum;
		int tx_status = serialmessagehandler->encode_ModeSerial(buffer,&length,node_state);
		bool status = gather_message_info(SERIAL_Mode_ID, "transmit");
		tx_buffers.push_back(std::vector<unsigned char>(buffer,buffer+sizeof(buffer)/sizeof(buffer[0])));
		send_nodemode.state = true;
		if (send_nodemode.retrying == false)
		{
			gettimeofday(&send_nodemode.orig_send_time,NULL);
			send_nodemode.retries = 0;
		}
		send_nodemode.trigger = false;
	}
	if(send_set_DIO_PortA.trigger == true)
	{
		nothing_triggered = false;
		//printf("send_set_DIO_PortA Triggered.\n");
		char buffer[12];
		int length;
		int computed_checksum;
		std::string BoardName = myboards.at(send_configure_DIO_PortA.flag1).DeviceName;
		Port_Info Port = get_PortInfo(BoardName,"DIO_PortA");
		if(Port.PortName == "")
		{
			nothing_triggered = false;
			return false;
			printf("SHOULD NOT GET HERE.\n");
			mylogger->log_error("COULDN'T LOOK UP PORT NAME!!!");
		}
		int tx_status = serialmessagehandler->encode_Set_DIO_PortASerial(buffer,&length,Port.Value[0],Port.Value[1],Port.Value[2],Port.Value[3],
				Port.Value[4],Port.Value[5],Port.Value[6],Port.Value[7]);
		bool status = gather_message_info(SERIAL_Set_DIO_PortA_ID, "transmit");
		//char tempstr[128];
		//sprintf(tempstr,"Setting Pin 0 to: %d",Port.Value[0]);
		//mylogger->log_debug(tempstr);
		tx_buffers.push_back(std::vector<unsigned char>(buffer,buffer+sizeof(buffer)/sizeof(buffer[0])));
		send_set_DIO_PortA.state = true;
		if (send_set_DIO_PortA.retrying == false)
		{
			gettimeofday(&send_set_DIO_PortA.orig_send_time,NULL);
			send_set_DIO_PortA.retries = 0;
		}
		send_set_DIO_PortA.trigger = false;
	}

	if(send_set_DIO_PortB.trigger == true)
	{
		nothing_triggered = false;
		//printf("send_set_DIO_PortB Triggered.\n");
		char buffer[12];
		int length;
		int computed_checksum;
		std::string BoardName = myboards.at(send_configure_DIO_PortB.flag1).DeviceName;
		Port_Info Port = get_PortInfo(BoardName,"DIO_PortB");
		if(Port.PortName == "")
		{
			nothing_triggered = false;
			return false;
			printf("SHOULD NOT GET HERE.\n");
			mylogger->log_error("COULDN'T LOOK UP PORT NAME!!!");
		}
		int tx_status = serialmessagehandler->encode_Set_DIO_PortBSerial(buffer,&length,DIO_PortB.Value[0],DIO_PortB.Value[1],DIO_PortB.Value[2],DIO_PortB.Value[3],
				DIO_PortB.Value[4],DIO_PortB.Value[5],DIO_PortB.Value[6],DIO_PortB.Value[7]);
		bool status = gather_message_info(SERIAL_Set_DIO_PortB_ID, "transmit");
		tx_buffers.push_back(std::vector<unsigned char>(buffer,buffer+sizeof(buffer)/sizeof(buffer[0])));
		send_set_DIO_PortB.state = true;
		if (send_set_DIO_PortB.retrying == false)
		{
			gettimeofday(&send_set_DIO_PortB.orig_send_time,NULL);
			send_set_DIO_PortB.retries = 0;
		}
		send_set_DIO_PortB.trigger = false;
	}
	if(nothing_triggered == true)
	{
		tx_buffers.clear();
		return false;
	}
	else
	{
		return true;
	}
}
icarus_rover_v2::diagnostic GPIONodeProcess::new_commandmsg(icarus_rover_v2::command msg)
{
	if (msg.Command ==  DIAGNOSTIC_ID)
	{
		if(msg.Option1 == LEVEL1)
		{
		}
		else if(msg.Option1 == LEVEL2)
		{
			if((board_state == GPIO_MODE_RUNNING) && (node_state == GPIO_MODE_RUNNING))
			{
				send_testmessage_command.trigger = true;
			}
			else
			{
				diagnostic.Level = WARN;
				diagnostic.Diagnostic_Message = DIAGNOSTIC_FAILED;
				diagnostic.Description = "Diagnostic not run.";
			}
		}
		else if(msg.Option1 == LEVEL3)
		{

		}
		else if(msg.Option1 == LEVEL4)
		{

		}
		else
		{
			mylogger->log_error("Shouldn't get here!!!");
		}
	}
	return diagnostic;
}
icarus_rover_v2::diagnostic GPIONodeProcess::new_serialmessage_TestMessageCounter(int packet_type,unsigned char* inpacket)
{
	bool status = gather_message_info(SERIAL_TestMessageCounter_ID, "receive");
	if(status == false)
	{
		diagnostic.Level = ERROR;
		diagnostic.Diagnostic_Type = COMMUNICATIONS;
		diagnostic.Diagnostic_Message = DROPPING_PACKETS;
		diagnostic.Description = "TestMessageCounter not received correctly.";
	}
	else
	{
		diagnostic.Level = WARN;
		diagnostic.Diagnostic_Type = COMMUNICATIONS;
		diagnostic.Diagnostic_Message = DROPPING_PACKETS;
		diagnostic.Description = "TestMessageCounter Message Not Implemented Yet.";
	}
	return diagnostic;
	
}
icarus_rover_v2::diagnostic GPIONodeProcess::new_serialmessage_FirmwareVersion(int packet_type,unsigned char* inpacket)
{
	bool status = gather_message_info(SERIAL_FirmwareVersion_ID, "receive");
	if(status == false)
	{
		diagnostic.Level = ERROR;
		diagnostic.Diagnostic_Type = COMMUNICATIONS;
		diagnostic.Diagnostic_Message = DROPPING_PACKETS;
		diagnostic.Description = "FirmwareVersion not received correctly.";
	}
	else
	{
		diagnostic.Level = WARN;
		diagnostic.Diagnostic_Type = COMMUNICATIONS;
		diagnostic.Diagnostic_Message = DROPPING_PACKETS;
		diagnostic.Description = "FirmwareVersion Message Not Implemented Yet.";
	}
	return diagnostic;
}
icarus_rover_v2::diagnostic GPIONodeProcess::new_serialmessage_Diagnostic(int packet_type,unsigned char* inpacket)
{
	if(packet_type ==SERIAL_Diagnostic_ID)
	{
		bool status = gather_message_info(SERIAL_Diagnostic_ID, "receive");
		if(status == false)
		{
			diagnostic.Level = ERROR;
			diagnostic.Diagnostic_Type = COMMUNICATIONS;
			diagnostic.Diagnostic_Message = DROPPING_PACKETS;
			diagnostic.Description = "Diagnostic not received correctly.";
		}
		else
		{
			mylogger->log_debug("Got Diagnostic from GPIO Board.");
			char gpio_board_system,gpio_board_subsystem,gpio_board_component,gpio_board_diagtype,gpio_board_level,gpio_board_message;
			serialmessagehandler->decode_DiagnosticSerial(inpacket,&gpio_board_system,&gpio_board_subsystem,&gpio_board_component,&gpio_board_diagtype,&gpio_board_level,&gpio_board_message);
			if(gpio_board_level > INFO)
			{
				diagnostic.Level = gpio_board_level;
				diagnostic.Diagnostic_Message = gpio_board_level;
				diagnostic.Description = "GPIO Board Empty Message.";
			}
		}
	}
	else
	{
		diagnostic.Level = ERROR;
		diagnostic.Description = "Diagnostic Message Not Decoded successfully.";
		diagnostic.Diagnostic_Message = DROPPING_PACKETS;
	}
	return diagnostic;
}
icarus_rover_v2::diagnostic GPIONodeProcess::new_serialmessage_Get_ANA_PortA(int packet_type,unsigned char* inpacket)
{
	bool status = gather_message_info(SERIAL_Get_ANA_PortA_ID, "receive");
	if(status == false)
	{
		diagnostic.Level = ERROR;
		diagnostic.Diagnostic_Type = COMMUNICATIONS;
		diagnostic.Diagnostic_Message = DROPPING_PACKETS;
		diagnostic.Description = "Get_ANA_PortA not received correctly.";
	}
	else
	{
		if(packet_type ==SERIAL_Get_ANA_PortA_ID)
		{
			char tempstr[128];
			int value1,value2,value3,value4;
			serialmessagehandler->decode_Get_ANA_PortASerial(inpacket,&value1,&value2,&value3,&value4);
			if(ANA_PortA.Mode[0] == PINMODE_ANALOG_INPUT)
			{
				ANA_PortA.Value[0] = value1;
			}
			else if(ANA_PortA.Mode[0] == PINMODE_FORCESENSOR_INPUT)
			{
				ANA_PortA.Value[0] = transducer_model(ANA_PortA.Mode[0],ANA_PortA.ConnectingDevice.at(0),(double)(value1));
			}

			if(ANA_PortA.Mode[1] == PINMODE_ANALOG_INPUT)
			{
				ANA_PortA.Value[1] = value2;
			}
			else if(ANA_PortA.Mode[1] == PINMODE_FORCESENSOR_INPUT)
			{
				ANA_PortA.Value[1] = transducer_model(ANA_PortA.Mode[1],ANA_PortA.ConnectingDevice.at(1),(double)(value2));
			}

			if(ANA_PortA.Mode[2] == PINMODE_ANALOG_INPUT)
			{
				ANA_PortA.Value[2] = value3;
			}
			else if(ANA_PortA.Mode[2] == PINMODE_FORCESENSOR_INPUT)
			{
				ANA_PortA.Value[2] = transducer_model(ANA_PortA.Mode[2],ANA_PortA.ConnectingDevice.at(2),(double)(value3));
			}

			if(ANA_PortA.Mode[3] == PINMODE_ANALOG_INPUT)
			{
				ANA_PortA.Value[3] = value4;
			}
			else if(ANA_PortA.Mode[3] == PINMODE_FORCESENSOR_INPUT)
			{
				ANA_PortA.Value[3] = transducer_model(ANA_PortA.Mode[3],ANA_PortA.ConnectingDevice.at(3),(double)(value4));
			}
		}
		else
		{
			diagnostic.Level = ERROR;
			diagnostic.Description = "Get_ANA_PortA Not Decoded successfully.";
			diagnostic.Diagnostic_Message = DROPPING_PACKETS;
		}
	}
	return diagnostic;
}

icarus_rover_v2::diagnostic GPIONodeProcess::new_serialmessage_Get_ANA_PortB(int packet_type,unsigned char* inpacket)
{
	bool status = gather_message_info(SERIAL_Get_ANA_PortB_ID, "receive");
	if(status == false)
	{
		diagnostic.Level = ERROR;
		diagnostic.Diagnostic_Type = COMMUNICATIONS;
		diagnostic.Diagnostic_Message = DROPPING_PACKETS;
		diagnostic.Description = "Get_ANA_PortB not received correctly.";
	}
	else
	{
		if(packet_type ==SERIAL_Get_ANA_PortB_ID)
		{
			char tempstr[128];
			int value1,value2,value3,value4;
			serialmessagehandler->decode_Get_ANA_PortBSerial(inpacket,&value1,&value2,&value3,&value4);

			if(ANA_PortB.Mode[0] == PINMODE_ANALOG_INPUT)
			{
				ANA_PortB.Value[0] = value1;
			}
			else if(ANA_PortB.Mode[0] == PINMODE_FORCESENSOR_INPUT)
			{
				ANA_PortB.Value[0] = transducer_model(ANA_PortB.Mode[0],ANA_PortB.ConnectingDevice.at(0),(double)(value1));
				//printf("input: %d out: %d\n",value1,ANA_PortB.Value[0]);
			}

			if(ANA_PortB.Mode[1] == PINMODE_ANALOG_INPUT)
			{
				ANA_PortB.Value[1] = value2;
			}
			else if(ANA_PortB.Mode[1] == PINMODE_FORCESENSOR_INPUT)
			{
				ANA_PortB.Value[1] = transducer_model(ANA_PortB.Mode[1],ANA_PortB.ConnectingDevice.at(1),(double)(value2));
			}

			if(ANA_PortB.Mode[2] == PINMODE_ANALOG_INPUT)
			{
				ANA_PortB.Value[2] = value3;
			}
			else if(ANA_PortB.Mode[2] == PINMODE_FORCESENSOR_INPUT)
			{
				ANA_PortB.Value[2] = transducer_model(ANA_PortB.Mode[2],ANA_PortB.ConnectingDevice.at(2),(double)(value3));
			}

			if(ANA_PortB.Mode[3] == PINMODE_ANALOG_INPUT)
			{
				ANA_PortB.Value[3] = value4;
			}
			else if(ANA_PortB.Mode[3] == PINMODE_FORCESENSOR_INPUT)
			{
				ANA_PortB.Value[3] = transducer_model(ANA_PortB.Mode[3],ANA_PortB.ConnectingDevice.at(3),(double)(value4));
			}
		}
		else
		{
			diagnostic.Level = ERROR;
			diagnostic.Description = "Get_ANA_PortB Not Decoded successfully.";
			diagnostic.Diagnostic_Message = DROPPING_PACKETS;
		}
	}
	return diagnostic;
}
icarus_rover_v2::diagnostic GPIONodeProcess::new_serialmessage_Get_DIO_PortA(int packet_type,unsigned char* inpacket)
{
	bool status = gather_message_info(SERIAL_Get_DIO_PortA_ID, "receive");
	if(status == false)
	{
		diagnostic.Level = ERROR;
		diagnostic.Diagnostic_Type = COMMUNICATIONS;
		diagnostic.Diagnostic_Message = DROPPING_PACKETS;
		diagnostic.Description = "Get_DIO_PortA not received correctly.";
	}
	else
	{
		diagnostic.Level = WARN;
		diagnostic.Diagnostic_Type = COMMUNICATIONS;
		diagnostic.Diagnostic_Message = DROPPING_PACKETS;
		diagnostic.Description = "Get_DIO_PortA Message Not Implemented Yet.";
	}
	return diagnostic;
}
icarus_rover_v2::diagnostic GPIONodeProcess::new_serialmessage_Get_DIO_PortB(int packet_type,unsigned char* inpacket)
{
	bool status = gather_message_info(SERIAL_Get_DIO_PortB_ID, "receive");
	if(status == false)
	{
		diagnostic.Level = ERROR;
		diagnostic.Diagnostic_Type = COMMUNICATIONS;
		diagnostic.Diagnostic_Message = DROPPING_PACKETS;
		diagnostic.Description = "Get_DIO_PortB not received correctly.";
	}
	else
	{
		diagnostic.Level = WARN;
		diagnostic.Diagnostic_Type = COMMUNICATIONS;
		diagnostic.Diagnostic_Message = DROPPING_PACKETS;
		diagnostic.Description = "Get_DIO_PortB Message Not Implemented Yet.";
	}
	return diagnostic;
}
icarus_rover_v2::diagnostic GPIONodeProcess::new_serialmessage_Get_Mode(int packet_type,unsigned char* inpacket)
{
	bool status = gather_message_info(SERIAL_Mode_ID, "receive");
	if(status == false)
	{
		diagnostic.Level = ERROR;
		diagnostic.Diagnostic_Type = COMMUNICATIONS;
		diagnostic.Diagnostic_Message = DROPPING_PACKETS;
		diagnostic.Description = "GPIO Board Mode not received correctly.";
	}
	else
	{
		if(packet_type ==SERIAL_Mode_ID)
		{
			char tempstr[128];
			char value1;
			serialmessagehandler->decode_ModeSerial(inpacket,&value1);
			board_state = value1;
			//sprintf(tempstr,"Got GPIO Board Mode: %d from GPIO Board.  Node Mode is: %d",board_state,node_state);
			//mylogger->log_debug(tempstr);
			diagnostic.Level = NOERROR;
			diagnostic.Description = "GPIO Board Mode Decoded successfully.";
			diagnostic.Diagnostic_Message = NOERROR;
			if(board_state == GPIO_MODE_BOOT)
			{
				send_configure_DIO_PortA.trigger = false;
				send_configure_DIO_PortB.trigger = false;
			}
			else if(node_state == GPIO_MODE_INITIALIZING)
			{
				send_configure_DIO_PortA.trigger = true;
				send_configure_DIO_PortB.trigger = true;
				prev_node_state = node_state;
				node_state = GPIO_MODE_INITIALIZED;
			}
			else if(board_state == GPIO_MODE_RUNNING)
			{
				if(node_state == GPIO_MODE_INITIALIZED)
				{
					node_state = prev_node_state;
					node_state = GPIO_MODE_RUNNING;
				}
			}
			else if((board_state == GPIO_MODE_INITIALIZING) && (node_state == GPIO_MODE_RUNNING))
			{
				node_state = GPIO_MODE_INITIALIZING;
			}
			/*	else if((board_state == GPIO_MODE_INITIALIZING) && (node_state == GPIO_MODE_INITIALIZED))
		{
			node_state = GPIO_MODE_INITIALIZING;
		}
			 */
			else
			{
			}

		}
		else
		{
			diagnostic.Level = ERROR;
			diagnostic.Description = "GPIO Board Mode Not Decoded successfully.";
			diagnostic.Diagnostic_Message = DROPPING_PACKETS;
		}
	}
	return diagnostic;
}
double GPIONodeProcess::find_slope(std::vector<double> x,std::vector<double> y)
{
	double slope = 0.0;
	for(int i = 1; i < x.size()-1;i++)
	{
		double temp = (y.at(i)-y.at(i-1))/(x.at(i)-x.at(i-1));
		slope = slope + temp;
	}
	slope = slope/(double)(x.size()-2);
	return slope;
}
double GPIONodeProcess::find_intercept(double slope,std::vector<double> x,std::vector<double> y)
{
	double intercept = 0.0;
	for(int i = 0; i < x.size();i++)
	{
		double temp = y.at(i)-(slope*x.at(i));
		intercept = intercept + temp;
	}
	intercept = intercept/(double)(x.size());
	return intercept;
}
icarus_rover_v2::diagnostic GPIONodeProcess::new_devicemsg(icarus_rover_v2::device newdevice)
{
	if( (all_board_info_received == true) &&
		(all_sensor_info_received == true))
	{
		all_device_info_received = true;
	}
	if((newdevice.DeviceParent == "None") && (newdevice.DeviceName == myhostname))// && (all_device_info_received == false))
	{
		mydevice = newdevice;
	}

	if((newdevice.DeviceType == "GPIOBoard") && (newdevice.DeviceParent == mydevice.DeviceName) && (mydevice.DeviceName != "") && (all_board_info_received == false))
	{
		icarus_rover_v2::device board;
		board.DeviceName = newdevice.DeviceName;
		board.pins = newdevice.pins;
		myboards.push_back(board);
		printf("Board Count so far: %d\n",myboards.size());
		printf("Board pin count: %d\n",board.pins.size());
		for(int i = 0; i < board.pins.size();i++)
		{
			if(configure_pin(board.DeviceName,board.pins.at(i).Port,board.pins.at(i).Number,board.pins.at(i).Function,board.pins.at(i).ConnectedDevice)==false)
			{
				char tempstr[256];
				sprintf(tempstr,"Board: %s Couldn't Configure Pin on Port: %s Number: %d Function: %s Connecting Device: %s",
						board.DeviceName.c_str(),
						board.pins.at(i).Port.c_str(),
						board.pins.at(i).Number,
						board.pins.at(i).Function.c_str(),
						board.pins.at(i).ConnectedDevice.c_str());
				//printf("%s\n",tempstr);
				mylogger->log_error(tempstr);
				diagnostic.Diagnostic_Type = SOFTWARE;
				diagnostic.Level = ERROR;
				diagnostic.Diagnostic_Message = INITIALIZING_ERROR;
				diagnostic.Description = std::string(tempstr);
			}
			else
			{
				char tempstr[256];
				sprintf(tempstr,"Board: %s Configured Pin on Port: %s Number: %d Function: %s Connected Device: %s",
						board.DeviceName.c_str(),
						board.pins.at(i).Port.c_str(),
						board.pins.at(i).Number,
						board.pins.at(i).Function.c_str(),
						board.pins.at(i).ConnectedDevice.c_str());
				//printf("%s\n",tempstr);
				mylogger->log_debug(tempstr);
			}
		}
		if((mydevice.BoardCount == myboards.size()) && (mydevice.BoardCount > 0) && (all_board_info_received == false))
		{
			diagnostic.Diagnostic_Type = NOERROR;
			diagnostic.Level = INFO;
			diagnostic.Diagnostic_Message = NOERROR;
			diagnostic.Description = "Received all Board Info.";
			mylogger->log_info("Received all Device Info.");
			for(int b = 0; b < myboards.size(); b++)
			{
				for(int p = 0; p < myboards.at(b).pins.size(); p++)
				{
					char tempstr[128];
					icarus_rover_v2::pin newpin = myboards.at(b).pins.at(p);
					sprintf(tempstr,"Board: %d Port: %s Pin: %d Function: %s",b,newpin.Port.c_str(),newpin.Number,newpin.Function.c_str());
					mylogger->log_debug(tempstr);
				}
			}
			all_board_info_received = true;
			ms_timer = 0;
			//printf("Received all device info.\n");
			prev_node_state = node_state;
			node_state = GPIO_MODE_INITIALIZING;
		}
	}
	else if((newdevice.DeviceType == "ForceSensor") && (newdevice.DeviceParent == mydevice.DeviceName) && (mydevice.DeviceName != "") && (all_sensor_info_received == false))
	{
		icarus_rover_v2::device dev_sensor;
		dev_sensor.DeviceName = newdevice.DeviceName;
		mysensors.push_back(dev_sensor);
		printf("Device Name: %s\n",newdevice.DeviceName.c_str());
		printf("Sensor Count so far: %d/%d\n",mysensors.size(),mydevice.SensorCount);
		Sensor newsensor;
		newsensor.type = newdevice.DeviceType;
		newsensor.name = newdevice.DeviceName;
		newsensor.spec_path = sensor_spec_path + "/" + newdevice.DeviceType + "/" + newdevice.DeviceName + ".csv";
		ifstream specpath(newsensor.spec_path.c_str());
		if( specpath.good() == false)
		{
			diagnostic.Diagnostic_Type = SENSORS;
			diagnostic.Level = FATAL;
			diagnostic.Diagnostic_Message = DEVICE_NOT_AVAILABLE;
			char tempstr[1024];
			sprintf(tempstr,"Specification for Sensor: %s not found at %s",
					newsensor.name.c_str(),
					newsensor.spec_path.c_str());
			diagnostic.Description = tempstr;
			mylogger->log_fatal(tempstr);
			return diagnostic;
		}
		string line;
		std::string name,type,relationship;
		bool voltage_reference_read,adc_resolution_read,rm_read,relationship_read = false;
		double voltage_reference,rm_ohms = 0.0;
		int adc_resolution = 0;
		std::vector<double> input_vector;
		std::vector<double> output_vector;
		bool reading_inputoutputmap = false;
		if (specpath.is_open())
		{
			while ( getline (specpath,line) )
			{
				std::vector<std::string> items;
				boost::split(items,line,boost::is_any_of(","),boost::token_compress_on);
				if(reading_inputoutputmap == false)
				{
					if(items.size() != 2)
					{
						diagnostic.Diagnostic_Type = SENSORS;
						diagnostic.Level = FATAL;
						diagnostic.Diagnostic_Message = DEVICE_NOT_AVAILABLE;
						char tempstr[1024];
						sprintf(tempstr,"Specification for Sensor: %s not in correct format at %s",
								newsensor.name.c_str(),
								newsensor.spec_path.c_str());
						diagnostic.Description = tempstr;
						mylogger->log_fatal(tempstr);
						return diagnostic;
					}
					if(items.at(0) == "Name")
					{
						name = items.at(1);
					}
					else if(items.at(0) == "Type")
					{
						type = items.at(1);
					}
					else if(items.at(0) == "Voltage Reference")
					{
						voltage_reference_read = true;
						voltage_reference = std::atof(items.at(1).c_str());
					}
					else if(items.at(0) == "ADC Resolution")
					{
						adc_resolution_read = true;
						adc_resolution = std::atoi(items.at(1).c_str());
					}
					else if(items.at(0) == "Rm")
					{
						rm_read = true;
						rm_ohms = std::atof(items.at(1).c_str());
					}
					else if(items.at(0) == "Relationship")
					{
						relationship_read = true;
						relationship = items.at(1);
					}
					else if(items.at(0) == "Specification:")
					{
						getline (specpath,line);
						boost::split(items,line,boost::is_any_of(","),boost::token_compress_on);

						if((items.at(0) == "Input") && (items.at(1) == "Output"))
						{
							reading_inputoutputmap = true;
						}
					}
				}
				else
				{
					if(items.size() != 2)
					{
						diagnostic.Diagnostic_Type = SENSORS;
						diagnostic.Level = FATAL;
						diagnostic.Diagnostic_Message = DEVICE_NOT_AVAILABLE;
						char tempstr[1024];
						sprintf(tempstr,"Specification for Sensor: %s not in correct format at %s",
								newsensor.name.c_str(),
								newsensor.spec_path.c_str());
						diagnostic.Description = tempstr;
						mylogger->log_fatal(tempstr);
						return diagnostic;
					}
					double input = std::atof(items.at(0).c_str());
					double output = std::atof(items.at(1).c_str());
					input_vector.push_back(input);
					output_vector.push_back(output);
				}
			}
		}
		specpath.close();
		if(	(name == newsensor.name) &&
			(type == newsensor.type) &&
			(voltage_reference_read == true) &&
			(adc_resolution_read == true) &&
			(rm_read == true) &&
			(relationship_read == true) &&
			(input_vector.size() == output_vector.size()) &&
			(reading_inputoutputmap == true))
		{
			newsensor.input_vector = input_vector;
			newsensor.output_vector = output_vector;
			newsensor.adc_resolution = adc_resolution;
			newsensor.voltage_reference = voltage_reference;
			newsensor.Rm_ohms = rm_ohms;
			newsensor.spec_relationship = relationship;
			std::vector<double> x,y;
			if(newsensor.spec_relationship == "log-log")
			{
				for(int i = 0; i < newsensor.input_vector.size(); i++)
				{
					x.push_back(log10(newsensor.input_vector.at(i)));
					y.push_back(log10(newsensor.output_vector.at(i)));
				}
				double slope = find_slope(x,y);
				double intercept = find_intercept(slope,x,y);
				newsensor.slope = slope;
				newsensor.intercept = intercept;

			}
			else
			{
				diagnostic.Diagnostic_Type = SENSORS;
				diagnostic.Level = FATAL;
				diagnostic.Diagnostic_Message = DEVICE_NOT_AVAILABLE;
				char tempstr[1024];
				sprintf(tempstr,"Specification for Sensor: %s relationship: %s Not Currently Supported",
						newsensor.name.c_str(),
						newsensor.spec_relationship.c_str());
				diagnostic.Description = tempstr;
				mylogger->log_fatal(tempstr);
				return diagnostic;
			}
			SensorSpecs.push_back(newsensor);
			diagnostic.Diagnostic_Type = SENSORS;
			diagnostic.Level = NOTICE;
			diagnostic.Diagnostic_Message = INITIALIZING;
			char tempstr[255];
			sprintf(tempstr,"Sensor Spec for: %s read correctly. Using slope: %f intercept: %f",newsensor.name.c_str(),
					newsensor.slope,newsensor.intercept);
			diagnostic.Description = "Received all Sensor Info.";
			mylogger->log_notice(tempstr);
		}
		else
		{
			diagnostic.Diagnostic_Type = SENSORS;
			diagnostic.Level = FATAL;
			diagnostic.Diagnostic_Message = DEVICE_NOT_AVAILABLE;
			char tempstr[1024];
			sprintf(tempstr,"Specification for Sensor: %s not read correctly at %s",
					newsensor.name.c_str(),
					newsensor.spec_path.c_str());
			diagnostic.Description = tempstr;
			mylogger->log_fatal(tempstr);
			char tempstr2[1024];
			sprintf(tempstr2,"Found: Sensor Name: %s/%s Sensor Type: %s/%s Voltage Reference: %f "
					"ADC Resolution: %d Rm: %f I/O Map Read: %d Size: %d/%d Please Correct as appropriate.",
					name.c_str(),newsensor.name.c_str(),
					type.c_str(),newsensor.type.c_str(),
					voltage_reference,adc_resolution,rm_ohms,
					reading_inputoutputmap,
					input_vector.size(),output_vector.size());
			mylogger->log_fatal(tempstr2);
			return diagnostic;
		}
		if((mydevice.SensorCount == mysensors.size()) && (mydevice.SensorCount > 0) && (all_sensor_info_received == false))
		{
			diagnostic.Diagnostic_Type = NOERROR;
			diagnostic.Level = INFO;
			diagnostic.Diagnostic_Message = NOERROR;
			diagnostic.Description = "Received all Sensor Info.";
			mylogger->log_info("Received all Sensor Info.");
			all_sensor_info_received = true;
			ms_timer = 0;
			//printf("Received all device info.\n");
			prev_node_state = node_state;
			node_state = GPIO_MODE_INITIALIZING;
		}
	}
	return diagnostic;
}
bool GPIONodeProcess::configure_pin(std::string BoardName,std::string Port, uint8_t Number, std::string Function,std::string ConnectedDevice)
{
	bool status = true;
	int function = map_PinFunction_ToInt(Function);
	if(function == PINMODE_UNDEFINED) { return false; }
	if((Number < 1) || (Number > 8)) { return false; }
	if(Port == "DIO_PortA")
	{
		DIO_PortA.Number[Number-1] = Number;
		DIO_PortA.Mode[Number-1] = function;
		DIO_PortA.Available[Number-1] = true;
		DIO_PortA.Value[Number-1] = 0;
		DIO_PortA.ConnectingDevice.at(Number-1) = ConnectedDevice;
	}
	else if(Port == "DIO_PortB")
	{
		DIO_PortB.Number[Number-1] = Number;
		DIO_PortB.Mode[Number-1] = function;
		DIO_PortB.Available[Number-1] = true;
		DIO_PortB.Value[Number-1] = 0;
		DIO_PortB.ConnectingDevice.at(Number-1) = ConnectedDevice;
	}
	else if(Port == "ANA_PortA")
	{
		ANA_PortA.Number[Number-1] = Number;
		ANA_PortA.Mode[Number-1] = function;
		ANA_PortA.Available[Number-1] = true;
		ANA_PortA.Value[Number-1] = 0;
		ANA_PortA.ConnectingDevice.at(Number-1) = ConnectedDevice;
	}
	else if(Port == "ANA_PortB")
	{
		ANA_PortB.Number[Number-1] = Number;
		ANA_PortB.Mode[Number-1] = function;
		ANA_PortB.Available[Number-1] = true;
		ANA_PortB.Value[Number-1] = 0;
		ANA_PortB.ConnectingDevice.at(Number-1) = ConnectedDevice;
	}
	else { 	status = false; }
	return status;
}
std::string GPIONodeProcess::map_mode_ToString(int mode)
{

	switch(mode)
	{
	case GPIO_MODE_UNDEFINED: 					return "Undefined";					break;
	case GPIO_MODE_BOOT:						return "Boot";						break;
	case GPIO_MODE_INITIALIZING:				return "Initializing";				break;
	case GPIO_MODE_INITIALIZED:					return "Initialized";				break;
	case GPIO_MODE_RUNNING:						return "Running";					break;
	case GPIO_MODE_STOPPED:						return "Stopped";					break;
	default:
		std::string tempstr;
		tempstr = "Mode: " + boost::lexical_cast<std::string>(mode) + " Not Supported";
		return tempstr;
	}
}

std::string GPIONodeProcess::map_PinFunction_ToString(int function)
{
	switch(function)
	{
	case PINMODE_UNDEFINED:						return "Undefined";					break;
	case PINMODE_DIGITAL_OUTPUT: 				return "DigitalOutput"; 			break;
	case PINMODE_DIGITAL_INPUT: 				return "DigitalInput"; 				break;
	case PINMODE_ANALOG_INPUT: 					return "AnalogInput"; 				break;
	case PINMODE_FORCESENSOR_INPUT: 			return "ForceSensorInput"; 			break;
	case PINMODE_ULTRASONIC_INPUT: 				return "UltraSonicSensorInput"; 	break;
	case PINMODE_QUADRATUREENCODER_INPUT: 		return "QuadratureEncoderInput";	break;
	case PINMODE_PWM_OUTPUT: 					return "PwmOutput"; 				break;
	default: 									return ""; 							break;
	}
}
int GPIONodeProcess::map_PinFunction_ToInt(std::string Function)
{
	if(Function == "DigitalInput")				{	return PINMODE_DIGITAL_INPUT;		}
	else if(Function == "DigitalOutput")		{	return PINMODE_DIGITAL_OUTPUT;		}
	else if(Function == "AnalogInput")			{	return PINMODE_ANALOG_INPUT;		}
	else if(Function == "ForceSensorInput")		{	return PINMODE_FORCESENSOR_INPUT;	}
	else if(Function == "UltraSonicSensorInput"){	return PINMODE_ULTRASONIC_INPUT;	}
	else if(Function == "PWMOutput")			{	return PINMODE_PWM_OUTPUT;			}
	else { return PINMODE_UNDEFINED; }
}
Port_Info GPIONodeProcess::get_PortInfo(std::string BoardName,std::string PortName)
{
	for(int i = 0; i<myboards.size();i++)
	{
		if(myboards.at(i).DeviceName == BoardName)
		{
			if		(PortName == "DIO_PortA") { return DIO_PortA; }
			else if	(PortName == "DIO_PortB") { return DIO_PortB; }
			else if	(PortName == "ANA_PortA") { return ANA_PortA; }
			else if	(PortName == "ANA_PortB") { return ANA_PortB; }
			else
			{
				Port_Info blank;
				blank.PortName == "";
				return blank;
			}
		}
	}
	Port_Info blank;
	blank.PortName == "";
	return blank;
}
void GPIONodeProcess::initialize_stateack_messages()
{
	send_configure_DIO_PortA.name = "Send Configure DIO PortA";
	send_configure_DIO_PortA.trigger = false;
	send_configure_DIO_PortA.state = false;
	gettimeofday(&send_configure_DIO_PortA.orig_send_time,NULL);
	send_configure_DIO_PortA.retries = 0;
	send_configure_DIO_PortA.timeout_counter = 0;
	send_configure_DIO_PortA.retry_mode = false;
	send_configure_DIO_PortA.failed = false;
	send_configure_DIO_PortA.flag1 = 0; //This flag represents the Board Index

	send_configure_DIO_PortB.name = "Send Configure DIO PortB";
	send_configure_DIO_PortB.trigger = false;
	send_configure_DIO_PortB.state = false;
	gettimeofday(&send_configure_DIO_PortB.orig_send_time,NULL);
	send_configure_DIO_PortB.retries = 0;
	send_configure_DIO_PortB.timeout_counter = 0;
	send_configure_DIO_PortB.retry_mode = false;
	send_configure_DIO_PortB.failed = false;
	send_configure_DIO_PortB.flag1 = 0; //This flag represents the Board Index


	send_testmessage_command.name = "Send Test Message Command";
	send_testmessage_command.trigger = false;
	send_testmessage_command.state = false;
	gettimeofday(&send_testmessage_command.orig_send_time,NULL);
	send_testmessage_command.retries = 0;
	send_testmessage_command.timeout_counter = 0;
	send_testmessage_command.retry_mode = false;
	send_testmessage_command.failed = false;

	send_nodemode.name = "Send Node Mode";
	send_nodemode.trigger = false;
	send_nodemode.state = false;
	gettimeofday(&send_nodemode.orig_send_time,NULL);
	send_nodemode.retries = 0;
	send_nodemode.timeout_counter = 0;
	send_nodemode.retry_mode = false;
	send_nodemode.failed = false;

	send_set_DIO_PortA.name = "Send Set DIO PortA";
	send_set_DIO_PortA.trigger = false;
	send_set_DIO_PortA.state = false;
	gettimeofday(&send_set_DIO_PortA.orig_send_time,NULL);
	send_set_DIO_PortA.retries = 0;
	send_set_DIO_PortA.timeout_counter = 0;
	send_set_DIO_PortA.retry_mode = false;
	send_set_DIO_PortA.failed = false;

	send_set_DIO_PortB.name = "Send Set DIO PortB";
	send_set_DIO_PortB.trigger = false;
	send_set_DIO_PortB.state = false;
	gettimeofday(&send_set_DIO_PortB.orig_send_time,NULL);
	send_set_DIO_PortB.retries = 0;
	send_set_DIO_PortB.timeout_counter = 0;
	send_set_DIO_PortB.retry_mode = false;
	send_set_DIO_PortB.failed = false;

}
bool GPIONodeProcess::gather_message_info(int id, std::string mode)
{
	bool found = false;
	for(int i = 0; i < messages.size();i++)
	{
		if(messages.at(i).id == id)
		{
			found = true;
			if(mode == "transmit")
			{
				double run_time = time_diff(ros::Time::now(),init_time);
				messages.at(i).sent_counter++;
                messages.at(i).transmitted_rate = (double)(messages.at(i)).sent_counter/run_time;
				messages.at(i).last_time_transmitted = ros::Time::now();
			}
			else if(mode == "receive")
			{
                double run_time = time_diff(ros::Time::now(),init_time);
				messages.at(i).received_counter++;
                messages.at(i).received_rate = (double)(messages.at(i)).received_counter/run_time;
				messages.at(i).last_time_received = ros::Time::now();
			}
			break;
		}
	}
	return found;
}
double GPIONodeProcess::time_diff(ros::Time timer_a, ros::Time timer_b)
{
	ros::Duration etime = timer_a - timer_b;
	return etime.toSec();
}
void GPIONodeProcess::initialize_message_info()
{
	{
		message_info newmessage;
		newmessage.id = SERIAL_Diagnostic_ID;
		messages.push_back(newmessage);
	}

	{
		message_info newmessage;
		newmessage.id = SERIAL_TestMessageCounter_ID;
		messages.push_back(newmessage);
	}

	{
		message_info newmessage;
		newmessage.id = SERIAL_TestMessageCommand_ID;
		messages.push_back(newmessage);
	}

	{
		message_info newmessage;
		newmessage.id = SERIAL_Configure_DIO_PortA_ID;
		messages.push_back(newmessage);
	}

	{
		message_info newmessage;
		newmessage.id = SERIAL_Mode_ID;
		messages.push_back(newmessage);
	}

	{
		message_info newmessage;
		newmessage.id = SERIAL_Set_DIO_PortA_ID;
		messages.push_back(newmessage);
	}

	{
		message_info newmessage;
		newmessage.id = SERIAL_Get_ANA_PortA_ID;
		messages.push_back(newmessage);
	}

	{
		message_info newmessage;
		newmessage.id = SERIAL_Get_ANA_PortB_ID;
		messages.push_back(newmessage);
	}

	{
		message_info newmessage;
		newmessage.id = SERIAL_Configure_DIO_PortB_ID;
		messages.push_back(newmessage);
	}

	{
		message_info newmessage;
		newmessage.id = SERIAL_Set_DIO_PortB_ID;
		messages.push_back(newmessage);
	}

	{
		message_info newmessage;
		newmessage.id = SERIAL_Get_DIO_PortA_ID;
		messages.push_back(newmessage);
	}

	{
		message_info newmessage;
		newmessage.id = SERIAL_Get_DIO_PortB_ID;
		messages.push_back(newmessage);
	}

	{
		message_info newmessage;
		newmessage.id = SERIAL_FirmwareVersion_ID;
		messages.push_back(newmessage);
	}

	{
		message_info newmessage;
		newmessage.id = SERIAL_StopMovement_ID;
		messages.push_back(newmessage);
	}

	for(int i = 0; i < messages.size(); i++)
	{
		messages.at(i).protocol = "serial";
		messages.at(i).last_time_received = ros::Time::now();
		messages.at(i).last_time_transmitted = ros::Time::now();
		messages.at(i).received_counter = 0;
		messages.at(i).sent_counter = 0;
		messages.at(i).received_rate = 0.0;
		messages.at(i).transmitted_rate = 0.0;
	}
}
int GPIONodeProcess::transducer_model(int mode,std::string SensorName,double input)
{
	for(int i = 0; i < SensorSpecs.size();i++)
	{
		Sensor mysensor = SensorSpecs.at(i);
		if(mysensor.name == SensorName)
		{
			if(mode == PINMODE_FORCESENSOR_INPUT)
			{
				double maxvalue = pow(2.0,(double)(mysensor.adc_resolution));
				double scaled_voltage = (double)(input)*mysensor.voltage_reference/(double)(maxvalue);
				double res = mysensor.Rm_ohms*((mysensor.voltage_reference/scaled_voltage) - 1.0);
				//NEED TO DO INTERPOLATION/EXTRAPOLATION NOW
				if(mysensor.spec_relationship == "log-log")
				{
					double x,y;
					if(extrapolate_values == false)
					{
						if(res > mysensor.input_vector.at(0))
						{
							
							y = mysensor.output_vector.at(0);
							printf("res too small: %f using output: %f\n",res,y);
							return (int)y;
						}
						else if(res < mysensor.input_vector.at(mysensor.input_vector.size()-1))
						{
							y = mysensor.output_vector.at(mysensor.output_vector.size()-1);
							printf("res too big: %f using output: %f\n",res,y);
							return (int)y;
						}
					}

					x = log10(res);
					double out = mysensor.slope*x + mysensor.intercept;
					y = pow(10.0,out);
					//printf("res: %f x: %f m: %f int: %f out: %f y: %f [y]: %d\n",res,x,mysensor.slope,mysensor.intercept,
					//		out,y,(int)y);
					return (int)y;

				}
			}
			return 0;
		}
	}
	return 0;
}
