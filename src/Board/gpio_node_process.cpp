#include "gpio_node_process.h"

GPIONodeProcess::GPIONodeProcess()
{
	board_state = GPIO_MODE_UNDEFINED;
	node_state = GPIO_MODE_BOOT;
	all_device_info_received = false;
	initialize_Ports();
	ms_timer = 0;
	timeout_value_ms = 0;
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
		
		DIO_PortB.Number[i] = i-1;
		DIO_PortB.Mode[i] = PINMODE_UNDEFINED;
		DIO_PortB.Available[i] = false;
		DIO_PortB.Value[i] = 0;
	}
	ANA_PortA.PortName = "ANA_PortA";
	ANA_PortB.PortName = "ANA_PortB";
	for(int i = 0; i < 4; i++)
	{
		ANA_PortA.Number[i] = i-1;
		ANA_PortA.Mode[i] = PINMODE_ANALOG_INPUT;
		ANA_PortA.Available[i] = false;
		ANA_PortA.Value[i] = 0;
		
		ANA_PortB.Number[i] = 4+i+1;
		ANA_PortB.Mode[i] = PINMODE_ANALOG_INPUT;
		ANA_PortB.Available[i] = false;
		ANA_PortB.Value[i] = 0;
	}
}
icarus_rover_v2::diagnostic GPIONodeProcess::init(icarus_rover_v2::diagnostic indiag,Logger *log,std::string hostname)
{
	initialize_stateack_messages();
	serialmessagehandler = new SerialMessageHandler();
	board_state = GPIO_MODE_UNDEFINED;

	node_state = GPIO_MODE_BOOT;
	prev_node_state = GPIO_MODE_BOOT;
	myhostname = hostname;
	diagnostic = indiag;
	mylogger = log;
	mydevice.DeviceName = hostname;
	timeout_value_ms = INITIAL_TIMEOUT_VALUE_MS;
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
	/*diagnostic.Level = WARN;
	diagnostic.Description = "Not implemented yet.";
	*/
	return diagnostic;
	
}
icarus_rover_v2::diagnostic GPIONodeProcess::new_serialmessage_FirmwareVersion(int packet_type,unsigned char* inpacket)
{
	//diagnostic.Level = WARN;
	//diagnostic.Description = "Not implemented yet.";
	return diagnostic;
}
icarus_rover_v2::diagnostic GPIONodeProcess::new_serialmessage_Diagnostic(int packet_type,unsigned char* inpacket)
{
	if(packet_type ==SERIAL_Diagnostic_ID)
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
	//diagnostic.Level = WARN;
	//diagnostic.Description = "Not implemented yet.";
	return diagnostic;
}
icarus_rover_v2::diagnostic GPIONodeProcess::new_serialmessage_Get_ANA_PortB(int packet_type,unsigned char* inpacket)
{
	//diagnostic.Level = WARN;
	//diagnostic.Description = "Not implemented yet.";
	return diagnostic;
}
icarus_rover_v2::diagnostic GPIONodeProcess::new_serialmessage_Get_DIO_PortA(int packet_type,unsigned char* inpacket)
{
	//diagnostic.Level = WARN;
	//diagnostic.Description = "Not implemented yet.";
	return diagnostic;
}
icarus_rover_v2::diagnostic GPIONodeProcess::new_serialmessage_Get_DIO_PortB(int packet_type,unsigned char* inpacket)
{
	//diagnostic.Level = WARN;
	//diagnostic.Description = "Not implemented yet.";
	return diagnostic;
}
icarus_rover_v2::diagnostic GPIONodeProcess::new_serialmessage_Get_Mode(int packet_type,unsigned char* inpacket)
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
	return diagnostic;
}
icarus_rover_v2::diagnostic GPIONodeProcess::new_devicemsg(icarus_rover_v2::device newdevice)
{
	//cout << "Got Device: " << newdevice.DeviceName << " Parent: " << newdevice.DeviceParent << endl;
	if((newdevice.DeviceParent == "None") && (newdevice.DeviceName == myhostname) && (all_device_info_received == false))
	{
		mydevice = newdevice;
	}

	if((newdevice.DeviceParent == mydevice.DeviceName) && (mydevice.DeviceName != "") && (all_device_info_received == false))
	{
		icarus_rover_v2::device board;
		board.DeviceName = newdevice.DeviceName;
		board.pins = newdevice.pins;
		myboards.push_back(board);
		printf("Device Name: %s\n",newdevice.DeviceName.c_str());
		printf("Board Count so far: %d\n",myboards.size());
		printf("Board pin count: %d\n",board.pins.size());
		for(int i = 0; i < board.pins.size();i++)
		{
			if(configure_pin(board.DeviceName,board.pins.at(i).Port,board.pins.at(i).Number,board.pins.at(i).Function)==false)
			{
				char tempstr[256];
				sprintf(tempstr,"Board: %s Couldn't Configure Pin on Port: %s Number: %d Function: %s",
						board.DeviceName.c_str(),
						board.pins.at(i).Port.c_str(),
						board.pins.at(i).Number,
						board.pins.at(i).Function.c_str());
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
				sprintf(tempstr,"Board: %s Configured Pin on Port: %s Number: %d Function: %s",
						board.DeviceName.c_str(),
						board.pins.at(i).Port.c_str(),
						board.pins.at(i).Number,
						board.pins.at(i).Function.c_str());
				//printf("%s\n",tempstr);
				mylogger->log_debug(tempstr);
			}
		}
		if((mydevice.BoardCount == myboards.size()) && (mydevice.BoardCount > 0) && (all_device_info_received == false))
		{
			diagnostic.Diagnostic_Type = NOERROR;
			diagnostic.Level = INFO;
			diagnostic.Diagnostic_Message = NOERROR;
			diagnostic.Description = "Received all Device Info.";
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
			all_device_info_received = true;
			ms_timer = 0;
			//printf("Received all device info.\n");
			prev_node_state = node_state;
			node_state = GPIO_MODE_INITIALIZING;
		}
	}
	return diagnostic;
}
bool GPIONodeProcess::configure_pin(std::string BoardName,std::string Port, uint8_t Number, std::string Function)
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
	}
	else if(Port == "DIO_PortB")
	{
		DIO_PortB.Number[Number-1] = Number;
		DIO_PortB.Mode[Number-1] = function;
		DIO_PortB.Available[Number-1] = true;
		DIO_PortB.Value[Number-1] = 0;
	}
	else if(Port == "ANA_PortA")
	{
		ANA_PortA.Number[Number-1] = Number;
		ANA_PortA.Mode[Number-1] = function;
		ANA_PortA.Available[Number-1] = true;
		ANA_PortA.Value[Number-1] = 0;
	}
	else if(Port == "ANA_PortB")
	{
		ANA_PortB.Number[Number-1] = Number;
		ANA_PortB.Mode[Number-1] = function;
		ANA_PortB.Available[Number-1] = true;
		ANA_PortB.Value[Number-1] = 0;
	}
	else { 	status = false; }
	return status;
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
