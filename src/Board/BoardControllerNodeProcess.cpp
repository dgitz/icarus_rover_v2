#include "BoardControllerNodeProcess.h"
eros::diagnostic  BoardControllerNodeProcess::finish_initialization()
{
	eros::diagnostic diag = root_diagnostic;
	init_messages();
	current_command.Command = ROVERCOMMAND_NONE;
	LEDPixelMode = LEDPIXELMODE_ERROR;
	encoder_count = 0;
	diag = update_diagnostic(COMMUNICATIONS,INFO,NOERROR,"No Error."); //Using Boards for Device specific comm diagnostics
	diag = update_diagnostic(SENSORS,INFO,NOERROR,"No Error."); //Using Sensors for Device specific sensor diagnostics
	return diag;
}
eros::diagnostic BoardControllerNodeProcess::update(double t_dt,double t_ros_time)
{
	eros::diagnostic diag = root_diagnostic;
	LEDPixelMode = LEDPIXELMODE_NORMAL;
	diag = update_baseprocess(t_dt,t_ros_time);
	bool boards_ready = true;
	for(std::size_t i = 0; i < boards_running.size(); i++)
	{
		if(boards_running.at(i) == true) { boards_ready = boards_ready and true; }
		else
		{
			boards_ready = false;

		}
		if(((run_time - boards.at(i).lasttime_rx) > BOARDCOMM_TIMEOUT_WARN) and
				((run_time - boards.at(i).lasttime_rx) < BOARDCOMM_TIMEOUT_ERROR))
		{
			char tempstr[512];
			sprintf(tempstr,"Have not had comm with Board: %s in %4.2f Seconds.  Timing out Soon.",
					boards.at(i).device.DeviceName.c_str(),run_time - boards.at(i).lasttime_rx);
			diag = update_diagnostic(boards.at(i).device.DeviceName,COMMUNICATIONS,WARN,DROPPING_PACKETS,std::string(tempstr));
			ready_to_arm = false;
		}
		else if(((run_time - boards.at(i).lasttime_rx) > BOARDCOMM_TIMEOUT_ERROR))
		{
			char tempstr[512];
			sprintf(tempstr,"Have not had comm with Board: %s in %4.2f Seconds.  Disarming.",
					boards.at(i).device.DeviceName.c_str(),(run_time - boards.at(i).lasttime_rx));
			diag = update_diagnostic(boards.at(i).device.DeviceName,COMMUNICATIONS,ERROR,DROPPING_PACKETS,std::string(tempstr));
			ready_to_arm = false;
		}
	}

	if(boards_running.size() == 0) {
		boards_ready = false; }
	for(std::size_t i = 0; i < messages.size(); i++)
	{
		if(messages.at(i).sent_counter > 0)
		{
			messages.at(i).sent_rate = (double)(messages.at(i).sent_counter)/run_time;
		}

		if(messages.at(i).recv_counter > 0)
		{
			messages.at(i).recv_rate = (double)(messages.at(i).recv_counter)/run_time;
		}
	}
	bool status = true;
	for(std::size_t i = 0; i < diagnostics.size(); ++i)
	{
		if((diagnostics.at(i).Level >= WARN) or (diagnostics.at(i).Diagnostic_Message == INITIALIZING))
		{
			status = false;
		}
	}
	if(boards_ready == true)
	{
		status = status and true;
		if(status == true)
		{
			diag = update_diagnostic(SOFTWARE,INFO,NOERROR,"Node Running.");
			ready_to_arm = true;
		}
		else
		{
			ready_to_arm = false;
		}
		
	}
	else
	{
		status = false;
		ready_to_arm = false;
		if(mydevice.BoardCount == 0)
		{
			char tempstr[512];
			sprintf(tempstr,"This node requires at least 1 Board, but 0 are defined.");
			diag = update_diagnostic(DATA_STORAGE,ERROR,INITIALIZING_ERROR,std::string(tempstr));
		}
		else
		{
			char tempstr[512];
			sprintf(tempstr,"All info for Boards not received yet.  Expected Board Count: %d",mydevice.BoardCount);
			diag = update_diagnostic(DATA_STORAGE,NOTICE,INITIALIZING,std::string(tempstr));
		}
		
	}
	return diag;
}
eros::diagnostic BoardControllerNodeProcess::new_devicemsg(const eros::device::ConstPtr& t_newdevice)
{
	eros::diagnostic diag = root_diagnostic;
	if(initialized == true)
	{
		if(ready == false)
		{
			if(t_newdevice->DeviceParent == host_name)
			{
				if(board_present(t_newdevice) == true)
				{
					char tempstr[512];
					sprintf(tempstr,"Board: %s already loaded.",
							t_newdevice->DeviceName.c_str());
					diag = update_diagnostic(DATA_STORAGE,WARN,INITIALIZING_ERROR,std::string(tempstr));
					return diag;
				}
				std::size_t board_message = t_newdevice->DeviceType.find("Board");
				if((board_message != std::string::npos))
				{
					eros::device board_device = convert_fromptr(t_newdevice);
					if(t_newdevice->DeviceType == "ArduinoBoard")
					{
						for(std::size_t i = 0; i < t_newdevice->pins.size(); i++)
						{
							if((t_newdevice->pins.at(i).Function == map_PinFunction_ToString(PINMODE_ANALOG_INPUT) or
									(t_newdevice->pins.at(i).Function == map_PinFunction_ToString(PINMODE_QUADRATUREENCODER_INPUT))))
							{
								Sensor new_sensor;
								new_sensor.initialized = false;
								new_sensor.signal.value = 0.0;
								new_sensor.name = t_newdevice->pins.at(i).ConnectedSensor;
								new_sensor.connected_board = board_device;
								new_sensor.connected_pin = t_newdevice->pins.at(i);
								new_sensor.signal.status = SIGNALSTATE_UNDEFINED;
								if(t_newdevice->pins.at(i).Function == "QuadratureEncoderInput")
								{
									new_sensor.signal.type = SIGNALTYPE_TICKSPEED;
								}
								diag = update_diagnostic(new_sensor.name,DATA_STORAGE,NOTICE,NOERROR,"Sensor Initialized.");
								sensors.push_back(new_sensor);
								if(load_sensorinfo(new_sensor.name) == false)
								{
									char tempstr[512];
									sprintf(tempstr,"Unable to load info for Sensor: %s",new_sensor.name.c_str());
									diag = update_diagnostic(DATA_STORAGE,ERROR,INITIALIZING_ERROR,std::string(tempstr));
									return diag;
								}

							}
							else
							{
								char tempstr[512];
								sprintf(tempstr,"Board Type: %s Pin Function: %s Not supported.",
										t_newdevice->DeviceType.c_str(),t_newdevice->pins.at(i).Function.c_str());
								diag = update_diagnostic(DATA_STORAGE,ERROR,INITIALIZING_ERROR,std::string(tempstr));
								return diag;
							}
						}
						Board board;
						board.device = board_device;
						board.lasttime_rx = 0.0;
						boards.push_back(board);
						diag = update_diagnostic(board_device.DeviceName,DATA_STORAGE,NOTICE,NOERROR,"No Error.");
						diag = update_diagnostic(board_device.DeviceName,COMMUNICATIONS,NOTICE,NOERROR,"No Error.");
						boards_running.push_back(true);
					}
					else
					{
						char tempstr[512];
						sprintf(tempstr,"Device Type: %s Not supported.",t_newdevice->DeviceType.c_str());
						diag = update_diagnostic(DATA_STORAGE,ERROR,INITIALIZING_ERROR,std::string(tempstr));
						return diag;
					}

				}

			}
		}
		else
		{

		}
	}
	if(((uint16_t)(boards.size()) == mydevice.BoardCount) and
			(sensors_initialized() == true))
	{
		ready = true;
		diag = update_diagnostic(DATA_STORAGE,INFO,NOERROR,"All Devices Initialized and Running.");
	}
	if(ready == false)
	{
		char tempstr[512];
		sprintf(tempstr,"Initialized: %d Ready: %d",initialized,ready);
		diag = update_diagnostic(DATA_STORAGE,NOTICE,INITIALIZING,std::string(tempstr));
	}
	return diag;
}
std::vector<eros::diagnostic> BoardControllerNodeProcess::new_commandmsg(const eros::command::ConstPtr& t_msg)
{
	std::vector<eros::diagnostic> diaglist;
	eros::diagnostic diag = root_diagnostic;
	current_command = convert_fromptr(t_msg);
	if (t_msg->Command == ROVERCOMMAND_RUNDIAGNOSTIC)
	{
		if (t_msg->Option1 == LEVEL1)
		{
			diaglist.push_back(diag);
		}
		else if (t_msg->Option1 == LEVEL2)
		{
			diaglist = check_programvariables();
			return diaglist;
		}
		else if (t_msg->Option1 == LEVEL3)
		{
			diaglist = run_unittest();
			return diaglist;
		}
		else if (t_msg->Option1 == LEVEL4)
		{
		}
	}
	return diaglist;
}
void BoardControllerNodeProcess::new_armedstatemsg(uint8_t t_armed_state)
{
	armed_state = t_armed_state;
}
std::vector<eros::diagnostic> BoardControllerNodeProcess::check_programvariables()
{
	std::vector<eros::diagnostic> diaglist;
	eros::diagnostic diag = root_diagnostic;
	bool status = true;

	if (status == true) {
		diag = update_diagnostic(SOFTWARE,INFO,DIAGNOSTIC_PASSED,"Checked Program Variables -> PASSED.");
		diaglist.push_back(diag);
	} else {
		diag = update_diagnostic(SOFTWARE,WARN,DIAGNOSTIC_FAILED,"Checked Program Variables -> FAILED.");
		diaglist.push_back(diag);
	}
	return diaglist;
}
std::vector<BoardControllerNodeProcess::Message> BoardControllerNodeProcess::get_querymessages_tosend()
{
	std::vector<BoardControllerNodeProcess::Message> querymessages;
	for(std::size_t i = 0; i < messages.size(); i++)
	{
		if((messages.at(i).type == "Query") and (messages.at(i).send_me == true))
		{
			querymessages.push_back(messages.at(i));
		}
	}
	return querymessages;
}
std::vector<BoardControllerNodeProcess::BoardControllerNodeProcess::Message> BoardControllerNodeProcess::get_commandmessages_tosend()
{
	std::vector<Message> commandmessages;
	for(std::size_t i = 0; i < messages.size(); i++)
	{
		if((messages.at(i).type == "Command") and (messages.at(i).send_me == true))
		{
			commandmessages.push_back(messages.at(i));
		}
	}
	return commandmessages;
}
eros::diagnostic BoardControllerNodeProcess::new_message_sent(unsigned char id)
{
	eros::diagnostic diag = root_diagnostic;
	for(std::size_t i = 0; i < messages.size(); i++)
	{
		if(messages.at(i).id == id)
		{
			messages.at(i).sent_counter++;
			messages.at(i).send_me = false;
		}
	}
	return diag;
}
eros::diagnostic BoardControllerNodeProcess::new_message_recv(unsigned char id)
{
	eros::diagnostic diag = root_diagnostic;
	for(std::size_t i = 0; i < messages.size(); i++)
	{
		if(messages.at(i).id == id)
		{
			messages.at(i).recv_counter++;
		}
	}
	return diag;
}
void BoardControllerNodeProcess::init_messages()
{
	/*
    {
        Message newmessage;
        newmessage.id = SPIMessageHandler::SPI_TestMessageCounter_ID;
        newmessage.name = "TestMessageCounter";
        newmessage.type = "Query";
        newmessage.send_me = false;
        messages.push_back(newmessage);
    }
    {
    	Message newmessage;
    	newmessage.id = SPIMessageHandler::SPI_Get_ANA_Port1_ID;
    	newmessage.name = "GetANAPort1";
    	newmessage.type = "Query";
    	newmessage.send_me = false;
    	messages.push_back(newmessage);
    }
	 */
	{
		Message newmessage;
		newmessage.id = SPIMessageHandler::SPI_Get_DIO_Port1_ID;
		newmessage.name = "GetDIOPort1";
		newmessage.type = "Query";
		newmessage.send_me = false;
		messages.push_back(newmessage);
	}

	{
		Message newmessage;
		newmessage.id = SPIMessageHandler::SPI_LEDStripControl_ID;
		newmessage.name = "LEDStripControl";
		newmessage.type = "Command";
		newmessage.send_me = false;
		messages.push_back(newmessage);
	}
	{
		Message newmessage;
		newmessage.id = SPIMessageHandler::SPI_Diagnostic_ID;
		newmessage.name = "Diagnostic";
		newmessage.type = "Query";
		newmessage.send_me = false;
		messages.push_back(newmessage);
	}
	{
		Message newmessage;
		newmessage.id = SPIMessageHandler::SPI_Command_ID;
		newmessage.name = "Command";
		newmessage.type = "Command";
		newmessage.send_me = false;
		messages.push_back(newmessage);
	}
	{
		Message newmessage;
		newmessage.id = SPIMessageHandler::SPI_Arm_Status_ID;
		newmessage.name = "ArmStatus";
		newmessage.type = "Command";
		newmessage.send_me = false;
		messages.push_back(newmessage);
	}
	for(std::size_t i = 0; i < messages.size(); i++)
	{
		messages.at(i).sent_counter = 0;
		messages.at(i).recv_counter = 0;
		messages.at(i).sent_rate = 0.0;
		messages.at(i).recv_rate = 0.0;
	}
}
eros::diagnostic BoardControllerNodeProcess::get_LEDStripControlParameters
(unsigned char& Mode,unsigned char& Param1,unsigned char& Param2)
{
	eros::diagnostic diag = root_diagnostic;
	Mode = LEDPixelMode;
	Param1 = 0;
	Param2 = 0;
	return diag;
}
eros::diagnostic BoardControllerNodeProcess::send_commandmessage(unsigned char id)
{
	eros::diagnostic diag = root_diagnostic;
	for(std::size_t i = 0; i <messages.size(); i++)
	{
		if((messages.at(i).id == id) && (messages.at(i).type == "Command"))
		{
			messages.at(i).send_me = true;
			diag.Diagnostic_Type = COMMUNICATIONS;
			diag.Level = INFO;
			diag.Diagnostic_Message = NOERROR;
			diag = update_diagnostic(COMMUNICATIONS,INFO,NOERROR,"Processed Command Message.");
			return diag;
		}
	}
	char tempstr[255];
	sprintf(tempstr,"Couldn't send Command Message: AB%0X",id);
	diag = update_diagnostic(COMMUNICATIONS,WARN,DROPPING_PACKETS,std::string(tempstr));
	return diag;
}

eros::diagnostic BoardControllerNodeProcess::send_querymessage(unsigned char id)
{
	eros::diagnostic diag = root_diagnostic;
	for(std::size_t i = 0; i <messages.size(); i++)
	{
		if((messages.at(i).id == id) && (messages.at(i).type == "Query"))
		{
			messages.at(i).send_me = true;
			diag = update_diagnostic(COMMUNICATIONS,INFO,NOERROR,"Processed Query Message.");
			return diag;
		}
	}
	char tempstr[255];
	sprintf(tempstr,"Couldn't send Query Message: AB%0X",id);
	diag = update_diagnostic(COMMUNICATIONS,WARN,DROPPING_PACKETS,std::string(tempstr));
	return diag;
}
std::string BoardControllerNodeProcess::get_messageinfo(bool v)
{
	char tempstr[2048];
	sprintf(tempstr,"%s","");

	for(std::size_t i = 0; i < messages.size(); i++)
	{
		if((v == true) || (messages.at(i).sent_counter > 0) || (messages.at(i).recv_counter > 0))
		{
			sprintf(tempstr,"%s\nMessage: %s(0xAB%0X) Sent: %d @%0.2f Hz Recv: %d @%0.2f Hz",tempstr,
					messages.at(i).name.c_str(),
					messages.at(i).id,
					messages.at(i).sent_counter,
					messages.at(i).sent_rate,
					messages.at(i).recv_counter,
					messages.at(i).recv_rate);
		}
	}
	return std::string(tempstr);
}
eros::diagnostic BoardControllerNodeProcess::new_message_TestMessageCounter(uint8_t boardid,unsigned char v1,unsigned char v2,unsigned char v3,unsigned char v4,
		unsigned char v5,unsigned char v6,unsigned char v7,unsigned char v8,
		unsigned char v9,unsigned char v10,unsigned char v11,unsigned char v12)
{
	eros::diagnostic diag = root_diagnostic;
	char tempstr[256];
	sprintf(tempstr,"Process Message: AB%0X Not Supported",SPIMessageHandler::SPI_TestMessageCounter_ID);
	diag = update_diagnostic(COMMUNICATIONS,WARN,DROPPING_PACKETS,std::string(tempstr));
	return diag;
}
eros::diagnostic BoardControllerNodeProcess::new_message_GetDIOPort1(uint8_t boardid,double tov,
		int16_t v1,int16_t v2)
{
	int16_t v[6] = {v1,v2};
	eros::diagnostic diag = root_diagnostic;
	eros::device board = find_board(boardid);
	eros::device::ConstPtr board_ptr(new eros::device(board));
	if(board.DeviceName == "")
	{
		char tempstr[255];
		sprintf(tempstr,"Board ID: %d Not Found\n",boardid);
		diag = update_diagnostic(COMMUNICATIONS,ERROR,DROPPING_PACKETS,std::string(tempstr));
		return diag;
	}
	for(uint8_t i = 0; i < 2; i++)
	{
		eros::pin pin = find_pin(board_ptr,"QuadratureEncoderInput",i);
		eros::pin::ConstPtr pin_ptr(new eros::pin(pin));
		if(pin.Name != "")
		{
			if(update_sensor(board_ptr,pin_ptr,tov,(double)v[i]) == false)
			{
			}
		}
	}
	diag = update_diagnostic(COMMUNICATIONS,INFO,NOERROR,"Updated.");
	return diag;
}
eros::diagnostic BoardControllerNodeProcess::new_message_Diagnostic(uint8_t boardid,
		unsigned char System,unsigned char SubSystem,
		unsigned char Component,unsigned char Diagnostic_Type,
		unsigned char Level,unsigned char Message)
{
	eros::diagnostic diag = root_diagnostic;
	eros::diagnostic worst_diagnostic;
	worst_diagnostic.Level = DEBUG;
	eros::device board = find_board(boardid);
	if(board.DeviceName == "")
	{
		char tempstr[255];
		sprintf(tempstr,"Board ID: %d Not Found\n",boardid);
		diag = update_diagnostic(COMMUNICATIONS,ERROR,DROPPING_PACKETS,std::string(tempstr));
		return diag;
	}
	diag = update_diagnostic(board.DeviceName,COMMUNICATIONS,Level,Message,"");
	for(std::size_t i = 0; i < boards.size(); ++i)
	{
		if(boards.at(i).device.DeviceName == board.DeviceName)
		{
			boards.at(i).lasttime_rx = run_time;
		}
	}
	return diag;
}
eros::diagnostic BoardControllerNodeProcess::new_message_GetANAPort1(uint8_t boardid,double tov,uint16_t v1,uint16_t v2,uint16_t v3,
		uint16_t v4,uint16_t v5,uint16_t v6)
{
	uint16_t v[6] = {v1,v2,v3,v4,v5,v6};
	eros::diagnostic diag = root_diagnostic;
	eros::device board = find_board(boardid);
	eros::device::ConstPtr board_ptr(new eros::device(board));
	if(board.DeviceName == "")
	{
		char tempstr[255];
		sprintf(tempstr,"Board ID: %d Not Found\n",boardid);
		diag = update_diagnostic(COMMUNICATIONS,ERROR,DROPPING_PACKETS,std::string(tempstr));
		return diag;
	}
	for(uint8_t i = 0; i < 6; i++)
	{
		eros::pin pin = find_pin(board_ptr,"AnalogInput",i);
		eros::pin::ConstPtr pin_ptr(new eros::pin(pin));
		if(pin.Name != "")
		{
			if(update_sensor(board_ptr,pin_ptr,tov,(double)v[i]) == false)
			{

			}
		}
	}
	char tempstr[255];
	sprintf(tempstr,"Updated");
	diag = update_diagnostic(board.DeviceName,COMMUNICATIONS,INFO,NOERROR,"Updated.");
	return diag;


}
bool BoardControllerNodeProcess::board_present(const eros::device::ConstPtr& device)
{
	for(std::size_t i = 0; i < boards.size(); i++)
	{
		if((boards.at(i).device.DeviceName == device->DeviceName))
		{
			return true;
		}
	}
	return false;
}
std::string BoardControllerNodeProcess::map_PinFunction_ToString(int function)
{
	switch(function)
	{
	case PINMODE_UNDEFINED:						return "Undefined";					break;
	case PINMODE_DIGITAL_OUTPUT: 				return "DigitalOutput"; 			break;
	case PINMODE_DIGITAL_INPUT: 				return "DigitalInput"; 				break;
	case PINMODE_ANALOG_INPUT: 					return "AnalogInput"; 				break;
	case PINMODE_ULTRASONIC_INPUT: 				return "UltraSonicSensorInput"; 	break;
	case PINMODE_QUADRATUREENCODER_INPUT: 		return "QuadratureEncoderInput";	break;
	case PINMODE_PWM_OUTPUT: 					return "PWMOutput"; 				break;
	case PINMODE_PWM_OUTPUT_NON_ACTUATOR:		return "PWMOutput-NonActuator";		break;
	case PINMODE_DIGITAL_OUTPUT_NON_ACTUATOR:	return "DigitalOutput-NonActuator"; break;
	default: 									return ""; 							break;
	}
}
int BoardControllerNodeProcess::map_PinFunction_ToInt(std::string Function)
{
	if(Function == "DigitalInput")						{	return PINMODE_DIGITAL_INPUT;				}
	else if(Function == "DigitalOutput")				{	return PINMODE_DIGITAL_OUTPUT;				}
	else if(Function == "AnalogInput")					{	return PINMODE_ANALOG_INPUT;				}
	else if(Function == "UltraSonicSensorInput")		{	return PINMODE_ULTRASONIC_INPUT;			}
	else if(Function == "PWMOutput")					{	return PINMODE_PWM_OUTPUT;					}
	else if(Function == "PWMOutput-NonActuator") 		{ 	return PINMODE_PWM_OUTPUT_NON_ACTUATOR; 	}
	else if(Function == "DigitalOutput-NonActuator")	{ 	return PINMODE_DIGITAL_OUTPUT_NON_ACTUATOR;	}
	else 												{ 	return PINMODE_UNDEFINED; 					}
}
bool BoardControllerNodeProcess::find_capability(std::vector<std::string> capabilities,std::string name)
{
	for(std::size_t i = 0; i < capabilities.size(); i++)
	{
		if(capabilities.at(i) == name)
		{
			return true;
		}
	}
	return false;
}
BoardControllerNodeProcess::Sensor BoardControllerNodeProcess::find_sensor(std::string name)
{
	for(std::size_t i = 0; i < sensors.size(); i++)
	{
		if(sensors.at(i).name == name)
		{
			return sensors.at(i);
		}
	}
	Sensor empty_sensor;
	empty_sensor.name = "";
	empty_sensor.initialized = false;
	empty_sensor.type = "";
	return empty_sensor;
}
bool BoardControllerNodeProcess::update_sensorinfo(Sensor sensor)
{
	for(std::size_t i = 0; i < sensors.size(); i++)
	{
		if(sensors.at(i).name == sensor.name)
		{
			sensors.at(i) = sensor;
			return true;
		}
	}
	return false;
}
bool BoardControllerNodeProcess::sensors_initialized()
{
	bool sensors_init = true;
	if((sensors.size() == 0) and (mydevice.SensorCount == 0))
	{
		return true;
	}
	for(std::size_t i=0; i < sensors.size(); i++)
	{
		if(sensors.at(i).initialized == false)
		{
			sensors_init = false;
		}
	}
	return sensors_init;
}
bool BoardControllerNodeProcess::update_sensor(const eros::device::ConstPtr& board,const eros::pin::ConstPtr& pin,double tov,double value)
{
	for(std::size_t i = 0; i < sensors.size(); i++)
	{
		if((sensors.at(i).connected_board.DeviceName == board->DeviceName) and
				(sensors.at(i).connected_pin.Name == pin->Name))
		{
			sensors.at(i).signal.tov = tov;
			sensors.at(i).signal.status = SIGNALSTATE_UPDATED;
			if(sensors.at(i).convert == false)
			{
				sensors.at(i).signal.value = value*sensors.at(i).conversion_factor;
			}
			else
			{
				sensors.at(i).signal.value = map_input_to_output(value,sensors.at(i).min_inputvalue*sensors.at(i).conversion_factor,
						sensors.at(i).max_inputvalue*sensors.at(i).conversion_factor,
						sensors.at(i).min_inputvalue*sensors.at(i).conversion_factor,
						sensors.at(i).max_outputvalue*sensors.at(i).conversion_factor);
			}
			return true;
		}
	}
	return false;
}
eros::device BoardControllerNodeProcess::find_board(uint8_t boardid)
{
	for(std::size_t i = 0; i < boards.size(); i++)
	{
		if(boards.at(i).device.ID == boardid)
		{
			return boards.at(i).device;
		}
	}
	eros::device empty_device;
	empty_device.DeviceName = "";
	empty_device.DeviceType = "";
	return empty_device;
}
eros::pin BoardControllerNodeProcess::find_pin(const eros::device::ConstPtr& board,std::string pinfunction,uint8_t pinnumber)
{
	for(std::size_t i = 0; i < board->pins.size(); i++)
	{
		if((board->pins.at(i).Function == pinfunction) and (board->pins.at(i).Number == pinnumber))
		{
			return board->pins.at(i);
		}
	}
	eros::pin empty_pin;
	empty_pin.Name = "";
	empty_pin.Function = "";
	return empty_pin;
}
eros::pin find_pin(const eros::device::ConstPtr& board,std::string pinname)
{
	for(std::size_t i = 0; i < board->pins.size(); i++)
	{
		if((board->pins.at(i).ConnectedDevice == pinname))
		{
			return board->pins.at(i);
		}
	}
	eros::pin empty_pin;
	empty_pin.Name = "";
	empty_pin.Function = "";
	return empty_pin;
}

bool BoardControllerNodeProcess::load_sensorinfo(std::string name)
{
	bool loaded = false;
	std::string sensor_folder = "/home/robot/config/sensors/" + name;
	std::string sensor_descriptor = "/home/robot/config/sensors/" + name + "/" + name + ".xml";
	TiXmlDocument sensor_doc(sensor_descriptor);
	bool sensorfile_loaded = sensor_doc.LoadFile();
	if(sensorfile_loaded == true)
	{
		if(parse_sensorfile(sensor_doc,name) == true)
		{
			return true;
		}
		else
		{
			printf("Could not parse Sensor File: %s\n",sensor_descriptor.c_str());
			return false;
		}
	}
	else
	{
		printf("Could not load Sensor File: %s\n",sensor_descriptor.c_str());
		return false;
	}
}
bool BoardControllerNodeProcess::parse_sensorfile(TiXmlDocument doc,std::string name)
{
	int sensor_index = -1;
	for(std::size_t i = 0; i < sensors.size(); i++)
	{
		if(sensors.at(i).name == name)
		{
			sensor_index = i;
		}
	}
	if(sensor_index < 0)
	{
		printf("Did not find Sensor: %s\n",name.c_str());
		return false;
	}
	TiXmlElement *l_pRootElement = doc.RootElement();

	if( NULL != l_pRootElement )
	{
		bool convert = true;
		TiXmlElement *l_pSensorType = l_pRootElement->FirstChildElement( "Type" );
		if(NULL != l_pSensorType)
		{
			sensors.at(sensor_index).type = l_pSensorType->GetText();
		}
		else { printf("Element: Type not found.\n"); return false; }
		if((sensors.at(sensor_index).type == "Potentiometer") or
				(sensors.at(sensor_index).type == "QuadratureEncoder"))
		{

		}
		else
		{
			printf("Sensor Type: %s Not Supported.\n",sensors.at(sensor_index).type.c_str());
			return false;
		}

		TiXmlElement *l_pOutputDataType = l_pRootElement->FirstChildElement( "OutputDataType" );
		if(NULL != l_pSensorType)
		{
			sensors.at(sensor_index).output_datatype = l_pOutputDataType->GetText();
		}
		else { printf("Element: OutputDataType not found.\n"); return false; }
		if((sensors.at(sensor_index).output_datatype == "signal"))

		{

		}
		else
		{
			printf("Sensor OutputDataType: %s Not Supported.\n",sensors.at(sensor_index).output_datatype.c_str());
			return false;
		}

		TiXmlElement *l_pMinInput = l_pRootElement->FirstChildElement( "MinInputValue" );
		if(NULL != l_pMinInput)
		{
			sensors.at(sensor_index).convert = true;
			sensors.at(sensor_index).min_inputvalue = std::atof(l_pMinInput->GetText());
		}
		else { printf("Sensor: %s Element: MinInputValue not found.\n",sensors.at(sensor_index).name.c_str()); convert = false; }

		TiXmlElement *l_pMaxInput = l_pRootElement->FirstChildElement( "MaxInputValue" );
		if(NULL != l_pMaxInput)
		{
			sensors.at(sensor_index).convert = true;
			sensors.at(sensor_index).max_inputvalue = std::atof(l_pMaxInput->GetText());
		}
		else { printf("Sensor: %s Element: MaxInputValue not found.\n",sensors.at(sensor_index).name.c_str()); convert = false; }

		TiXmlElement *l_pMinOutput = l_pRootElement->FirstChildElement( "MinOutputValue" );
		if(NULL != l_pMinOutput)
		{
			sensors.at(sensor_index).convert = true;
			sensors.at(sensor_index).min_outputvalue = std::atof(l_pMinOutput->GetText());
		}
		else { printf("Sensor: %s Element: MinOutputValue not found.\n",sensors.at(sensor_index).name.c_str()); convert = false; }

		TiXmlElement *l_pMaxOutput = l_pRootElement->FirstChildElement( "MaxOutputValue" );
		if(NULL != l_pMaxOutput)
		{
			sensors.at(sensor_index).convert = true;
			sensors.at(sensor_index).max_outputvalue = std::atof(l_pMaxOutput->GetText());
		}
		else { printf("Sensor: %s Element: MaxOutputValue not found.\n",sensors.at(sensor_index).name.c_str()); convert = false; }

		TiXmlElement *l_pUnits = l_pRootElement->FirstChildElement( "Units" );
		if(NULL != l_pUnits)
		{
			double conversion_factor = 1.0;
			sensors.at(sensor_index).signal.type = convert_signaltype(l_pUnits->GetText(),&conversion_factor);
			sensors.at(sensor_index).conversion_factor = conversion_factor;
		}
		else { printf("Sensor: %s Element: Units not found.\n",sensors.at(sensor_index).name.c_str()); return false; }

		sensors.at(sensor_index).convert = convert;
		if((sensors.at(sensor_index).output_datatype == "signal"))
		{

		}
		else
		{
			printf("Sensor OutputDataType: %s Not Supported.\n",sensors.at(sensor_index).output_datatype.c_str());
			return false;
		}
		sensors.at(sensor_index).initialized = true;
	}
	else {	printf("Element: Sensor not found.\n"); return false; }
	return true;
}
double BoardControllerNodeProcess::map_input_to_output(double input_value,double min_input,double max_input,double min_output,double max_output)
{
	double m = (max_output-min_output)/(max_input-min_input);
	//y-y1 = m*(x-x1);
	double out = (m*(input_value-min_input))+min_output;
	if(max_output >= min_output)
	{
		if(out > max_output) { out = max_output; }
		if(out < min_output) { out = min_output; }
	}
	else// if(min_output > max_output)
	{
		if(out > min_output) { out = min_output; }
		if(out < max_output) { out = max_output; }
	}
	return out;
}
