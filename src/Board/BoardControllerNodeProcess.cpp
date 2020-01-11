#include "BoardControllerNodeProcess.h"
bool BoardControllerNodeProcess::initialize_supportedboards()
{
	supported_partnumbers.push_back(PN_100005);
	{ //BOARD: ARDUINOBOARD PN: 100005
		BoardControllerNodeProcess::BoardMap board;
		board.FAST_PN = PN_100005;
		board.DeviceType = DEVICETYPE_ARDUINOBOARD;
		//DIO PORT1 MAX SIZE=2
		board.PinMap.push_back(create_pindefinition("DIO1-0", PORT_DIGIPORT_1, 0));
		board.PinMap.push_back(create_pindefinition("DIO1-1", PORT_DIGIPORT_1, 1));

		//ANA PORT1 MAX SIZE=6
		board.PinMap.push_back(create_pindefinition("A0", PORT_ANAPORT_1, 0));
		board.PinMap.push_back(create_pindefinition("A1", PORT_ANAPORT_1, 1));
		board.PinMap.push_back(create_pindefinition("A2", PORT_ANAPORT_1, 2));
		board.PinMap.push_back(create_pindefinition("A3", PORT_ANAPORT_1, 3));
		board.PinMap.push_back(create_pindefinition("A4", PORT_ANAPORT_1, 4));
		board.PinMap.push_back(create_pindefinition("A5", PORT_ANAPORT_1, 5));

		//ANA PORT2 MAX SIZE=6
		board.PinMap.push_back(create_pindefinition("A6", PORT_ANAPORT_2, 0));
		board.PinMap.push_back(create_pindefinition("A7", PORT_ANAPORT_2, 1));
		board.PinMap.push_back(create_pindefinition("A8", PORT_ANAPORT_2, 2));
		board.PinMap.push_back(create_pindefinition("A9", PORT_ANAPORT_2, 3));
		board.PinMap.push_back(create_pindefinition("A10", PORT_ANAPORT_2, 4));
		board.PinMap.push_back(create_pindefinition("A11", PORT_ANAPORT_2, 5));
		supported_boards.push_back(board);
	}
	return true;
}
eros::diagnostic BoardControllerNodeProcess::finish_initialization()
{
	eros::diagnostic diag = root_diagnostic;
	init_messages();
	reset();
	current_command.Command = ROVERCOMMAND_NONE;
	LEDPixelMode = LEDPIXELMODE_ERROR;
	encoder_count = 0;
	bool status = initialize_supportedboards();
	if (status == false)
	{
		diag = update_diagnostic(DATA_STORAGE, ERROR, INITIALIZING_ERROR, "Unable to initialize Supported Boards.");
	}
	else
	{
		diag = update_diagnostic(DATA_STORAGE, INFO, NOERROR, "No Error.");
	}
	diag = update_diagnostic(COMMUNICATIONS, INFO, NOERROR, "No Error."); //Using Boards for Device specific comm diagnostics
	diag = update_diagnostic(SENSORS, INFO, NOERROR, "No Error.");		  //Using Sensors for Device specific sensor diagnostics
	return diag;
}
eros::diagnostic BoardControllerNodeProcess::update(double t_dt, double t_ros_time)
{
	eros::diagnostic diag = root_diagnostic;
	LEDPixelMode = LEDPIXELMODE_NORMAL;
	if (task_state == TASKSTATE_PAUSE)
	{
	}
	else if (task_state == TASKSTATE_RESET)
	{
		bool v = request_statechange(TASKSTATE_RUNNING);
		if (v == false)
		{
			diag = update_diagnostic(SOFTWARE, ERROR, DIAGNOSTIC_FAILED,
									 "Unallowed State Transition: From: " + map_taskstate_tostring(task_state) + " To: " + map_taskstate_tostring(TASKSTATE_RUNNING));
		}
	}
	else if (task_state == TASKSTATE_INITIALIZED)
	{
		diag = update_diagnostic(DATA_STORAGE, NOTICE, INITIALIZING, "Node Initializing");
	}
	else if (task_state == TASKSTATE_RUNNING)
	{
	}
	else if (task_state != TASKSTATE_RUNNING)
	{
		bool v = request_statechange(TASKSTATE_RUNNING);
		if (v == false)
		{
			diag = update_diagnostic(SOFTWARE, ERROR, DIAGNOSTIC_FAILED,
									 "Unallowed State Transition: From: " + map_taskstate_tostring(task_state) + " To: " + map_taskstate_tostring(TASKSTATE_RUNNING));
		}
	}
	diag = update_baseprocess(t_dt, t_ros_time);
	bool boards_ready = true;
	for (std::size_t i = 0; i < boards_running.size(); i++)
	{
		if (boards_running.at(i) == true)
		{
			boards_ready = boards_ready and true;
		}
		else
		{
			boards_ready = false;
		}

		if (((run_time - boards.at(i).lasttime_rx) > BOARDCOMM_TIMEOUT_WARN) and
			((run_time - boards.at(i).lasttime_rx) < BOARDCOMM_TIMEOUT_ERROR))
		{
			char tempstr[512];
			sprintf(tempstr, "Have not had comm with Board: %s in %4.2f Seconds.  Timing out Soon.",
					boards.at(i).device.DeviceName.c_str(), run_time - boards.at(i).lasttime_rx);
			diag = update_diagnostic(boards.at(i).device.DeviceName, COMMUNICATIONS, WARN, DROPPING_PACKETS, std::string(tempstr));
			ready_to_arm = false;
		}
		else if (((run_time - boards.at(i).lasttime_rx) > BOARDCOMM_TIMEOUT_ERROR))
		{
			char tempstr[512];
			sprintf(tempstr, "Have not had comm with Board: %s in %4.2f Seconds.  Disarming.",
					boards.at(i).device.DeviceName.c_str(), (run_time - boards.at(i).lasttime_rx));
			diag = update_diagnostic(boards.at(i).device.DeviceName, COMMUNICATIONS, ERROR, DROPPING_PACKETS, std::string(tempstr));
			ready_to_arm = false;
		}
		else
		{
			diag = update_diagnostic(boards.at(i).device.DeviceName, COMMUNICATIONS, INFO, NOERROR, "Last Comm with with Board: " + boards.at(i).device.DeviceName + " " + std::to_string(run_time - boards.at(i).lasttime_rx) + " seconds ago.");
		}
	}

	if (boards_running.size() == 0)
	{
		boards_ready = false;
	}
	for (std::size_t i = 0; i < messages.size(); i++)
	{
		if (messages.at(i).sent_counter > 0)
		{
			messages.at(i).sent_rate = (double)(messages.at(i).sent_counter) / run_time;
		}

		if (messages.at(i).recv_counter > 0)
		{
			messages.at(i).recv_rate = (double)(messages.at(i).recv_counter) / run_time;
		}
	}
	bool status = true;
	for (std::size_t i = 0; i < diagnostics.size(); ++i)
	{
		if ((diagnostics.at(i).Level >= WARN) or (diagnostics.at(i).Diagnostic_Message == INITIALIZING))
		{
			status = false;
		}
	}
	if (boards_ready == true)
	{
		status = status and true;
		if (status == true)
		{
			diag = update_diagnostic(SOFTWARE, INFO, NOERROR, "Node Running.");
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
		if (mydevice.BoardCount == 0)
		{
			char tempstr[512];
			sprintf(tempstr, "This node requires at least 1 Board, but 0 are defined.");
			diag = update_diagnostic(DATA_STORAGE, ERROR, INITIALIZING_ERROR, std::string(tempstr));
		}
		else
		{
			char tempstr[512];
			sprintf(tempstr, "All info for Boards not received yet.  Expected Board Count: %d", mydevice.BoardCount);
			diag = update_diagnostic(DATA_STORAGE, NOTICE, INITIALIZING, std::string(tempstr));
		}
	}
	return diag;
}
eros::diagnostic BoardControllerNodeProcess::new_devicemsg(const eros::device::ConstPtr &t_newdevice)
{
	eros::diagnostic diag = root_diagnostic;

	if (task_state == TASKSTATE_INITIALIZED)
	{
		if (t_newdevice->DeviceParent == host_name)
		{
			if (board_present(t_newdevice) == true)
			{
				char tempstr[512];
				sprintf(tempstr, "Board: %s already loaded.",
						t_newdevice->DeviceName.c_str());
				diag = update_diagnostic(DATA_STORAGE, WARN, INITIALIZING_ERROR, std::string(tempstr));
				return diag;
			}
			std::size_t board_message = t_newdevice->DeviceType.find("Board");
			if ((board_message != std::string::npos))
			{
				eros::device board_device = convert_fromptr(t_newdevice);
				if (t_newdevice->DeviceType == "ArduinoBoard")
				{
					for (std::size_t i = 0; i < t_newdevice->pins.size(); ++i)
					{
						uint8_t function = map_PinFunction_ToInt(t_newdevice->pins.at(i).Function);
						if (function == PINMODE_UNDEFINED)
						{
							diag = update_diagnostic(DATA_STORAGE, ERROR, INITIALIZING_ERROR, "Board Type: " + t_newdevice->DeviceType + " Pin Function: " + t_newdevice->pins.at(i).Function + " Not Supported.");
							return diag;
						}
						else if ((t_newdevice->pins.at(i).ConnectedSensor != ""))
						{
							Sensor new_sensor;
							new_sensor.initialized = false;
							new_sensor.signal.value = 0.0;
							new_sensor.name = t_newdevice->pins.at(i).ConnectedSensor;
							new_sensor.connected_board = board_device;
							new_sensor.connected_pin = t_newdevice->pins.at(i);
							new_sensor.signal.status = SIGNALSTATE_UNDEFINED;
							if (t_newdevice->pins.at(i).Function == "QuadratureEncoderInput")
							{
								new_sensor.signal.type = SIGNALTYPE_TICKSPEED;
							}
							diag = update_diagnostic(new_sensor.name, DATA_STORAGE, NOTICE, NOERROR, "Sensor Initialized.");
							sensors.push_back(new_sensor);
							if (load_sensorinfo(new_sensor.name) == false)
							{
								char tempstr[512];
								sprintf(tempstr, "Unable to load info for Sensor: %s", new_sensor.name.c_str());
								diag = update_diagnostic(DATA_STORAGE, ERROR, INITIALIZING_ERROR, std::string(tempstr));
								return diag;
							}
						}
					}
					Board board;
					board.device = board_device;
					board.lasttime_rx = 0.0;
					boards.push_back(board);
					diag = update_diagnostic(board_device.DeviceName, DATA_STORAGE, NOTICE, NOERROR, "No Error.");
					diag = update_diagnostic(board_device.DeviceName, COMMUNICATIONS, NOTICE, NOERROR, "No Error.");
					boards_running.push_back(true);
				}
				else
				{
					char tempstr[512];
					sprintf(tempstr, "Device Type: %s Not supported.", t_newdevice->DeviceType.c_str());
					diag = update_diagnostic(DATA_STORAGE, ERROR, INITIALIZING_ERROR, std::string(tempstr));
					return diag;
				}
			}
		}
	}
	else
	{
	}

	if (((uint16_t)(boards.size()) == mydevice.BoardCount) and
		(sensors_initialized() == true))
	{
		request_statechange(TASKSTATE_RUNNING);
		diag = update_diagnostic(DATA_STORAGE, INFO, NOERROR, "All Devices Initialized and Running.");
	}
	return diag;
}
std::vector<eros::diagnostic> BoardControllerNodeProcess::new_commandmsg(const eros::command::ConstPtr &t_msg)
{
	std::vector<eros::diagnostic> diaglist;
	eros::diagnostic diag = root_diagnostic;
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
	else if (t_msg->Command == ROVERCOMMAND_TASKCONTROL)
	{
		if (node_name.find(t_msg->CommandText) != std::string::npos)
		{
			uint8_t prev_taskstate = get_taskstate();
			bool v = request_statechange(t_msg->Option2);
			if (v == false)
			{
				diag = update_diagnostic(SOFTWARE, ERROR, DIAGNOSTIC_FAILED,
										 "Unallowed State Transition: From: " + map_taskstate_tostring(prev_taskstate) + " To: " + map_taskstate_tostring(t_msg->Option2));
				diaglist.push_back(diag);
			}
			else
			{
				if (task_state == TASKSTATE_RESET)
				{
					reset();
				}
				diag = update_diagnostic(SOFTWARE, NOTICE, DIAGNOSTIC_PASSED,
										 "Commanded State Transition: From: " + map_taskstate_tostring(prev_taskstate) + " To: " + map_taskstate_tostring(t_msg->Option2));
				diaglist.push_back(diag);
			}
		}
	}
	for (std::size_t i = 0; i < diaglist.size(); ++i)
	{
		diag = update_diagnostic(diaglist.at(i));
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

	if (status == true)
	{
		diag = update_diagnostic(SOFTWARE, INFO, DIAGNOSTIC_PASSED, "Checked Program Variables -> PASSED.");
		diaglist.push_back(diag);
	}
	else
	{
		diag = update_diagnostic(SOFTWARE, WARN, DIAGNOSTIC_FAILED, "Checked Program Variables -> FAILED.");
		diaglist.push_back(diag);
	}
	return diaglist;
}
std::vector<BoardControllerNodeProcess::Message> BoardControllerNodeProcess::get_querymessages_tosend()
{
	std::vector<BoardControllerNodeProcess::Message> querymessages;
	for (std::size_t i = 0; i < messages.size(); i++)
	{
		if ((messages.at(i).type == "Query") and (messages.at(i).send_me == true))
		{
			querymessages.push_back(messages.at(i));
		}
	}
	return querymessages;
}
std::vector<BoardControllerNodeProcess::BoardControllerNodeProcess::Message> BoardControllerNodeProcess::get_commandmessages_tosend()
{
	std::vector<Message> commandmessages;
	for (std::size_t i = 0; i < messages.size(); i++)
	{
		if ((messages.at(i).type == "Command") and (messages.at(i).send_me == true))
		{
			commandmessages.push_back(messages.at(i));
		}
	}
	return commandmessages;
}
eros::diagnostic BoardControllerNodeProcess::new_message_sent(unsigned char id)
{
	eros::diagnostic diag = root_diagnostic;
	for (std::size_t i = 0; i < messages.size(); i++)
	{
		if (messages.at(i).id == id)
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
	for (std::size_t i = 0; i < messages.size(); i++)
	{
		if (messages.at(i).id == id)
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
	for (std::size_t i = 0; i < messages.size(); i++)
	{
		messages.at(i).sent_counter = 0;
		messages.at(i).recv_counter = 0;
		messages.at(i).sent_rate = 0.0;
		messages.at(i).recv_rate = 0.0;
	}
}
eros::diagnostic BoardControllerNodeProcess::get_LEDStripControlParameters(unsigned char &Mode, unsigned char &Param1, unsigned char &Param2)
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
	for (std::size_t i = 0; i < messages.size(); i++)
	{
		if ((messages.at(i).id == id) && (messages.at(i).type == "Command"))
		{
			messages.at(i).send_me = true;
			diag.Diagnostic_Type = COMMUNICATIONS;
			diag.Level = INFO;
			diag.Diagnostic_Message = NOERROR;
			diag = update_diagnostic(COMMUNICATIONS, INFO, NOERROR, "Processed Command Message.");
			return diag;
		}
	}
	char tempstr[255];
	sprintf(tempstr, "Couldn't send Command Message: AB%0X", id);
	diag = update_diagnostic(COMMUNICATIONS, WARN, DROPPING_PACKETS, std::string(tempstr));
	return diag;
}

eros::diagnostic BoardControllerNodeProcess::send_querymessage(unsigned char id)
{
	eros::diagnostic diag = root_diagnostic;
	for (std::size_t i = 0; i < messages.size(); i++)
	{
		if ((messages.at(i).id == id) && (messages.at(i).type == "Query"))
		{
			messages.at(i).send_me = true;
			diag = update_diagnostic(COMMUNICATIONS, INFO, NOERROR, "Processed Query Message.");
			return diag;
		}
	}
	char tempstr[255];
	sprintf(tempstr, "Couldn't send Query Message: AB%0X", id);
	diag = update_diagnostic(COMMUNICATIONS, WARN, DROPPING_PACKETS, std::string(tempstr));
	return diag;
}
std::string BoardControllerNodeProcess::get_messageinfo(bool v)
{
	char tempstr[2048];
	sprintf(tempstr, "%s", "");

	for (std::size_t i = 0; i < messages.size(); i++)
	{
		if ((v == true) || (messages.at(i).sent_counter > 0) || (messages.at(i).recv_counter > 0))
		{
			sprintf(tempstr, "%s\nMessage: %s(0xAB%0X) Sent: %d @%0.2f Hz Recv: %d @%0.2f Hz", tempstr,
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
eros::diagnostic BoardControllerNodeProcess::new_message_TestMessageCounter(
	__attribute__((unused)) uint8_t boardid,
	__attribute__((unused)) unsigned char v1,
	__attribute__((unused)) unsigned char v2,
	__attribute__((unused)) unsigned char v3,
	__attribute__((unused)) unsigned char v4,
	__attribute__((unused)) unsigned char v5,
	__attribute__((unused)) unsigned char v6,
	__attribute__((unused)) unsigned char v7,
	__attribute__((unused)) unsigned char v8,
	__attribute__((unused)) unsigned char v9,
	__attribute__((unused)) unsigned char v10,
	__attribute__((unused)) unsigned char v11,
	__attribute__((unused)) unsigned char v12)
{
	eros::diagnostic diag = root_diagnostic;
	char tempstr[256];
	sprintf(tempstr, "Process Message: AB%0X Not Supported", SPIMessageHandler::SPI_TestMessageCounter_ID);
	diag = update_diagnostic(COMMUNICATIONS, WARN, DROPPING_PACKETS, std::string(tempstr));
	return diag;
}
eros::diagnostic BoardControllerNodeProcess::new_message_GetDIOPort1(std::string device_type, uint8_t boardid, double tov,
																	 int16_t v1, int16_t v2)
{
	eros::diagnostic diag = root_diagnostic;
	if ((device_type == "ArduinoBoard"))
	{
	}
	else
	{
		diag = update_diagnostic(COMMUNICATIONS, ERROR, DROPPING_PACKETS, "DeviceType: " + device_type + " Not Supported.");
		return diag;
	}
	uint16_t port_width = 2;
	int16_t v[port_width] = {v1, v2};
	eros::device board = find_board(boardid);
	eros::device::ConstPtr board_ptr(new eros::device(board));
	if (board.DeviceName == "")
	{
		char tempstr[255];
		sprintf(tempstr, "Board ID: %d Not Found\n", boardid);
		diag = update_diagnostic(COMMUNICATIONS, ERROR, DROPPING_PACKETS, std::string(tempstr));
		return diag;
	}
	for (uint8_t i = 0; i < port_width; ++i)
	{
		eros::pin pin = find_pin(board_ptr, PORT_DIGIPORT_1, i);
		eros::pin::ConstPtr pin_ptr(new eros::pin(pin));
		if (pin.ConnectedSensor != "")
		{
			if (update_sensor(board_ptr, pin_ptr, tov, (double)v[i]) == false)
			{
				diag = update_diagnostic(board_ptr->DeviceName, COMMUNICATIONS, WARN, DROPPING_PACKETS, "Unable to Update Pin: " + pin.ConnectedSensor);
			}
		}
		if (pin.Name != "")
		{
			pin.Value = v[i];
			diag = update_pin(device_type, boardid, pin.Name, pin);
			diag = update_diagnostic(diag);
		}
	}
	return diag;
}
eros::diagnostic BoardControllerNodeProcess::update_pin(std::string device_type, uint8_t device_id, std::string pin_name, eros::pin new_pin)
{
	eros::diagnostic diag = root_diagnostic;
	bool found = false;
	for (std::size_t i = 0; i < boards.size(); ++i)
	{
		if ((boards.at(i).device.DeviceType == device_type) && (boards.at(i).device.ID == device_id))
		{
			for (std::size_t j = 0; j < boards.at(i).device.pins.size(); ++j)
			{
				if (boards.at(i).device.pins.at(j).Name == pin_name)
				{
					found = true;
					boards.at(i).device.pins.at(j) = new_pin;
				}
			}
		}
	}
	if (found == false)
	{
		diag = update_diagnostic(COMMUNICATIONS, WARN, DROPPING_PACKETS, "Unable to find Pin " + pin_name + " in Board: " + device_type + " ID: " + std::to_string(device_id));
	}
	else
	{
		diag = update_diagnostic(COMMUNICATIONS, INFO, NOERROR, "Updated Pin: " + pin_name);
	}
	return diag;
}

eros::diagnostic BoardControllerNodeProcess::new_message_Diagnostic(
	uint8_t boardid,
	__attribute__((unused)) unsigned char System,
	__attribute__((unused)) unsigned char SubSystem,
	__attribute__((unused)) unsigned char Component,
	unsigned char Diagnostic_Type,
	unsigned char Level,
	unsigned char Message)
{
	eros::diagnostic diag = root_diagnostic;
	eros::diagnostic worst_diagnostic;
	worst_diagnostic.Level = DEBUG;
	eros::device board = find_board(boardid);
	if (board.DeviceName == "")
	{
		char tempstr[255];
		sprintf(tempstr, "Board ID: %d Not Found\n", boardid);
		diag = update_diagnostic(COMMUNICATIONS, ERROR, DROPPING_PACKETS, std::string(tempstr));
		return diag;
	}
	diag = update_diagnostic(board.DeviceName, Diagnostic_Type, Level, Message, "");
	//diag = update_diagnostic(Diagnostic_Type,Level,Message,"Received from: Board: " + board.DeviceName);
	bool found = false;
	for (std::size_t i = 0; i < boards.size(); ++i)
	{
		if (boards.at(i).device.DeviceName == board.DeviceName)
		{
			found = true;
			boards.at(i).lasttime_rx = run_time;
		}
	}
	if (found == false)
	{
		diag = update_diagnostic(COMMUNICATIONS, WARN, DROPPING_PACKETS, "Could not find Board: " + board.DeviceName);
	}
	return diag;
}
eros::diagnostic BoardControllerNodeProcess::new_message_GetANAPort1(std::string device_type, uint8_t boardid, double tov, uint16_t v1, uint16_t v2, uint16_t v3,
																	 uint16_t v4, uint16_t v5, uint16_t v6)
{
	eros::diagnostic diag = root_diagnostic;
	if ((device_type == "ArduinoBoard"))
	{
	}
	else
	{
		diag = update_diagnostic(COMMUNICATIONS, ERROR, DROPPING_PACKETS, "DeviceType: " + device_type + " Not Supported.");
		return diag;
	}
	uint16_t port_width = 6;
	uint16_t v[port_width] = {v1, v2, v3, v4, v5, v6};
	eros::device board = find_board(boardid);
	eros::device::ConstPtr board_ptr(new eros::device(board));
	if (board.DeviceName == "")
	{
		char tempstr[255];
		sprintf(tempstr, "Board ID: %d Not Found\n", boardid);
		diag = update_diagnostic(COMMUNICATIONS, ERROR, DROPPING_PACKETS, std::string(tempstr));
		return diag;
	}
	for (uint8_t i = 0; i < port_width; ++i)
	{
		eros::pin pin = find_pin(board_ptr, PORT_ANAPORT_1, i);
		eros::pin::ConstPtr pin_ptr(new eros::pin(pin));
		if (pin.ConnectedSensor != "")
		{
			if (update_sensor(board_ptr, pin_ptr, tov, (double)v[i]) == false)
			{
				diag = update_diagnostic(board_ptr->DeviceName, COMMUNICATIONS, WARN, DROPPING_PACKETS, "Unable to Update Pin: " + pin.ConnectedSensor);
			}
		}
		if (pin.Name != "")
		{
			pin.Value = v[i];
			diag = update_pin(device_type, boardid, pin.Name, pin);
			diag = update_diagnostic(diag);
		}
	}
	return diag;
}
bool BoardControllerNodeProcess::board_present(const eros::device::ConstPtr &device)
{
	for (std::size_t i = 0; i < boards.size(); i++)
	{
		if ((boards.at(i).device.DeviceName == device->DeviceName))
		{
			return true;
		}
	}
	return false;
}
std::string BoardControllerNodeProcess::map_PinFunction_ToString(int function)
{
	switch (function)
	{
	case PINMODE_UNDEFINED:
		return "Undefined";
		break;
	case PINMODE_DIGITAL_OUTPUT:
		return "DigitalOutput";
		break;
	case PINMODE_DIGITAL_INPUT:
		return "DigitalInput";
		break;
	case PINMODE_ANALOG_INPUT:
		return "AnalogInput";
		break;
	case PINMODE_ULTRASONIC_INPUT:
		return "UltraSonicSensorInput";
		break;
	case PINMODE_QUADRATUREENCODER_INPUT:
		return "QuadratureEncoderInput";
		break;
	case PINMODE_PWM_OUTPUT:
		return "PWMOutput";
		break;
	case PINMODE_PWM_OUTPUT_NON_ACTUATOR:
		return "PWMOutput-NonActuator";
		break;
	case PINMODE_DIGITAL_OUTPUT_NON_ACTUATOR:
		return "DigitalOutput-NonActuator";
		break;
	default:
		return "";
		break;
	}
}
int BoardControllerNodeProcess::map_PinFunction_ToInt(std::string Function)
{
	if (Function == "DigitalInput")
	{
		return PINMODE_DIGITAL_INPUT;
	}
	else if (Function == "DigitalOutput")
	{
		return PINMODE_DIGITAL_OUTPUT;
	}
	else if (Function == "AnalogInput")
	{
		return PINMODE_ANALOG_INPUT;
	}
	else if (Function == "UltraSonicSensorInput")
	{
		return PINMODE_ULTRASONIC_INPUT;
	}
	else if (Function == "QuadratureEncoderInput")
	{
		return PINMODE_QUADRATUREENCODER_INPUT;
	}
	else if (Function == "PWMOutput")
	{
		return PINMODE_PWM_OUTPUT;
	}
	else if (Function == "PWMOutput-NonActuator")
	{
		return PINMODE_PWM_OUTPUT_NON_ACTUATOR;
	}
	else if (Function == "DigitalOutput-NonActuator")
	{
		return PINMODE_DIGITAL_OUTPUT_NON_ACTUATOR;
	}
	else
	{
		return PINMODE_UNDEFINED;
	}
}
bool BoardControllerNodeProcess::find_capability(std::vector<std::string> capabilities, std::string name)
{
	for (std::size_t i = 0; i < capabilities.size(); i++)
	{
		if (capabilities.at(i) == name)
		{
			return true;
		}
	}
	return false;
}
BoardControllerNodeProcess::Sensor BoardControllerNodeProcess::find_sensor(std::string name)
{
	for (std::size_t i = 0; i < sensors.size(); i++)
	{
		if (sensors.at(i).name == name)
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
	for (std::size_t i = 0; i < sensors.size(); i++)
	{
		if (sensors.at(i).name == sensor.name)
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
	if ((sensors.size() == 0) and (mydevice.SensorCount == 0))
	{
		return true;
	}
	for (std::size_t i = 0; i < sensors.size(); i++)
	{
		if (sensors.at(i).initialized == false)
		{
			sensors_init = false;
		}
	}
	return sensors_init;
}
bool BoardControllerNodeProcess::update_sensor(const eros::device::ConstPtr &board, const eros::pin::ConstPtr &pin, double tov, double value)
{
	for (std::size_t i = 0; i < sensors.size(); i++)
	{
		if ((sensors.at(i).connected_board.DeviceName == board->DeviceName) and
			(sensors.at(i).connected_pin.Name == pin->Name))
		{
			sensors.at(i).signal.tov = tov;
			sensors.at(i).signal.status = SIGNALSTATE_UPDATED;
			if (sensors.at(i).convert == false)
			{
				sensors.at(i).signal.value = value * sensors.at(i).conversion_factor;
			}
			else
			{
				sensors.at(i).signal.value = map_input_to_output(value, sensors.at(i).min_inputvalue * sensors.at(i).conversion_factor,
																 sensors.at(i).max_inputvalue * sensors.at(i).conversion_factor,
																 sensors.at(i).min_inputvalue * sensors.at(i).conversion_factor,
																 sensors.at(i).max_outputvalue * sensors.at(i).conversion_factor);
			}
			return true;
		}
	}
	return false;
}
eros::device BoardControllerNodeProcess::find_board(uint8_t boardid)
{
	for (std::size_t i = 0; i < boards.size(); i++)
	{
		if (boards.at(i).device.ID == boardid)
		{
			return boards.at(i).device;
		}
	}
	eros::device empty_device;
	empty_device.DeviceName = "";
	empty_device.DeviceType = "";
	return empty_device;
}
eros::pin BoardControllerNodeProcess::find_pin(const eros::device::ConstPtr &t_device, uint8_t port_id, uint8_t port_pinnumber)
{
	for (std::size_t i = 0; i < supported_boards.size(); ++i)
	{
		if (t_device->PartNumber == supported_boards.at(i).FAST_PN)
		{
			for (std::size_t j = 0; j < supported_boards.at(i).PinMap.size(); ++j)
			{
				if ((supported_boards.at(i).PinMap.at(j).port_id == port_id) &&
					(supported_boards.at(i).PinMap.at(j).port_pinnumber == port_pinnumber))
				{
					return find_pin(t_device, supported_boards.at(i).PinMap.at(j).PinName);
				}
			}
		}
	}
	eros::pin empty_pin;
	empty_pin.Name = "";
	empty_pin.Function = "";
	return empty_pin;
}
eros::pin BoardControllerNodeProcess::find_pin(const eros::device::ConstPtr &board, std::string pin_name)
{
	for (std::size_t i = 0; i < board->pins.size(); i++)
	{
		if ((board->pins.at(i).Name == pin_name))
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
	std::string sensor_folder = "/home/robot/config/sensors/" + name;
	std::string sensor_descriptor = "/home/robot/config/sensors/" + name + "/" + name + ".xml";
	TiXmlDocument sensor_doc(sensor_descriptor);
	bool sensorfile_loaded = sensor_doc.LoadFile();
	if (sensorfile_loaded == true)
	{
		if (parse_sensorfile(sensor_doc, name) == true)
		{
			return true;
		}
		else
		{
			printf("[ERROR]: Could not parse Sensor File: %s\n", sensor_descriptor.c_str());
			return false;
		}
	}
	else
	{
		printf("[ERROR]: Could not load Sensor File: %s\n", sensor_descriptor.c_str());
		return false;
	}
}
bool BoardControllerNodeProcess::parse_sensorfile(TiXmlDocument doc, std::string name)
{
	int sensor_index = -1;
	for (std::size_t i = 0; i < sensors.size(); i++)
	{
		if (sensors.at(i).name == name)
		{
			sensor_index = i;
		}
	}
	if (sensor_index < 0)
	{
		printf("[ERROR]: Did not find Sensor: %s\n", name.c_str());
		return false;
	}
	TiXmlElement *l_pRootElement = doc.RootElement();

	if (NULL != l_pRootElement)
	{
		bool convert = true;
		TiXmlElement *l_pSensorType = l_pRootElement->FirstChildElement("Type");
		if (NULL != l_pSensorType)
		{
			sensors.at(sensor_index).type = l_pSensorType->GetText();
		}
		else
		{
			printf("[ERROR]: Element: Type not found.\n");
			return false;
		}
		if ((sensors.at(sensor_index).type == "Potentiometer") or
			(sensors.at(sensor_index).type == "QuadratureEncoder"))
		{
		}
		else
		{
			printf("[ERROR]: Sensor Type: %s Not Supported.\n", sensors.at(sensor_index).type.c_str());
			return false;
		}

		TiXmlElement *l_pOutputDataType = l_pRootElement->FirstChildElement("OutputDataType");
		if (NULL != l_pSensorType)
		{
			sensors.at(sensor_index).output_datatype = l_pOutputDataType->GetText();
		}
		else
		{
			printf("[ERROR]: Element: OutputDataType not found.\n");
			return false;
		}
		if ((sensors.at(sensor_index).output_datatype == "signal"))

		{
		}
		else
		{
			printf("[ERROR]: Sensor OutputDataType: %s Not Supported.\n", sensors.at(sensor_index).output_datatype.c_str());
			return false;
		}

		TiXmlElement *l_pMinInput = l_pRootElement->FirstChildElement("MinInputValue");
		if (NULL != l_pMinInput)
		{
			sensors.at(sensor_index).convert = true;
			sensors.at(sensor_index).min_inputvalue = std::atof(l_pMinInput->GetText());
		}
		else
		{
			printf("[ERROR]: Sensor: %s Element: MinInputValue not found.\n", sensors.at(sensor_index).name.c_str());
			convert = false;
		}

		TiXmlElement *l_pMaxInput = l_pRootElement->FirstChildElement("MaxInputValue");
		if (NULL != l_pMaxInput)
		{
			sensors.at(sensor_index).convert = true;
			sensors.at(sensor_index).max_inputvalue = std::atof(l_pMaxInput->GetText());
		}
		else
		{
			printf("[ERROR]: Sensor: %s Element: MaxInputValue not found.\n", sensors.at(sensor_index).name.c_str());
			convert = false;
		}

		TiXmlElement *l_pMinOutput = l_pRootElement->FirstChildElement("MinOutputValue");
		if (NULL != l_pMinOutput)
		{
			sensors.at(sensor_index).convert = true;
			sensors.at(sensor_index).min_outputvalue = std::atof(l_pMinOutput->GetText());
		}
		else
		{
			printf("[ERROR]: Sensor: %s Element: MinOutputValue not found.\n", sensors.at(sensor_index).name.c_str());
			convert = false;
		}

		TiXmlElement *l_pMaxOutput = l_pRootElement->FirstChildElement("MaxOutputValue");
		if (NULL != l_pMaxOutput)
		{
			sensors.at(sensor_index).convert = true;
			sensors.at(sensor_index).max_outputvalue = std::atof(l_pMaxOutput->GetText());
		}
		else
		{
			printf("[ERROR]: Sensor: %s Element: MaxOutputValue not found.\n", sensors.at(sensor_index).name.c_str());
			convert = false;
		}

		TiXmlElement *l_pUnits = l_pRootElement->FirstChildElement("Units");
		if (NULL != l_pUnits)
		{
			double conversion_factor = 1.0;
			sensors.at(sensor_index).signal.type = convert_signaltype(l_pUnits->GetText(), &conversion_factor);
			sensors.at(sensor_index).conversion_factor = conversion_factor;
		}
		else
		{
			printf("[ERROR]: Sensor: %s Element: Units not found.\n", sensors.at(sensor_index).name.c_str());
			return false;
		}

		sensors.at(sensor_index).convert = convert;
		if ((sensors.at(sensor_index).output_datatype == "signal"))
		{
		}
		else
		{
			printf("[ERROR]: Sensor OutputDataType: %s Not Supported.\n", sensors.at(sensor_index).output_datatype.c_str());
			return false;
		}
		sensors.at(sensor_index).initialized = true;
	}
	else
	{
		printf("[ERROR]: Element: Sensor not found.\n");
		return false;
	}
	return true;
}
double BoardControllerNodeProcess::map_input_to_output(double input_value, double min_input, double max_input, double min_output, double max_output)
{
	double m = (max_output - min_output) / (max_input - min_input);
	//y-y1 = m*(x-x1);
	double out = (m * (input_value - min_input)) + min_output;
	if (max_output >= min_output)
	{
		if (out > max_output)
		{
			out = max_output;
		}
		if (out < min_output)
		{
			out = min_output;
		}
	}
	else // if(min_output > max_output)
	{
		if (out > min_output)
		{
			out = min_output;
		}
		if (out < max_output)
		{
			out = max_output;
		}
	}
	return out;
}
BoardControllerNodeProcess::BoardMap BoardControllerNodeProcess::get_boardmap_bypartnumber(std::string partnumber)
{
	for (std::size_t i = 0; i < supported_boards.size(); ++i)
	{
		if (supported_boards.at(i).FAST_PN == partnumber)
		{
			return supported_boards.at(i);
		}
	}
	BoardMap empty_map;
	return empty_map;
}