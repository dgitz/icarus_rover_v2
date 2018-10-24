#include "boardcontroller_node_process.h"
BoardControllerNodeProcess::BoardControllerNodeProcess()
{
	run_time = 0.0;
	init_messages();
	initialized = false;
	ready = false;
	ready_to_arm = false;
	LEDPixelMode = LEDPIXELMODE_ERROR;
	encoder_count = 0;
	imu1.id = 1;
	imu1.msg.xacc.status = SIGNALSTATE_INITIALIZING;
	imu1.msg.yacc.status = SIGNALSTATE_INITIALIZING;
	imu1.msg.zacc.status = SIGNALSTATE_INITIALIZING;

	imu1.initialized = true;
	imu1.ready = true;


	imu2.id = 2;
	imu1.msg.xacc.status = SIGNALSTATE_INITIALIZING;
	imu1.msg.yacc.status = SIGNALSTATE_INITIALIZING;
	imu1.msg.zacc.status = SIGNALSTATE_INITIALIZING;
	imu2.initialized = true;
	imu2.ready = true;
}
BoardControllerNodeProcess::~BoardControllerNodeProcess()
{

}
icarus_rover_v2::diagnostic BoardControllerNodeProcess::init(icarus_rover_v2::diagnostic indiag,std::string hostname)
{
	myhostname = hostname;
	diagnostic = indiag;
	return diagnostic;
}
icarus_rover_v2::diagnostic BoardControllerNodeProcess::update(double dt)
{
	icarus_rover_v2::diagnostic diag = diagnostic;
	run_time += dt;
	LEDPixelMode = LEDPIXELMODE_NORMAL;
	bool boards_ready = true;
	for(std::size_t i = 0; i < boards_running.size(); i++)
	{
		if(boards_running.at(i) == true) { boards_ready = boards_ready and true; }
		else
		{
			boards_ready = false;

		}
		if((run_time - board_diagnostics.at(i).lasttime_rx > 5.0) and
				(run_time - board_diagnostics.at(i).lasttime_rx < 10.0))
		{
			diag.Level = WARN;
			diag.Diagnostic_Type = COMMUNICATIONS;
			diag.Diagnostic_Message = DROPPING_PACKETS;
			char tempstr[512];
			sprintf(tempstr,"Have not had comm with Board: %d in %f Seconds.  Disarming.",
					board_diagnostics.at(i).id,run_time - board_diagnostics.at(i).lasttime_rx);
			diag.Description = std::string(tempstr);
			ready_to_arm = false;
		}
		else if((run_time - board_diagnostics.at(i).lasttime_rx > 10.0))
		{
			diag.Level = ERROR;
			diag.Diagnostic_Type = COMMUNICATIONS;
			diag.Diagnostic_Message = DROPPING_PACKETS;
			char tempstr[512];
			sprintf(tempstr,"Have not had comm with Board: %d in %f Seconds.  Disarming.",
					board_diagnostics.at(i).id,run_time - board_diagnostics.at(i).lasttime_rx);
			diag.Description = std::string(tempstr);
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
	if(boards_ready == true)
	{
		status = status and true;
		diag.Level = INFO;
		diag.Diagnostic_Type = SOFTWARE;
		diag.Diagnostic_Message = NOERROR;
		diag.Description = "Node Running";
		diagnostic = diag;
	}
	else
	{
		status = false;
		diag.Level = NOTICE;
		diag.Diagnostic_Type = SOFTWARE;
		diag.Diagnostic_Message = DEVICE_NOT_AVAILABLE;
		char tempstr[512];
		sprintf(tempstr,"All info for Boards not received yet.",mydevice.DeviceName.c_str());
		diag.Description = std::string(tempstr);
	}
	diagnostic = diag;
	return diag;
}
std::vector<Message> BoardControllerNodeProcess::get_querymessages_tosend()
{
	std::vector<Message> querymessages;
	for(std::size_t i = 0; i < messages.size(); i++)
	{
		if((messages.at(i).type == "Query") and (messages.at(i).send_me == true))
		{
			querymessages.push_back(messages.at(i));
		}
	}
	return querymessages;
}
std::vector<Message> BoardControllerNodeProcess::get_commandmessages_tosend()
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
icarus_rover_v2::diagnostic BoardControllerNodeProcess::new_message_sent(unsigned char id)
{
	icarus_rover_v2::diagnostic diag = diagnostic;
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
icarus_rover_v2::diagnostic BoardControllerNodeProcess::new_message_recv(unsigned char id)
{
	icarus_rover_v2::diagnostic diag = diagnostic;
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
		newmessage.id = SPIMessageHandler::SPI_Get_IMUAcc_ID;
		newmessage.name = "GetIMUAcc";
		newmessage.type = "Query";
		newmessage.send_me = false;
		messages.push_back(newmessage);
	}
	{
		Message newmessage;
		newmessage.id = SPIMessageHandler::SPI_Get_IMUGyro_ID;
		newmessage.name = "GetIMUGyro";
		newmessage.type = "Query";
		newmessage.send_me = false;
		messages.push_back(newmessage);
	}
	{
		Message newmessage;
		newmessage.id = SPIMessageHandler::SPI_Get_IMUMag_ID;
		newmessage.name = "GetIMUMag";
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
	for(std::size_t i = 0; i < messages.size(); i++)
	{
		messages.at(i).sent_counter = 0;
		messages.at(i).recv_counter = 0;
		messages.at(i).sent_rate = 0.0;
		messages.at(i).recv_rate = 0.0;
	}
}
icarus_rover_v2::diagnostic BoardControllerNodeProcess::get_LEDStripControlParameters
(unsigned char& Mode,unsigned char& Param1,unsigned char& Param2)
{
	icarus_rover_v2::diagnostic diag = diagnostic;
	Mode = LEDPixelMode;
	Param1 = 0;
	Param2 = 0;
	return diag;
}
icarus_rover_v2::diagnostic BoardControllerNodeProcess::send_commandmessage(unsigned char id)
{
	icarus_rover_v2::diagnostic diag;
	diag = diagnostic;
	for(std::size_t i = 0; i <messages.size(); i++)
	{
		if((messages.at(i).id == id) && (messages.at(i).type == "Command"))
		{
			messages.at(i).send_me = true;
			diag.Diagnostic_Type = COMMUNICATIONS;
			diag.Level = INFO;
			diag.Diagnostic_Message = NOERROR;
			return diag;
		}
	}
	char tempstr[255];
	diag.Diagnostic_Type = COMMUNICATIONS;
	diag.Level = WARN;
	diag.Diagnostic_Message = DROPPING_PACKETS;
	sprintf(tempstr,"Couldn't send Command Message: AB%0X",id);
	diag.Description = std::string(tempstr);
	return diag;
}

icarus_rover_v2::diagnostic BoardControllerNodeProcess::send_querymessage(unsigned char id)
{
	icarus_rover_v2::diagnostic diag;
	diag = diagnostic;
	for(std::size_t i = 0; i <messages.size(); i++)
	{
		if((messages.at(i).id == id) && (messages.at(i).type == "Query"))
		{
			messages.at(i).send_me = true;
			diag.Diagnostic_Type = COMMUNICATIONS;
			diag.Level = INFO;
			diag.Diagnostic_Message = NOERROR;
			return diag;
		}
	}
	char tempstr[255];
	diag.Diagnostic_Type = COMMUNICATIONS;
	diag.Level = WARN;
	diag.Diagnostic_Message = DROPPING_PACKETS;
	sprintf(tempstr,"Couldn't send Query Message: AB%0X",id);
	diag.Description = std::string(tempstr);
	return diag;
}
std::string BoardControllerNodeProcess::get_messageinfo(bool v)
{
	char tempstr[2048];
	sprintf(tempstr,"");

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
icarus_rover_v2::diagnostic BoardControllerNodeProcess::new_message_TestMessageCounter(uint8_t boardid,unsigned char v1,unsigned char v2,unsigned char v3,unsigned char v4,
		unsigned char v5,unsigned char v6,unsigned char v7,unsigned char v8,
		unsigned char v9,unsigned char v10,unsigned char v11,unsigned char v12)
{
	icarus_rover_v2::diagnostic diag;
	diag = diagnostic;
	char tempstr[255];
	diag.Diagnostic_Type = COMMUNICATIONS;
	diag.Level = WARN;
	diag.Diagnostic_Message = DROPPING_PACKETS;
	sprintf(tempstr,"Process Message: AB%0X Not Supported",SPIMessageHandler::SPI_TestMessageCounter_ID);
	diag.Description = std::string(tempstr);
	return diag;
}
icarus_rover_v2::diagnostic BoardControllerNodeProcess::new_message_GetDIOPort1(uint8_t boardid,double tov,
		int16_t v1,int16_t v2)
{
	int16_t v[6] = {v1,v2};
	icarus_rover_v2::diagnostic diag = diagnostic;
	icarus_rover_v2::device board = find_board(boardid);
	if(board.DeviceName == "")
	{
		char tempstr[255];
		diag.Diagnostic_Type = COMMUNICATIONS;
		diag.Level = ERROR;
		diag.Diagnostic_Message = DROPPING_PACKETS;
		sprintf(tempstr,"Board ID: %d Not Found\n",boardid);
		diag.Description = std::string(tempstr);
		return diag;
	}
	for(uint8_t i = 0; i < 2; i++)
	{
		icarus_rover_v2::pin pin = find_pin(board,"QuadratureEncoderInput",i);
		if(pin.Name != "")
		{
			if(update_sensor(board,pin,tov,(double)v[i]) == false)
			{
			}
		}
	}
	char tempstr[255];
	diag.Diagnostic_Type = COMMUNICATIONS;
	diag.Level = INFO;
	diag.Diagnostic_Message = NOERROR;
	sprintf(tempstr,"Updated");
	diag.Description = std::string(tempstr);
	return diag;
}
icarus_rover_v2::diagnostic BoardControllerNodeProcess::new_message_Diagnostic(uint8_t boardid,
		unsigned char System,unsigned char SubSystem,
		unsigned char Component,unsigned char Diagnostic_Type,
		unsigned char Level,unsigned char Message)
{
	icarus_rover_v2::diagnostic diag = diagnostic;
	icarus_rover_v2::diagnostic worst_diagnostic;
	worst_diagnostic.Level = DEBUG;
	icarus_rover_v2::device board = find_board(boardid);
	if(board.DeviceName == "")
	{
		char tempstr[255];
		diag.Diagnostic_Type = COMMUNICATIONS;
		diag.Level = ERROR;
		diag.Diagnostic_Message = DROPPING_PACKETS;
		sprintf(tempstr,"Board ID: %d Not Found\n",boardid);
		diag.Description = std::string(tempstr);
		return diag;
	}

	bool should_i_arm = true;
	bool found = false;
	for(std::size_t i = 0; i < board_diagnostics.size(); i++)
	{
		if(board_diagnostics.at(i).id == boardid)
		{
			found = true;
			board_diagnostics.at(i).lasttime_rx = run_time;
			board_diagnostics.at(i).diagnostic.System = System;
			board_diagnostics.at(i).diagnostic.SubSystem = SubSystem;
			board_diagnostics.at(i).diagnostic.Component = Component;
			board_diagnostics.at(i).diagnostic.Diagnostic_Type = Diagnostic_Type;
			board_diagnostics.at(i).diagnostic.Level = Level;
			board_diagnostics.at(i).diagnostic.Diagnostic_Message = Message;
			board_diagnostics.at(i).diagnostic.Description = "";
			if(Level > worst_diagnostic.Level)
			{
				worst_diagnostic = board_diagnostics.at(i).diagnostic;
			}
			if(Message == INITIALIZING)
			{
				should_i_arm = false;
			}
			else if((Level <= NOTICE) or (Message == NOERROR))
			{
				should_i_arm = should_i_arm and true;
			}
			else
			{
				should_i_arm = false;
			}
		}
	}
	if(found == false)
	{
		should_i_arm = false;
	}
	if(should_i_arm == true)
	{
		ready_to_arm = true;
	}
	if(worst_diagnostic.Level > NOTICE)
	{
		return worst_diagnostic;
	}
	else
	{
		char tempstr[255];
		diag.Diagnostic_Type = COMMUNICATIONS;
		diag.Level = INFO;
		diag.Diagnostic_Message = NOERROR;
		sprintf(tempstr,"Updated");
		diag.Description = std::string(tempstr);
		return diag;
	}
}
icarus_rover_v2::diagnostic BoardControllerNodeProcess::new_message_GetANAPort1(uint8_t boardid,double tov,uint16_t v1,uint16_t v2,uint16_t v3,
		uint16_t v4,uint16_t v5,uint16_t v6)
{
	uint16_t v[6] = {v1,v2,v3,v4,v5,v6};
	icarus_rover_v2::diagnostic diag = diagnostic;
	icarus_rover_v2::device board = find_board(boardid);
	if(board.DeviceName == "")
	{
		char tempstr[255];
		diag.Diagnostic_Type = COMMUNICATIONS;
		diag.Level = ERROR;
		diag.Diagnostic_Message = DROPPING_PACKETS;
		sprintf(tempstr,"Board ID: %d Not Found\n",boardid);
		diag.Description = std::string(tempstr);
		return diag;
	}
	for(uint8_t i = 0; i < 6; i++)
	{
		icarus_rover_v2::pin pin = find_pin(board,"AnalogInput",i);
		if(pin.Name != "")
		{
			if(update_sensor(board,pin,tov,(double)v[i]) == false)
			{

			}
		}
	}
	char tempstr[255];
	diag.Diagnostic_Type = COMMUNICATIONS;
	diag.Level = INFO;
	diag.Diagnostic_Message = NOERROR;
	sprintf(tempstr,"Updated");
	diag.Description = std::string(tempstr);
	return diag;


}
icarus_rover_v2::diagnostic BoardControllerNodeProcess::new_message_IMUAcc(uint8_t boardid,double tov,uint16_t v1,uint16_t v2,uint16_t v3,uint16_t v4,uint16_t v5,uint16_t v6)
{
	bool any_error = false;
	icarus_rover_v2::diagnostic diag = diagnostic;
	icarus_rover_v2::signal xacc1_sig,yacc1_sig,zacc1_sig,xacc2_sig,yacc2_sig,zacc2_sig;
	xacc1_sig.value = (double)(v1-32768)/100.0;
	xacc1_sig.status = SIGNALSTATE_UPDATED;
	yacc1_sig.value = (double)(v2-32768)/100.0;
	yacc1_sig.status = SIGNALSTATE_UPDATED;
	zacc1_sig.value = (double)(v3-32768)/100.0;
	zacc1_sig.status = SIGNALSTATE_UPDATED;
	xacc2_sig.value = (double)(v4-32768)/100.0;
	xacc2_sig.status = SIGNALSTATE_UPDATED;
	yacc2_sig.value = (double)(v5-32768)/100.0;
	yacc2_sig.status = SIGNALSTATE_UPDATED;
	zacc2_sig.value = (double)(v6-32768)/100.0;
	zacc2_sig.status = SIGNALSTATE_UPDATED;
	imu1.msg.xacc = xacc1_sig;
	imu1.msg.yacc = yacc1_sig;
	imu1.msg.zacc = zacc1_sig;
	imu2.msg.xacc = xacc2_sig;
	imu2.msg.yacc = yacc2_sig;
	imu2.msg.zacc = zacc2_sig;
	imu1.msg.tov = tov;
	imu1.msg.tov = tov;
	imu1.updated = true;
	imu2.updated = true;

	if(any_error == false)
	{
		char tempstr[255];
		diag.Diagnostic_Type = COMMUNICATIONS;
		diag.Level = INFO;
		diag.Diagnostic_Message = NOERROR;
		sprintf(tempstr,"Updated");
		diag.Description = std::string(tempstr);
	}
	return diag;
}
icarus_rover_v2::diagnostic BoardControllerNodeProcess::new_message_IMUGyro(uint8_t boardid,double tov,uint16_t v1,uint16_t v2,uint16_t v3,uint16_t v4,uint16_t v5,uint16_t v6)
{
	bool any_error = false;
	icarus_rover_v2::diagnostic diag = diagnostic;
	icarus_rover_v2::signal xgyro1_sig,ygyro1_sig,zgyro1_sig,xgyro2_sig,ygyro2_sig,zgyro2_sig;
	xgyro1_sig.value = (double)(v1-32768)/1000.0;
	xgyro1_sig.status = SIGNALSTATE_UPDATED;
	ygyro1_sig.value = (double)(v2-32768)/1000.0;
	ygyro1_sig.status = SIGNALSTATE_UPDATED;
	zgyro1_sig.value = (double)(v3-32768)/1000.0;
	zgyro1_sig.status = SIGNALSTATE_UPDATED;
	xgyro2_sig.value = (double)(v4-32768)/1000.0;
	xgyro2_sig.status = SIGNALSTATE_UPDATED;
	ygyro2_sig.value = (double)(v5-32768)/1000.0;
	ygyro2_sig.status = SIGNALSTATE_UPDATED;
	zgyro2_sig.value = (double)(v6-32768)/1000.0;
	zgyro2_sig.status = SIGNALSTATE_UPDATED;
	imu1.msg.xgyro = xgyro1_sig;
	imu1.msg.ygyro = ygyro1_sig;
	imu1.msg.zgyro = zgyro1_sig;
	imu2.msg.xgyro = xgyro2_sig;
	imu2.msg.ygyro = ygyro2_sig;
	imu2.msg.zgyro = zgyro2_sig;
	imu1.msg.tov = tov;
	imu1.msg.tov = tov;
	imu1.updated = true;
	imu2.updated = true;

	if(any_error == false)
	{
		char tempstr[255];
		diag.Diagnostic_Type = COMMUNICATIONS;
		diag.Level = INFO;
		diag.Diagnostic_Message = NOERROR;
		sprintf(tempstr,"Updated");
		diag.Description = std::string(tempstr);
	}
	return diag;
}
icarus_rover_v2::diagnostic BoardControllerNodeProcess::new_message_IMUMag(uint8_t boardid,double tov,uint16_t v1,uint16_t v2,uint16_t v3,uint16_t v4,uint16_t v5,uint16_t v6)
{
	bool any_error = false;
	icarus_rover_v2::diagnostic diag = diagnostic;
	icarus_rover_v2::signal xmag1_sig,ymag1_sig,zmag1_sig,xmag2_sig,ymag2_sig,zmag2_sig;
	xmag1_sig.value = (double)(v1-32768)/1000.0;
	xmag1_sig.status = SIGNALSTATE_UPDATED;
	ymag1_sig.value = (double)(v2-32768)/1000.0;
	ymag1_sig.status = SIGNALSTATE_UPDATED;
	zmag1_sig.value = (double)(v3-32768)/1000.0;
	zmag1_sig.status = SIGNALSTATE_UPDATED;
	xmag2_sig.value = (double)(v4-32768)/1000.0;
	xmag2_sig.status = SIGNALSTATE_UPDATED;
	ymag2_sig.value = (double)(v5-32768)/1000.0;
	ymag2_sig.status = SIGNALSTATE_UPDATED;
	zmag2_sig.value = (double)(v6-32768)/1000.0;
	zmag2_sig.status = SIGNALSTATE_UPDATED;
	imu1.msg.xmag = xmag1_sig;
	imu1.msg.ymag = ymag1_sig;
	imu1.msg.zmag = zmag1_sig;
	imu2.msg.xmag = xmag2_sig;
	imu2.msg.ymag = ymag2_sig;
	imu2.msg.zmag = zmag2_sig;
	imu1.msg.tov = tov;
	imu1.msg.tov = tov;
	imu1.updated = true;
	imu2.updated = true;

	if(any_error == false)
	{
		char tempstr[255];
		diag.Diagnostic_Type = COMMUNICATIONS;
		diag.Level = INFO;
		diag.Diagnostic_Message = NOERROR;
		sprintf(tempstr,"Updated");
		diag.Description = std::string(tempstr);
	}
	return diag;
}
icarus_rover_v2::diagnostic BoardControllerNodeProcess::new_devicemsg(icarus_rover_v2::device newdevice)
{
	icarus_rover_v2::diagnostic diag = diagnostic;
	if(initialized == true)
	{
		if(ready == false)
		{
			if(newdevice.DeviceParent == myhostname)
			{
				if(board_present(newdevice) == true)
				{
					diag.Level = WARN;
					diag.Diagnostic_Type = SOFTWARE;
					diag.Diagnostic_Message = DEVICE_NOT_AVAILABLE;
					char tempstr[512];
					sprintf(tempstr,"Board: %s already loaded.",
							newdevice.DeviceName.c_str());
					diag.Description = std::string(tempstr);
					return diag;
				}
				std::size_t board_message = newdevice.DeviceType.find("Board");
				if((board_message != std::string::npos))
				{
					if(newdevice.DeviceType == "ArduinoBoard")
					{
						for(std::size_t i = 0; i < newdevice.pins.size(); i++)
						{
							if((newdevice.pins.at(i).Function == map_PinFunction_ToString(PINMODE_ANALOG_INPUT) or
									(newdevice.pins.at(i).Function == map_PinFunction_ToString(PINMODE_QUADRATUREENCODER_INPUT))))
							{
								Sensor new_sensor;
								new_sensor.initialized = false;
								new_sensor.value = 0.0;
								new_sensor.name = newdevice.pins.at(i).ConnectedSensor;
								new_sensor.connected_board = newdevice;
								new_sensor.connected_pin = newdevice.pins.at(i);
								new_sensor.status = SIGNALSTATE_UNDEFINED;

								sensors.push_back(new_sensor);
								if(load_sensorinfo(new_sensor.name) == false)
								{
									diag.Level = ERROR;
									diag.Diagnostic_Type = SOFTWARE;
									diag.Diagnostic_Message = DEVICE_NOT_AVAILABLE;
									char tempstr[512];
									sprintf(tempstr,"Unable to load info for Sensor: %s",new_sensor.name.c_str());
									diag.Description = std::string(tempstr);
									return diag;
								}

							}
							else
							{
								diag.Level = ERROR;
								diag.Diagnostic_Type = SOFTWARE;
								diag.Diagnostic_Message = DEVICE_NOT_AVAILABLE;
								char tempstr[512];
								sprintf(tempstr,"Board Type: %s Pin Function: %s Not supported.",
										newdevice.DeviceType.c_str(),newdevice.pins.at(i).Function.c_str());
								diag.Description = std::string(tempstr);
								return diag;
							}
						}
						printf("Added Board\n");
						boards.push_back(newdevice);
						BoardDiagnostic board_diag;
						board_diag.id = newdevice.ID;
						board_diag.diagnostic.System = SYSTEM_UNKNOWN;
						board_diag.diagnostic.SubSystem = SUBSYSTEM_UNKNOWN;
						board_diag.diagnostic.Component = COMPONENT_UNKNOWN;
						board_diag.diagnostic.Diagnostic_Type = GENERAL_ERROR;
						board_diag.diagnostic.Diagnostic_Message = UNKNOWN_STATE;
						board_diag.diagnostic.Level = LEVEL_UNKNOWN;
						board_diag.diagnostic.Description = "No Info received yet for this Board.";
						board_diagnostics.push_back(board_diag);
						boards_running.push_back(true);
					}
					else
					{
						diag.Level = ERROR;
						diag.Diagnostic_Type = SOFTWARE;
						diag.Diagnostic_Message = DEVICE_NOT_AVAILABLE;
						char tempstr[512];
						sprintf(tempstr,"Device Type: %s Not supported.",newdevice.DeviceType.c_str());
						diag.Description = std::string(tempstr);
						return diag;
					}

				}

			}
		}
		else
		{

		}
	}
	if((boards.size() == mydevice.BoardCount) and
			(sensors_initialized() == true))
	{
		ready = true;
	}
	diag.Level = INFO;
	diag.Diagnostic_Type = COMMUNICATIONS;
	diag.Diagnostic_Message = NOERROR;
	char tempstr[512];
	sprintf(tempstr,"Initialized: %d Ready: %d",initialized,ready);
	diag.Description = std::string(tempstr);
	return diag;
}
bool BoardControllerNodeProcess::board_present(icarus_rover_v2::device device)
{
	for(std::size_t i = 0; i < boards.size(); i++)
	{
		if((boards.at(i).DeviceName == device.DeviceName))
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
bool BoardControllerNodeProcess::update_sensor(icarus_rover_v2::device board,icarus_rover_v2::pin pin,double tov,double value)
{
	for(std::size_t i = 0; i < sensors.size(); i++)
	{
		if((sensors.at(i).connected_board.DeviceName == board.DeviceName) and
				(sensors.at(i).connected_pin.Name == pin.Name))
		{
			sensors.at(i).tov = tov;
			sensors.at(i).status = SIGNALSTATE_UPDATED;
			if(sensors.at(i).convert == false)
			{
				sensors.at(i).value = value;
			}
			else
			{
				sensors.at(i).value = map_input_to_output(value,sensors.at(i).min_inputvalue,
						sensors.at(i).max_inputvalue,
						sensors.at(i).min_inputvalue,
						sensors.at(i).max_outputvalue);
			}
			return true;
		}
	}
	return false;
}
icarus_rover_v2::device BoardControllerNodeProcess::find_board(uint8_t boardid)
{
	for(std::size_t i = 0; i < boards.size(); i++)
	{
		if(boards.at(i).ID == boardid)
		{
			return boards.at(i);
		}
	}
	icarus_rover_v2::device empty_device;
	empty_device.DeviceName = "";
	empty_device.DeviceType = "";
	return empty_device;
}
icarus_rover_v2::pin BoardControllerNodeProcess::find_pin(icarus_rover_v2::device board,std::string pinfunction,uint8_t pinnumber)
{
	for(std::size_t i = 0; i < board.pins.size(); i++)
	{
		if((board.pins.at(i).Function == pinfunction) and (board.pins.at(i).Number == pinnumber))
		{
			return board.pins.at(i);
		}
	}
	icarus_rover_v2::pin empty_pin;
	empty_pin.Name = "";
	empty_pin.Function = "";
	return empty_pin;
}
icarus_rover_v2::pin find_pin(icarus_rover_v2::device board,std::string pinname)
{
	for(std::size_t i = 0; i < board.pins.size(); i++)
	{
		if((board.pins.at(i).ConnectedDevice == pinname))
		{
			return board.pins.at(i);
		}
	}
	icarus_rover_v2::pin empty_pin;
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
			sensors.at(sensor_index).units = l_pUnits->GetText();
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
