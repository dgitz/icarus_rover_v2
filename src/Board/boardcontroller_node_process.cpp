#include "boardcontroller_node_process.h"
BoardControllerNodeProcess::BoardControllerNodeProcess(std::string loc, int v)
{
	location = loc;
	id = v;
	my_boardname = "";
	board_state = BOARDMODE_UNDEFINED;
	node_state = BOARDMODE_BOOT;
	all_sensor_info_received = false;
	all_shield_info_received = false;
	all_device_info_received = false;
	extrapolate_values = false;
	initialize_Ports();
	ms_timer = 0;
	timeout_value_ms = 0;
	armed_state = ARMEDSTATUS_DISARMED_CANNOTARM;
	armed_command = ARMEDSTATUS_DISARMED_CANNOTARM;
	shield_count = -1;
	run_time = 0.0;
    ready_to_arm = false;
	gettimeofday(&init_time,NULL);
	//init_time = ros::Time::now();
}
BoardControllerNodeProcess::BoardControllerNodeProcess()
{
}
BoardControllerNodeProcess::~BoardControllerNodeProcess()
{

}
bool BoardControllerNodeProcess::initialize_Ports()
{
	/*
	DIO_Port.PortName = "DIO_Port";
	for(int i = 0; i < 8; i++)
	{
		DIO_Port.Number[i] = i+1;
		DIO_Port.Mode[i] = PINMODE_UNDEFINED;
		DIO_Port.Available[i] = false;
		DIO_Port.Value[i] = 0;
		DIO_Port.DefaultValue[i] = 0;
		DIO_Port.ConnectingDevice.push_back("");
	}
	ANA_Port.PortName = "ANA_Port";
	for(int i = 0; i < 4; i++)
	{
		ANA_Port.Number[i] = i-1;
		ANA_Port.Mode[i] = PINMODE_ANALOG_INPUT;
		ANA_Port.Available[i] = false;
		ANA_Port.Value[i] = 0;
		ANA_Port.DefaultValue[i] = 0;
		ANA_Port.ConnectingDevice.push_back("");
		
	}
	*/
}
icarus_rover_v2::diagnostic BoardControllerNodeProcess::init(icarus_rover_v2::diagnostic indiag
		,std::string hostname,std::string sensorspecpath,bool extrapolate)
{
	initialize_stateack_messages();
	initialize_message_info();
	serialmessagehandler = new SerialMessageHandler();
	board_state = BOARDMODE_UNDEFINED;

	node_state = BOARDMODE_BOOT;
	prev_node_state = BOARDMODE_BOOT;
	myhostname = hostname;
	diagnostic = indiag;
	mydevice.DeviceName = hostname;
	timeout_value_ms = INITIAL_TIMEOUT_VALUE_MS;
	sensor_spec_path = sensorspecpath;
	extrapolate_values = extrapolate;
	return diagnostic;
}
icarus_rover_v2::diagnostic BoardControllerNodeProcess::init(icarus_rover_v2::diagnostic indiag,
		std::string hostname,int temp_id)
{
	UsbDevice_id = temp_id;
	initialize_stateack_messages();
	initialize_message_info();
	serialmessagehandler = new SerialMessageHandler();
	board_state = BOARDMODE_UNDEFINED;

	node_state = BOARDMODE_BOOT;
	prev_node_state = BOARDMODE_BOOT;
	myhostname = hostname;
	diagnostic = indiag;
	mydevice.DeviceName = hostname;
	timeout_value_ms = INITIAL_TIMEOUT_VALUE_MS;
	sensor_spec_path = "";
	extrapolate_values = false;
	return diagnostic;
}
void  BoardControllerNodeProcess::transmit_armedstate()
{
	send_armedstate.trigger = true;
}
icarus_rover_v2::diagnostic BoardControllerNodeProcess::update(double dt)
{
	if((shield_count == myshields.size() && (shield_count > 0))) { all_shield_info_received = true; }
	else if(shield_count == 0) { all_shield_info_received = true; }
	if((all_shield_info_received == true) &&
			(all_device_info_received == false))//Other checks?
	{

		node_state = BOARDMODE_INITIALIZING;
		all_device_info_received = true;
	}
	run_time += dt;
	//send_configure_shields.trigger = true;  //Debug
	//send_nodemode.trigger = true;
	ms_timer += dt;
	if(ms_timer >= timeout_value_ms) { timer_timeout = true; }
	if(timer_timeout == true)
	{
		timer_timeout = false;

	}
	if(prev_node_state != node_state)
	{
		send_nodemode.trigger = true;
		prev_node_state = node_state;
	}
	if((board_state == BOARDMODE_INITIALIZING) && (node_state == BOARDMODE_INITIALIZING))
	{
		send_configure_shields.trigger = true;
	}
	if((board_state == BOARDMODE_SHIELDS_CONFIGURED) && (node_state == BOARDMODE_INITIALIZING))
	{
		node_state = BOARDMODE_SHIELDS_CONFIGURED;
		send_configure_shields.trigger = false;
		send_configure_DIO_Ports.trigger = true;
	}
	if((board_state == BOARDMODE_INITIALIZED) && (node_state == BOARDMODE_SHIELDS_CONFIGURED))
	{
		node_state = BOARDMODE_INITIALIZED;
        send_defaultvalue_DIO_Port.trigger = true;
		send_configure_DIO_Ports.trigger = false;
	}
	if((board_state == BOARDMODE_RUNNING) && (node_state == BOARDMODE_INITIALIZED))
	{
        armed_state = ARMEDSTATUS_DISARMED;
		node_state = BOARDMODE_RUNNING;
	}

	if((board_state == BOARDMODE_RUNNING) && (node_state == BOARDMODE_RUNNING))
	{
		send_set_DIO_Port.trigger = true;
        
	}
	if((board_state == BOARDMODE_BOOT) and (node_state > BOARDMODE_INITIALIZING))
	{
		node_state = BOARDMODE_INITIALIZING;
	}
    if((board_state == BOARDMODE_RUNNING) && (node_state == BOARDMODE_RUNNING))
    {
        ready_to_arm = true;
    }
    else
    {
        ready_to_arm = false;
    }
	diagnostic.Level = INFO;
	diagnostic.Diagnostic_Message = NOERROR;
	diagnostic.Description = "Node Executing.";
	return diagnostic;
}
icarus_rover_v2::diagnostic BoardControllerNodeProcess::new_armedstatemsg(uint8_t v)
{
	armed_state = v;
	return diagnostic;
}
state_ack BoardControllerNodeProcess::get_stateack(std::string name)
{

	if(name == send_configure_DIO_Ports.name)
	{
		return send_configure_DIO_Ports;
	}
	else if(name == send_configure_shields.name)
	{
		return send_configure_shields;
	}
	else if(name == send_testmessage_command.name)
	{
		return send_testmessage_command;
	}
	else if(name == send_nodemode.name)
	{
		return send_nodemode;
	}
	else if(name == send_set_DIO_Port.name)
	{
		return send_set_DIO_Port;
	}
	else if(name == send_defaultvalue_DIO_Port.name)
	{
		return send_defaultvalue_DIO_Port;
	}
    else if(name == send_armedcommand.name)
    {
        return send_armedcommand;
    }
	else
	{
		state_ack emptystateack;
		emptystateack.name = "";
		return emptystateack;
	}
}
bool BoardControllerNodeProcess::set_stateack(state_ack stateack)
{
	if(stateack.name == "Send Configure DIO Ports")
	{
		send_configure_DIO_Ports = stateack;
	}
	else if(stateack.name == "Send Test Message Command")
	{
		send_testmessage_command = stateack;
	}
	else if(stateack.name == "Send Node Mode")
	{
		send_nodemode = stateack;
	}
	else if(stateack.name == "Send Set DIO Port")
	{
		send_set_DIO_Port = stateack;
	}
    else if(stateack.name == "Send Arm Command")
    {
        send_armedcommand = stateack;
    }
	else if(stateack.name == "Send DefaultValue DIO Port")
	{
		send_defaultvalue_DIO_Port = stateack;
	}
	else
	{
		return false;
	}
	return true;
}
icarus_rover_v2::diagnostic BoardControllerNodeProcess::new_pinmsg(icarus_rover_v2::pin pinmsg)
{
	if((board_state == BOARDMODE_RUNNING) && (node_state == BOARDMODE_RUNNING))
	{
		int portID = floor(pinmsg.Number/8)+1;
		bool found = false;
		for(int s = 0; s < myports.size();s++)
		{
			if((myports.at(s).ShieldID == pinmsg.ShieldID) &&
			   (myports.at(s).PortID == portID))
			{
				myports.at(s).Updated = true;
				int pinnumber;
				if(pinmsg.Number < 8){ pinnumber = pinmsg.Number;		}
				else	 			 { pinnumber = pinmsg.Number % 8;	}
				if(myports.at(s).Mode.at(pinnumber) == map_PinFunction_ToInt(pinmsg.Function))
				{
					myports.at(s).Value.at(pinnumber) = pinmsg.Value;
					diagnostic.Level = INFO;
					diagnostic.Diagnostic_Type = SOFTWARE;
					diagnostic.Diagnostic_Message = NOERROR;
					char tempstr[512];
					sprintf(tempstr,"Pin Message for Shield: %d and Pin Number: %d and Mode: %s New Value: %d",
							pinmsg.ShieldID,
							pinmsg.Number,
							map_PinFunction_ToString(myports.at(s).Mode.at(pinnumber)).c_str(),
							pinmsg.Value);
					diagnostic.Description = tempstr;

				}
				else
				{
					diagnostic.Level = FATAL;
					diagnostic.Diagnostic_Type = SOFTWARE;
					diagnostic.Diagnostic_Message = DEVICE_NOT_AVAILABLE;
					char tempstr[512];
					sprintf(tempstr,"Pin Message for Shield: %d and Pin Number: %d and Mode: %s Not Supported, got: %s",
							pinmsg.ShieldID,pinmsg.Number,map_PinFunction_ToString(myports.at(s).Mode.at(pinnumber)).c_str(),
							pinmsg.Function.c_str());
					diagnostic.Description = tempstr;
				}
				found = true;
			}
		}
		if(found == false)
		{
			diagnostic.Level = FATAL;
			diagnostic.Diagnostic_Type = SOFTWARE;
			diagnostic.Diagnostic_Message = DEVICE_NOT_AVAILABLE;
			char tempstr[512];
			sprintf(tempstr,"Pin Message for Shield: %d and Pin Number: %d Not Supported",
					pinmsg.ShieldID,portID);
			diagnostic.Description = tempstr;
		}
	}
	else
	{
		diagnostic.Level = WARN;
		diagnostic.Diagnostic_Type = REMOTE_CONTROL;
		diagnostic.Diagnostic_Message = INITIALIZING_ERROR;
		char tempstr[512];
		sprintf(tempstr,"Device is not RUNNING Yet.");
		diagnostic.Description = tempstr;
	}
	return diagnostic;
}
bool BoardControllerNodeProcess::checkTriggers(std::vector<std::vector<unsigned char > > &tx_buffers)
{
	bool nothing_triggered = true;
	//send_nodemode.trigger = true;
	if(send_nodemode.trigger == true)
	{
		bool send_me = false;
		if(send_nodemode.stream_rate < 0) //Normal operation, should send
		{
			send_me = true;
			send_nodemode.trigger = false;
		}
		else
		{
			struct timeval now;
			gettimeofday(&now,NULL);
			double etime = time_diff(send_nodemode.orig_send_time,now);
			double delay = 1.0/(send_nodemode.stream_rate);
			if(etime > delay){ send_me = true; }
			else{ send_me = false; }
		}
		if(send_me == true)
		{
			nothing_triggered = false;
			//printf("send_nodemode Triggered.\n");
			char buffer[16];
			int length;
			int computed_checksum;
			int tx_status = serialmessagehandler->encode_ModeSerial(buffer,&length,0,0,node_state);
			bool status = gather_message_info(SERIAL_Mode_ID, "transmit");
			tx_buffers.push_back(std::vector<unsigned char>(buffer,buffer+sizeof(buffer)/sizeof(buffer[0])));
			send_nodemode.state = true;
			if (send_nodemode.retrying == false)
			{
				gettimeofday(&send_nodemode.orig_send_time,NULL);
				send_nodemode.retries = 0;
			}
		}

	}
	if(send_armedcommand.trigger == true)
	{
		bool send_me = false;
		if(send_armedcommand.stream_rate < 0) //Normal operation, should send
		{
			send_me = true;
			send_armedcommand.trigger = false;
		}
		else
		{
			struct timeval now;
			gettimeofday(&now,NULL);
			double etime = time_diff(send_armedcommand.orig_send_time,now);
			double delay = 1.0/(send_armedcommand.stream_rate);
			if(etime > delay){ send_me = true; }
			else{ send_me = false; }
		}
		if(send_me == true)
		{
			nothing_triggered = false;
			char buffer[16];
			int length;
			int computed_checksum;
			int index=0;
			int tx_status = serialmessagehandler->encode_Arm_CommandSerial(buffer,&length,
					armed_command);
			bool status = gather_message_info(SERIAL_Arm_Command_ID, "transmit");
			//printf("%d Sending shield config for shield: %d\n",get_boardid(),myshields.at(i).ID);
			tx_buffers.push_back(std::vector<unsigned char>(buffer,buffer+sizeof(buffer)/sizeof(buffer[0])));
			/*for(int i = 0; i < 16; i++)
			{
				printf("%0x ",buffer[i]);
			}
			printf("\n");
			*/

			send_armedcommand.state = true;
			if (send_armedcommand.retrying == false)
			{
				gettimeofday(&send_armedcommand.orig_send_time,NULL);
				send_armedcommand.retries = 0;
			}
			send_armedcommand.trigger = false;
		}
	}
	if(send_configure_shields.trigger == true)
	{
		bool send_me = false;
		if(send_configure_shields.stream_rate < 0) //Normal operation, should send
		{
			send_me = true;
			send_configure_shields.trigger = false;
		}
		else
		{
			struct timeval now;
			gettimeofday(&now,NULL);
			double etime = time_diff(send_configure_shields.orig_send_time,now);
			double delay = 1.0/(send_configure_shields.stream_rate);
			if(etime > delay){ send_me = true; }
			else{ send_me = false; }
		}
		if(send_me == true)
		{
			nothing_triggered = false;
			for(int i = 0; i < myshields.size();i++)
			{
				char buffer[16];
				int length;
				int computed_checksum;
				int index=0;
				int tx_status = serialmessagehandler->encode_Configure_ShieldSerial(buffer,&length,
						myshields.size(),
						map_DeviceType_ToInt(myshields.at(i).DeviceType),
						myshields.at(i).ID,
						get_portlist(myshields.at(i).ID).size());
				bool status = gather_message_info(SERIAL_Configure_Shield_ID, "transmit");
				//printf("%d Sending shield config for shield: %d\n",get_boardid(),myshields.at(i).ID);
				tx_buffers.push_back(std::vector<unsigned char>(buffer,buffer+sizeof(buffer)/sizeof(buffer[0])));
				/*
				for(int i = 0; i < 16; i++)
				{
					printf("%0x ",buffer[i]);
				}
				printf("\n");
				*/
				
			}
			send_configure_shields.state = true;
			if (send_configure_shields.retrying == false)
			{
				gettimeofday(&send_configure_shields.orig_send_time,NULL);
				send_configure_shields.retries = 0;
			}
		}
	}
	if(send_configure_DIO_Ports.trigger == true)
	{
		bool send_me = false;
		if(send_configure_DIO_Ports.stream_rate < 0) //Normal operation, should send
		{
			send_me = true;
			send_configure_DIO_Ports.trigger = false;
		}
		else
		{
			struct timeval now;
			gettimeofday(&now,NULL);
			double etime = time_diff(send_configure_DIO_Ports.orig_send_time,now);
			double delay = 1.0/(send_configure_DIO_Ports.stream_rate);
			if(etime > delay){ send_me = true; }
			else{ send_me = false; }
		}
		if(send_me == true)
		{
			nothing_triggered = false;
			for(int i = 0; i < myports.size();i++)
			{
				char buffer[16];
				int length;
				int computed_checksum;
				int index=0;
				int tx_status = serialmessagehandler->encode_Configure_DIO_PortSerial(buffer,&length,
						myports.at(i).ShieldID,
						myports.at(i).PortID,
						myports.at(i).Mode.at(0),
						myports.at(i).Mode.at(1),
						myports.at(i).Mode.at(2),
						myports.at(i).Mode.at(3),
						myports.at(i).Mode.at(4),
						myports.at(i).Mode.at(5),
						myports.at(i).Mode.at(6),
						myports.at(i).Mode.at(7));

				bool status = gather_message_info(SERIAL_Configure_DIO_Port_ID, "transmit");
				/*
				for(int i = 0; i < 16; i++)
				{
					printf("%0x ",buffer[i]);
				}
				printf("\n");
				*/
				tx_buffers.push_back(std::vector<unsigned char>(buffer,buffer+sizeof(buffer)/sizeof(buffer[0])));
			}
			send_configure_DIO_Ports.state = true;
			if (send_configure_DIO_Ports.retrying == false)
			{
				gettimeofday(&send_configure_DIO_Ports.orig_send_time,NULL);
				send_configure_DIO_Ports.retries = 0;
			}
		}

	}
    if(send_defaultvalue_DIO_Port.trigger == true)
	{
    	bool send_me = false;
		if(send_defaultvalue_DIO_Port.stream_rate < 0) //Normal operation, should send
		{
			send_me = true;
			send_defaultvalue_DIO_Port.trigger = false;
		}
		else
		{
			struct timeval now;
			gettimeofday(&now,NULL);
			double etime = time_diff(send_defaultvalue_DIO_Port.orig_send_time,now);
			double delay = 1.0/(send_defaultvalue_DIO_Port.stream_rate);
			if(etime > delay){ send_me = true; }
			else{ send_me = false; }
		}
		if(send_me == true)
		{
			nothing_triggered = false;
			for(int i = 0; i < myports.size();i++)
			{
				char buffer[16];
				int length;
				int computed_checksum;
				int index=0;
				int tx_status = serialmessagehandler->encode_Set_DIO_Port_DefaultValueSerial(buffer,&length,
						myports.at(i).ShieldID,
						myports.at(i).PortID,
						myports.at(i).DefaultValue.at(0),
						myports.at(i).DefaultValue.at(1),
						myports.at(i).DefaultValue.at(2),
						myports.at(i).DefaultValue.at(3),
						myports.at(i).DefaultValue.at(4),
						myports.at(i).DefaultValue.at(5),
						myports.at(i).DefaultValue.at(6),
						myports.at(i).DefaultValue.at(7));

				bool status = gather_message_info(SERIAL_Set_DIO_Port_DefaultValue_ID, "transmit");
				/*for(int i = 0; i < 16; i++)
				{
					printf("%0x ",buffer[i]);
				}
				printf("\n");
				*/
				tx_buffers.push_back(std::vector<unsigned char>(buffer,buffer+sizeof(buffer)/sizeof(buffer[0])));
			}
			send_defaultvalue_DIO_Port.state = true;
			if (send_defaultvalue_DIO_Port.retrying == false)
			{
				gettimeofday(&send_defaultvalue_DIO_Port.orig_send_time,NULL);
				send_defaultvalue_DIO_Port.retries = 0;
			}
		}

	}
	if(send_set_DIO_Port.trigger == true)
	{
		bool send_me = false;
		if(send_set_DIO_Port.stream_rate < 0) //Normal operation, should send
		{
			send_me = true;
			send_set_DIO_Port.trigger = false;
		}
		else
		{
			struct timeval now;
			gettimeofday(&now,NULL);
			double etime = time_diff(send_set_DIO_Port.orig_send_time,now);
			double delay = 1.0/(send_set_DIO_Port.stream_rate);
			if(etime > delay){ send_me = true; }
			else{ send_me = false; }
		}
		if(send_me == true)
		{
			nothing_triggered = false;
			for(int i = 0; i < myports.size();i++)
			{
				if(myports.at(i).Updated == true)
				{
					//printf("port: %d Updated\n",myports.at(i).PortID);
					myports.at(i).Updated = false;
					char buffer[16];
					int length;
					int computed_checksum;
					int index=0;
					int tx_status = serialmessagehandler->encode_Set_DIO_PortSerial(buffer,&length,
							myports.at(i).ShieldID,
							myports.at(i).PortID,
							myports.at(i).Value.at(0),
							myports.at(i).Value.at(1),
							myports.at(i).Value.at(2),
							myports.at(i).Value.at(3),
							myports.at(i).Value.at(4),
							myports.at(i).Value.at(5),
							myports.at(i).Value.at(6),
							myports.at(i).Value.at(7));

					bool status = gather_message_info(SERIAL_Set_DIO_Port_ID, "transmit");
					/*
					for(int i = 0; i < 16; i++)
					{
						printf("%02x ",buffer[i]);
					}
					printf("\n");
					*/
					tx_buffers.push_back(std::vector<unsigned char>(buffer,buffer+sizeof(buffer)/sizeof(buffer[0])));
				}
			}
			send_set_DIO_Port.state = true;
			if (send_set_DIO_Port.retrying == false)
			{
				gettimeofday(&send_set_DIO_Port.orig_send_time,NULL);
				send_set_DIO_Port.retries = 0;
			}
		}
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
icarus_rover_v2::diagnostic BoardControllerNodeProcess::new_commandmsg(icarus_rover_v2::command newcommand)
{
	if (newcommand.Command ==  DIAGNOSTIC_ID)
	{
		if(newcommand.Option1 == LEVEL1)
		{
		}
		else if(newcommand.Option2 == LEVEL2)
		{
			if((board_state == BOARDMODE_RUNNING) && (node_state == BOARDMODE_RUNNING))
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
		else if(newcommand.Option1 == LEVEL3)
		{

		}
		else if(newcommand.Option1 == LEVEL4)
		{

		}
		else
		{
			char tempstr[255];
			sprintf(tempstr,"Got: Diagnostic ID w/ Option: %d but not implemented yet.",newcommand.Option1);
		}
	}
	else if(newcommand.Command == ARM_COMMAND_ID)
	{
		armed_command = newcommand.Option1;
		send_armedcommand.trigger = true;
	}
	return diagnostic;
}
icarus_rover_v2::diagnostic BoardControllerNodeProcess::new_serialmessage_TestMessageCounter(int packet_type,unsigned char* inpacket)
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
icarus_rover_v2::diagnostic BoardControllerNodeProcess::new_serialmessage_FirmwareVersion(int packet_type,unsigned char* inpacket)
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
        unsigned char major_version,minor_version,build_number;
        serialmessagehandler->decode_FirmwareVersionSerial(inpacket,&major_version,&minor_version,&build_number);
        board_firmware.Generic_Node_Name = node_firmware.Generic_Node_Name;
        board_firmware.Node_Name = node_firmware.Node_Name + ":" + my_boardname;
        board_firmware.Major_Release = major_version;
        board_firmware.Minor_Release = minor_version;
        board_firmware.Build_Number = build_number;
        board_firmware.Description = "";
        firmware_received = true;
	}
	return diagnostic;
}
icarus_rover_v2::diagnostic BoardControllerNodeProcess::new_serialmessage_Diagnostic(int packet_type,unsigned char* inpacket)
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
			unsigned char gpio_board_system,gpio_board_subsystem,gpio_board_component,gpio_board_diagtype,gpio_board_level,gpio_board_message;
			serialmessagehandler->decode_DiagnosticSerial(inpacket,&gpio_board_system,&gpio_board_subsystem,&gpio_board_component,&gpio_board_diagtype,&gpio_board_level,&gpio_board_message);
			if(gpio_board_level > INFO)
			{
				diagnostic.Level = gpio_board_level;
				diagnostic.Diagnostic_Message = gpio_board_level;
				diagnostic.Description = "";
			}
            if(gpio_board_level > NOTICE)
            {
                ready_to_arm = false;
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
icarus_rover_v2::diagnostic BoardControllerNodeProcess::new_serialmessage_Get_ANA_Port(int packet_type,unsigned char* inpacket)
{
	/*
	bool status = gather_message_info(SERIAL_Get_ANA_Port_ID, "receive");
	if(status == false)
	{
		diagnostic.Level = ERROR;
		diagnostic.Diagnostic_Type = COMMUNICATIONS;
		diagnostic.Diagnostic_Message = DROPPING_PACKETS;
		diagnostic.Description = "Get_ANA_PORT not received correctly.";
	}
	else
	{
		if(packet_type ==SERIAL_Get_ANA_Port_ID)
		{
			char tempstr[128];
			int value1,value2,value3,value4;
			serialmessagehandler->decode_Get_ANA_PortSerial(inpacket,&value1,&value2,&value3,&value4);
			if(ANA_Port.Mode[0] == PINMODE_ANALOG_INPUT)
			{
				ANA_Port.Value[0] = value1;
			}
			else if(ANA_Port.Mode[0] == PINMODE_FORCESENSOR_INPUT)
			{
				ANA_Port.Value[0] = transducer_model(ANA_PORT.Mode[0],ANA_Port.ConnectingDevice.at(0),(double)(value1));
			}

			if(ANA_PORT.Mode[1] == PINMODE_ANALOG_INPUT)
			{
				ANA_PORT.Value[1] = value2;
			}
			else if(ANA_PORT.Mode[1] == PINMODE_FORCESENSOR_INPUT)
			{
				ANA_PORT.Value[1] = transducer_model(ANA_PORT.Mode[1],ANA_PORT.ConnectingDevice.at(1),(double)(value2));
			}

			if(ANA_PORT.Mode[2] == PINMODE_ANALOG_INPUT)
			{
				ANA_PORT.Value[2] = value3;
			}
			else if(ANA_PORT.Mode[2] == PINMODE_FORCESENSOR_INPUT)
			{
				ANA_PORT.Value[2] = transducer_model(ANA_PORT.Mode[2],ANA_PORT.ConnectingDevice.at(2),(double)(value3));
			}

			if(ANA_PORT.Mode[3] == PINMODE_ANALOG_INPUT)
			{
				ANA_PORT.Value[3] = value4;
			}
			else if(ANA_PORT.Mode[3] == PINMODE_FORCESENSOR_INPUT)
			{
				ANA_PORT.Value[3] = transducer_model(ANA_PORT.Mode[3],ANA_PORT.ConnectingDevice.at(3),(double)(value4));
			}
		}
		else
		{
			diagnostic.Level = ERROR;
			diagnostic.Description = "Get_ANA_PORT Not Decoded successfully.";
			diagnostic.Diagnostic_Message = DROPPING_PACKETS;
		}
	}
	*/
	return diagnostic;
}

icarus_rover_v2::diagnostic BoardControllerNodeProcess::new_serialmessage_Get_DIO_Port(int packet_type,unsigned char* inpacket)
{
	bool status = gather_message_info(SERIAL_Get_DIO_Port_ID, "receive");
	if(status == false)
	{
		diagnostic.Level = ERROR;
		diagnostic.Diagnostic_Type = COMMUNICATIONS;
		diagnostic.Diagnostic_Message = DROPPING_PACKETS;
		diagnostic.Description = "Get_DIO_Port not received correctly.";
	}
	else
	{
		diagnostic.Level = WARN;
		diagnostic.Diagnostic_Type = COMMUNICATIONS;
		diagnostic.Diagnostic_Message = DROPPING_PACKETS;
		diagnostic.Description = "Get_DIO_Port Message Not Implemented Yet.";
	}
	return diagnostic;
}
icarus_rover_v2::diagnostic BoardControllerNodeProcess::new_serialmessage_UserMessage(int packet_type,unsigned char* inpacket)
{
	if(packet_type ==SERIAL_UserMessage_ID)
	{
		unsigned char value1,value2,value3,value4,value5,value6,value7,value8,value9,value10,value11,value12;
		serialmessagehandler->decode_UserMessageSerial(inpacket,&value1,
																&value2,
																&value3,
																&value4,
																&value5,
																&value6,
																&value7,
																&value8,
																&value9,
																&value10,
																&value11,
																&value12);
		//printf("got: %d %d\n",value1,value2);
	}
	return diagnostic;
}
icarus_rover_v2::diagnostic BoardControllerNodeProcess::new_serialmessage_Get_Mode(int packet_type,unsigned char* inpacket)
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
			char tempstr[255];
			unsigned char value1,value2,value3;
			serialmessagehandler->decode_ModeSerial(inpacket,&value1,&value2,&value3);
			board_state = value3;
			/*
			printf("Board Mode: %s\n",map_mode_ToString(board_state).c_str());
			printf("Name: %s\n",my_boardname.c_str());
			printf("ID: %d\n",get_boardid());
			printf("Node Mode: %s\n",map_mode_ToString(node_state).c_str());
			sprintf(tempstr,"Got Board Mode: %s from Board: %s:%d.  Node Mode is: %s",
					map_mode_ToString(board_state).c_str(),
					my_boardname.c_str(),
					get_boardid(),
					map_mode_ToString(node_state).c_str());
			printf("%s\n",tempstr);
			*/
			diagnostic.Level = NOERROR;
			diagnostic.Description = "Board Mode Decoded successfully.";
			diagnostic.Diagnostic_Message = NOERROR;
		}
		else
		{
			diagnostic.Level = ERROR;
			diagnostic.Description = "Board Mode Not Decoded successfully.";
			diagnostic.Diagnostic_Message = DROPPING_PACKETS;
		}
	}
	return diagnostic;
}
double BoardControllerNodeProcess::find_slope(std::vector<double> x,std::vector<double> y)
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
double BoardControllerNodeProcess::find_intercept(double slope,std::vector<double> x,std::vector<double> y)
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
bool BoardControllerNodeProcess::configure_port(int ShieldID,std::vector<icarus_rover_v2::pin> pins)
{
	//Port 1: Pin Numbers 0-7
	//Port 2: Pin Numbers 8-15
	//Port 3: Pin Numbers 16-23...
	bool status = true;
	int port_offsetcount = myports.size();
	if(pins.size() == 0) { return true; }
	std::vector<int> pinnumbers;
	for(int i = 0; i < pins.size();i++)
	{
		pinnumbers.push_back(pins.at(i).Number);
	}
	int high_pinnumber = *std::max_element(pinnumbers.begin(),pinnumbers.end());

	int port_count = (high_pinnumber/PORT_SIZE)+1; //Use pin number to determine port ID
	int pinindex = 0;

	for(int p = 1; p <= port_count; p++)
	{
		Port_Info newport;
		newport.PortID = p;
		newport.ShieldID = ShieldID;
		newport.Updated = false;
		for(int j = 0; j < PORT_SIZE;j++)
		{

			newport.Available.push_back(0);
			newport.ConnectingDevice.push_back("");
			newport.DefaultValue.push_back(0);//
			newport.Mode.push_back(PINMODE_UNDEFINED);
			newport.Number.push_back(pinindex++);
			newport.Value.push_back(0);
		}
		myports.push_back(newport);
	}
	for(int i = 0; i < pins.size();i++)
	{
		int p = (pins.at(i).Number/PORT_SIZE);;
		int portpin_index = pins.at(i).Number-(PORT_SIZE*(p));
		myports.at(p+port_offsetcount).Number.at(portpin_index) = pins.at(i).Number;
		myports.at(p+port_offsetcount).Mode.at(portpin_index) = map_PinFunction_ToInt(pins.at(i).Function);
		if(myports.at(p+port_offsetcount).Mode.at(portpin_index) == PINMODE_UNDEFINED)
		{
			status = false;
		}
		myports.at(p+port_offsetcount).DefaultValue.at(portpin_index) = pins.at(i).DefaultValue;
		myports.at(p+port_offsetcount).ConnectingDevice.at(portpin_index) =  pins.at(i).ConnectedDevice;
		myports.at(p+port_offsetcount).Available.at(portpin_index) = 1;
		myports.at(p+port_offsetcount).Value.at(portpin_index) = pins.at(i).DefaultValue;
	}

	for(int i = 0; i < myports.size();i++)
	{
		for(int j = 0; j < PORT_SIZE;j++)
		{
			char tempstr[512];
			sprintf(tempstr,"Configured Pin for Shield: %d Port: %d Pin: %d Mode: %d Default: %d Conn: %s Av: %d",
					myports.at(i).ShieldID,
					myports.at(i).PortID,
					myports.at(i).Number.at(j),
					myports.at(i).Mode.at(j),
					myports.at(i).DefaultValue.at(j),
					myports.at(i).ConnectingDevice.at(j).c_str(),
					myports.at(i).Available.at(j));
		}
	}
	return status;
}
std::vector<int> BoardControllerNodeProcess::get_portlist(int ShieldID)
{
	std::vector<int> portlist;
	for(int i = 0; i < myports.size();i++)
	{
		if(myports.at(i).ShieldID == ShieldID)
		{
			portlist.push_back(myports.at(i).PortID);
		}
	}
	return portlist;
}
icarus_rover_v2::diagnostic BoardControllerNodeProcess::new_devicemsg(icarus_rover_v2::device newdevice)
{
	printf("Got device msg: %s: %d\n",newdevice.DeviceName.c_str(),(int)newdevice.ID);
	std::size_t board_message = newdevice.DeviceType.find("Board");
	std::size_t shield_message = newdevice.DeviceType.find("Shield");
	if(board_message != std::string::npos)
	{
		if(newdevice.ID == get_boardid())
		{
			my_boardname = newdevice.DeviceName;
			shield_count = newdevice.ShieldCount;
		}
	}

	else if ((shield_message!=std::string::npos) && (my_boardname != ""))
	{
		if(my_boardname == newdevice.DeviceParent)
		{
			bool add_me = true;
			for(int i = 0; i < myshields.size();i++)
			{
				if(myshields.at(i).ID == newdevice.ID)
				{

					add_me = false;
				}
			}
			if(add_me == true)
			{
				myshields.push_back(newdevice);
				char tempstr[255];
				sprintf(tempstr,"Adding Shield: %s:%d to Board: %s:%d",
						newdevice.DeviceName.c_str(),
						(int)newdevice.ID,
						my_boardname.c_str(),
						(int)get_boardid());
				bool status =  configure_port(newdevice.ID,newdevice.pins);
				if(status == false)
				{
					diagnostic.Diagnostic_Type = SENSORS;
					diagnostic.Level = FATAL;
					diagnostic.Diagnostic_Message = DEVICE_NOT_AVAILABLE;
					char tempstr[1024];
					sprintf(tempstr,"Unable to configure Port/Pin Combination for: %s:%d",newdevice.DeviceName.c_str(),(int)newdevice.ID);
					diagnostic.Description = tempstr;
					return diagnostic;
				}


			}
		}

	}
	else
	{
		char tempstr[255];
		sprintf(tempstr,"Device: %s not currently supported.",newdevice.DeviceName.c_str());
	}


	/*
	printf("Board info recvd: %d sensor info recvd: %d all info recvd: %d\n",
			all_board_info_received,
			all_sensor_info_received,
			all_device_info_received);

	if( (all_board_info_received == true) &&
		(all_sensor_info_received == true) &&
		(all_device_info_received == false))
	{
		mylogger->log_info("Received all Device Info.");
		all_device_info_received = true;
	}
	else if((newdevice.DeviceParent == "None") && (newdevice.DeviceName == myhostname))// && (all_device_info_received == false))
	{
		mydevice = newdevice;
		if(mydevice.SensorCount == 0)
		{
			all_sensor_info_received = true;
		}
	}

	else if(((newdevice.DeviceType == "ArduinoBoard"))
			&& (newdevice.DeviceParent == mydevice.DeviceName) && (mydevice.DeviceName != "") && (all_board_info_received == false))
	{
		icarus_rover_v2::device board;
		board.DeviceName = newdevice.DeviceName;
		board.pins = newdevice.pins;
		myboards.push_back(board);
		//printf("Board Count so far: %d\n",myboards.size());
		//printf("Board pin count: %d\n",board.pins.size());
		for(int i = 0; i < board.pins.size();i++)
		{
			if(configure_pin(board.DeviceName,board.pins.at(i).Port,board.pins.at(i).Number,board.pins.at(i).Function,board.pins.at(i).ConnectedDevice,(uint8_t)board.pins.at(i).DefaultValue)==false)
			{
				char tempstr[256];
				sprintf(tempstr,"Board: %s Couldn't Configure Pin on Port: %s Number: %d Function: %s Connecting Device: %s Default Value: %d",
						board.DeviceName.c_str(),
						board.pins.at(i).Port.c_str(),
						board.pins.at(i).Number,
						board.pins.at(i).Function.c_str(),
						board.pins.at(i).ConnectedDevice.c_str(),
						board.pins.at(i).DefaultValue);
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
				sprintf(tempstr,"Board: %s Configured Pin on Port: %s Number: %d Function: %s Connected Device: %s Default Value: %d",
						board.DeviceName.c_str(),
						board.pins.at(i).Port.c_str(),
						board.pins.at(i).Number,
						board.pins.at(i).Function.c_str(),
						board.pins.at(i).ConnectedDevice.c_str(),
						board.pins.at(i).DefaultValue);
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
			node_state = BOARDMODE_INITIALIZING;
		}
	}
	else if((newdevice.DeviceType == "ForceSensor") && (newdevice.DeviceParent == mydevice.DeviceName) && (mydevice.DeviceName != "") && (all_sensor_info_received == false))
	{
		icarus_rover_v2::device dev_sensor;
		dev_sensor.DeviceName = newdevice.DeviceName;
		mysensors.push_back(dev_sensor);
		//printf("Device Name: %s\n",newdevice.DeviceName.c_str());
		//printf("Sensor Count so far: %d/%d\n",mysensors.size(),mydevice.SensorCount);
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
			node_state = BOARDMODE_INITIALIZING;
		}
	}
	*/
	return diagnostic;
}
/*bool BoardControllerNodeProcess::configure_pin(std::string ShieldName,int PortID, uint8_t Number, std::string Function,std::string ConnectedDevice,uint8_t defaultvalue)
{
	bool status = true;
	int function = map_PinFunction_ToInt(Function);
	if(function == PINMODE_UNDEFINED) { return false; }
	if((Number < 1) || (Number > 8)) { return false; }
	if(Port == "DIO_Port")
	{
		DIO_PortA.Number[Number-1] = Number;
		DIO_PortA.Mode[Number-1] = function;
		DIO_PortA.Available[Number-1] = true;
		DIO_PortA.Value[Number-1] = 0;
		DIO_PortA.DefaultValue[Number-1] = DefaultValue;
		DIO_PortA.ConnectingDevice.at(Number-1) = ConnectedDevice;
	}
	else if(Port == "ANA_Port")
	{
		ANA_PORT.Number[Number-1] = Number;
		ANA_PORT.Mode[Number-1] = function;
		ANA_PORT.Available[Number-1] = true;
		ANA_PORT.Value[Number-1] = 0;
		ANA_PORT.DefaultValue[Number-1] = DefaultValue;
		ANA_PORT.ConnectingDevice.at(Number-1) = ConnectedDevice;
	}
	else { 	status = false; }

	return status;
}
*/
std::string BoardControllerNodeProcess::map_mode_ToString(int mode)
{

	switch(mode)
	{
	case BOARDMODE_UNDEFINED: 					return "Undefined";					break;
	case BOARDMODE_BOOT:						return "Boot";						break;
	case BOARDMODE_INITIALIZING:				return "Initializing";				break;
	case BOARDMODE_SHIELDS_CONFIGURED:			return "Shields Configured";		break;
	case BOARDMODE_INITIALIZED:					return "Initialized";				break;
	case BOARDMODE_RUNNING:						return "Running";					break;
	case BOARDMODE_STOPPED:						return "Stopped";					break;
	default:
		std::string tempstr;
		tempstr = "Mode: " + boost::lexical_cast<std::string>(mode) + " Not Supported";
		return tempstr;
	}
}
int BoardControllerNodeProcess::map_DeviceType_ToInt(std::string devicetype)
{
	if(devicetype == "ServoShield")						{	return SHIELDTYPE_SERVOSHIELD; 				}
	else if(devicetype == "RelayShield")				{ 	return SHIELDTYPE_RELAYSHIELD;				}
	else												{	return SHIELDTYPE_UNDEFINED;				}


}
std::string BoardControllerNodeProcess::map_PinFunction_ToString(int function)
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
	else if(Function == "ForceSensorInput")				{	return PINMODE_FORCESENSOR_INPUT;			}
	else if(Function == "UltraSonicSensorInput")		{	return PINMODE_ULTRASONIC_INPUT;			}
	else if(Function == "PWMOutput")					{	return PINMODE_PWM_OUTPUT;					}
	else if(Function == "PWMOutput-NonActuator") 		{ 	return PINMODE_PWM_OUTPUT_NON_ACTUATOR; 	}
	else if(Function == "DigitalOutput-NonActuator")	{ 	return PINMODE_DIGITAL_OUTPUT_NON_ACTUATOR;	}
	else 												{ 	return PINMODE_UNDEFINED; 					}
}
/*
Port_Info BoardControllerNodeProcess::get_PortInfo(std::string BoardName,std::string PortName)
{

	for(int i = 0; i<myboards.size();i++)
	{
		if(myboards.at(i).DeviceName == BoardName)
		{
			if		(PortName == "DIO_Port") { return DIO_Port; }
			else if	(PortName == "ANA_Port") { return ANA_Port; }
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
*/
void BoardControllerNodeProcess::initialize_stateack_messages()
{
	send_configure_DIO_Ports.name = "Send Configure DIO Ports";
	send_configure_DIO_Ports.trigger = false;
	send_configure_DIO_Ports.state = false;
	gettimeofday(&send_configure_DIO_Ports.orig_send_time,NULL);
	send_configure_DIO_Ports.retries = 0;
	send_configure_DIO_Ports.timeout_counter = 0;
	send_configure_DIO_Ports.retry_mode = false;
	send_configure_DIO_Ports.failed = false;
	send_configure_DIO_Ports.flag1 = 0; //This flag represents the Board Index
	send_configure_DIO_Ports.stream_rate = -1.0;  //Don't stream this

	send_configure_shields.name = "Send Configure Shields";
	send_configure_shields.trigger = false;
	send_configure_shields.state = false;
	gettimeofday(&send_configure_shields.orig_send_time,NULL);
	send_configure_shields.retries = 0;
	send_configure_shields.timeout_counter = 0;
	send_configure_shields.retry_mode = false;
	send_configure_shields.failed = false;
	send_configure_shields.flag1 = 0;
	send_configure_shields.stream_rate = -1.0;  //Don't stream this

	send_testmessage_command.name = "Send Test Message Command";
	send_testmessage_command.trigger = false;
	send_testmessage_command.state = false;
	gettimeofday(&send_testmessage_command.orig_send_time,NULL);
	send_testmessage_command.retries = 0;
	send_testmessage_command.timeout_counter = 0;
	send_testmessage_command.retry_mode = false;
	send_testmessage_command.failed = false;
	send_testmessage_command.stream_rate = -1.0;  //Don't stream this

	send_nodemode.name = "Send Node Mode";
	send_nodemode.trigger = false;
	send_nodemode.state = false;
	gettimeofday(&send_nodemode.orig_send_time,NULL);
	send_nodemode.retries = 0;
	send_nodemode.timeout_counter = 0;
	send_nodemode.retry_mode = false;
	send_nodemode.failed = false;
	send_nodemode.stream_rate = 2.0;

	send_set_DIO_Port.name = "Send Set DIO Port";
	send_set_DIO_Port.trigger = false;
	send_set_DIO_Port.state = false;
	gettimeofday(&send_set_DIO_Port.orig_send_time,NULL);
	send_set_DIO_Port.retries = 0;
	send_set_DIO_Port.timeout_counter = 0;
	send_set_DIO_Port.retry_mode = false;
	send_set_DIO_Port.failed = false;
	send_set_DIO_Port.stream_rate = -1.0; //Don't stream this, only send on Callback

	send_armedcommand.name = "Send Arm Command";
	send_armedcommand.trigger = false;
	send_armedcommand.state = false;
	gettimeofday(&send_armedcommand.orig_send_time,NULL);
	send_armedcommand.retries = 0;
	send_armedcommand.timeout_counter = 0;
	send_armedcommand.retry_mode = false;
	send_armedcommand.failed = false;
	send_armedcommand.stream_rate = 10.0;

	send_armedstate.name = "Send Armed State";
	send_armedstate.trigger = false;
	send_armedstate.state = false;
	gettimeofday(&send_armedstate.orig_send_time,NULL);
	send_armedstate.retries = 0;
	send_armedstate.timeout_counter = 0;
	send_armedstate.retry_mode = false;
	send_armedstate.failed = false;
	send_armedstate.stream_rate = 2.0;

	send_defaultvalue_DIO_Port.name = "Send DefaultValue DIO Port";
	send_defaultvalue_DIO_Port.trigger = false;
	send_defaultvalue_DIO_Port.state = false;
	gettimeofday(&send_defaultvalue_DIO_Port.orig_send_time,NULL);
	send_defaultvalue_DIO_Port.retries = 0;
	send_defaultvalue_DIO_Port.timeout_counter = 0;
	send_defaultvalue_DIO_Port.retry_mode = false;
	send_defaultvalue_DIO_Port.failed = false;
	send_defaultvalue_DIO_Port.flag1 = 0; //This flag represents the Board Index
	send_defaultvalue_DIO_Port.stream_rate = -1.0;

}
bool BoardControllerNodeProcess::gather_message_info(int id, std::string mode)
{
	bool found = false;
	for(int i = 0; i < messages.size();i++)
	{
		if(messages.at(i).id == id)
		{
			found = true;
			if(mode == "transmit")
			{
				struct timeval now;
				gettimeofday(&now,NULL);
				double run_time = time_diff(init_time,now);
				messages.at(i).sent_counter++;
                messages.at(i).transmitted_rate = (double)(messages.at(i)).sent_counter/run_time;
				gettimeofday(&messages.at(i).last_time_transmitted,NULL);
			}
			else if(mode == "receive")
			{
				struct timeval now;
				gettimeofday(&now,NULL);
				double run_time = time_diff(init_time,now);
				messages.at(i).received_counter++;
                messages.at(i).received_rate = (double)(messages.at(i)).received_counter/run_time;
				gettimeofday(&messages.at(i).last_time_received,NULL);
			}
			break;
		}
	}
	return found;
}
double BoardControllerNodeProcess::time_diff(struct timeval timea, struct timeval timeb)
{
	long mtime, seconds, useconds;
	seconds  = timeb.tv_sec  - timea.tv_sec;
	useconds = timeb.tv_usec - timea.tv_usec;

	mtime = ((seconds) * 1000 + useconds/1000.0) + 0.5;
	return (double)(mtime)/1000.0;

}
void BoardControllerNodeProcess::initialize_message_info()
{
	{
		message_info newmessage;
		newmessage.id = SERIAL_Diagnostic_ID;
		newmessage.name = "Diagnostic";
		messages.push_back(newmessage);
	}

	{
		message_info newmessage;
		newmessage.id = SERIAL_TestMessageCounter_ID;
		newmessage.name = "Test Message Counter";
		messages.push_back(newmessage);
	}

	{
		message_info newmessage;
		newmessage.id = SERIAL_TestMessageCommand_ID;
		newmessage.name = "Test Message Command";
		messages.push_back(newmessage);
	}

	{
		message_info newmessage;
		newmessage.id = SERIAL_Configure_DIO_Port_ID;
		newmessage.name = "Configure DIO Port";
		messages.push_back(newmessage);
	}

	{
		message_info newmessage;
		newmessage.id = SERIAL_Mode_ID;
		newmessage.name = "Mode";
		messages.push_back(newmessage);
	}

	{
		message_info newmessage;
		newmessage.id = SERIAL_Set_DIO_Port_ID;
		newmessage.name = "Set DIO Port";
		messages.push_back(newmessage);
	}

	{
		message_info newmessage;
		newmessage.id = SERIAL_Get_ANA_Port_ID;
		newmessage.name = "Get ANA Port";
		messages.push_back(newmessage);
	}

	{
		message_info newmessage;
		newmessage.id = SERIAL_Get_DIO_Port_ID;
		newmessage.name = "Get DIO Port";
		messages.push_back(newmessage);
	}

	{
		message_info newmessage;
		newmessage.id = SERIAL_FirmwareVersion_ID;
		newmessage.name = "Firmware";
		messages.push_back(newmessage);
	}

	{
		message_info newmessage;
		newmessage.id = SERIAL_Arm_Command_ID;
		newmessage.name = "Arm Command";
		messages.push_back(newmessage);
	}

	{
		message_info newmessage;
		newmessage.id = SERIAL_Arm_Status_ID;
		newmessage.name = "Arm Status";
		messages.push_back(newmessage);
	}

	{
		message_info newmessage;
		newmessage.id = SERIAL_Set_DIO_Port_DefaultValue_ID;
		newmessage.name = "Set DIO Port Default Value";
		messages.push_back(newmessage);
	}

	{
		message_info newmessage;
		newmessage.id = SERIAL_Configure_Shield_ID;
		newmessage.name = "Configure Shield";
		messages.push_back(newmessage);
	}

	for(int i = 0; i < messages.size(); i++)
	{
		messages.at(i).protocol = "serial";
		//messages.at(i).last_time_received = ros::Time::now();
		//messages.at(i).last_time_transmitted = ros::Time::now();
		messages.at(i).received_counter = 0;
		messages.at(i).sent_counter = 0;
		messages.at(i).received_rate = 0.0;
		messages.at(i).transmitted_rate = 0.0;
	}
}
int BoardControllerNodeProcess::transducer_model(int mode,std::string SensorName,double input)
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
