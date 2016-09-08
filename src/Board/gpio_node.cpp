#include "gpio_node.h"
//Start User Code: Functions
bool initialize_gpioboard_ports()
{
	ANA_PortA.PortName = "ANA_PortA";
	ANA_PortB.PortName = "ANA_PortB";
	DIO_PortA.PortName = "DIO_PortA";
	DIO_PortB.PortName = "DIO_PortB";
	for(int i = 0; i < 4; i++) //These Pins can only be used as an ANALOG INPUT
	{
		ANA_PortA.Value[i] = 0;
		ANA_PortA.Mode[i] = PINMODE_ANALOG_INPUT;
		ANA_PortA.Available[i] = false;
	}
	for(int i = 4; i < 8; i++) //These Pins are not used on the current GPIO Board.
	{
		ANA_PortA.Value[i] = 0;
		ANA_PortA.Mode[i] = PINMODE_UNDEFINED;
		ANA_PortA.Available[i] = false;
	}

	for(int i = 0; i < 4; i++) //These Pins can only be used as an ANALOG INPUT or as a FORCESENSOR INPUT
	{
		ANA_PortB.Value[i] = 0;
		ANA_PortB.Mode[i] = PINMODE_UNDEFINED;
		ANA_PortB.Available[i] = false;
	}
	for(int i = 4; i < 8; i++) //These Pins are not used on the current GPIO Board.
	{
		ANA_PortB.Value[i] = 0;
		ANA_PortB.Mode[i] = PINMODE_UNDEFINED;
		ANA_PortB.Available[i] = false;
	}
	for(int i = 0; i < 8; i++)
	{
		DIO_PortA.Value[i] = 0;
		DIO_PortA.Mode[i] = PINMODE_UNDEFINED;
		DIO_PortA.Available[i] = false;
		DIO_PortB.Value[i] = 0;
		DIO_PortB.Mode[i] = PINMODE_UNDEFINED;
		DIO_PortB.Available[i] = false;
	}
	return true;
}
std::string map_PinFunction_ToString(int function)  //Put in helper class
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
int map_PinFunction_ToInt(std::string Function)  //Put in helper class
{
	if(Function == "DigitalInput")				{	return PINMODE_DIGITAL_INPUT;		}
	else if(Function == "DigitalOutput")		{	return PINMODE_DIGITAL_OUTPUT;		}
	else if(Function == "AnalogInput")			{	return PINMODE_ANALOG_INPUT;		}
	else if(Function == "ForceSensorInput")		{	return PINMODE_FORCESENSOR_INPUT;	}
	else if(Function == "UltraSonicSensorInput"){	return PINMODE_ULTRASONIC_INPUT;	}
	else if(Function == "PWMOutput")			{	return PINMODE_PWM_OUTPUT;			}
	else { return PINMODE_UNDEFINED; }
}
bool configure_pin(std::string BoardName,std::string Port, uint8_t Number, std::string Function)
{
	bool status = true;
	int function = map_PinFunction_ToInt(Function);
	if(function == PINMODE_UNDEFINED) { return false; }
	if((Number < 1) || (Number > 8)) { return false; }

	if(Port == "DIO_PortA")
	{
		DIO_PortA.Mode[Number-1] = function;
		DIO_PortA.Available[Number-1] = true;
	}
	else if(Port == "DIO_PortB")
	{
		DIO_PortB.Mode[Number-1] = function;
		DIO_PortB.Available[Number-1] = true;
	}
	else if(Port == "ANA_PortA")
	{
		ANA_PortA.Mode[Number-1] = function;
		ANA_PortA.Available[Number-1] = true;
	}
	else if(Port == "ANA_PortB")
	{
		ANA_PortB.Mode[Number-1] = function;
		ANA_PortB.Available[Number-1] = true;
	}
	else { 	status = false; }
	return status;
}
void process_message_thread()
{
	while(1)
	{
		unsigned char rx_buffer[64];
		
		memset(rx_buffer, 0, sizeof(rx_buffer));
		//memset(packet,0,sizeof(packet));
		//ros::Time start_time = ros::Time::now();
		int rx_length = read(device_fid, rx_buffer, sizeof(rx_buffer));
		
		for(int i = 0; i < rx_length; i++)
		{
			if((rx_buffer[i] == 0xAB) and (message_started == false))
			{
				memset(message_buffer, 0, sizeof(message_buffer));
				message_started = true;
				message_completed = false;
				message_buffer_index = 0;
				message_buffer[message_buffer_index++] = rx_buffer[i];	
			}
			else if(message_started == true)
			{
				message_buffer[message_buffer_index++] = rx_buffer[i];	
				if(message_buffer_index == 12)
				{
					message_completed = true;
					
					message_started = false;
				}
			}
		}
		if(message_completed == true)
		{
			packet_length = message_buffer[2];
			int checksum = 0;
			for(int i = 3; i < 3+packet_length;i++)
			{
				packet[i-3] = message_buffer[i];
				checksum ^= packet[i-3];
			}
			int recv_checksum = message_buffer[11];
			if(recv_checksum == checksum)
			{
				good_checksum_counter++;
				packet_type = message_buffer[1];
				new_message = true;
			}
			else
			{
				bad_checksum_counter++;
			}
		}
	}
}
bool run_fastrate_code()
{
	/*
	unsigned char rx_buffer[64];
	unsigned char packet[8];
	memset(rx_buffer, 0, sizeof(rx_buffer));
	//memset(packet,0,sizeof(packet));
	//ros::Time start_time = ros::Time::now();
	int rx_length = read(device_fid, rx_buffer, sizeof(rx_buffer));
	
	for(int i = 0; i < rx_length; i++)
	{
		if((rx_buffer[i] == 0xAB) and (message_started == false))
		{
			memset(message_buffer, 0, sizeof(message_buffer));
			message_started = true;
			message_completed = false;
			message_buffer_index = 0;
			message_buffer[message_buffer_index++] = rx_buffer[i];	
		}
		else if(message_started == true)
		{
			message_buffer[message_buffer_index++] = rx_buffer[i];	
			if(message_buffer_index == 12)
			{
				message_completed = true;
				
				message_started = false;
			}
		}
	}
	if(message_completed == true)
	{
		packet_length = message_buffer[2];
		int checksum = 0;
		for(int i = 3; i < 3+packet_length;i++)
		{
			packet[i-3] = message_buffer[i];
			checksum ^= packet[i-3];
		}
		int recv_checksum = message_buffer[11];
		if(recv_checksum == checksum)
		{
			good_checksum_counter++;
			packet_type = message_buffer[1];
			new_message = true;
		}
		else
		{
			bad_checksum_counter++;
		}
	}
	*/
	if(new_message == true)
	{
		new_message = false;
		if(packet_type ==SERIAL_TestMessageCounter_ID)
		{
			logger->log_debug("Got TestMessageCounter from GPIO Board.");
			char value1,value2,value3,value4,value5,value6,value7,value8;
			serialmessagehandler->decode_TestMessageCounterSerial(packet,&value1,&value2,&value3,&value4,&value5,&value6,&value7,&value8);
			//printf("TestMessage Counter: 1: %d 2: %d 3: %d 4: %d 5: %d 6: %d 7: %d 8: %d\r\n",value1,value2,value3,value4,value5,value6,value7,value8);
			last_num = current_num;
			current_num = value1;
			int num = abs(current_num-last_num)-1;
			if(num > 150) { num = 0; }
			if(num < 0){ num = 0; }
			missed_counter += num;
			if(checking_gpio_comm == true)
			{
				if(num == 0) { message_receive_counter++; }
			}
			//if(num > 0){printf("M: %d/%d/%d\r\n",current_num,last_num,num);};
			
		}
		if(packet_type == SERIAL_FirmwareVersion_ID)
		{
			logger->log_debug("Got FirmwareVersion from GPIO Board.");
			char majorVersion,minorVersion,buildNumber;
			serialmessagehandler->decode_FirmwareVersionSerial(packet,&majorVersion,&minorVersion,&buildNumber);
			char tempstr[128];
			sprintf(tempstr,"GPIO Board Major Version: %d Minor Version: %d Build Number: %d",majorVersion,minorVersion,buildNumber);
			logger->log_notice(tempstr);
			diagnostic_status.Level = NOTICE;
			diagnostic_status.Diagnostic_Message = INITIALIZING;
			diagnostic_status.Description = tempstr;
			diagnostic_pub.publish(diagnostic_status);
			
		}
		if(packet_type == SERIAL_Diagnostic_ID)
		{
			logger->log_debug("Got Diagnostics from GPIO Board.");
			char gpio_board_system,gpio_board_subsystem,gpio_board_component,gpio_board_diagtype,gpio_board_level,gpio_board_message;
			serialmessagehandler->decode_DiagnosticSerial(packet,&gpio_board_system,&gpio_board_subsystem,&gpio_board_component,&gpio_board_diagtype,&gpio_board_level,&gpio_board_message);
			diagnostic_status.Level = gpio_board_level;
			diagnostic_status.Diagnostic_Message = gpio_board_level;
			diagnostic_status.Description = "GPIO Board Empty Message.";
			diagnostic_pub.publish(diagnostic_status);
		}
		if(packet_type == SERIAL_Get_ANA_PortA_ID)
		{
			logger->log_debug("Got ANA PortA from GPIO Board.");
			int v1,v2,v3,v4;
			int status = serialmessagehandler->decode_Get_ANA_PortASerial(packet,&v1,&v2,&v3,&v4);
			if(status == 1)
			{
				ANA_PortA.Value[0] = v1;
				ANA_PortA.Value[1] = v2;
				ANA_PortA.Value[2] = v3;
				ANA_PortA.Value[3] = v4;
			}
		}
		if(packet_type == SERIAL_Get_ANA_PortB_ID)
		{
			logger->log_debug("Got ANA PortB from GPIO Board.");
			int v1,v2,v3,v4;
			int status = serialmessagehandler->decode_Get_ANA_PortBSerial(packet,&v1,&v2,&v3,&v4);
			if(status == 1)
			{
				ANA_PortB.Value[0] = v1;
				ANA_PortB.Value[1] = v2;
				ANA_PortB.Value[2] = v3;
				ANA_PortB.Value[3] = v4;
			}
		}
		if(packet_type == SERIAL_Get_DIO_PortA_ID)
		{
			logger->log_debug("Got DIO PortA from GPIO Board.");
			char v1,v2,v3,v4,v5,v6,v7,v8;
			int status = serialmessagehandler->decode_Get_DIO_PortASerial(packet,&v1,&v2,&v3,&v4,&v5,&v6,&v7,&v8);
			if(status == 1)
			{
				if((DIO_PortA.Mode[0] == PINMODE_DIGITAL_INPUT) || (DIO_PortA.Mode[0] == PINMODE_ULTRASONIC_INPUT) || (DIO_PortA.Mode[0] == PINMODE_QUADRATUREENCODER_INPUT))
				{
					DIO_PortA.Value[0] = (int)v1;
				}
				if((DIO_PortA.Mode[1] == PINMODE_DIGITAL_INPUT) || (DIO_PortA.Mode[1] == PINMODE_ULTRASONIC_INPUT) || (DIO_PortA.Mode[1] == PINMODE_QUADRATUREENCODER_INPUT))
				{
					DIO_PortA.Value[1] = (int)v2;
				}
				if((DIO_PortA.Mode[2] == PINMODE_DIGITAL_INPUT) || (DIO_PortA.Mode[2] == PINMODE_ULTRASONIC_INPUT) || (DIO_PortA.Mode[2] == PINMODE_QUADRATUREENCODER_INPUT))
				{
					DIO_PortA.Value[2] = (int)v3;
				}
				if((DIO_PortA.Mode[3] == PINMODE_DIGITAL_INPUT) || (DIO_PortA.Mode[3] == PINMODE_ULTRASONIC_INPUT) || (DIO_PortA.Mode[3] == PINMODE_QUADRATUREENCODER_INPUT))
				{
					DIO_PortA.Value[3] = (int)v4;
				}
				if((DIO_PortA.Mode[4] == PINMODE_DIGITAL_INPUT) || (DIO_PortA.Mode[4] == PINMODE_ULTRASONIC_INPUT) || (DIO_PortA.Mode[4] == PINMODE_QUADRATUREENCODER_INPUT))
				{
					DIO_PortA.Value[4] = (int)v5;
				}
				if((DIO_PortA.Mode[5] == PINMODE_DIGITAL_INPUT) || (DIO_PortA.Mode[5] == PINMODE_ULTRASONIC_INPUT) || (DIO_PortA.Mode[5] == PINMODE_QUADRATUREENCODER_INPUT))
				{
					DIO_PortA.Value[5] = (int)v6;
				}
				if((DIO_PortA.Mode[6] == PINMODE_DIGITAL_INPUT) || (DIO_PortA.Mode[6] == PINMODE_ULTRASONIC_INPUT) || (DIO_PortA.Mode[6] == PINMODE_QUADRATUREENCODER_INPUT))
				{
					DIO_PortA.Value[6] = (int)v7;
				}
				if((DIO_PortA.Mode[7] == PINMODE_DIGITAL_INPUT) || (DIO_PortA.Mode[7] == PINMODE_ULTRASONIC_INPUT) || (DIO_PortA.Mode[7] == PINMODE_QUADRATUREENCODER_INPUT))
				{
					DIO_PortA.Value[7] = (int)v8;
				}
			}
		}
		if(packet_type == SERIAL_Get_DIO_PortB_ID)
		{
			logger->log_debug("Got DIO PortB from GPIO Board.");
			char v1,v2,v3,v4,v5,v6,v7,v8;
			int status = serialmessagehandler->decode_Get_DIO_PortBSerial(packet,&v1,&v2,&v3,&v4,&v5,&v6,&v7,&v8);
			if(status == 1)
			{
				if((DIO_PortB.Mode[0] == PINMODE_DIGITAL_INPUT) || (DIO_PortB.Mode[0] == PINMODE_ULTRASONIC_INPUT) || (DIO_PortB.Mode[0] == PINMODE_QUADRATUREENCODER_INPUT))
				{
					DIO_PortB.Value[0] = (int)v1;
				}
				if((DIO_PortB.Mode[1] == PINMODE_DIGITAL_INPUT) || (DIO_PortB.Mode[1] == PINMODE_ULTRASONIC_INPUT) || (DIO_PortB.Mode[1] == PINMODE_QUADRATUREENCODER_INPUT))
				{
					DIO_PortB.Value[1] = (int)v2;
				}
				if((DIO_PortB.Mode[2] == PINMODE_DIGITAL_INPUT) || (DIO_PortB.Mode[2] == PINMODE_ULTRASONIC_INPUT) || (DIO_PortB.Mode[2] == PINMODE_QUADRATUREENCODER_INPUT))
				{
					DIO_PortB.Value[2] = (int)v3;
				}
				if((DIO_PortB.Mode[3] == PINMODE_DIGITAL_INPUT) || (DIO_PortB.Mode[3] == PINMODE_ULTRASONIC_INPUT) || (DIO_PortB.Mode[3] == PINMODE_QUADRATUREENCODER_INPUT))
				{
					DIO_PortB.Value[3] = (int)v4;
				}
				if((DIO_PortB.Mode[4] == PINMODE_DIGITAL_INPUT) || (DIO_PortB.Mode[4] == PINMODE_ULTRASONIC_INPUT) || (DIO_PortB.Mode[4] == PINMODE_QUADRATUREENCODER_INPUT))
				{
					DIO_PortB.Value[4] = (int)v5;
				}
				if((DIO_PortB.Mode[5] == PINMODE_DIGITAL_INPUT) || (DIO_PortB.Mode[5] == PINMODE_ULTRASONIC_INPUT) || (DIO_PortB.Mode[5] == PINMODE_QUADRATUREENCODER_INPUT))
				{
					DIO_PortB.Value[5] = (int)v6;
				}
				if((DIO_PortB.Mode[6] == PINMODE_DIGITAL_INPUT) || (DIO_PortB.Mode[6] == PINMODE_ULTRASONIC_INPUT) || (DIO_PortB.Mode[6] == PINMODE_QUADRATUREENCODER_INPUT))
				{
					DIO_PortB.Value[6] = (int)v7;
				}
				if((DIO_PortB.Mode[7] == PINMODE_DIGITAL_INPUT) || (DIO_PortB.Mode[7] == PINMODE_ULTRASONIC_INPUT) || (DIO_PortB.Mode[7] == PINMODE_QUADRATUREENCODER_INPUT))
				{
					DIO_PortB.Value[7] = (int)v8;
				}
			}
		}
	}
	if(checking_gpio_comm == true)
	{
		double test_runtime = measure_time_diff(ros::Time::now(),gpio_comm_test_start);
		
		if(test_runtime > 5.0)
		{
			checking_gpio_comm = false;
			char tempstr[128];
			sprintf(tempstr,"Finished testing GPIO comm, received %d messages.",message_receive_counter);
			logger->log_debug(tempstr);
			if(message_receive_counter > 10)
			{
				icarus_rover_v2::diagnostic diag=diagnostic_status;
				diag.Diagnostic_Type = COMMUNICATIONS;
				diag.Level = NOTICE;
				diag.Diagnostic_Message = DIAGNOSTIC_PASSED;
				diag.Description = "Checked Communication w/ GPIO Board -> PASSED";
				diagnostic_pub.publish(diag);
			}
			else if(message_receive_counter > 5)
			{
				icarus_rover_v2::diagnostic diag=diagnostic_status;
				diag.Diagnostic_Type = COMMUNICATIONS;
				diag.Level = WARN;
				diag.Diagnostic_Message = DIAGNOSTIC_PASSED;
				diag.Description = "Checked Communication w/ GPIO Board -> PASSED MARGINALLY";
				diagnostic_pub.publish(diag);
			}
			else
			{
				icarus_rover_v2::diagnostic diag=diagnostic_status;
				diag.Diagnostic_Type = COMMUNICATIONS;
				diag.Level = ERROR;
				diag.Diagnostic_Message = DIAGNOSTIC_FAILED;
				diag.Description = "Checked Communication w/ GPIO Board -> FAILED";
				diagnostic_pub.publish(diag);
			}
		}
	}
	return true;
}
bool run_mediumrate_code()
{
	if(gpio_board_mode == GPIOBOARD_MODE_INITIALIZING)
	{
		unsigned char tx_buffer[12];
		int length;
		int tx_status = serialmessagehandler->encode_Configure_DIO_PortASerial(tx_buffer,&length,
		DIO_PortA.Mode[0],DIO_PortA.Mode[1],DIO_PortA.Mode[2],DIO_PortA.Mode[3],DIO_PortA.Mode[4],DIO_PortA.Mode[5],DIO_PortA.Mode[6],DIO_PortA.Mode[7]);
	
		if (tx_status == 1)
		{
			int count = write(device_fid, &tx_buffer[0], length);
			if (count < 0)
			{
				logger->log_error("UART TX error\n");
				icarus_rover_v2::diagnostic diag=diagnostic_status;
				diag.Diagnostic_Type = COMMUNICATIONS;
				diag.Level = ERROR;
				diag.Diagnostic_Message = DROPPING_PACKETS;
				diag.Description = "Cannot write to UART.";
				diagnostic_pub.publish(diag);
			}
			else
			{
				gpio_board_mode = GPIOBOARD_MODE_START;
			}
		}
		
	}
	else if(gpio_board_mode == GPIOBOARD_MODE_START)
	{
		unsigned char tx_buffer[12];
		int length;
		int tx_status = serialmessagehandler->encode_GPIO_Board_ModeSerial(tx_buffer,&length,GPIOBOARD_MODE_START);
		if (tx_status == 1)
		{
			int count = write(device_fid, &tx_buffer[0], length);
			if (count < 0)
			{
				logger->log_error("UART TX error\n");
				icarus_rover_v2::diagnostic diag=diagnostic_status;
				diag.Diagnostic_Type = COMMUNICATIONS;
				diag.Level = ERROR;
				diag.Diagnostic_Message = DROPPING_PACKETS;
				diag.Description = "Cannot write to UART.";
				diagnostic_pub.publish(diag);
			}
			
		}
		gpio_board_mode = GPIOBOARD_MODE_RUNNING;
	}
	else if((gpio_board_mode == GPIOBOARD_MODE_RUNNING) and (checking_gpio_comm == false))
	{
		unsigned char tx_buffer[12];
		int length;
		int tx_status = serialmessagehandler->encode_Set_DIO_PortASerial(tx_buffer,&length,DIO_PortA.Value[0],DIO_PortA.Value[1],DIO_PortA.Value[2],DIO_PortA.Value[3],
		DIO_PortA.Value[4],DIO_PortA.Value[5],DIO_PortA.Value[6],DIO_PortA.Value[7]);
		if (tx_status == 1)
		{
			int count = write(device_fid, &tx_buffer[0], length);
			if (count < 0)
			{
				logger->log_error("UART TX error\n");
				icarus_rover_v2::diagnostic diag=diagnostic_status;
				diag.Diagnostic_Type = COMMUNICATIONS;
				diag.Level = ERROR;
				diag.Diagnostic_Message = DROPPING_PACKETS;
				diag.Description = "Cannot write to UART.";
				diagnostic_pub.publish(diag);
			}
			
		}
		publish_port_info(DIO_PortA);
		publish_port_info(DIO_PortB);
		publish_port_info(ANA_PortA);
		publish_port_info(ANA_PortB);
		gpio_board_mode = GPIOBOARD_MODE_RUNNING;
	}
	double dropped_ratio = 100*(double)((double)bad_checksum_counter/((double)bad_checksum_counter+(double)good_checksum_counter));
	{
		
		double runtime = measure_time_diff(now,boot_time);
		double missed_rate = (double)(missed_counter)/runtime;
		char tempstr[256];
		sprintf(tempstr,"Missed Counter: %d Bad checksum: %d Good checksum: %d Dropped Ratio: %f Missed Rate: %f\r\n",missed_counter,bad_checksum_counter,good_checksum_counter,dropped_ratio,missed_rate);
		logger->log_debug(tempstr);
	}
	if(dropped_ratio < 3.0)
	{
		diagnostic_status.Diagnostic_Type = SOFTWARE;
		diagnostic_status.Level = INFO;
		diagnostic_status.Diagnostic_Message = NOERROR;
		diagnostic_status.Description = "Node Executing.";
	}
	else if((dropped_ratio >= 3.0) && (dropped_ratio < 6.0)) 
	{
		diagnostic_status.Diagnostic_Type = SOFTWARE;
		diagnostic_status.Level = WARN;
		diagnostic_status.Diagnostic_Message = DROPPING_PACKETS;
		diagnostic_status.Description = "Warning: Dropping Serial Packets.";
		logger->log_warn(diagnostic_status.Description);
	}
	else if(dropped_ratio >= 6.0) 
	{
		diagnostic_status.Diagnostic_Type = SOFTWARE;
		diagnostic_status.Level = FATAL;
		diagnostic_status.Diagnostic_Message = DROPPING_PACKETS;
		diagnostic_status.Description = "Error: Dropping Serial Packets.";
		logger->log_fatal(diagnostic_status.Description);
	}
	
	diagnostic_pub.publish(diagnostic_status);
	//char tempstr[128];
	//sprintf(tempstr,"ANA Port A p1: %d p2: %d p3: %d p4: %d Port B: p1: %d p2: %d p3: %d p4: %d",
	//ANA_PortA.Pin1_Value,ANA_PortA.Pin2_Value,ANA_PortA.Pin3_Value,ANA_PortA.Pin4_Value,
	//ANA_PortB.Pin1_Value,ANA_PortB.Pin2_Value,ANA_PortB.Pin3_Value,ANA_PortB.Pin4_Value);
	//logger->log_debug(tempstr);
	return true;
}
bool run_slowrate_code()
{
	if(device_initialized == true)
	{
		pid = get_pid();
		if(pid < 0)
		{
			logger->log_warn("Couldn't retrieve PID.");
		}
		else
		{
			if(check_resources(pid))
			{
				resource_pub.publish(resources_used);
			}
			else
			{
				logger->log_warn("Couldn't read resources used.");
			}
		}
	}
	
	
	return true;
}
std::vector<icarus_rover_v2::diagnostic> check_gpioboard_comm()
{
}
bool publish_port_info(Port_Info pi)
{
	for(int i = 0; i < 8; i++)
	{
		if(pi.Available[i] == true)
		{
			int mode = pi.Mode[i];
			icarus_rover_v2::pin newpin;
			newpin.Function = map_PinFunction_ToString(pi.Mode[i]);
			newpin.Number = i+1;
			newpin.Port = pi.PortName;
			newpin.Value = pi.Value[i];
			switch (mode)
			{
				case PINMODE_DIGITAL_INPUT:
					digitalinput_pub.publish(newpin);
					break;
				case PINMODE_ANALOG_INPUT:
					analoginput_pub.publish(newpin);
					break;
				case PINMODE_FORCESENSOR_INPUT:
					forcesensorinput_pub.publish(newpin);
					break;
				default:
					break;
			}
		}
	}
	return true;
}
bool run_veryslowrate_code()
{
	//logger->log_debug("Running very slow rate code.");
	logger->log_info("Node Running.");
	return true;
}
void DigitalOutput_Callback(const icarus_rover_v2::pin::ConstPtr& msg)
{
	if(msg->Function == "DigitalOutput")
	{
		char tempstr[128];
		sprintf(tempstr,"Got DigitalOutput for Port: %s Pin: %d for Value: %d",msg->Port.c_str(),msg->Number, msg->Value);
		logger->log_debug(tempstr);
		if(msg->Port == DIO_PortA.PortName) 		
		{ 	
			DIO_PortA.Value[msg->Number-1] = msg->Value;	
		} 	
		if(msg->Port == DIO_PortB.PortName)
		{
			DIO_PortB.Value[msg->Number-1] = msg->Value;
		}
		
	}
}
void PwmOutput_Callback(const icarus_rover_v2::pin::ConstPtr& msg)
{
	if(msg->Function == "PWMOutput")
	{
		char tempstr[128];
		sprintf(tempstr,"Got PWMOutput for Port: %s Pin: %d",msg->Port.c_str(),msg->Number);
		logger->log_debug(tempstr);
		if(msg->Port == DIO_PortA.PortName) 		
		{ 	
			logger->log_debug("Setting DIO_PortA.");
			DIO_PortA.Value[msg->Number-1] = msg->Value;	
		} 	
		if(msg->Port == DIO_PortB.PortName)
		{
			DIO_PortB.Value[msg->Number-1] = msg->Value;
		}
	}
}
//End User Code: Functions

//Start Template Code: Functions
void Command_Callback(const icarus_rover_v2::command::ConstPtr& msg)
{
	char tempstr[128];
	sprintf(tempstr,"Got Command: %d Level: %d",msg->Command,msg->Option1);
	logger->log_info(tempstr);
	if (msg->Command ==  DIAGNOSTIC_ID)
	{
		if(msg->Option1 == LEVEL1)
		{
			diagnostic_pub.publish(diagnostic_status);
		}
		else if(msg->Option1 == LEVEL2)
		{
			checking_gpio_comm = true;
			gpio_comm_test_start = ros::Time::now();
			message_receive_counter = 0;
			unsigned char tx_buffer[12];
			int length;
			int tx_status = serialmessagehandler->encode_TestMessageCommandSerial(tx_buffer,&length,0,0,0,0,0,0,0,0);
			if (tx_status == 1)
			{
				int count = write(device_fid, &tx_buffer[0], length);
				if (count < 0)
				{
					logger->log_error("UART TX error\n");
					icarus_rover_v2::diagnostic diag=diagnostic_status;
					diag.Diagnostic_Type = COMMUNICATIONS;
					diag.Level = ERROR;
					diag.Diagnostic_Message = DROPPING_PACKETS;
					diag.Description = "Cannot write to UART.";
					diagnostic_pub.publish(diag);
				}
			}
			//std::vector<icarus_rover_v2::diagnostic> diaglist1 = check_program_variables();
			//for(int i = 0; i < diaglist1.size();i++) { diagnostic_pub.publish(diaglist1.at(i)); }
			//std::vector<icarus_rover_v2::diagnostic> diaglist2 = check_gpioboard_comm();
			//for(int i = 0; i < diaglist2.size();i++) { diagnostic_pub.publish(diaglist2.at(i)); }

		}
		else if(msg->Option1 == LEVEL3)
		{

		}
		else if(msg->Option1 == LEVEL4)
		{

		}
		else
		{
			logger->log_error("Shouldn't get here!!!");
		}
	}
}
int main(int argc, char **argv)
{
 
	node_name = "gpio_node";


    ros::init(argc, argv, node_name);
    node_name = ros::this_node::getName();

    ros::NodeHandle n;
    
    if(initialize(n) == false)
    {
        logger->log_fatal("Unable to Initialize.  Exiting.");
    	diagnostic_status.Diagnostic_Type = SOFTWARE;
		diagnostic_status.Level = FATAL;
		diagnostic_status.Diagnostic_Message = INITIALIZING_ERROR;
		diagnostic_status.Description = "Node Initializing Error.";
		diagnostic_pub.publish(diagnostic_status);
        return 0; 
    }
    ros::Rate loop_rate(rate);
    now = ros::Time::now();
	boot_time = now;
    fast_timer = now;
    medium_timer = now;
    slow_timer = now;
    veryslow_timer = now;
	boost::thread processmessage_thread(&process_message_thread);
    while (ros::ok())
    {
		
    	bool ok_to_start = false;
		if(require_pps_to_start == false) { ok_to_start = true;}
		else if(require_pps_to_start == true && received_pps == true) { ok_to_start = true; }
    	if(ok_to_start == true)
    	{
    		now = ros::Time::now();
    		mtime = measure_time_diff(now,fast_timer);
			if(mtime > .001)
			{
				run_fastrate_code();
				fast_timer = ros::Time::now();
			}
			mtime = measure_time_diff(now,medium_timer);
			if(mtime > 0.1)
			{
				//printf("Executing\n");
				run_mediumrate_code();
				medium_timer = ros::Time::now();
			}
			mtime = measure_time_diff(now,slow_timer);
			if(mtime > 1.0)
			{
				run_slowrate_code();
				slow_timer = ros::Time::now();
			}
			mtime = measure_time_diff(now,veryslow_timer);
			if(mtime > 10.0)
			{
				run_veryslowrate_code();
				veryslow_timer = ros::Time::now();
			}
		}
		else
		{
			logger->log_warn("Waiting on PPS to Start.");
		}
		ros::spinOnce();
		loop_rate.sleep();
    }
	close(device_fid);
	processmessage_thread.join();
    return 0;
}

bool initialize(ros::NodeHandle nh)
{
    //Start Template Code: Initialization and Parameters
    myDevice.DeviceName = "";
    myDevice.Architecture = "";
    myDevice.DeviceParent = "";
    myDevice.DeviceType = "";
    myDevice.BoardCount = 0;
    device_initialized = false;
    pid = -1;
    std::string diagnostic_topic = "/" + node_name + "/diagnostic";
	diagnostic_pub =  nh.advertise<icarus_rover_v2::diagnostic>(diagnostic_topic,1000);
	diagnostic_status.Node_Name = node_name;
	diagnostic_status.System = ROVER;
	diagnostic_status.SubSystem = ROBOT_CONTROLLER;
	diagnostic_status.Component = GPIO_NODE;

	diagnostic_status.Diagnostic_Type = NOERROR;
	diagnostic_status.Level = INFO;
	diagnostic_status.Diagnostic_Message = INITIALIZING;
	diagnostic_status.Description = "Node Initializing";
	diagnostic_pub.publish(diagnostic_status);

	std::string resource_topic = "/" + node_name + "/resource";
	resource_pub = nh.advertise<icarus_rover_v2::resource>(resource_topic,1000);

    std::string param_verbosity_level = node_name +"/verbosity_level";
    if(nh.getParam(param_verbosity_level,verbosity_level) == false)
    {
        logger = new Logger("WARN",ros::this_node::getName());
        logger->log_warn("Missing Parameter: verbosity_level");
        return false;
    }
    else
    {
        logger = new Logger(verbosity_level,ros::this_node::getName());      
    }
    std::string param_loop_rate = node_name +"/loop_rate";
    if(nh.getParam(param_loop_rate,rate) == false)
    {
        logger->log_warn("Missing Parameter: loop_rate.");
        return false;
    }
    char hostname[1024];
	hostname[1023] = '\0';
	gethostname(hostname,1023);
    std::string device_topic = "/" + std::string(hostname) +"_master_node/device";
	printf("Subscribing to: %s\n",device_topic.c_str());
    device_sub = nh.subscribe<icarus_rover_v2::device>(device_topic,1000,Device_Callback);
	if(device_sub)
	{
		printf("Created subscriber\n");
	}
	else
	{
		printf("Couldn't create subscriber\n");
	}
	
    pps_sub = nh.subscribe<std_msgs::Bool>("/pps",1000,PPS_Callback);  //This is a pps consumer.
	command_sub = nh.subscribe<icarus_rover_v2::command>("/command",1000,Command_Callback);
    std::string param_require_pps_to_start = node_name +"/require_pps_to_start";
    if(nh.getParam(param_require_pps_to_start,require_pps_to_start) == false)
	{
		logger->log_warn("Missing Parameter: require_pps_to_start.");
		return false;
	}

    //End Template Code: Initialization and Parameters

    //Start User Code: Initialization and Parameters
    std::string digitalinput_topic = "/" + node_name + "/DigitalInput";
    digitalinput_pub = nh.advertise<icarus_rover_v2::pin>(digitalinput_topic,1000);
    std::string analoginput_topic = "/" + node_name + "/AnalogInput";
	analoginput_pub = nh.advertise<icarus_rover_v2::pin>(analoginput_topic,1000);
	std::string forcesensorinput_topic = "/" + node_name + "/ForceSensorInput";
	forcesensorinput_pub = nh.advertise<icarus_rover_v2::pin>(forcesensorinput_topic,1000);
	std::string digitaloutput_topic = "/" + node_name + "/DigitalOutput";
	digitaloutput_sub = nh.subscribe<icarus_rover_v2::pin>(digitaloutput_topic,1000,DigitalOutput_Callback);
	std::string pwmoutput_topic = "/" + node_name + "/PWMOutput";
	pwmoutput_sub = nh.subscribe<icarus_rover_v2::pin>(pwmoutput_topic,1000,PwmOutput_Callback);

	current_num = -1;
	last_num = -1;
	missed_counter = 0;
	bad_checksum_counter = 0;
	good_checksum_counter = 0;
	message_started = false;
	message_completed = false;
	new_message = false;
	checking_gpio_comm = false;
	message_receive_counter = 0;
	if(initialize_gpioboard_ports() == false)
	{
		logger->log_fatal("Couldn't initialize gpio board ports. Exiting.");
	}
	gpio_board_mode = GPIOBOARD_MODE_UNDEFINED;
	device_fid = open("/dev/ttyAMA0", O_RDWR | O_NOCTTY /*| O_NDELAY*/);
    if(device_fid < 0)
    {
    	logger->log_fatal("Unable to setup UART.  Exiting.");
		return false;
    }
	struct termios options;
	tcgetattr(device_fid, &options);
	options.c_cflag = B115200 | CS8 | CLOCAL | CREAD;		//<Set baud rate
	options.c_iflag = IGNPAR;
	options.c_oflag = 0;
	options.c_lflag = 0;
	tcflush(device_fid, TCIFLUSH);
	tcsetattr(device_fid, TCSANOW, &options);

	serialmessagehandler = new SerialMessageHandler();
    //Finish User Code: Initialization and Parameters

    //Start Template Code: Final Initialization.
	diagnostic_status.Diagnostic_Type = NOERROR;
	diagnostic_status.Level = INFO;
	diagnostic_status.Diagnostic_Message = NOERROR;
	diagnostic_status.Description = "Node Initialized";
	diagnostic_pub.publish(diagnostic_status);
    logger->log_info("Initialized!");
    return true;
    //End Template Code: Finish Initialization.
}
double measure_time_diff(ros::Time timer_a, ros::Time timer_b)
{
	ros::Duration etime = timer_a - timer_b;
	return etime.toSec();
}
void PPS_Callback(const std_msgs::Bool::ConstPtr& msg)
{
	logger->log_info("Got pps");
	received_pps = true;
}
bool check_resources(int procid)
{
	if(procid <= 0)
	{
		resources_used.PID = procid;
		return false;
	}
	std::string resource_filename;
	resource_filename = "/home/robot/logs/output/RESOURCE/" + node_name;
	char tempstr[130];
	sprintf(tempstr,"top -bn1 | grep %d > %s",procid,resource_filename.c_str());
	//printf("Command: %s\r\n",tempstr);
	system(tempstr); //RAM used is column 6, in KB.  CPU used is column 8, in percentage.
	ifstream myfile;
	myfile.open(resource_filename.c_str());
	if(myfile.is_open())
	{
		std::string line;
		getline(myfile,line);
		std::vector<std::string> strs;
		boost::split(strs,line,boost::is_any_of(" "),boost::token_compress_on);
		resources_used.Node_Name = node_name;
		resources_used.PID = procid;
		resources_used.CPU_Perc = atoi(strs.at(8).c_str());
		resources_used.RAM_MB = atoi(strs.at(6).c_str())/1000.0;
		return true;
	}
	else
	{
		return false;
	}
	myfile.close();
	return false;
}
int get_pid()
{
	int id = -1;
	std::string local_node_name;
	local_node_name = node_name.substr(1,node_name.size());
	std::string pid_filename;
	pid_filename = "/home/robot/logs/output/PID" + node_name;
	char tempstr[130];
	sprintf(tempstr,"ps aux | grep __name:=%s > %s",local_node_name.c_str(),pid_filename.c_str());
	system(tempstr);
	ifstream myfile;
	myfile.open(pid_filename.c_str());
	if(myfile.is_open())
	{
		std::string line;
		getline(myfile,line);
		//printf("Line:%s\r\n",line.c_str());
		std::size_t found = line.find("icarus_rover_v2/gpio_node");
		if(found != std::string::npos)
		{
			std::vector <string> fields;
			boost::split(fields,line,boost::is_any_of("\t "),boost::token_compress_on);
			id =  atoi(fields.at(1).c_str());
		}
	}
	else
	{
		id = -1;
	}
	myfile.close();
	return id;
}
std::vector<icarus_rover_v2::diagnostic> check_program_variables()
{
	std::vector<icarus_rover_v2::diagnostic> diaglist;
	bool status = true;
	logger->log_notice("checking program variables.");
	
	if(status == true)
	{
		icarus_rover_v2::diagnostic diag=diagnostic_status;
		diag.Diagnostic_Type = SOFTWARE;
		diag.Level = NOTICE;
		diag.Diagnostic_Message = DIAGNOSTIC_PASSED;
		diag.Description = "Checked Program Variables -> PASSED";
		diaglist.push_back(diag);
	}
	else
	{
		icarus_rover_v2::diagnostic diag=diagnostic_status;
		diag.Diagnostic_Type = SOFTWARE;
		diag.Level = WARN;
		diag.Diagnostic_Message = DIAGNOSTIC_FAILED;
		diag.Description = "Checked Program Variables -> FAILED";
		diaglist.push_back(diag);
	}
	return diaglist;
}
void Device_Callback(const icarus_rover_v2::device::ConstPtr& msg)
{
	icarus_rover_v2::device newdevice;
	newdevice.DeviceName = msg->DeviceName;
	newdevice.Architecture = msg->Architecture;
	newdevice.DeviceType = msg->DeviceType;
	newdevice.DeviceParent = msg->DeviceParent;
	newdevice.BoardCount = msg->BoardCount;
	printf("Got device for: %s/%s",newdevice.DeviceParent.c_str(),newdevice.DeviceName.c_str());
	if((newdevice.DeviceParent == "None") && (device_initialized == false))
	{
		myDevice = newdevice;
	}
	else if((newdevice.DeviceParent == myDevice.DeviceName) && (myDevice.DeviceName != "") && (device_initialized == false))
	{
		icarus_rover_v2::device board;
		board.DeviceName = msg->DeviceName;
		board.pins = msg->pins;
		boards.push_back(board);
		for(int i = 0; i < board.pins.size();i++)
		{

			if(configure_pin(board.DeviceName,board.pins.at(i).Port,board.pins.at(i).Number,board.pins.at(i).Function)==false)
			{
				char tempstr[256];
				sprintf(tempstr,"Couldn't Configure Pin on Port: %s Number: %d Function: %s",
					board.pins.at(i).Port.c_str(),
					board.pins.at(i).Number,
					board.pins.at(i).Function.c_str());
				logger->log_error(tempstr);
			}
		}
		if((myDevice.BoardCount == boards.size()) && (myDevice.BoardCount > 0) && (device_initialized == false))
		{
			diagnostic_status.Diagnostic_Type = NOERROR;
			diagnostic_status.Level = INFO;
			diagnostic_status.Diagnostic_Message = NOERROR;
			diagnostic_status.Description = "Received all Device Info.";
			diagnostic_pub.publish(diagnostic_status);
			logger->log_info("Received all Device Info.");
			for(int b = 0; b < boards.size(); b++)
			{
				for(int p = 0; p < boards.at(b).pins.size(); p++)
				{
					char tempstr[128];
					icarus_rover_v2::pin newpin = boards.at(b).pins.at(p);
					sprintf(tempstr,"Board: %d Port: %s Pin: %d Function: %s",b,newpin.Port.c_str(),newpin.Number,newpin.Function.c_str());
					logger->log_debug(tempstr);
				}
			}
			device_initialized = true;
			gpio_board_mode = GPIOBOARD_MODE_INITIALIZING;
		}
	}
}
//End Template Code: Functions
