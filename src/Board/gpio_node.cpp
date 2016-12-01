#include "gpio_node.h"
//Start User Code: Firmware Definition
#define GPIONODE_MAJOR_RELEASE 3
#define GPIONODE_MINOR_RELEASE 1
#define GPIONODE_BUILD_NUMBER 3
//End User Code: Firmware Definition
//Start User Code: Functions

void process_message_thread()
{
	while(kill_node == 0)
	{
		unsigned char rx_buffer[64];
		
		memset(rx_buffer, 0, sizeof(rx_buffer));
		//memset(packet,0,sizeof(packet));
		//ros::Time start_time = ros::Time::now();
		int rx_length = read(device_fid, rx_buffer, sizeof(rx_buffer));
		if(rx_length > 0)
		{
			last_message_received_time = ros::Time::now();
		}
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
void ArmedState_Callback(const std_msgs::UInt8::ConstPtr& msg)
{
	diagnostic_status = process->new_armedstatemsg(msg->data);
	if(diagnostic_status.Level > NOTICE)
	{
		diagnostic_pub.publish(diagnostic_status);
	}
}
bool run_fastrate_code()
{
	uint8_t armed_command = process->get_armedcommand();
	uint8_t armed_state = process->get_armedcommand();
	if(new_message == true)
	{
		new_message = false;
		if(packet_type == SERIAL_TestMessageCounter_ID)
		{
			//logger->log_debug("Starting to process SERIAL_TestMessageCounter_ID.");
			diagnostic_status = process->new_serialmessage_TestMessageCounter(packet_type,packet);
			//logger->log_debug("Finished processing SERIAL_TestMessageCounter_ID.");
			if(WARN_ON_SOFTWARE_NOT_IMPLEMENTED == 1)
			{
				if(diagnostic_status.Level > NOTICE)
				{
					logger->log_warn(diagnostic_status.Description);
					diagnostic_pub.publish(diagnostic_status);
				}
			}
		}
		else if(packet_type == SERIAL_FirmwareVersion_ID)
		{
			//logger->log_debug("Starting to process SERIAL_FirmwareVersion_ID.");
			diagnostic_status = process->new_serialmessage_FirmwareVersion(packet_type,packet);
			//logger->log_debug("Finished processing SERIAL_FirmwareVersion_ID.");
			if(WARN_ON_SOFTWARE_NOT_IMPLEMENTED == 1)
			{
				if(diagnostic_status.Level > NOTICE)
				{
					logger->log_warn(diagnostic_status.Description);
					diagnostic_pub.publish(diagnostic_status);
				}
			}
		}
		else if(packet_type == SERIAL_Diagnostic_ID)
		{
			//logger->log_debug("Starting to process SERIAL_Diagnostic_ID.");
			diagnostic_status = process->new_serialmessage_Diagnostic(packet_type,packet);
			//logger->log_debug("Finished processing SERIAL_Diagnostic_ID.");
			if(diagnostic_status.Level > NOTICE)
			{
				logger->log_warn(diagnostic_status.Description);
				diagnostic_pub.publish(diagnostic_status);
			}
		}
		else if(packet_type == SERIAL_Get_ANA_PortA_ID)
		{
			//logger->log_debug("Starting to process SERIAL_Get_ANA_PortA_ID.");
			diagnostic_status = process->new_serialmessage_Get_ANA_PortA(packet_type,packet);
			//logger->log_debug("Finished processing SERIAL_Get_ANA_PortA_ID.");
			if(diagnostic_status.Level > NOTICE)
			{
				logger->log_warn(diagnostic_status.Description);
				diagnostic_pub.publish(diagnostic_status);
			}
		}
		else if(packet_type == SERIAL_Get_ANA_PortB_ID)
		{
			//logger->log_debug("Starting to process SERIAL_Get_ANA_PortB_ID.");
			diagnostic_status = process->new_serialmessage_Get_ANA_PortB(packet_type,packet);
			//logger->log_debug("Finished processing SERIAL_Get_ANA_PortB_ID.");
			if(diagnostic_status.Level > NOTICE)
			{
				logger->log_warn(diagnostic_status.Description);
				diagnostic_pub.publish(diagnostic_status);
			}
		}
		else if(packet_type == SERIAL_Get_DIO_PortA_ID)
		{
			//logger->log_debug("Starting to process SERIAL_Get_DIO_PortA_ID.");
			diagnostic_status = process->new_serialmessage_Get_DIO_PortA(packet_type,packet);
			//logger->log_debug("Finished processing SERIAL_Get_DIO_PortA_ID.");
			if(WARN_ON_SOFTWARE_NOT_IMPLEMENTED == 1)
			{
				if(diagnostic_status.Level > NOTICE)
				{
					logger->log_warn(diagnostic_status.Description);
					diagnostic_pub.publish(diagnostic_status);
				}
			}
		}
		else if(packet_type == SERIAL_Get_DIO_PortB_ID)
		{
			//logger->log_debug("Starting to process SERIAL_Get_DIO_PortB_ID.");
			diagnostic_status = process->new_serialmessage_Get_DIO_PortB(packet_type,packet);
			//logger->log_debug("Finished processing SERIAL_Get_DIO_PortB_ID.");
			if(WARN_ON_SOFTWARE_NOT_IMPLEMENTED == 1)
			{
				if(diagnostic_status.Level > NOTICE)
				{
					logger->log_warn(diagnostic_status.Description);
					diagnostic_pub.publish(diagnostic_status);
				}
			}
		}
		else if(packet_type == SERIAL_Mode_ID)
		{
			//logger->log_debug("Starting to process SERIAL_Mode_ID.");
			diagnostic_status = process->new_serialmessage_Get_Mode(packet_type,packet);
			//logger->log_debug("Finished processing SERIAL_Mode_ID.");
			if(diagnostic_status.Level > NOTICE)
			{
				logger->log_warn(diagnostic_status.Description);
				diagnostic_pub.publish(diagnostic_status);
			}
		}
		else
		{
			if(WARN_ON_SOFTWARE_NOT_IMPLEMENTED == 1)
			{
				diagnostic_status.Level = WARN;
				diagnostic_status.Diagnostic_Type= COMMUNICATIONS;
				diagnostic_status.Diagnostic_Message = DROPPING_PACKETS;
				char tempstr[255];
				sprintf(tempstr,"Message: %0x not processed.",packet_type);
				diagnostic_status.Description = tempstr;
				logger->log_warn(tempstr);
			}

		}
	}
	//printf("Running process update.\n");
	diagnostic_status = process->update(20);  //Need to change 20 to the actual dt!!!
	if(diagnostic_status.Level > NOTICE)
	{
		logger->log_warn(diagnostic_status.Description);
		diagnostic_pub.publish(diagnostic_status);
	}
	//printf("diag: %s\n",diagnostic_status.Description.c_str());
	std::vector<std::vector<unsigned char> > tx_buffers;
	//logger->log_debug("Checking message output triggers.");
	bool send = process->checkTriggers(tx_buffers);
	if(send == true)
	{
		for(int i = 0; i < tx_buffers.size();i++)
		{
			unsigned char tx_buffer[12];
			std::vector<unsigned char> tempstr = tx_buffers.at(i);
			#if(USE_UART == 1)
				int count = write(device_fid,reinterpret_cast<char*> (&tempstr[0]),12);
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
			#endif
		}
	}
	//logger->log_debug("Finished message output triggers.");

	if((process->get_boardstate() == GPIO_MODE_RUNNING) &&
	   (process->get_nodestate() == GPIO_MODE_RUNNING))
	{
		ready_to_arm = true;
		std::vector<icarus_rover_v2::device> boards = process->get_boards();
		for(int i = 0; i < boards.size(); i++)
		{
			std::string boardname = boards.at(i).DeviceName;
			Port_Info ANA_PortA = process->get_PortInfo(boardname,"ANA_PortA");
			if(ANA_PortA.PortName == "")
			{
			}
			else
			{
				for(int j = 0; j < 4; j++)
				{
					icarus_rover_v2::pin newpin;
					newpin.Function = process->map_PinFunction_ToString(ANA_PortA.Mode[j]);
					newpin.Number = ANA_PortA.Number[j];
					newpin.Port = "ANA_PortA";
					newpin.Value = ANA_PortA.Value[j];
					newpin.ConnectedDevice = ANA_PortA.ConnectingDevice.at(j);
					//printf("Port Name: %s Pin Number: %d Pin Function: %d Pin Value: %d\n",
					//		ANA_PortA.PortName.c_str(),ANA_PortA.Number[j],ANA_PortA.Mode[j],ANA_PortA.Value[j]);
					if(ANA_PortA.Mode[j] == PINMODE_ANALOG_INPUT)
					{
						analoginput_pub.publish(newpin);
					}
					else if(ANA_PortA.Mode[j] == PINMODE_FORCESENSOR_INPUT)
					{
						forcesensorinput_pub.publish(newpin);
					}
				}
			}
			Port_Info ANA_PortB = process->get_PortInfo(boardname,"ANA_PortB");
			if(ANA_PortB.PortName == "")
			{
			}
			else
			{
				for(int j = 0; j < 4; j++)
				{
					icarus_rover_v2::pin newpin;
					newpin.Function = process->map_PinFunction_ToString(ANA_PortB.Mode[j]);
					newpin.Number = ANA_PortB.Number[j];
					newpin.Port = "ANA_PortB";
					newpin.Value = ANA_PortB.Value[j];
					newpin.ConnectedDevice = ANA_PortB.ConnectingDevice.at(j);
					//printf("Port Name: %s Pin Number: %d Pin Function: %d Pin Value: %d\n",
					//		ANA_PortA.PortName.c_str(),ANA_PortA.Number[j],ANA_PortA.Mode[j],ANA_PortA.Value[j]);
					if(ANA_PortB.Mode[j] == PINMODE_ANALOG_INPUT)
					{
						analoginput_pub.publish(newpin);
					}
					else if(ANA_PortB.Mode[j] == PINMODE_FORCESENSOR_INPUT)
					{
						forcesensorinput_pub.publish(newpin);
					}
				}
			}
		}
		//logger->log_debug("Finished reading ANA Ports.");
	}
	else
	{
		ready_to_arm = false;
	}
	std_msgs::Bool bool_ready_to_arm;
	bool_ready_to_arm.data = ready_to_arm;
	ready_to_arm_pub.publish(bool_ready_to_arm);

	//diagnostic_pub.publish(diagnostic_status);
	return true;
}
bool run_mediumrate_code()
{
	process->transmit_armedstate();
	double time_since_last_message = measure_time_diff(ros::Time::now(),last_message_received_time);
	if((time_since_last_message > 3.0) && (time_since_last_message < 6.0))
	{
		diagnostic_status.Level = WARN;
		diagnostic_status.Diagnostic_Message = DROPPING_PACKETS;
		char tempstr[255];
		sprintf(tempstr,"No Message received from GPIO Board in %f seconds",time_since_last_message);
		diagnostic_status.Description = tempstr;
	}
	else if(time_since_last_message >= 6.0)
	{
		diagnostic_status.Level = ERROR;
		diagnostic_status.Diagnostic_Message = DROPPING_PACKETS;
		char tempstr[255];
		sprintf(tempstr,"No Message received from GPIO Board in %f seconds",time_since_last_message);
		diagnostic_status.Description = tempstr;
	}

	beat.stamp = ros::Time::now();
	heartbeat_pub.publish(beat);
	diagnostic_pub.publish(diagnostic_status);
	return true;
}
bool run_slowrate_code()
{
	{
		icarus_rover_v2::diagnostic diag = diagnostic_status;
		char tempstr[255];
		sprintf(tempstr,"Board Mode: %s Node Mode: %s",
			process->map_mode_ToString(process->get_boardstate()).c_str(),
			process->map_mode_ToString(process->get_nodestate()).c_str());
		if((process->get_boardstate() == GPIO_MODE_RUNNING) &&
			   (process->get_nodestate() == GPIO_MODE_RUNNING))
		{
			logger->log_info(tempstr);
			diag.Level = INFO;
		}
		else
		{
			logger->log_warn(tempstr);
			diag.Level = WARN;
		}
		diag.Diagnostic_Type = SOFTWARE;
		diag.Diagnostic_Message = NOERROR;
		diag.Description = tempstr;
		diagnostic_pub.publish(diag);
	}

	if(device_initialized == true)
	{
		icarus_rover_v2::diagnostic resource_diagnostic;
		resource_diagnostic = resourcemonitor->update();
		if(resource_diagnostic.Diagnostic_Message == DEVICE_NOT_AVAILABLE)
		{
			diagnostic_pub.publish(resource_diagnostic);
			logger->log_warn("Couldn't read resources used.");
		}
		else if(resource_diagnostic.Level >= WARN)
		{
			resources_used = resourcemonitor->get_resourceused();
			resource_pub.publish(resources_used);
			diagnostic_pub.publish(resource_diagnostic);
		}
		else if(resource_diagnostic.Level <= NOTICE)
		{
			resources_used = resourcemonitor->get_resourceused();
			resource_pub.publish(resources_used);
		}
	}

	{
		char tempstr[255];
		sprintf(tempstr,"Checksum passed: %d failed: %d",good_checksum_counter,bad_checksum_counter);
		logger->log_info(tempstr);
	}
	return true;
}

bool run_veryslowrate_code()
{
	//logger->log_debug("Running very slow rate code.");
	logger->log_info("Node Running.");
	icarus_rover_v2::firmware fw;
	fw.Generic_Node_Name = "gpio_node";
	fw.Node_Name = node_name;
	fw.Description = "Latest Rev: 30-Nov-2016";
	fw.Major_Release = GPIONODE_MAJOR_RELEASE;
	fw.Minor_Release = GPIONODE_MINOR_RELEASE;
	fw.Build_Number = GPIONODE_BUILD_NUMBER;
	firmware_pub.publish(fw);
	std::vector<message_info> allmessage_info = process->get_allmessage_info();
	for(int i = 0; i < allmessage_info.size(); i++)
	{
		char tempstr[255];
		message_info message = allmessage_info.at(i);
		sprintf(tempstr,"Message: AB%0X Received Counter: %d Received Rate: %f (Hz) Transmitted Counter: %d Transmitted Rate: %f (Hz)",
				message.id,
				message.received_counter,
				message.received_rate,
				message.sent_counter,
				message.transmitted_rate);
		logger->log_info(tempstr);
	}
	return true;
}
void DigitalOutput_Callback(const icarus_rover_v2::pin::ConstPtr& msg)
{
	icarus_rover_v2::pin pinmsg;
	pinmsg.Port = msg->Port;
	pinmsg.Function = msg->Function;
	pinmsg.Number = msg->Number;
	pinmsg.Value = msg->Value;
	if(pinmsg.Function == "DigitalOutput")
	{
		//char tempstr[128];
		//sprintf(tempstr,"Pin: %d Mode: %s Value: %d",pinmsg.Number,pinmsg.Function.c_str(),pinmsg.Value);
		//logger->log_debug(tempstr);
		process->new_pinmsg(pinmsg);
	}
}
void PwmOutput_Callback(const icarus_rover_v2::pin::ConstPtr& msg)
{
	icarus_rover_v2::pin pinmsg;
	pinmsg.Port = msg->Port;
	pinmsg.Function = msg->Function;
	pinmsg.Number = msg->Number;
	pinmsg.Value = msg->Value;
	if(pinmsg.Function == "PWMOutput")
	{

		process->new_pinmsg(pinmsg);
	}
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


void Command_Callback(const icarus_rover_v2::command::ConstPtr& msg)
{
	diagnostic_status = process->new_commandmsg(
			msg->Command,msg->Option1,msg->Option2,msg->Option3,msg->CommandText,msg->CommandText);
	if(diagnostic_status.Level > NOTICE)
	{
		diagnostic_pub.publish(diagnostic_status);
	}
	if (msg->Command ==  DIAGNOSTIC_ID)
	{
		if(msg->Option1 == LEVEL1)
		{
			diagnostic_pub.publish(diagnostic_status);
		}
		else if(msg->Option1 == LEVEL2)
		{
			std::vector<icarus_rover_v2::diagnostic> diaglist = check_program_variables();
			for(int i = 0; i < diaglist.size();i++) { diagnostic_pub.publish(diaglist.at(i)); }
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
//End User Code: Functions

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
		kill_node = 1;
    }
    ros::Rate loop_rate(rate);
	boot_time = ros::Time::now();
    now = ros::Time::now();
    fast_timer = now;
    medium_timer = now;
    slow_timer = now;
    veryslow_timer = now;
	#if( USE_UART == 1)

    	boost::thread processmessage_thread(&process_message_thread);

	#endif
    while (ros::ok() && (kill_node == 0))
    {
    	bool ok_to_start = false;
		if(require_pps_to_start == false) { ok_to_start = true;}
		else if(require_pps_to_start == true && received_pps == true) { ok_to_start = true; }
    	if(ok_to_start == true)
    	{
    		now = ros::Time::now();
    		mtime = measure_time_diff(now,fast_timer);
			if(mtime > .02)
			{
				run_fastrate_code();
				fast_timer = ros::Time::now();
			}
			mtime = measure_time_diff(now,medium_timer);
			if(mtime > 0.1)
			{
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

	#if(USE_UART == 1)

    	processmessage_thread.join();
	#endif
    kill_node = true;
    logger->log_notice("Node Finished Safely.");
    return 0;
}

bool initialize(ros::NodeHandle nh)
{
    //Start Template Code: Initialization, Parameters and Topics
	kill_node = 0;
	signal(SIGINT,signalinterrupt_handler);
    myDevice.DeviceName = "";
    myDevice.Architecture = "";
    device_initialized = false;
    hostname[1023] = '\0';
    gethostname(hostname,1023);
    std::string diagnostic_topic = "/" + node_name + "/diagnostic";
	diagnostic_pub =  nh.advertise<icarus_rover_v2::diagnostic>(diagnostic_topic,1000);
    diagnostic_status.DeviceName = hostname;
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
    
    std::string heartbeat_topic = "/" + node_name + "/heartbeat";
    heartbeat_pub = nh.advertise<icarus_rover_v2::heartbeat>(heartbeat_topic,1000);
    beat.Node_Name = node_name;
    std::string device_topic = "/" + std::string(hostname) +"_master_node/device";
    device_sub = nh.subscribe<icarus_rover_v2::device>(device_topic,1000,Device_Callback);

    pps_sub = nh.subscribe<std_msgs::Bool>("/pps",1000,PPS_Callback);  //This is a pps consumer.
    command_sub = nh.subscribe<icarus_rover_v2::command>("/command",1000,Command_Callback);
    std::string param_require_pps_to_start = node_name +"/require_pps_to_start";
    if(nh.getParam(param_require_pps_to_start,require_pps_to_start) == false)
	{
		logger->log_warn("Missing Parameter: require_pps_to_start.");
		return false;
	}
    std::string firmware_topic = "/" + node_name + "/firmware";
    firmware_pub =  nh.advertise<icarus_rover_v2::firmware>(firmware_topic,1000);
    //End Template Code: Initialization and Parameters

    //Start User Code: Initialization and Parameters
    ready_to_arm  = false;

    std::string sensor_spec_path;
    std::string armed_state_topic = "/armed_state";
    armed_state_sub = nh.subscribe<std_msgs::UInt8>(armed_state_topic,1000,ArmedState_Callback);
    std::string param_sensor_spec_path = node_name +"/sensor_spec_path";
	if(nh.getParam(param_sensor_spec_path,sensor_spec_path) == false)
	{
		logger->log_error("Missing Parameter: sensor_spec_path.");
		return false;
	}
	bool extrapolate;
	std::string param_extrapolate_sensor_values = node_name +"/extrapolate_sensor_values";
	if(nh.getParam(param_sensor_spec_path,sensor_spec_path) == false)
	{
		logger->log_warn("Missing Parameter: extrapolate_sensor_values. Using default: False.");
		extrapolate = false;
	}

    process = new GPIONodeProcess;
	diagnostic_status = process->init(diagnostic_status,logger,std::string(hostname),sensor_spec_path,extrapolate);
    last_message_received_time = ros::Time::now();
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
	#if(USE_UART == 1)
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
	#endif

	std::string ready_to_arm_topic = node_name + "/ready_to_arm";
	ready_to_arm_pub = nh.advertise<std_msgs::Bool>(ready_to_arm_topic,1000);
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
//Start Template Code: Functions
double measure_time_diff(ros::Time timer_a, ros::Time timer_b)
{
	ros::Duration etime = timer_a - timer_b;
	return etime.toSec();
}
void PPS_Callback(const std_msgs::Bool::ConstPtr& msg)
{
	//logger->log_info("Got pps");
	received_pps = true;
}
void Device_Callback(const icarus_rover_v2::device::ConstPtr& msg)
{
	icarus_rover_v2::device newdevice;
	newdevice.DeviceName = msg->DeviceName;
	newdevice.Architecture = msg->Architecture;
	newdevice.DeviceType = msg->DeviceType;
	newdevice.DeviceParent = msg->DeviceParent;
	newdevice.BoardCount = msg->BoardCount;
	newdevice.SensorCount = msg->SensorCount;
	newdevice.pins = msg->pins;

	if(device_initialized == false)
	{
		//printf("Starting to process device message for device: %s\n",newdevice.DeviceName.c_str());
		for(int i = 0; i < newdevice.pins.size();i++)
		{
			printf("port: %s pin: %d def value: %d\n",newdevice.pins.at(i).Port.c_str(),newdevice.pins.at(i).Number,newdevice.pins.at(i).DefaultValue);
		}
		diagnostic_status = process->new_devicemsg(newdevice);
		//printf("Processed device message.\n");
	}
	if(diagnostic_status.Level == FATAL)
	{
		logger->log_fatal(diagnostic_status.Description);
		logger->log_fatal("This is a Safety Issue!  Killing Node.");
		kill_node = true;
	}
	if((device_initialized == false) and (process->is_finished_initializing() == true))
	{
		//printf("Almost done.\n");
		myDevice = process->get_mydevice();
		//printf("Got device: %s\n",myDevice.DeviceName.c_str());
		resourcemonitor = new ResourceMonitor(diagnostic_status,myDevice.Architecture,myDevice.DeviceName,node_name);
		//logger->log_info("Resource Monitor Initialized.");
		device_initialized = true;
		logger->log_info("Device Initialized.");
	}


}
void signalinterrupt_handler(int sig)
{
	kill_node = 1;
}
//End Template Code: Functions
