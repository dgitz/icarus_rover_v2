#include "boardcontroller_node.h"
//Start User Code: Firmware Definition
#define BOARDCONTROLLERNODE_MAJOR_RELEASE 0
#define BOARDCONTROLLERNODE_MINOR_RELEASE 0
#define BOARDCONTROLLERNODE_BUILD_NUMBER 1
//End User Code: Firmware Definition
//Start User Code: Functions
bool run_loop1_code()
{
	process->send_commandmessage(SPIMessageHandler::SPI_LEDStripControl_ID);
	process->send_querymessage(SPIMessageHandler::SPI_Diagnostic_ID);
	icarus_rover_v2::diagnostic diag = process->get_diagnostic();
	diagnostic_pub.publish(diag);
	{
		/*
		icarus_rover_v2::diagnostic diag = process->send_querymessage(SPIMessageHandler::SPI_Get_ANA_Port1_ID);
		if(diag.Level >= WARN)
		{
			diagnostic_pub.publish(diag);
			logger->log_diagnostic(diag);
		}
		 */
	}
	/*
	{
		icarus_rover_v2::diagnostic diag = process->send_querymessage(SPIMessageHandler::SPI_Get_DIO_Port1_ID);
		if(diag.Level >= WARN)
		{
			diagnostic_pub.publish(diag);
			logger->log_diagnostic(diag);
		}
	}
	 */
	return true;
}
bool run_loop2_code()
{

	icarus_rover_v2::diagnostic diag = process->update(measure_time_diff(ros::Time::now(),last_loop2_timer));
	if(diag.Level >= WARN)
	{
		diagnostic_pub.publish(diag);
		logger->log_diagnostic(diag);
	}
	process->send_querymessage(SPIMessageHandler::SPI_Get_IMUAcc_ID);
		process->send_querymessage(SPIMessageHandler::SPI_Get_IMUGyro_ID);
		process->send_querymessage(SPIMessageHandler::SPI_Get_IMUMag_ID);
	diag = process->send_querymessage(SPIMessageHandler::SPI_Get_DIO_Port1_ID);
	std::vector<BoardControllerNodeProcess::Sensor> sensors = process->get_sensordata();
	for(std::size_t i = 0; i < sensors.size(); i++)
	{
		for(std::size_t j = 0; j < signal_sensor_names.size(); j++)
		{
			if(signal_sensor_names.at(j) == sensors.at(i).name)
			{
				icarus_rover_v2::signal v;
				v.tov = ros::Time(sensors.at(i).tov);
				v.value = sensors.at(i).value;
				v.units = sensors.at(i).units;
				v.status = sensors.at(i).status;
				v.rms = -1.0;
				signal_sensor_pubs.at(j).publish(v);
			}
		}
	}
	bool ready_to_arm = process->get_ready_to_arm();
	std_msgs::Bool bool_ready_to_arm;
	bool_ready_to_arm.data = ready_to_arm;
	ready_to_arm_pub.publish(bool_ready_to_arm);
	return true;
}
bool run_loop3_code()
{
	std::vector<Message> querymessages_tosend = process->get_querymessages_tosend();
	for(std::size_t i = 0; i < querymessages_tosend.size(); i++)
	{
		unsigned char inputbuffer[12];
		int passed_checksum_calc = sendMessageQuery(querymessages_tosend.at(i).id,inputbuffer);
		process->new_message_sent(querymessages_tosend.at(i).id);
		if(passed_checksum_calc > 0)
		{
			passed_checksum++;

			int length;
			int success;
			unsigned char v1,v2,v3,v4,v5,v6,v7,v8,v9,v10,v11,v12;
			uint16_t a1,a2,a3,a4,a5,a6;
			int16_t b1,b2;
			icarus_rover_v2::diagnostic diag;
			switch(querymessages_tosend.at(i).id)
			{
			case SPIMessageHandler::SPI_TestMessageCounter_ID:
				success = spimessagehandler->decode_TestMessageCounterSPI(inputbuffer,&length,&v1,&v2,&v3,&v4,&v5,&v6,&v7,&v8,&v9,&v10,&v11,&v12);
				if(success == 1)
				{
					process->new_message_recv(querymessages_tosend.at(i).id);
					diag = process->new_message_TestMessageCounter(BOARD_ID,v1,v2,v3,v4,v5,v6,v7,v8,v9,v10,v11,v12);
					if(diag.Level >= WARN)
					{
						diagnostic_pub.publish(diag);
						logger->log_diagnostic(diag);
					}
				}
				break;
			case SPIMessageHandler::SPI_Get_ANA_Port1_ID:
				success = spimessagehandler->decode_Get_ANA_Port1SPI(inputbuffer,&length,&a1,&a2,&a3,&a4,&a5,&a6);
				if(success == 1)
				{
					process->new_message_recv(querymessages_tosend.at(i).id);
					diag = process->new_message_GetANAPort1(BOARD_ID,ros::Time::now().toSec(),a1,a2,a3,a4,a5,a6);
					if(diag.Level >= WARN)
					{
						diagnostic_pub.publish(diag);
						logger->log_diagnostic(diag);
					}
				}
				break;
			case SPIMessageHandler::SPI_Get_DIO_Port1_ID:
				success = spimessagehandler->decode_Get_DIO_Port1SPI(inputbuffer,&length,&a1,&a2);
				if(success == 1)
				{
					process->new_message_recv(querymessages_tosend.at(i).id);
					diag = process->new_message_GetDIOPort1(BOARD_ID,ros::Time::now().toSec(),
							a1-BYTE2_OFFSET,a2-BYTE2_OFFSET);
					if(diag.Level >= WARN)
					{
						diagnostic_pub.publish(diag);
						logger->log_diagnostic(diag);
					}
				}
				break;
			case SPIMessageHandler::SPI_Diagnostic_ID:

				success = spimessagehandler->decode_DiagnosticSPI(inputbuffer,&length,&v1,&v2,&v3,&v4,&v5,&v6);
				if(success == 1)
				{
					process->new_message_recv(querymessages_tosend.at(i).id);
					diag = process->new_message_Diagnostic(BOARD_ID,v1,v2,v3,v4,v5,v6);
					if(diag.Level >= WARN)
					{
						diagnostic_pub.publish(diag);
						logger->log_diagnostic(diag);
					}
				}
				break;
			case SPIMessageHandler::SPI_Get_IMUAcc_ID:

				success = spimessagehandler->decode_Get_IMUAccSPI(inputbuffer,&length,&a1,&a2,&a3,&a4,&a5,&a6);
				if(success == 1)
				{
					process->new_message_recv(querymessages_tosend.at(i).id);
					diag = process->new_message_IMUAcc(BOARD_ID,ros::Time::now().toSec(),a1,a2,a3,a4,a5,a6);
					if(diag.Level >= WARN)
					{
						diagnostic_pub.publish(diag);
						logger->log_diagnostic(diag);
					}
				}
				break;
			case SPIMessageHandler::SPI_Get_IMUGyro_ID:

				success = spimessagehandler->decode_Get_IMUGyroSPI(inputbuffer,&length,&a1,&a2,&a3,&a4,&a5,&a6);
				if(success == 1)
				{
					process->new_message_recv(querymessages_tosend.at(i).id);
					diag = process->new_message_IMUGyro(BOARD_ID,ros::Time::now().toSec(),a1,a2,a3,a4,a5,a6);
					if(diag.Level >= WARN)
					{
						diagnostic_pub.publish(diag);
						logger->log_diagnostic(diag);
					}
				}
				break;
			case SPIMessageHandler::SPI_Get_IMUMag_ID:

				success = spimessagehandler->decode_Get_IMUMagSPI(inputbuffer,&length,&a1,&a2,&a3,&a4,&a5,&a6);
				if(success == 1)
				{
					process->new_message_recv(querymessages_tosend.at(i).id);
					diag = process->new_message_IMUMag(BOARD_ID,ros::Time::now().toSec(),a1,a2,a3,a4,a5,a6);
					if(diag.Level >= WARN)
					{
						diagnostic_pub.publish(diag);
						logger->log_diagnostic(diag);
					}
				}
				break;
			default:
				break;
			}
		}
		else if(passed_checksum_calc == 0)
		{
			failed_checksum++;
		}
		else
		{
			printf("[BoardControllerNode]: No Comm with device.\n");
		}
	}
	std::vector<Message> commandmessages_tosend = process->get_commandmessages_tosend();
	for(std::size_t i = 0; i < commandmessages_tosend.size(); i++)
	{
		int success;
		int length;
		unsigned char outputbuffer[12];
		unsigned char v1,v2,v3;
		switch(commandmessages_tosend.at(i).id)
		{
		case SPIMessageHandler::SPI_LEDStripControl_ID:
			diagnostic_status = process->get_LEDStripControlParameters(v1,v2,v3);
			success = spimessagehandler->encode_LEDStripControlSPI(outputbuffer,&length,v1,v2,v3);
			break;
		default:
			break;
		}
		sendMessageCommand(commandmessages_tosend.at(i).id,outputbuffer);
		process->new_message_sent(SPIMessageHandler::SPI_LEDStripControl_ID);
	}
	icarus_rover_v2::imu imu1,imu2;
	int v1 = process->get_imu(&imu1,1);
	if(v1 > 0)
	{
		imu1raw_pub.publish(imu1);
	}
	else if(v1 == 0)
	{
	}
	int v2 = process->get_imu(&imu2,2);
	if(v2 > 0)
	{
		imu2raw_pub.publish(imu2);
	}
	return true;
}

void PPS01_Callback(const std_msgs::Bool::ConstPtr& msg)
{
	icarus_rover_v2::firmware fw;
	fw.Generic_Node_Name = "boardcontroller_node";
	fw.Node_Name = node_name;
	fw.Description = "Latest Rev: 7-Aug-2018";
	fw.Major_Release = BOARDCONTROLLERNODE_MAJOR_RELEASE;
	fw.Minor_Release = BOARDCONTROLLERNODE_MINOR_RELEASE;
	fw.Build_Number = BOARDCONTROLLERNODE_BUILD_NUMBER;
	firmware_pub.publish(fw);
	printf("t=%4.2f (sec) [%s]: %s\n",ros::Time::now().toSec(),node_name.c_str(),process->get_diagnostic().Description.c_str());
}
void PPS1_Callback(const std_msgs::Bool::ConstPtr& msg)
{
	received_pps = true;
	if((process->is_ready() == true) and (process->is_initialized() == true))
	{
		icarus_rover_v2::diagnostic resource_diagnostic = resourcemonitor->update();
		if(resource_diagnostic.Diagnostic_Message == DEVICE_NOT_AVAILABLE)
		{
			diagnostic_pub.publish(resource_diagnostic);
			logger->log_diagnostic(resource_diagnostic);
		}
		else if(resource_diagnostic.Level >= WARN)
		{
			resources_used = resourcemonitor->get_resourceused();
			resource_pub.publish(resources_used);
			diagnostic_pub.publish(resource_diagnostic);
			logger->log_diagnostic(resource_diagnostic);
		}
		else if(resource_diagnostic.Level <= NOTICE)
		{
			resources_used = resourcemonitor->get_resourceused();
			resource_pub.publish(resources_used);
		}
	}
	else if((process->is_ready() == false) and (process->is_initialized() == true))
	{
		{
			icarus_rover_v2::srv_device srv;
			srv.request.query = "DeviceType=ArduinoBoard";
			if(srv_device.call(srv) == true)
			{
				for(std::size_t i = 0; i < srv.response.data.size(); i++)
				{
					bool status = new_devicemsg(srv.request.query,srv.response.data.at(i));
				}
			}
		}
	}
	else if(process->is_initialized() == false)
	{
		{
			icarus_rover_v2::srv_device srv;
			srv.request.query = "SELF";
			if(srv_device.call(srv) == true)
			{
				if(srv.response.data.size() != 1)
				{
					logger->log_error("Got unexpected device message");
				}
				else
				{
					bool status = new_devicemsg(srv.request.query,srv.response.data.at(0));
				}
			}
		}
	}
	else
	{
	}

	/*
    {
    	icarus_rover_v2::diagnostic diag = process->send_querymessage(SPIMessageHandler::SPI_TestMessageCounter_ID);
    	if(diag.Level >= WARN)
    	{
    		diagnostic_pub.publish(diag);
    		logger->log_diagnostic(diag);
    	}
    }
	 */
	char tempstr[512];
	sprintf(tempstr,"Passed Checksum: %d @ %f Failed Checksum: %d @ %f",
			passed_checksum,passed_checksum/(measure_time_diff(ros::Time::now(),boot_time)),
			failed_checksum,failed_checksum/(measure_time_diff(ros::Time::now(),boot_time)));
	logger->log_info(tempstr);
	logger->log_info(process->get_messageinfo(false));
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
	//logger->log_info("Got command");
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
			process->send_querymessage(SPIMessageHandler::SPI_Diagnostic_ID);
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
int spiTxRx(unsigned char txDat)
{
	unsigned char rxDat;
	struct spi_ioc_transfer spi;
	memset (&spi, 0, sizeof (spi));
	spi.tx_buf        = (unsigned long)&txDat;
	spi.rx_buf        = (unsigned long)&rxDat;
	spi.len           = 1;
	ioctl (spi_device, SPI_IOC_MESSAGE(1), &spi);
	return rxDat;
}
int sendMessageCommand(unsigned char command,unsigned char * outputbuffer)
{
	unsigned char resultByte;
	bool ack;
	int wait_time_us = 1;
	int counter = 0;
	do
	{
		ack = false;
		spiTxRx(0xAB);
		usleep (wait_time_us);
		resultByte = spiTxRx(command);
		if (resultByte == 'a')
		{
			ack = true;
		}
		else { counter++; }
		if(counter > 10000)
		{
			printf("No Comm with device after %d tries. Exiting.\n",counter);
			return -1;
		}
		usleep (wait_time_us);
	}
	while (ack == false);
	usleep(wait_time_us);
	unsigned char v;
	unsigned char running_checksum = 0;
	for(int i = 0; i < 12; i++)
	{
		v = spiTxRx(outputbuffer[i]);
		running_checksum ^= v;
		outputbuffer[i] = v;
		usleep(wait_time_us);

	}
	resultByte = spiTxRx(running_checksum);
	usleep(wait_time_us);
	return 1;
}
int sendMessageQuery(unsigned char query, unsigned char * inputbuffer)
{
	unsigned char resultByte;
	bool ack;
	int wait_time_us = 1;
	int counter = 0;
	do
	{
		ack = false;
		spiTxRx(0xAB);
		usleep (wait_time_us);
		resultByte = spiTxRx(query);
		if (resultByte == 'a')
		{
			ack = true;
		}
		else { counter++; }
		if(counter > 10000)
		{
			logger->log_fatal("No com with device after %d tries.");
			return -1;
		}
		usleep (wait_time_us);
	}
	while (ack == false);
	usleep(wait_time_us);
	resultByte = spiTxRx(0);
	usleep(wait_time_us);
	unsigned char v;
	unsigned char running_checksum = 0;
	for(int i = 0; i < 12; i++)
	{
		v = spiTxRx(0);
		running_checksum ^= v;
		inputbuffer[i] = v;
		usleep(wait_time_us);

	}
	resultByte = spiTxRx(0);
	usleep(wait_time_us);
	if(resultByte == running_checksum) { return 1; }
	else { return 0; }
}

//End User Code: Functions
bool run_10Hz_code()
{
	beat.stamp = ros::Time::now();
	heartbeat_pub.publish(beat);

	if(diagnostic_status.Level > NOTICE)
	{
		diagnostic_pub.publish(diagnostic_status);
	}
	return true;
}
int main(int argc, char **argv)
{
	node_name = "boardcontroller_node";
	ros::init(argc, argv, node_name);
	n.reset(new ros::NodeHandle);
	node_name = ros::this_node::getName();
	ros::NodeHandle n;

	if(initializenode() == false)
	{
		logger->log_fatal("Unable to Initialize.  Exiting.");
		diagnostic_status.Diagnostic_Type = SOFTWARE;
		diagnostic_status.Level = FATAL;
		diagnostic_status.Diagnostic_Message = INITIALIZING_ERROR;
		diagnostic_status.Description = "Node Initializing Error.";
		diagnostic_pub.publish(diagnostic_status);
		kill_node = 1;
	}
	ros::Rate loop_rate(ros_rate);
	boot_time = ros::Time::now();
	last_10Hz_timer = ros::Time::now();
	double mtime;
	while (ros::ok() && (kill_node == 0))
	{
		bool ok_to_start = false;
		if(require_pps_to_start == false) { ok_to_start = true;}
		else if(require_pps_to_start == true && received_pps == true) { ok_to_start = true; }
		if(ok_to_start == true)
		{
			if(run_loop1 == true)
			{
				mtime = measure_time_diff(ros::Time::now(),last_loop1_timer);
				if(mtime >= (1.0/loop1_rate))
				{
					run_loop1_code();
					last_loop1_timer = ros::Time::now();
				}
			}
			if(run_loop2 == true)
			{
				mtime = measure_time_diff(ros::Time::now(),last_loop2_timer);
				if(mtime >= (1.0/loop2_rate))
				{
					run_loop2_code();
					last_loop2_timer = ros::Time::now();
				}
			}
			if(run_loop3 == true)
			{
				mtime = measure_time_diff(ros::Time::now(),last_loop3_timer);
				if(mtime >= (1.0/loop3_rate))
				{
					run_loop3_code();
					last_loop3_timer = ros::Time::now();
				}
			}

			mtime = measure_time_diff(ros::Time::now(),last_10Hz_timer);
			if(mtime >= 0.1)
			{
				run_10Hz_code();
				last_10Hz_timer = ros::Time::now();
			}
		}
		else
		{
			logger->log_warn("Waiting on PPS to Start.");
		}
		ros::spinOnce();
		loop_rate.sleep();
	}
	logger->log_notice("Node Finished Safely.");
	return 0;
}
bool initializenode()
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
	diagnostic_pub =  n->advertise<icarus_rover_v2::diagnostic>(diagnostic_topic,1000);
	diagnostic_status.DeviceName = hostname;
	diagnostic_status.Node_Name = node_name;
	diagnostic_status.System = ROVER;
	diagnostic_status.SubSystem = ROBOT_CONTROLLER;
	diagnostic_status.Component = TIMING_NODE;

	diagnostic_status.Diagnostic_Type = NOERROR;
	diagnostic_status.Level = INFO;
	diagnostic_status.Diagnostic_Message = INITIALIZING;
	diagnostic_status.Description = "Node Initializing";
	diagnostic_pub.publish(diagnostic_status);

	std::string resource_topic = "/" + node_name + "/resource";
	resource_pub = n->advertise<icarus_rover_v2::resource>(resource_topic,1000);

	std::string param_verbosity_level = node_name +"/verbosity_level";
	if(n->getParam(param_verbosity_level,verbosity_level) == false)
	{
		logger = new Logger("WARN",ros::this_node::getName());
		logger->log_warn("Missing Parameter: verbosity_level");
		return false;
	}
	else
	{
		logger = new Logger(verbosity_level,ros::this_node::getName());
	}

	std::string heartbeat_topic = "/" + node_name + "/heartbeat";
	heartbeat_pub = n->advertise<icarus_rover_v2::heartbeat>(heartbeat_topic,1000);
	beat.Node_Name = node_name;

	std::string param_startup_delay = node_name + "/startup_delay";
	double startup_delay = 0.0;
	if(n->getParam(param_startup_delay,startup_delay) == false)
	{
		logger->log_notice("Missing Parameter: startup_delay.  Using Default: 0.0 sec.");
	}
	else
	{
		char tempstr[128];
		sprintf(tempstr,"Using Parameter: startup_delay = %4.2f sec.",startup_delay);
		logger->log_notice(std::string(tempstr));
	}
	printf("[%s] Using Parameter: startup_delay = %4.2f sec.\n",node_name.c_str(),startup_delay);
	ros::Duration(startup_delay).sleep();

	std::string device_topic = "/" + std::string(hostname) + "_master_node/srv_device";
	srv_device = n->serviceClient<icarus_rover_v2::srv_device>(device_topic);
	pps01_sub = n->subscribe<std_msgs::Bool>("/01PPS",1000,PPS01_Callback);
	pps1_sub = n->subscribe<std_msgs::Bool>("/1PPS",1000,PPS1_Callback);
	command_sub = n->subscribe<icarus_rover_v2::command>("/command",1000,Command_Callback);
	std::string param_require_pps_to_start = node_name +"/require_pps_to_start";
	if(n->getParam(param_require_pps_to_start,require_pps_to_start) == false)
	{
		logger->log_warn("Missing Parameter: require_pps_to_start.");
		return false;
	}
	std::string firmware_topic = "/" + node_name + "/firmware";
	firmware_pub =  n->advertise<icarus_rover_v2::firmware>(firmware_topic,1000);

	double max_rate = 0.0;
	std::string param_loop1_rate = node_name + "/loop1_rate";
	if(n->getParam(param_loop1_rate,loop1_rate) == false)
	{
		logger->log_warn("Missing parameter: loop1_rate.  Not running loop1 code.");
		run_loop1 = false;
	}
	else
	{
		last_loop1_timer = ros::Time::now();
		run_loop1 = true;
		if(loop1_rate > max_rate) { max_rate = loop1_rate; }
	}

	std::string param_loop2_rate = node_name + "/loop2_rate";
	if(n->getParam(param_loop2_rate,loop2_rate) == false)
	{
		logger->log_warn("Missing parameter: loop1_rate.  Not running loop2 code.");
		run_loop2 = false;
	}
	else
	{
		last_loop2_timer = ros::Time::now();
		run_loop2 = true;
		if(loop2_rate > max_rate) { max_rate = loop2_rate; }
	}

	std::string param_loop3_rate = node_name + "/loop3_rate";
	if(n->getParam(param_loop3_rate,loop3_rate) == false)
	{
		logger->log_warn("Missing parameter: loop3_rate.  Not running loop3 code.");
		run_loop3 = false;
	}
	else
	{
		last_loop3_timer = ros::Time::now();
		run_loop3 = true;
		if(loop3_rate > max_rate) { max_rate = loop3_rate; }
	}
	ros_rate = max_rate * 50.0;
	if(ros_rate < 100.0) { ros_rate = 100.0; }
	char tempstr[512];
	sprintf(tempstr,"Running Node at Rate: %f",ros_rate);
	logger->log_notice(std::string(tempstr));
	//End Template Code: Initialization and Parameters

	//Start User Code: Initialization and Parameters
	std::string ready_to_arm_topic = node_name + "/ready_to_arm";
	ready_to_arm_pub = n->advertise<std_msgs::Bool>(ready_to_arm_topic,1);
	passed_checksum = 0;
	failed_checksum = 0;
	process = new BoardControllerNodeProcess;
	diagnostic_status = process->init(diagnostic_status,std::string(hostname));
	spi_device = open("/dev/spidev0.0", O_RDWR);
	unsigned int clock_rate = 1000000;
	ioctl (spi_device, SPI_IOC_WR_MAX_SPEED_HZ, &clock_rate);
	std::string imu1raw_topic = "/IMU1_raw";
		imu1raw_pub = n->advertise<icarus_rover_v2::imu>(imu1raw_topic,1);
		std::string imu2raw_topic = "/IMU2_raw";
		imu2raw_pub = n->advertise<icarus_rover_v2::imu>(imu2raw_topic,1);
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
bool new_devicemsg(std::string query,icarus_rover_v2::device device)
{
	if(query == "SELF")
	{
		if((device.DeviceName == hostname))
		{

			process->set_mydevice(device);
			myDevice = process->get_mydevice();
			resourcemonitor = new ResourceMonitor(diagnostic_status,myDevice.Architecture,myDevice.DeviceName,node_name);
		}
	}
	else
	{
		if(process->is_ready() == false)
		{
			logger->log_notice("Device not initialized yet.");

			diagnostic_status = process->new_devicemsg(device);
			logger->log_diagnostic(diagnostic_status);
			if(diagnostic_status.Level > INFO) { diagnostic_pub.publish(diagnostic_status); }
			if(process->is_ready() == true)
			{
				std::vector<BoardControllerNodeProcess::Sensor> sensors = process->get_sensordata();
				for(std::size_t i = 0; i < sensors.size(); i++)
				{
					if(sensors.at(i).output_datatype == "signal")
					{
						std::string topic = "/" + sensors.at(i).name;
						ros::Publisher pub = n->advertise<icarus_rover_v2::signal>(topic,10);
						signal_sensor_pubs.push_back(pub);
						signal_sensor_names.push_back(sensors.at(i).name);
					}
				}
				logger->log_notice("Device finished initializing.");
			}
			else
			{
			}
		}
		else
		{
		}
	}
	return true;
}
void Device_Callback(const icarus_rover_v2::device::ConstPtr& msg)
{

}
void signalinterrupt_handler(int sig)
{
	kill_node = 1;
}
//End Template Code: Functions
