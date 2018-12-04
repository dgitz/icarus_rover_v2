#include "boardcontroller_node.h"
bool kill_node = false;
bool BoardControllerNode::start(int argc,char **argv)
{
	bool status = false;
	process = new BoardControllerNodeProcess();
	set_basenodename(BASE_NODE_NAME);
	initialize_firmware(MAJOR_RELEASE_VERSION,MINOR_RELEASE_VERSION,BUILD_NUMBER,FIRMWARE_DESCRIPTION);
	initialize_diagnostic(DIAGNOSTIC_SYSTEM,DIAGNOSTIC_SUBSYSTEM,DIAGNOSTIC_COMPONENT);
	diagnostic = preinitialize_basenode(argc,argv);
	if(diagnostic.Level > WARN)
	{
		return false;
	}
	diagnostic = read_launchparameters();
	if(diagnostic.Level > WARN)
	{
		return false;
	}

	process->initialize(get_basenodename(),get_nodename(),get_hostname());
	process->set_diagnostic(diagnostic);
	process->finish_initialization();
	diagnostic = finish_initialization();
	if(diagnostic.Level > WARN)
	{
		return false;
	}
	if(diagnostic.Level < WARN)
	{
		diagnostic.Diagnostic_Type = NOERROR;
		diagnostic.Level = INFO;
		diagnostic.Diagnostic_Message = NOERROR;
		diagnostic.Description = "Node Configured.  Initializing.";
		get_logger()->log_diagnostic(diagnostic);
	}
	status = true;
	return status;
}

icarus_rover_v2::diagnostic BoardControllerNode::read_launchparameters()
{
	icarus_rover_v2::diagnostic diag = diagnostic;
	get_logger()->log_notice("Configuration Files Loaded.");
	return diagnostic;
}
icarus_rover_v2::diagnostic BoardControllerNode::finish_initialization()
{
	icarus_rover_v2::diagnostic diag = diagnostic;
	passed_checksum = 0;
	failed_checksum = 0;
	pps1_sub = n->subscribe<std_msgs::Bool>("/1PPS",1,&BoardControllerNode::PPS1_Callback,this);
	command_sub = n->subscribe<icarus_rover_v2::command>("/command",1,&BoardControllerNode::Command_Callback,this);
	std::string device_topic = "/" + std::string(host_name) + "_master_node/srv_device";
	srv_device = n->serviceClient<icarus_rover_v2::srv_device>(device_topic);
	spi_device = open("/dev/spidev0.0", O_RDWR);
	unsigned int clock_rate = 1000000;
	ioctl (spi_device, SPI_IOC_WR_MAX_SPEED_HZ, &clock_rate);
	return diagnostic;
}
bool BoardControllerNode::run_001hz()
{
	return true;
}
bool BoardControllerNode::run_01hz()
{
	icarus_rover_v2::diagnostic diag = process->get_diagnostic();
	{
		get_logger()->log_diagnostic(diag);
		diagnostic_pub.publish(diag);
	}
	char tempstr[512];
	sprintf(tempstr,"Passed Checksum: %d @ %f Failed Checksum: %d @ %f",
			passed_checksum,(double)passed_checksum/process->get_runtime(),
			failed_checksum,(double)failed_checksum/process->get_runtime());
	logger->log_info(tempstr);
	logger->log_info(process->get_messageinfo(false));
	return true;
}
bool BoardControllerNode::run_1hz()
{
	if((process->is_initialized() == true) and (process->is_ready() == true))
	{
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

					get_logger()->log_error("Got unexpected device message.");
				}
				else
				{
					bool status = new_devicemsg(srv.request.query,srv.response.data.at(0));
				}
			}
			else
			{
			}
		}
	}
	icarus_rover_v2::diagnostic diag = process->get_diagnostic();
	if(diag.Level >= NOTICE)
	{
		get_logger()->log_diagnostic(diag);
		diagnostic_pub.publish(diag);
	}

	return true;
}
bool BoardControllerNode::run_10hz()
{
	ready_to_arm = process->get_ready_to_arm();
	return true;
}
bool BoardControllerNode::run_loop1()
{
	process->send_commandmessage(SPIMessageHandler::SPI_LEDStripControl_ID);
	process->send_querymessage(SPIMessageHandler::SPI_Diagnostic_ID);
	icarus_rover_v2::diagnostic diag = process->get_diagnostic();
	if(diag.Level > WARN)
	{
		diagnostic_pub.publish(diag);
		logger->log_diagnostic(diag);
	}
	diag = process->send_querymessage(SPIMessageHandler::SPI_Get_DIO_Port1_ID);
	std::vector<BoardControllerNodeProcess::Sensor> sensors = process->get_sensordata();
	for(std::size_t i = 0; i < sensors.size(); i++)
	{
		signal_sensor_pubs.at(i).publish(sensors.at(i).signal);
	}
	return true;
}
bool BoardControllerNode::run_loop2()
{
	icarus_rover_v2::diagnostic diag = process->update(0.1,ros::Time::now().toSec());
	return true;
}
bool BoardControllerNode::run_loop3()
{
	icarus_rover_v2::diagnostic diag = process->get_diagnostic();
	std::vector<BoardControllerNodeProcess::Message> querymessages_tosend = process->get_querymessages_tosend();
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
	std::vector<BoardControllerNodeProcess::Message> commandmessages_tosend = process->get_commandmessages_tosend();
	for(std::size_t i = 0; i < commandmessages_tosend.size(); i++)
	{
		int success;
		int length;
		unsigned char outputbuffer[12];
		unsigned char v1,v2,v3;
		switch(commandmessages_tosend.at(i).id)
		{
		case SPIMessageHandler::SPI_LEDStripControl_ID:
			diag = process->get_LEDStripControlParameters(v1,v2,v3);
			success = spimessagehandler->encode_LEDStripControlSPI(outputbuffer,&length,v1,v2,v3);
			break;
		default:
			break;
		}
		sendMessageCommand(commandmessages_tosend.at(i).id,outputbuffer);
		process->new_message_sent(SPIMessageHandler::SPI_LEDStripControl_ID);
	}
	return true;
}

void BoardControllerNode::PPS1_Callback(const std_msgs::Bool::ConstPtr& msg)
{
	new_ppsmsg(msg);
}

void BoardControllerNode::Command_Callback(const icarus_rover_v2::command::ConstPtr& t_msg)
{
	std::vector<icarus_rover_v2::diagnostic> diaglist = process->new_commandmsg(t_msg);
	new_commandmsg_result(t_msg,diaglist);
}
bool BoardControllerNode::new_devicemsg(std::string query,icarus_rover_v2::device t_device)
{
	if(query == "SELF")
	{
		if(t_device.DeviceName == std::string(host_name))
		{
			set_mydevice(t_device);
			process->set_mydevice(t_device);
		}
	}


	if((process->is_initialized() == true))
	{
		icarus_rover_v2::device::ConstPtr device_ptr(new icarus_rover_v2::device(t_device));
		icarus_rover_v2::diagnostic diag = process->new_devicemsg(device_ptr);
	}
	if((process->is_ready() == true))
	{
		std::vector<BoardControllerNodeProcess::Sensor> sensors = process->get_sensordata();
		for(std::size_t i = 0; i < sensors.size(); i++)
		{
			if(sensors.at(i).output_datatype == "signal")
			{
				std::string topic = "/" + sensors.at(i).name;
				ros::Publisher pub = n->advertise<icarus_rover_v2::signal>(topic,10);
				signal_sensor_pubs.push_back(pub);
			}
		}
	}
	return true;
}
int BoardControllerNode::spiTxRx(unsigned char txDat)
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
int BoardControllerNode::sendMessageCommand(unsigned char command,unsigned char * outputbuffer)
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
int BoardControllerNode::sendMessageQuery(unsigned char query, unsigned char * inputbuffer)
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
void BoardControllerNode::thread_loop()
{
	while(kill_node == false)
	{
		ros::Duration(1.0).sleep();
	}
}
void BoardControllerNode::cleanup()
{
}
/*! \brief Attempts to kill a node when an interrupt is received.
 *
 */
void signalinterrupt_handler(int sig)
{
	kill_node = true;
	exit(0);
}
int main(int argc, char **argv) {
	signal(SIGINT, signalinterrupt_handler);
	signal(SIGTERM, signalinterrupt_handler);
	BoardControllerNode *node = new BoardControllerNode();
	bool status = node->start(argc,argv);
	std::thread thread(&BoardControllerNode::thread_loop, node);
	while((status == true) and (kill_node == false))
	{
		status = node->update();
	}
	node->get_logger()->log_info("Node Finished Safely.");
	return 0;
}

