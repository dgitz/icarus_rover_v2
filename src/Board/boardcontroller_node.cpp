#include "boardcontroller_node.h"
//Start User Code: Firmware Definition
#define BOARDCONTROLLERNODE_MAJOR_RELEASE 0
#define BOARDCONTROLLERNODE_MINOR_RELEASE 0
#define BOARDCONTROLLERNODE_BUILD_NUMBER 0
//End User Code: Firmware Definition
//Start User Code: Functions
bool run_loop1_code()
{
	{
		icarus_rover_v2::diagnostic diag = process->send_querymessage(SPIMessageHandler::SPI_Get_ANA_Port1_ID);
		if(diag.Level >= WARN)
		{
			diagnostic_pub.publish(diag);
			logger->log_diagnostic(diag);
		}
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
	std::vector<Sensor> sensors = process->get_sensordata();
	for(std::size_t i = 0; i < sensors.size(); i++)
	{
		for(std::size_t j = 0; j < analog_sensor_names.size(); j++)
		{
			if(analog_sensor_names.at(j) == sensors.at(i).name)
			{
				std_msgs::Float32 v;
				v.data = sensors.at(i).value;
				analog_sensor_pubs.at(j).publish(v);
			}
		}
	}
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
						diag = process->new_message_GetANAPort1(BOARD_ID,a1,a2,a3,a4,a5,a6);
						if(diag.Level >= WARN)
						{
							diagnostic_pub.publish(diag);
							logger->log_diagnostic(diag);
						}
					}
					break;
				case SPIMessageHandler::SPI_Get_DIO_Port1_ID:
					success = spimessagehandler->decode_Get_DIO_Port1SPI(inputbuffer,&length,&v1,&v2,&v3,&v4,&v5,&v6,&v7,&v8);
					if(success == 1)
					{
						process->new_message_recv(querymessages_tosend.at(i).id);
						diag = process->new_message_GetDIOPort1(BOARD_ID,v1,v2,v3,v4,v5,v6,v7,v8);
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
			printf("No Comm with device.\n");
		}
	}
 	return true;
}

void PPS01_Callback(const std_msgs::Bool::ConstPtr& msg)
{
	icarus_rover_v2::firmware fw;
	fw.Generic_Node_Name = "boardcontroller_node";
	fw.Node_Name = node_name;
	fw.Description = "Latest Rev: 14-Sep-2017";
	fw.Major_Release = BOARDCONTROLLERNODE_MAJOR_RELEASE;
	fw.Minor_Release = BOARDCONTROLLERNODE_MINOR_RELEASE;
	fw.Build_Number = BOARDCONTROLLERNODE_BUILD_NUMBER;
	firmware_pub.publish(fw);
}
void PPS1_Callback(const std_msgs::Bool::ConstPtr& msg)
{
	received_pps = true;
    if(device_initialized == true)
	{
		icarus_rover_v2::diagnostic resource_diagnostic = resourcemonitor->update();
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
    std::string device_topic = "/" + std::string(hostname) +"_master_node/device";
    device_sub = n->subscribe<icarus_rover_v2::device>(device_topic,1000,Device_Callback);
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
    passed_checksum = 0;
    failed_checksum = 0;
    process = new BoardControllerNodeProcess;
	diagnostic_status = process->init(diagnostic_status,std::string(hostname));
	spi_device = open("/dev/spidev0.0", O_RDWR);
	unsigned int clock_rate = 1000000;
	ioctl (spi_device, SPI_IOC_WR_MAX_SPEED_HZ, &clock_rate);

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
void Device_Callback(const icarus_rover_v2::device::ConstPtr& msg)
{
	if(process->is_ready() == false)
	{
		logger->log_notice("Device not initialized yet.");
		icarus_rover_v2::device newdevice;
		newdevice.Architecture = msg->Architecture;
		newdevice.BoardCount = msg->BoardCount;
		newdevice.Capabilities = msg->Capabilities;
		newdevice.DeviceName = msg->DeviceName;
		newdevice.DeviceParent = msg->DeviceParent;
		newdevice.DeviceType = msg->DeviceType;
		newdevice.ID = msg->ID;
		newdevice.SensorCount = msg->SensorCount;
		newdevice.ShieldCount = msg->ShieldCount;
		newdevice.pins = msg->pins;
		diagnostic_status = process->new_devicemsg(newdevice);
		logger->log_diagnostic(diagnostic_status);
		if(diagnostic_status.Level > INFO) { diagnostic_pub.publish(diagnostic_status); }
		if(process->is_ready() == true)
		{
			myDevice = process->get_mydevice();
			resourcemonitor = new ResourceMonitor(diagnostic_status,myDevice.Architecture,myDevice.DeviceName,node_name);
			std::vector<Sensor> sensors = process->get_sensordata();
			for(std::size_t i = 0; i < sensors.size(); i++)
			{
				if(sensors.at(i).output_datatype == "double")
				{
					std::string analog_topic = "/" + sensors.at(i).name;
					ros::Publisher pub = n->advertise<std_msgs::Float32>(analog_topic,10);
					analog_sensor_pubs.push_back(pub);
					analog_sensor_names.push_back(sensors.at(i).name);
				}
			}
			logger->log_notice("Device finished initializing.");
		}
	}
}
void signalinterrupt_handler(int sig)
{
	kill_node = 1;
}
//End Template Code: Functions
