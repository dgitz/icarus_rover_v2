#include "gpio_node.h"
//Start Template Code: Firmware Definition
#define GPIONODE_MAJOR_RELEASE 2
#define GPIONODE_MINOR_RELEASE 1
#define GPIONODE_BUILD_NUMBER 1
//End Template Code: Firmware Definition
//Start User Code: Functions

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
	if(new_message == true)
	{
		new_message = false;
		if(packet_type == SERIAL_TestMessageCounter_ID)
		{
			diagnostic_status = process->new_serialmessage_TestMessageCounter(packet_type,packet);
		}
		else if(packet_type == SERIAL_FirmwareVersion_ID)
		{
			diagnostic_status = process->new_serialmessage_FirmwareVersion(packet_type,packet);
		}
		else if(packet_type == SERIAL_Diagnostic_ID)
		{
			diagnostic_status = process->new_serialmessage_Diagnostic(packet_type,packet);
		}
		else if(packet_type == SERIAL_Get_ANA_PortA_ID)
		{
			diagnostic_status = process->new_serialmessage_Get_ANA_PortA(packet_type,packet);
		}
		else if(packet_type == SERIAL_Get_ANA_PortB_ID)
		{
			diagnostic_status = process->new_serialmessage_Get_ANA_PortB(packet_type,packet);
		}
		else if(packet_type == SERIAL_Get_DIO_PortA_ID)
		{
			diagnostic_status = process->new_serialmessage_Get_DIO_PortA(packet_type,packet);
		}
		else if(packet_type == SERIAL_Get_DIO_PortB_ID)
		{
			diagnostic_status = process->new_serialmessage_Get_DIO_PortB(packet_type,packet);
		}
		else if(packet_type == SERIAL_Mode_ID)
		{
			diagnostic_status = process->new_serialmessage_Get_Mode(packet_type,packet);
		}
	}
	diagnostic_status = process->update(20);  //Need to change 20 to the actual dt!!!
	std::vector<std::vector<unsigned char> > tx_buffers;
	bool send = process->checkTriggers(tx_buffers);
	if(send == true)
	{
		for(int i = 0; i < tx_buffers.size();i++)
		{
			unsigned char tx_buffer[12];
			std::vector<unsigned char> tempstr = tx_buffers.at(i);
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
		}
	}
	//diagnostic_pub.publish(diagnostic_status);
	return true;
}
bool run_mediumrate_code()
{
	
	
	diagnostic_pub.publish(diagnostic_status);
	return true;
}
bool run_slowrate_code()
{
	char tempstr[128];
	sprintf(tempstr,"Board Mode: %d Node Mode: %d",process->get_boardstate(),process->get_nodestate());
    logger->log_debug(tempstr);
	if(device_initialized == true)
	{
		bool status = resourcemonitor->update();
		if(status == true)
		{
			resources_used = resourcemonitor->get_resourceused();
			resource_pub.publish(resources_used);
		}
		else
		{
			logger->log_warn("Couldn't read resources used.");
		}
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
	fw.Description = "Latest Rev: 8-Sep-2016";
	fw.Major_Release = GPIONODE_MAJOR_RELEASE;
	fw.Minor_Release = GPIONODE_MINOR_RELEASE;
	fw.Build_Number = GPIONODE_BUILD_NUMBER;
	firmware_pub.publish(fw);
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
    resource_monitor_running = false;
    device_initialized = false;
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
	hostname[1023] = '\0';
	gethostname(hostname,1023);
	process = new GPIONodeProcess;
	diagnostic_status = process->init(diagnostic_status,logger,std::string(hostname));
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
    std::string firmware_topic = "/" + node_name + "/firmware";
    firmware_pub =  nh.advertise<icarus_rover_v2::firmware>(firmware_topic,1000);
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
	newdevice.pins = msg->pins;
	diagnostic_status = process->new_devicemsg(newdevice);
	if((newdevice.DeviceName == hostname) and (resource_monitor_running == false) and (process->is_finished_initializing() == true))
	{
		myDevice = process->get_mydevice();
		resourcemonitor = new ResourceMonitor(myDevice.Architecture,myDevice.DeviceName,node_name);
		resource_monitor_running = true;
	}

}
//End Template Code: Functions
