#include "gpio_node.h"
//Start User Code: Functions
bool run_fastrate_code()
{
	unsigned char rx_buffer[12];
	memset(rx_buffer, 0, sizeof(rx_buffer));
	memset(packet_data,0,sizeof(packet_data));
	ros::Time start_time = ros::Time::now();
	int rx_length = read(device_fid, rx_buffer, sizeof(rx_buffer));
	double readtime = measure_time_diff(ros::Time::now(),start_time);
	printf("Read Time: %f Bytes Read: %d\r\n",readtime,rx_length);
	if(rx_length > 0)
	{
		if(rx_buffer[0] == 0xAB)
		{
			packet_length = rx_buffer[2];
			int checksum = 0;
			for(int i = 3; i < 3+packet_length;i++)
			{
				packet_data[i-3] = rx_buffer[i];
				checksum ^= packet_data[i-3];
			}
			int recv_checksum = rx_buffer[11];
			if(recv_checksum == checksum)
			{
				packet_type = rx_buffer[1];
				new_message = true;
			}
			else
			{
				bad_checksum_counter++;
			}
		}
	}
	if(new_message == true)
	{
		new_message = false;
		if(packet_type ==SERIAL_TestMessageCounter_ID)
		{
			char value1,value2,value3,value4,value5,value6,value7,value8;
			serialmessagehandler->decode_TestMessageCounterSerial(packet_data,&value1,&value2,&value3,&value4,&value5,&value6,&value7,&value8);
			//printf("TestMessage Counter: 1: %d 2: %d 3: %d 4: %d 5: %d 6: %d 7: %d 8: %d\r\n",value1,value2,value3,value4,value5,value6,value7,value8);
			last_num = current_num;
			current_num = value1;
			missed_counter += abs(current_num-last_num)-1;
			
			
		}
		else if(packet_type ==SERIAL_TestMessageCommand_ID)
		{
			char value1,value2,value3,value4,value5,value6,value7,value8;
			serialmessagehandler->decode_TestMessageCommandSerial(packet_data,&value1,&value2,&value3,&value4,&value5,&value6,&value7,&value8);
			//printf("TestMessage Command: 1: %d 2: %d 3: %d 4: %d 5: %d 6: %d 7: %d 8: %d\r\n",value1,value2,value3,value4,value5,value6,value7,value8);
		}
	}
	return true;
}
bool run_mediumrate_code()
{

	printf("Missed Counter: %d Bad checksum: %d\r\n",missed_counter,bad_checksum_counter);
	//logger->log_debug("Running medium rate code.");
	diagnostic_status.Diagnostic_Type = SOFTWARE;
	diagnostic_status.Level = INFO;
	diagnostic_status.Diagnostic_Message = NOERROR;
	diagnostic_status.Description = "Node Executing.";
	diagnostic_pub.publish(diagnostic_status);

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
	unsigned char tx_buffer[12];
	int length;
	int status = serialmessagehandler->encode_TestMessageCommandSerial(tx_buffer,&length,0,1,2,3,4,5,6,6);
	if (status == 1)
	{
		/*int count = write(device_fid, &tx_buffer[0], length);
		if (count < 0)
		{
			logger->log_error("UART TX error\n");
			diagnostic_status.Diagnostic_Type = COMMUNICATIONS;
			diagnostic_status.Level = ERROR;
			diagnostic_status.Diagnostic_Message = DROPPING_PACKETS;
			diagnostic_status.Description = "Cannot write to UART.";
			diagnostic_pub.publish(diagnostic_status);
		}
		*/
	}

	return true;
}
bool run_veryslowrate_code()
{
	//logger->log_debug("Running very slow rate code.");
	logger->log_info("Node Running.");
	return true;
}
//End User Code: Functions

//Start Template Code: Functions
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
    fast_timer = now;
    medium_timer = now;
    slow_timer = now;
    veryslow_timer = now;
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
	diagnostic_status.Component = TIMING_NODE;

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
    device_sub = nh.subscribe<icarus_rover_v2::device>(device_topic,1000,Device_Callback);

    pps_sub = nh.subscribe<std_msgs::Bool>("/pps",1000,PPS_Callback);  //This is a pps consumer.
    std::string param_require_pps_to_start = node_name +"/require_pps_to_start";
    if(nh.getParam(param_require_pps_to_start,require_pps_to_start) == false)
	{
		logger->log_warn("Missing Parameter: require_pps_to_start.");
		return false;
	}

    //End Template Code: Initialization and Parameters

    //Start User Code: Initialization and Parameters
	current_num = -1;
	last_num = -1;
	missed_counter = 0;
	bad_checksum_counter = 0;
	new_message = false;
	device_fid = open("/dev/ttyAMA0", O_RDWR | O_NOCTTY | O_NDELAY);
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
void Device_Callback(const icarus_rover_v2::device::ConstPtr& msg)
{
	icarus_rover_v2::device newdevice;
	newdevice.DeviceName = msg->DeviceName;
	newdevice.Architecture = msg->Architecture;
	newdevice.DeviceType = msg->DeviceType;
	newdevice.DeviceParent = msg->DeviceParent;
	newdevice.BoardCount = msg->BoardCount;
	if((newdevice.DeviceParent == "None") && (device_initialized == false))
	{
		myDevice = newdevice;
	}
	else
	{
		icarus_rover_v2::device board;
		board = newdevice;
		board.pins = newdevice.pins;
		boards.push_back(board);
		if((myDevice.BoardCount == boards.size()) && (myDevice.BoardCount > 0) && (device_initialized == false))
		{
			diagnostic_status.Diagnostic_Type = NOERROR;
			diagnostic_status.Level = INFO;
			diagnostic_status.Diagnostic_Message = NOERROR;
			diagnostic_status.Description = "Received all Device Info.";
			diagnostic_pub.publish(diagnostic_status);
			logger->log_info("Received all Device Info.");
			device_initialized = true;
		}
	}
}
//End Template Code: Functions
