#include "network_transceiver_node.h"
//Start User Code: Functions
void diagnostic_Callback(const icarus_rover_v2::diagnostic::ConstPtr& msg)
{
	char tempstr[240];
	sprintf(tempstr,"Got Diagnostic from Task: %s with System: %d Level: %d",msg->Node_Name.c_str(),msg->Level,msg->System);
	logger->log_info(tempstr);
	std::string send_string = udpmessagehandler->encode_DiagnosticUDP(msg->Node_Name,
																(uint8_t)msg->System,
																(uint8_t)msg->SubSystem,
																(uint8_t)msg->Component,
																(uint8_t)msg->Diagnostic_Type,
																(uint8_t)msg->Level,
																(uint8_t)msg->Diagnostic_Message,
																msg->Description);
	if(sendto(device_sock, send_string.c_str(), send_string.size(), 0, (struct sockaddr *)&device_addr, sizeof(device_addr))!=send_string.size())
	{
		  logger->log_warn("Mismatch in number of bytes sent");

	}
}
void resource_Callback(const icarus_rover_v2::resource::ConstPtr& msg)
{
	char tempstr[240];
	sprintf(tempstr,"Got Resource from Task: %s",msg->Node_Name.c_str());
	logger->log_info(tempstr);
	std::string send_string = udpmessagehandler->encode_ResourceUDP(msg->Node_Name,
																	msg->RAM_MB,
																	msg->CPU_Perc);
	if(sendto(device_sock, send_string.c_str(), send_string.size(), 0, (struct sockaddr *)&device_addr, sizeof(device_addr))!=send_string.size())
	{
		  logger->log_warn("Mismatch in number of bytes sent");

	}
}
void device_Callback(const icarus_rover_v2::device::ConstPtr& msg)
{
	char tempstr[240];
	sprintf(tempstr,"Got Device from Task: %s",msg->DeviceName.c_str());
	logger->log_info(tempstr);
	std::string send_string = udpmessagehandler->encode_DeviceUDP(msg->DeviceParent,
																msg->DeviceName,
																msg->DeviceType,
																msg->Architecture);
	if(sendto(device_sock, send_string.c_str(), send_string.size(), 0, (struct sockaddr *)&device_addr, sizeof(device_addr))!=send_string.size())
	{
		  logger->log_warn("Mismatch in number of bytes sent");

	}
}
bool initialize_socket()
{
	memset(&device_addr, 0, sizeof(device_addr));
	device_addr.sin_family=AF_INET;
	//Create the socket
	if((device_sock=socket(AF_INET, SOCK_DGRAM, 0))<0)
	{
		logger->log_error("Failed to create socket");
		return false;
	}

	if(bind(device_sock,( struct sockaddr *) &device_addr, sizeof(device_addr))<0)
	{
		logger->log_error("Failed to bind socket");
		return false;
	}
	inet_pton(AF_INET,multicast_group.c_str(),&device_addr.sin_addr.s_addr);
	device_addr.sin_port=htons(multicast_port);
	return true;
}
bool run_fastrate_code()
{
	//logger->log_debug("Running fast rate code.");
	return true;
}
bool run_mediumrate_code()
{
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

	//logger->log_debug("Running slow rate code.");

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
 
	node_name = "network_transceiver_node";


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


	int counter = 0;
    while (ros::ok())
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
    return 0;
}

bool initialize(ros::NodeHandle nh)
{
	sleep(5);
    //Start Template Code: Initialization and Parameters
    myDevice.DeviceName = "";
    myDevice.Architecture = "";
    device_initialized = false;
    pid = -1;
    std::string diagnostic_topic = "/" + node_name + "/diagnostic";
	diagnostic_pub =  nh.advertise<icarus_rover_v2::diagnostic>(diagnostic_topic,1000);
	diagnostic_status.Node_Name = node_name;
	diagnostic_status.System = ROVER;
	diagnostic_status.SubSystem = ROBOT_CONTROLLER;
	diagnostic_status.Component = COMMUNICATION_NODE;

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
    std::string param_multicast_address = node_name +"/Multicast_Group";
    if(nh.getParam(param_multicast_address,multicast_group) == false)
   	{
   		logger->log_warn("Missing Parameter: Multicast_Group.");
   		return false;
   	}

    std::string param_multicast_port = node_name +"/Multicast_Port";
   	if(nh.getParam(param_multicast_port,multicast_port) == false)
   	{
   		logger->log_warn("Missing Parameter: Multicast_Port.");
   		return false;
   	}
   	if(initialize_socket() == false)
   	{
   		logger->log_error("Couldn't initialize socket.  Exiting.");
   		return false;
   	}
   	std::string param_Mode = node_name +"/Mode";
	if(nh.getParam(param_Mode,Mode) == false)
	{
		logger->log_warn("Missing Parameter: Mode.");
		return false;
	}
	sleep(5);
	if(Mode=="Diagnostics_GUI")
	{
		ros::master::V_TopicInfo master_topics;
		ros::master::getTopics(master_topics);

		std::vector<std::string> diagnostic_topics;
		for (ros::master::V_TopicInfo::iterator it = master_topics.begin() ; it != master_topics.end(); it++)
		{
			const ros::master::TopicInfo& info = *it;
			if(info.datatype == "icarus_rover_v2/diagnostic")
			{
				diagnostic_topics.push_back(info.name);
			}

		}
		ros::Subscriber * diagnostic_subs;
		diagnostic_subs = new ros::Subscriber[diagnostic_topics.size()];
		for(int i = 0; i < diagnostic_topics.size();i++)
		{
			char tempstr[50];
			sprintf(tempstr,"Subscribing to diagnostic topic: %s",diagnostic_topics.at(i).c_str());
			logger->log_info(tempstr);
			diagnostic_subs[i] = nh.subscribe<icarus_rover_v2::diagnostic>(diagnostic_topics.at(i),1000,diagnostic_Callback);
		}

		std::vector<std::string> device_topics;
		for (ros::master::V_TopicInfo::iterator it = master_topics.begin() ; it != master_topics.end(); it++)
		{
			const ros::master::TopicInfo& info = *it;
			if(info.datatype == "icarus_rover_v2/device")
			{
				device_topics.push_back(info.name);
			}

		}
		ros::Subscriber * device_subs;
		device_subs = new ros::Subscriber[device_topics.size()];
		for(int i = 0; i < device_topics.size();i++)
		{
			char tempstr[50];
			sprintf(tempstr,"Subscribing to device topic: %s",device_topics.at(i).c_str());
			logger->log_info(tempstr);
			device_subs[i] = nh.subscribe<icarus_rover_v2::device>(device_topics.at(i),1000,device_Callback);
		}

		std::vector<std::string> resource_topics;
		for (ros::master::V_TopicInfo::iterator it = master_topics.begin() ; it != master_topics.end(); it++)
		{
			const ros::master::TopicInfo& info = *it;
			if(info.datatype == "icarus_rover_v2/resource")
			{
				resource_topics.push_back(info.name);
			}

		}
		ros::Subscriber * resource_subs;
		resource_subs = new ros::Subscriber[resource_topics.size()];
		for(int i = 0; i < resource_topics.size();i++)
		{
			char tempstr[50];
			sprintf(tempstr,"Subscribing to resource topic: %s",resource_topics.at(i).c_str());
			logger->log_info(tempstr);
			resource_subs[i] = nh.subscribe<icarus_rover_v2::resource>(resource_topics.at(i),1000,resource_Callback);
		}
	}
	udpmessagehandler = new UDPMessageHandler();
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
		std::size_t found = line.find("icarus_rover_v2/network_transceiver_node");
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
	myDevice = newdevice;
	if(myDevice.DeviceName != "")
	{
		device_initialized = true;
	}
}
//End Template Code: Functions
