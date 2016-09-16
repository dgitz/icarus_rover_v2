#include "network_transceiver_node.h"
//Start Template Code: Firmware Definition
#define NETWORKTRANSCEIVERNODE_MAJOR_RELEASE 1
#define NETWORKTRANSCEIVERNODE_MINOR_RELEASE 1
#define NETWORKTRANSCEIVERNODE_BUILD_NUMBER 1
//End Template Code: Firmware Definition
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
	if(sendto(senddevice_sock, send_string.c_str(), send_string.size(), 0, (struct sockaddr *)&senddevice_addr, sizeof(senddevice_addr))!=send_string.size())
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
	if(sendto(senddevice_sock, send_string.c_str(), send_string.size(), 0, (struct sockaddr *)&senddevice_addr, sizeof(senddevice_addr))!=send_string.size())
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
	if(sendto(senddevice_sock, send_string.c_str(), send_string.size(), 0, (struct sockaddr *)&senddevice_addr, sizeof(senddevice_addr))!=send_string.size())
	{
		  logger->log_warn("Mismatch in number of bytes sent");

	}
}
bool process_udp_receive()
{
	unsigned char buf[RECV_BUFFERSIZE];
	socklen_t addrlen = sizeof(remote_addr);
	int recvlen = recvfrom(recvdevice_sock,buf,RECV_BUFFERSIZE,0,(struct sockaddr *)&remote_addr,&addrlen);
	char tempstr[128];
	sprintf(tempstr,"Received bytes: %d",recvlen);
	logger->log_debug(tempstr);

}
bool initialize_recvsocket()
{
	if((recvdevice_sock = socket(AF_INET,SOCK_DGRAM,0)) < 0)
	{
		logger->log_error("Failed to create recv socket. Exiting.");
		return false;
	}
	memset((char*)&my_addr,0,sizeof(my_addr));
	my_addr.sin_family = AF_INET;
	my_addr.sin_addr.s_addr = htonl(INADDR_ANY);
	my_addr.sin_port = htons(recv_unicast_port);
	if(bind(recvdevice_sock,(struct sockaddr *)&my_addr,sizeof(my_addr)) < 0)
	{
		logger->log_error("Failed to bind recv socket. Exiting.");
		return false;
	}
	fcntl(recvdevice_sock,F_SETFL,O_NONBLOCK);

	return true;
}
bool initialize_sendsocket()
{
	memset(&senddevice_addr, 0, sizeof(senddevice_addr));
	senddevice_addr.sin_family=AF_INET;
	//Create the socket
	if((senddevice_sock=socket(AF_INET, SOCK_DGRAM, 0))<0)
	{
		logger->log_error("Failed to create send socket. Exiting.");
		return false;
	}

	if(bind(senddevice_sock,( struct sockaddr *) &senddevice_addr, sizeof(senddevice_addr))<0)
	{
		logger->log_error("Failed to bind send socket. Exiting.");
		return false;
	}
	inet_pton(AF_INET,send_multicast_group.c_str(),&senddevice_addr.sin_addr.s_addr);
	senddevice_addr.sin_port=htons(send_multicast_port);
	return true;
}
bool run_fastrate_code()
{
	if(process_udp_receive() == false)
	{
		logger->log_warn("Unable to process UDP Receive message.");
	}
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
	//logger->log_debug("Running slow rate code.");

	return true;
}
bool run_veryslowrate_code()
{
	//logger->log_debug("Running very slow rate code.");
	logger->log_info("Node Running.");
	icarus_rover_v2::firmware fw;
	fw.Generic_Node_Name = "network_transceiver_Node";
	fw.Node_Name = node_name;
	fw.Description = "Latest Rev: 8-Sep-2016";
	fw.Major_Release = NETWORKTRANSCEIVERNODE_MAJOR_RELEASE;
	fw.Minor_Release = NETWORKTRANSCEIVERNODE_MINOR_RELEASE;
	fw.Build_Number = NETWORKTRANSCEIVERNODE_BUILD_NUMBER;
	firmware_pub.publish(fw);
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
    std::string firmware_topic = "/" + node_name + "/firmware";
    firmware_pub =  nh.advertise<icarus_rover_v2::firmware>(firmware_topic,1000);
    //End Template Code: Initialization and Parameters

    //Start User Code: Initialization and Parameters
    std::string param_send_multicast_address = node_name +"/Send_Multicast_Group";
    if(nh.getParam(param_send_multicast_address,send_multicast_group) == false)
   	{
   		logger->log_warn("Missing Parameter: Send_Multicast_Group. Exiting.");
   		return false;
   	}

    std::string param_send_multicast_port = node_name +"/Send_Multicast_Port";
   	if(nh.getParam(param_send_multicast_port,send_multicast_port) == false)
   	{
   		logger->log_warn("Missing Parameter: Send_Multicast_Port. Exiting.");
   		return false;
   	}
   	std::string param_recv_unicast_port = node_name +"/Recv_Unicast_Port";
	if(nh.getParam(param_recv_unicast_port,recv_unicast_port) == false)
	{
		logger->log_warn("Missing Parameter: Recv_Unicast_Port. Exiting.");
		return false;
	}
   	if(initialize_sendsocket() == false)
   	{
   		logger->log_error("Couldn't initialize send socket.  Exiting.");
   		return false;
   	}
   	if(initialize_recvsocket() == false)
	{
		logger->log_error("Couldn't initialize recv socket.  Exiting.");
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
void Device_Callback(const icarus_rover_v2::device::ConstPtr& msg)
{
	icarus_rover_v2::device newdevice;
	newdevice.DeviceName = msg->DeviceName;
	newdevice.Architecture = msg->Architecture;
	myDevice = newdevice;
	if(myDevice.DeviceName != "")
	{
		resourcemonitor = new ResourceMonitor(myDevice.Architecture,myDevice.DeviceName,node_name);
		device_initialized = true;
	}
}
//End Template Code: Functions
