#include "network_transceiver_node.h"
//Start User Code: Firmware Definition
#define NETWORKTRANSCEIVERNODE_MAJOR_RELEASE 3
#define NETWORKTRANSCEIVERNODE_MINOR_RELEASE 1
#define NETWORKTRANSCEIVERNODE_BUILD_NUMBER 4
//End User Code: Firmware Definition
//Start User Code: Functions
/*! \brief User Loop1 Code
 */
bool run_loop1_code()
{
	icarus_rover_v2::diagnostic diagnostic = rescan_topics(process->get_diagnostic());
	if(diagnostic.Level > NOTICE)
	{
		diagnostic_pub.publish(diagnostic);
    	logger->log_info(process->get_messageinfo(false));
	}
 	return true;
}
/*! \brief User Loop2 Code
 */
bool run_loop2_code()
{
       

	if((remote_heartbeat_pass == true) and
	   (1 == 1)) //Others??
	{
		ready_to_arm = true;
	}
	else
	{
		ready_to_arm = false;
	}
	std_msgs::Bool bool_ready_to_arm;
	bool_ready_to_arm.data = ready_to_arm;
	ready_to_arm_pub.publish(bool_ready_to_arm);
 	return true;
}
/*! \brief User Loop3 Code
 */
bool run_loop3_code()
{
   	icarus_rover_v2::diagnostic diagnostic = process->update(measure_time_diff(ros::Time::now(),last_loop3_timer));
	if(diagnostic.Level > NOTICE)
	{
		diagnostic_pub.publish(diagnostic);
    	logger->log_info(process->get_messageinfo(false));
	}
 	return true;
}
bool check_remoteHeartbeats()
{
	bool heartbeat_pass = true;
	for(int i = 0; i < remote_devices.size();i++)
	{
		gettimeofday(&now2,NULL);
		double last_beat_time_sec = remote_devices.at(i).current_beatepoch_sec + remote_devices.at(i).offset_sec;
		double now_sec = (double)now2.tv_sec + (double)(now2.tv_usec)/1000000.0;
		double time_since_last = now_sec - last_beat_time_sec;
		if(time_since_last > 1.0)
		{
			heartbeat_pass = false;
			char tempstr[255];
			sprintf(tempstr,"Haven't received Heartbeat from: %s in %f Seconds. Disarming.",
					remote_devices.at(i).Name.c_str(),time_since_last);
			logger->log_error(tempstr);
			icarus_rover_v2::diagnostic diagnostic = process->get_diagnostic();
			diagnostic.Diagnostic_Type = COMMUNICATIONS;
			diagnostic.Level = ERROR;
			diagnostic.Diagnostic_Message = MISSING_HEARTBEATS;
			diagnostic.Description = tempstr;
			logger->log_diagnostic(diagnostic);
			diagnostic_pub.publish(diagnostic);

		}
	}
	if(remote_devices.size() == 0)
	{
		icarus_rover_v2::diagnostic diagnostic = process->get_diagnostic();
		diagnostic.Diagnostic_Type = COMMUNICATIONS;
		diagnostic.Level = WARN;
		diagnostic.Diagnostic_Message = DEVICE_NOT_AVAILABLE;
		diagnostic.Description = "No Remote UI Devices Found Yet.";
		diagnostic_pub.publish(diagnostic);
		logger->log_diagnostic(diagnostic);
		heartbeat_pass = false;

	}
	return heartbeat_pass;
}
icarus_rover_v2::diagnostic rescan_topics(icarus_rover_v2::diagnostic diag)
{
	int found_new_topics = 0;
	ros::master::V_TopicInfo master_topics;
	ros::master::getTopics(master_topics);
	for (ros::master::V_TopicInfo::iterator it = master_topics.begin() ; it != master_topics.end(); it++)
	{
		const ros::master::TopicInfo& info = *it;
		if(info.datatype == "icarus_rover_v2/resource")
		{
			bool found = true;
			for(int i = 0; i < resource_topics.size();i++)
			{
				if(resource_topics.at(i) == info.name)
				{
					found = false;
					break;
				}
			}
			if(found == true)
			{
				found_new_topics++;
				resource_topics.push_back(info.name);
				char tempstr[255];
				sprintf(tempstr,"Subscribing to resource topic: %s",info.name.c_str());
				logger->log_info(tempstr);
				ros::Subscriber sub = n->subscribe<icarus_rover_v2::resource>(info.name,1000,resource_Callback);
				resource_subs.push_back(sub);
			}
		}
		else if(info.datatype == "icarus_rover_v2/diagnostic")
		{
			bool found = true;
			for(int i = 0; i < diagnostic_topics.size();i++)
			{
				if(diagnostic_topics.at(i) == info.name)
				{
					found = false;
					break;
				}
			}
			if(found == true)
			{
				found_new_topics++;
				diagnostic_topics.push_back(info.name);
				char tempstr[255];
				sprintf(tempstr,"Subscribing to diagnostic topic: %s",info.name.c_str());
				logger->log_info(tempstr);
				ros::Subscriber sub = n->subscribe<icarus_rover_v2::diagnostic>(info.name,1000,diagnostic_Callback);
				diagnostic_subs.push_back(sub);
			}
		}
		/*
		else if(info.datatype == "icarus_rover_v2/device")
		{
			bool found = true;
			for(int i = 0; i < device_topics.size();i++)
			{
				if(device_topics.at(i) == info.name)
				{
					found = false;
					break;
				}
			}
			if(found == true)
			{
				found_new_topics++;
				device_topics.push_back(info.name);
				char tempstr[255];
				sprintf(tempstr,"Subscribing to device topic: %s",info.name.c_str());
				logger->log_info(tempstr);
				//ros::Subscriber sub = n->subscribe<icarus_rover_v2::device>(info.name,1000,device_Callback);
				//device_subs.push_back(sub);
			}
		}
		*/

	}
	icarus_rover_v2::srv_device srv;
	srv.request.query = "ALL";
	if(srv_device.call(srv) == true)
	{
		if(srv.response.data.size() == 0)
		{
			logger->log_error("Got unexpected device message");
		}
		else
		{
			device_callback(srv.response.data);
		}
	}
	else
	{
	}

	diag.Diagnostic_Type = SOFTWARE;
	diag.Level = INFO;
	diag.Diagnostic_Message = NOERROR;

	char tempstr[255];
	if(found_new_topics > 0)
	{
		sprintf(tempstr,"Rescanned and found %d new topics.",found_new_topics);
	}
	else
	{
		sprintf(tempstr,"Rescanned and found no new topics.");
	}
	logger->log_info(tempstr);
	diag.Description = tempstr;
	return diag;
}
void estop_Callback(const icarus_rover_v2::estop::ConstPtr& msg)
{
	std::string send_string = udpmessagehandler->encode_EStopUDP(msg->name,msg->state);
	if(sendto(senddevice_sock, send_string.c_str(), send_string.size(), 0, (struct sockaddr *)&senddevice_addr, sizeof(senddevice_addr))!=send_string.size())
	{
		logger->log_warn("Mismatch in number of bytes sent");
	}
    else { process->new_message_sent(ESTOP_ID); }
}
void ArmedState_Callback(const std_msgs::UInt8::ConstPtr& msg)
{
	std::string send_string = udpmessagehandler->encode_Arm_StatusUDP(
																		msg->data);
	if(sendto(senddevice_sock, send_string.c_str(), send_string.size(), 0, (struct sockaddr *)&senddevice_addr, sizeof(senddevice_addr))!=send_string.size())
	{
		logger->log_warn("Mismatch in number of bytes sent");
	}
    else { process->new_message_sent(ARM_STATUS_ID); }
}
void diagnostic_Callback(const icarus_rover_v2::diagnostic::ConstPtr& msg)
{
	//char tempstr[255];
	//sprintf(tempstr,"Got Diagnostic from Task: %s with System: %d Level: %d",msg->Node_Name.c_str(),msg->Level,msg->System);
	//logger->log_info(tempstr);
	std::string send_string = udpmessagehandler->encode_DiagnosticUDP(
																		msg->DeviceName,
																		msg->Node_Name,
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
    else { process->new_message_sent(DIAGNOSTIC_ID); }
}
void resource_Callback(const icarus_rover_v2::resource::ConstPtr& msg)
{
	//char tempstr[240];
	//sprintf(tempstr,"Got Resource from Task: %s",msg->Node_Name.c_str());
	//logger->log_info(tempstr);
	std::string send_string = udpmessagehandler->encode_ResourceUDP(msg->Node_Name,
																	msg->RAM_MB,
																	msg->CPU_Perc);
	if(sendto(senddevice_sock, send_string.c_str(), send_string.size(), 0, (struct sockaddr *)&senddevice_addr, sizeof(senddevice_addr))!=send_string.size())
	{
		  logger->log_warn("Mismatch in number of bytes sent");
	}
    else { process->new_message_sent(RESOURCE_ID); }
}
void device_callback(std::vector<icarus_rover_v2::device> devicelist)
{
	for(std::size_t i = 0; i < devicelist.size(); i++)
	{
		std::string send_string = udpmessagehandler->encode_DeviceUDP(devicelist.at(i).DeviceParent,
				devicelist.at(i).DeviceName,
				devicelist.at(i).DeviceType,
				devicelist.at(i).Architecture);
		if(sendto(senddevice_sock, send_string.c_str(), send_string.size(), 0, (struct sockaddr *)&senddevice_addr, sizeof(senddevice_addr))!=send_string.size())
		{
			logger->log_warn("Mismatch in number of bytes sent");
		}
		else { process->new_message_sent(DEVICE_ID); }
	}
}

void process_udp_receive()
{
	while(kill_node == 0)
	{
		char buf[RECV_BUFFERSIZE] = {0};
		socklen_t addrlen = sizeof(remote_addr);
		int recvlen = recvfrom(recvdevice_sock,buf,RECV_BUFFERSIZE,0,(struct sockaddr *)&remote_addr,&addrlen);
		if(recvlen < 0) { continue; }
		buf[recvlen] = '\0';
		std::string buffer(buf);
		std::vector<std::string> items;
		boost::split(items,buffer,boost::is_any_of(","));
		int success;
		//std::string id = items.at(0);
		char tempstr[8];
		sprintf(tempstr,"0x%s",items.at(0).c_str());
		int id = (int)strtol(tempstr,NULL,0);
		uint8_t device,armcommand;
		int axis1,axis2,axis3,axis4,axis5,axis6,axis7,axis8,int_1,int_2,int_3;
		uint8_t command,option1,option2,option3;
		uint8_t button1,button2,button3,button4,button5,button6,button7,button8;
		std::string tempstr1,tempstr2;
		double v1, v2, v3;
		uint64_t t,t2;
        process->new_message_recv(id);
		switch (id)
		{

			case UDPMessageHandler::UDP_Command_ID:
				success = udpmessagehandler->decode_CommandUDP(items,&command,&option1,&option2,&option3,&tempstr1,&tempstr2);
				if(success == 1)
				{
					icarus_rover_v2::command newcommand;
					newcommand.Command = command;
					newcommand.Option1 = option1;
					newcommand.Option2 = option2;
					newcommand.Option3 = option3;
					newcommand.CommandText = tempstr1;
					newcommand.Description = tempstr2;
					user_command_pub.publish(newcommand);
				}
				else
				{
					printf("Couldn't decode message.\n");
				}
				break;
			case UDPMessageHandler::UDP_Heartbeat_ID:
				success = udpmessagehandler->decode_HeartbeatUDP(items,&tempstr1,&t,&t2);
				if(success == 1)
				{
					//printf("Dev: %s current t: %llu expected t: %llu\n",tempstr1.c_str(),t,t2);
					bool add_new_entry = true;
					for(int i = 0; i < remote_devices.size(); i++)
					{
						if(tempstr1 == remote_devices.at(i).Name)
						{
							add_new_entry = false;
							remote_devices.at(i).current_beatepoch_sec = t/1000.0;
							remote_devices.at(i).expected_beatepoch_sec = t2/1000.0;
							break;
						}
					}
					if(add_new_entry == true)
					{
						RemoteDevice dev;
						dev.Name = tempstr1;
						dev.current_beatepoch_sec = t/1000.0;
						dev.expected_beatepoch_sec = t2/1000.0;
						double now_sec = ros::Time::now().sec + ros::Time::now().nsec/1000000000.0;
						dev.offset_sec = now_sec-dev.current_beatepoch_sec;
						remote_devices.push_back(dev);
					}
				}
				else
				{
					printf("Couldn't decode message.\n");
				}
				break;
			case UDPMessageHandler::UDP_RemoteControl_ID:

				success = udpmessagehandler->decode_RemoteControlUDP(items,&t,&axis1,&axis2,&axis3,&axis4,&axis5,&axis6,&axis7,&axis8,
																	&button1,&button2,&button3,&button4,&button5,&button6,&button7,&button8);
				if(success == 1)
				{
					sensor_msgs::Joy newjoy;
                    double send_time = t/1000.0;
					newjoy.header.stamp.sec = floor(send_time);
                    newjoy.header.stamp.nsec = (send_time-newjoy.header.stamp.sec)*1000000000;
					newjoy.header.frame_id = "/world";
					newjoy.axes.push_back((float)(axis1/32768.0));
					newjoy.axes.push_back((float)(axis2/32768.0));
					newjoy.axes.push_back((float)(axis3/32768.0));
					newjoy.axes.push_back((float)(axis4/32768.0));
					newjoy.axes.push_back((float)(axis5/32768.0));
					newjoy.axes.push_back((float)(axis6/32768.0));
					newjoy.axes.push_back((float)(axis7/32768.0));
					newjoy.axes.push_back((float)(axis8/32768.0));
					newjoy.buttons.push_back(button1);
					newjoy.buttons.push_back(button2);
					newjoy.buttons.push_back(button3);
					newjoy.buttons.push_back(button4);
					newjoy.buttons.push_back(button5);
					newjoy.buttons.push_back(button6);
					newjoy.buttons.push_back(button7);
					newjoy.buttons.push_back(button8);
					joy_pub.publish(newjoy);
				}
				else
				{
					printf("Couldn't decode message.\n");
				}
				break;
			case UDPMessageHandler::UDP_ArmControl_ID:
				success = udpmessagehandler->decode_ArmControlUDP(items,&device,&axis1,&axis2,&axis3,&axis4,&axis5,&axis6,
						&button1,&button2,&button3,&button4,&button5,&button6);
				if(success == 1)
				{
					sensor_msgs::Joy newjoy;
					newjoy.header.stamp = ros::Time::now();
					newjoy.header.frame_id = "/world";
					newjoy.axes.push_back((float)(axis1/-32768.0));
					newjoy.axes.push_back((float)(axis2/-32768.0));
					newjoy.axes.push_back((float)(axis3/-32768.0));
					newjoy.axes.push_back((float)(axis4/-32768.0));
					newjoy.axes.push_back((float)(axis5/-32768.0));
					newjoy.axes.push_back((float)(axis6/-32768.0));
					newjoy.buttons.push_back(button1);
					newjoy.buttons.push_back(button2);
					newjoy.buttons.push_back(button3);
					newjoy.buttons.push_back(button4);
					newjoy.buttons.push_back(button5);
					newjoy.buttons.push_back(button6);
					arm1_joy_pub.publish(newjoy);
				}
				else
				{
					printf("Couldn't decode message.\n");
				}
				break;
			case UDPMessageHandler::UDP_TuneControlGroup_ID:
				success = udpmessagehandler->decode_TuneControlGroupUDP(items,&tempstr1,&tempstr2,&v1,&v2,&v3,&int_1,&int_2,&int_3);
				if(success == 1)
				{
					icarus_rover_v2::controlgroup cg;
					cg.name = tempstr1;
					cg.type = tempstr2;
					cg.value1 = v1;
					cg.value2 = v2;
					cg.value3 = v3;
					cg.maxvalue = int_1;
					cg.minvalue = int_2;
					cg.defaultvalue = int_3;
					controlgroup_pub.publish(cg);
					
				}
				else
				{
					printf("Couldn't decode message.\n");
				}
				break;
			default:
				char tempstr[512];

				sprintf(tempstr,"Message: %d Not Supported.",id);
				printf("%s\n",tempstr);
				logger->log_warn(std::string(tempstr));
				break;
		}

	}

}
bool initialize_recvsocket()
{
	if((recvdevice_sock = socket(AF_INET,SOCK_DGRAM,0)) < 0)
	{
		logger->log_error("Failed to create recv socket. Exiting.");
		return false;
	}
	struct timeval timeout;
	timeout.tv_sec = 1;
	timeout.tv_usec = 0;
	setsockopt(recvdevice_sock,SOL_SOCKET,SO_RCVTIMEO,(char *)&timeout,sizeof(struct timeval));
	memset((char*)&my_addr,0,sizeof(my_addr));
	my_addr.sin_family = AF_INET;
	my_addr.sin_addr.s_addr = htonl(INADDR_ANY);
	my_addr.sin_port = htons(recv_unicast_port);
	if(bind(recvdevice_sock,(struct sockaddr *)&my_addr,sizeof(my_addr)) < 0)
	{
		logger->log_error("Failed to bind recv socket. Exiting.");
		return false;
	}

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
/*! \brief 0.1 PULSE PER SECOND User Code
 */
void PPS01_Callback(const std_msgs::Bool::ConstPtr& msg)
{
	icarus_rover_v2::firmware fw;
	fw.Generic_Node_Name = "network_transceiver_Node";
	fw.Node_Name = node_name;
	fw.Description = "Latest Rev: 1-January-2018";
	fw.Major_Release = NETWORKTRANSCEIVERNODE_MAJOR_RELEASE;
	fw.Minor_Release = NETWORKTRANSCEIVERNODE_MINOR_RELEASE;
	fw.Build_Number = NETWORKTRANSCEIVERNODE_BUILD_NUMBER;
	firmware_pub.publish(fw);
	printf("t=%4.2f (sec) [%s]: %s\n",ros::Time::now().toSec(),node_name.c_str(),process->get_diagnostic().Description.c_str());
}
/*! \brief 1.0 PULSE PER SECOND User Code
 */
void PPS1_Callback(const std_msgs::Bool::ConstPtr& msg)
{
	received_pps = true;
    if((process->get_initialized() == true) and (process->get_ready() == true))
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
			logger->log_diagnostic(resource_diagnostic);
		}
		else if(resource_diagnostic.Level <= NOTICE)
		{
			resources_used = resourcemonitor->get_resourceused();
			resource_pub.publish(resources_used);
		}
	}
    else if(process->get_ready() == false)
    {
        
    }
	else if(process->get_initialized() == false)
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
    diagnostic_pub.publish(process->get_diagnostic());
}
void Command_Callback(const icarus_rover_v2::command::ConstPtr& msg)
{
	icarus_rover_v2::command command;
	command.Command = msg->Command;
	command.Option1 = msg->Option1;
	command.Option2 = msg->Option2;
	command.Option3 = msg->Option3;
	command.CommandText = msg->CommandText;
	command.Description = msg->Description;
	std::vector<icarus_rover_v2::diagnostic> diaglist = process->new_commandmsg(command);
	for(std::size_t i = 0; i < diaglist.size(); i++)
	{
		logger->log_diagnostic(diaglist.at(i));
		diagnostic_pub.publish(diaglist.at(i));
	}
}
//End User Code: Functions
bool run_10Hz_code()
{
    beat.stamp = ros::Time::now();
	heartbeat_pub.publish(beat);
    if(process->get_diagnostic().Level > NOTICE)
    {
        diagnostic_pub.publish(process->get_diagnostic());
        logger->log_diagnostic(process->get_diagnostic());
    }
    return true;
}
int main(int argc, char **argv)
{
	node_name = "network_transceiver_node";
    ros::init(argc, argv, node_name);
    n.reset(new ros::NodeHandle);
    node_name = ros::this_node::getName();
    ros::NodeHandle n;
    
    if(initializenode() == false)
    {
        char tempstr[256];
        sprintf(tempstr,"Unable to Initialize. Exiting.");
        printf("[%s]: %s\n",node_name.c_str(),tempstr);
        logger->log_fatal(tempstr);
		kill_node = 1;
    }
    ros::Rate loop_rate(ros_rate);
	boot_time = ros::Time::now();
    last_10Hz_timer = ros::Time::now();
    double mtime;
	boost::thread process_udpreceive_thread(&process_udp_receive);
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
    process_udpreceive_thread.join();
    close(recvdevice_sock);
    close(senddevice_sock);
    logger->log_notice("Node Finished Safely.");
    return 0;
}

bool initializenode()
{
    //Start Template Code: Initialization, Parameters and Topics
	kill_node = 0;
	signal(SIGINT,signalinterrupt_handler);
    hostname[1023] = '\0';
    gethostname(hostname,1023);
    std::string diagnostic_topic = "/" + node_name + "/diagnostic";
	diagnostic_pub =  n->advertise<icarus_rover_v2::diagnostic>(diagnostic_topic,5);
    icarus_rover_v2::diagnostic diagnostic;
    diagnostic.DeviceName = hostname;
	diagnostic.Node_Name = node_name;
	diagnostic.System = ROVER;
	diagnostic.SubSystem = ROBOT_CONTROLLER;
	diagnostic.Component = COMMUNICATION_NODE;

	diagnostic.Diagnostic_Type = NOERROR;
	diagnostic.Level = INFO;
	diagnostic.Diagnostic_Message = INITIALIZING;
	diagnostic.Description = "Node Initializing";
	diagnostic_pub.publish(diagnostic);

	std::string resource_topic = "/" + node_name + "/resource";
	resource_pub = n->advertise<icarus_rover_v2::resource>(resource_topic,5);

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
	std::string param_disabled = node_name +"/disable";
    bool disable_node;
    if(n->getParam(param_disabled,disable_node) == true)
    {
    	if(disable_node == true)
    	{
    		logger->log_notice("Node Disabled in Launch File.  Exiting.");
    		printf("[%s]: Node Disabled in Launch File. Exiting.\n",node_name.c_str());
    		return false;
    	}
    }
    
    std::string heartbeat_topic = "/" + node_name + "/heartbeat";
    heartbeat_pub = n->advertise<icarus_rover_v2::heartbeat>(heartbeat_topic,5);
    beat.Node_Name = node_name;
    std::string device_topic = "/" + std::string(hostname) + "_master_node/srv_device";
    srv_device = n->serviceClient<icarus_rover_v2::srv_device>(device_topic);

    pps01_sub = n->subscribe<std_msgs::Bool>("/01PPS",5,PPS01_Callback); 
    pps1_sub = n->subscribe<std_msgs::Bool>("/1PPS",5,PPS1_Callback); 
    command_sub = n->subscribe<icarus_rover_v2::command>("/command",5,Command_Callback);
    std::string param_require_pps_to_start = node_name +"/require_pps_to_start";
    if(n->getParam(param_require_pps_to_start,require_pps_to_start) == false)
	{
		logger->log_warn("Missing Parameter: require_pps_to_start.");
		return false;
	}
    std::string firmware_topic = "/" + node_name + "/firmware";
    firmware_pub =  n->advertise<icarus_rover_v2::firmware>(firmware_topic,1);
    
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
        logger->log_warn("Missing parameter: loop2_rate.  Not running loop2 code.");
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
	process = new NetworkTransceiverNodeProcess;
	diagnostic = process->init(diagnostic,std::string(hostname));
	if(diagnostic.Level > NOTICE)
	{
		logger->log_fatal(diagnostic.Description);
		printf("[%s]: %s\n",node_name.c_str(),diagnostic.Description.c_str());
		return false;
	}

    ros::Time::init();
    ready_to_arm = false;
    remote_heartbeat_pass = false;
    std::string armed_disarmed_state_topic = "/armed_state";
    armed_disarmed_state_sub = n->subscribe<std_msgs::UInt8>(armed_disarmed_state_topic,1000,ArmedState_Callback);
    std::string param_send_multicast_address = node_name +"/Send_Multicast_Group";
    if(n->getParam(param_send_multicast_address,send_multicast_group) == false)
   	{
   		logger->log_warn("Missing Parameter: Send_Multicast_Group. Exiting.");
   		return false;
   	}

    std::string param_send_multicast_port = node_name +"/Send_Multicast_Port";
   	if(n->getParam(param_send_multicast_port,send_multicast_port) == false)
   	{
   		logger->log_warn("Missing Parameter: Send_Multicast_Port. Exiting.");
   		return false;
   	}
   	std::string param_recv_unicast_port = node_name +"/Recv_Unicast_Port";
	if(n->getParam(param_recv_unicast_port,recv_unicast_port) == false)
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
	if(n->getParam(param_Mode,Mode) == false)
	{
		logger->log_warn("Missing Parameter: Mode.");
		return false;
	}
	if(Mode=="Diagnostics_GUI")
	{
		std::string joystick_topic = "/" + Mode + "/joystick";
		joy_pub =  n->advertise<sensor_msgs::Joy>(joystick_topic,1000);

		std::string arm1_joystick_topic = "/" + Mode + "/arm1_joystick";
		arm1_joy_pub =  n->advertise<sensor_msgs::Joy>(arm1_joystick_topic,1000);

		std::string arm2_joystick_topic = "/" + Mode + "/arm2_joystick";
		arm2_joy_pub =  n->advertise<sensor_msgs::Joy>(arm2_joystick_topic,1000);

		//std::string arm_command_topic = "/" + Mode + "/user_command";
		//arm_command_pub = n->advertise<icarus_rover_v2::command>(arm_command_topic,1000);


	}
	else if(Mode=="DriverStation")
	{
		std::string joystick_topic = "/" + Mode + "/joystick";
		joy_pub =  n->advertise<sensor_msgs::Joy>(joystick_topic,1000);

		std::string arm1_joystick_topic = "/" + Mode + "/arm1_joystick";
		arm1_joy_pub =  n->advertise<sensor_msgs::Joy>(arm1_joystick_topic,1000);

		std::string arm2_joystick_topic = "/" + Mode + "/arm2_joystick";
		arm2_joy_pub =  n->advertise<sensor_msgs::Joy>(arm2_joystick_topic,1000);

		std::string user_command_topic = "/" + Mode + "/user_command";
		user_command_pub = n->advertise<icarus_rover_v2::command>(user_command_topic,1000);

		std::string controlgroup_topic = "/" + Mode + "/controlgroup";
		controlgroup_pub = n->advertise<icarus_rover_v2::controlgroup>(controlgroup_topic,1000);


	}
	std::string ready_to_arm_topic = node_name + "/ready_to_arm";
	ready_to_arm_pub = n->advertise<std_msgs::Bool>(ready_to_arm_topic,1000);

	estop_sub = n->subscribe<icarus_rover_v2::estop>("/estop",5,estop_Callback);

	udpmessagehandler = new UDPMessageHandler();

	process->set_initialized(true);
    //Finish User Code: Initialization and Parameters

    //Start Template Code: Final Initialization.
	diagnostic.Diagnostic_Type = NOERROR;
	diagnostic.Level = INFO;
	diagnostic.Diagnostic_Message = NOERROR;
	diagnostic.Description = "Node Initialized";
	process->set_diagnostic(diagnostic);
	diagnostic_pub.publish(diagnostic);
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
			resourcemonitor = new ResourceMonitor(process->get_diagnostic(),device.Architecture,device.DeviceName,node_name);
			process->set_mydevice(device);
		}
	}

	if((process->get_initialized() == true))
	{
		icarus_rover_v2::diagnostic diag = process->new_devicemsg(device);
	}
	return true;
}
void signalinterrupt_handler(int sig)
{
	kill_node = 1;
}
//End Template Code: Functions
