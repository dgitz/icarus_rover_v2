#include "networktransceiver_node.h"
bool kill_node = false;
bool NetworkTransceiverNode::start(int argc,char **argv)
{
	bool status = false;
	process = new NetworkTransceiverNodeProcess();
	set_basenodename(BASE_NODE_NAME);
	initialize_firmware(MAJOR_RELEASE_VERSION,MINOR_RELEASE_VERSION,BUILD_NUMBER,FIRMWARE_DESCRIPTION);
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
	process->initialize(get_basenodename(),get_nodename(),get_hostname(),DIAGNOSTIC_SYSTEM,DIAGNOSTIC_SUBSYSTEM,DIAGNOSTIC_COMPONENT);
	std::vector<uint8_t> diagnostic_types;
	diagnostic_types.push_back(SOFTWARE);
	diagnostic_types.push_back(DATA_STORAGE);
	diagnostic_types.push_back(SYSTEM_RESOURCE);
	diagnostic_types.push_back(COMMUNICATIONS);
	diagnostic_types.push_back(REMOTE_CONTROL);
	process->enable_diagnostics(diagnostic_types);
	process->finish_initialization();
	diagnostic = finish_initialization();
	if(diagnostic.Level > WARN)
	{
		return false;
	}
	if(diagnostic.Level < WARN)
	{
		diagnostic = process->update_diagnostic(DATA_STORAGE,INFO,NOERROR,"Node Configured. Initializing.");
		get_logger()->log_diagnostic(diagnostic);
	}
	status = true;
	return status;
}

eros::diagnostic NetworkTransceiverNode::read_launchparameters()
{
	eros::diagnostic diag = diagnostic;
	/*
	std::string send_multicast_group;
	int send_multicast_port,recv_unicast_port;
	std::string param_send_multicast_group = node_name +"/Send_Multicast_Group";
	if(n->getParam(param_send_multicast_group,send_multicast_group) == false)
	{
		diag = process->update_diagnostic(DATA_STORAGE,ERROR,INITIALIZING_ERROR,"Missing Parameter: Send_Multicast_Group. Exiting.");
		logger->log_diagnostic(diag);
		return diag;
	}

	std::string param_send_multicast_port = node_name +"/Send_Multicast_Port";
	if(n->getParam(param_send_multicast_port,send_multicast_port) == false)
	{
		diag = process->update_diagnostic(DATA_STORAGE,ERROR,INITIALIZING_ERROR,"Missing Parameter: Send_Multicast_Port. Exiting.");
		logger->log_diagnostic(diag);
		return diag;
	}
	std::string param_recv_unicast_port = node_name +"/Recv_Unicast_Port";
	if(n->getParam(param_recv_unicast_port,recv_unicast_port) == false)
	{
		diag = process->update_diagnostic(DATA_STORAGE,ERROR,INITIALIZING_ERROR,"Missing Parameter: Recv_Unicast_Port. Exiting.");
		logger->log_diagnostic(diag);
		return diag;
	}
	process->set_networkconfiguration(send_multicast_group,send_multicast_port,recv_unicast_port);
	*/
	std::string UIMode;
	std::string param_Mode = node_name +"/Mode";
	if(n->getParam(param_Mode,UIMode) == false)
	{
		diag = process->update_diagnostic(DATA_STORAGE,ERROR,INITIALIZING_ERROR,"Missing Parameter: Mode. Exiting.");
		logger->log_diagnostic(diag);
		return diag;
	}
	process->set_UIMode(UIMode);
	get_logger()->log_notice(__FILE__,__LINE__,"Configuration Files Loaded.");
	return diag;
}
eros::diagnostic NetworkTransceiverNode::finish_initialization()
{
	eros::diagnostic diag = diagnostic;
	pps1_sub = n->subscribe<std_msgs::Bool>("/1PPS",1,&NetworkTransceiverNode::PPS1_Callback,this);
	command_sub = n->subscribe<eros::command>("/command",10,&NetworkTransceiverNode::Command_Callback,this);
	systemstate_sub = n->subscribe<eros::system_state>("/System/State",10,&NetworkTransceiverNode::systemstate_Callback,this);
	std::string armed_disarmed_state_topic = "/armed_state";
	armed_disarmed_state_sub = n->subscribe<std_msgs::UInt8>(armed_disarmed_state_topic,10,&NetworkTransceiverNode::ArmedState_Callback,this);
	std::string device_topic = "/" + std::string(host_name) + "_master_node/srv_device";
	srv_device = n->serviceClient<eros::srv_device>(device_topic);
	std::string subsystem_diagnostic_topic = "/System/Diagnostic/State";
	subsystem_diagnostic_sub = n->subscribe<eros::subsystem_diagnostic>(subsystem_diagnostic_topic,1,&NetworkTransceiverNode::subsystem_diagnostic_Callback,this);
	std::string systemsnapshot_state_topic = "/System/Snapshot/State";
	systemsnapshot_state_sub = n->subscribe<eros::systemsnapshot_state>(systemsnapshot_state_topic,1,&NetworkTransceiverNode::systemSnapshotState_Callback,this);
	

	if(process->get_UIMode()=="Diagnostics_GUI")
	{
		std::string joystick_topic = "/" + process->get_UIMode() + "/joystick";
		joy_pub =  n->advertise<sensor_msgs::Joy>(joystick_topic,1000);

		std::string arm1_joystick_topic = "/" + process->get_UIMode() + "/arm1_joystick";
		arm1_joy_pub =  n->advertise<sensor_msgs::Joy>(arm1_joystick_topic,1000);

		std::string arm2_joystick_topic = "/" + process->get_UIMode() + "/arm2_joystick";
		arm2_joy_pub =  n->advertise<sensor_msgs::Joy>(arm2_joystick_topic,1000);

	}
	else if(process->get_UIMode()=="DriverStation")
	{
		std::string joystick_topic = "/" + process->get_UIMode() + "/joystick";
		joy_pub =  n->advertise<sensor_msgs::Joy>(joystick_topic,1);

		std::string arm1_joystick_topic = "/" + process->get_UIMode() + "/arm1_joystick";
		arm1_joy_pub =  n->advertise<sensor_msgs::Joy>(arm1_joystick_topic,1);

		std::string arm2_joystick_topic = "/" + process->get_UIMode() + "/arm2_joystick";
		arm2_joy_pub =  n->advertise<sensor_msgs::Joy>(arm2_joystick_topic,1);

		std::string user_command_topic = "/" + process->get_UIMode() + "/user_command";
		user_command_pub = n->advertise<eros::command>(user_command_topic,1);

		std::string tune_controlgroup_topic = "/" + process->get_UIMode() + "/tune_controlgroup";
		tune_controlgroup_pub = n->advertise<eros::tune_controlgroup>(tune_controlgroup_topic,1);

		std::string view_controlgroup_topic = "/" + process->get_UIMode() + "/view_controlgroup";
		view_controlgroup_sub = n->subscribe<eros::view_controlgroup>(view_controlgroup_topic,1,&NetworkTransceiverNode::viewControlGroup_Callback,this);
	
	}
	diagnostic = process->load("/home/robot/config/MiscConfig.xml");
	logger->log_diagnostic(diagnostic);
	if(diagnostic.Level > NOTICE)
	{
		logger->log_diagnostic(diagnostic);
		return diagnostic;
	}
	udpmessagehandler = new UDPMessageHandler();
	if(initialize_sendsocket() == false)
	{
		diag = process->update_diagnostic(COMMUNICATIONS,ERROR,INITIALIZING_ERROR,"Couldn't initialize send socket.  Exiting.");
		logger->log_diagnostic(diag);
		return diag;
	}
	if(initialize_recvsocket() == false)
	{
		diag = process->update_diagnostic(COMMUNICATIONS,ERROR,INITIALIZING_ERROR,"Couldn't initialize recv socket.  Exiting.");
		logger->log_diagnostic(diag);
		return diag;
	}
	return diagnostic;
}
bool NetworkTransceiverNode::run_001hz()
{
	eros::diagnostic diag = rescan_topics();
	diag = process->update_diagnostic(diag);
	if(diag.Level > NOTICE)
	{
		logger->log_diagnostic(diag);
		diagnostic_pub.publish(diag);
	}
	return true;
}
bool NetworkTransceiverNode::run_01hz()
{

	logger->log_info(__FILE__,__LINE__,process->get_messageinfo(false));
	return true;
}
bool NetworkTransceiverNode::run_01hz_noisy()
{
	std::vector<eros::diagnostic> diaglist = process->get_diagnostics();
	for (std::size_t i = 0; i < diaglist.size(); ++i)
	{
		get_logger()->log_diagnostic(diaglist.at(i));
		diagnostic_pub.publish(diaglist.at(i));
	}
	return true;
}
bool NetworkTransceiverNode::run_1hz()
{
	process->update_diagnostic(process->check_remoteHeartbeats());
	process->update_diagnostic(get_resource_diagnostic());
	if(process->get_taskstate() == TASKSTATE_RUNNING)
	{
	}
	else if(process->get_taskstate() == TASKSTATE_INITIALIZED)
	{
	}
	else if(process->get_taskstate() == TASKSTATE_INITIALIZING)
	{
		{
			{
				eros::srv_device srv;
				srv.request.query = "SELF";
				if(srv_device.call(srv) == true)
				{
					if(srv.response.data.size() != 1)
					{

						get_logger()->log_error(__FILE__,__LINE__,"Got unexpected device message.");
					}
					else
					{
						new_devicemsg(srv.request.query,srv.response.data.at(0));
					}
				}
				else
				{
				}
				srv.request.query = "ALL";
				if(srv_device.call(srv) == true)
				{
					for(std::size_t i = 0; i < srv.response.data.size(); ++i)
					{
						new_devicemsg(srv.request.query,srv.response.data.at(i));
					}
				}
				else
				{
				}
			}
			{

			}

		}
	}
	else
	{
	}
	std::vector<eros::diagnostic> diaglist = process->get_diagnostics();
	for (std::size_t i = 0; i < diaglist.size(); ++i)
	{
		if (diaglist.at(i).Level == WARN)
		{
			get_logger()->log_diagnostic(diaglist.at(i));
			diagnostic_pub.publish(diaglist.at(i));
		}
	}
	return true;
}
bool NetworkTransceiverNode::run_10hz()
{
	ready_to_arm = process->get_ready_to_arm();
	if((process->get_remoteheartbeatresult() == true) and
			(1 == 1)) //Others??
	{
		ready_to_arm = true;
	}
	else
	{
		logger->log_warn(__FILE__,__LINE__,"Remote Heartbeat Failed.");
		ready_to_arm = false;
	}
	eros::diagnostic diag = process->update(0.1,ros::Time::now().toSec());
	diag = process->update_diagnostic(diag);
	if(diag.Level > WARN)
	{
		get_logger()->log_diagnostic(diag);
		diagnostic_pub.publish(diag);
	}
	std::vector<eros::diagnostic> diaglist = process->get_diagnostics();
	for (std::size_t i = 0; i < diaglist.size(); ++i)
	{
		if ((diaglist.at(i).Level > WARN))
		{
			get_logger()->log_diagnostic(diaglist.at(i));
			diagnostic_pub.publish(diaglist.at(i));
		}
	}
	return true;
}
bool NetworkTransceiverNode::run_loop1()
{
	{
		std::vector<NetworkTransceiverNodeProcess::QueueElement> buffer = process->get_sendqueue(NetworkTransceiverNodeProcess::PriorityLevel::HIGH);
		for(std::size_t i = 0; i < buffer.size(); i++)
		{
			if(sendto(senddevice_sock, buffer.at(i).item.c_str(), buffer.at(i).item.size(), 0, (struct sockaddr *)&senddevice_addr, 
				sizeof(senddevice_addr))!=(uint16_t)buffer.at(i).item.size())
			{
				logger->log_warn(__FILE__,__LINE__,"Mismatch in number of bytes sent");
			}
			else
			{
				process->new_message_sent(buffer.at(i).id);
			}
		}
	}

	{
		std::vector<NetworkTransceiverNodeProcess::QueueElement> buffer = process->get_sendqueue(NetworkTransceiverNodeProcess::PriorityLevel::MEDIUM);
		for(std::size_t i = 0; i < buffer.size(); i++)
		{
			if(sendto(senddevice_sock, buffer.at(i).item.c_str(), buffer.at(i).item.size(), 0, 
				(struct sockaddr *)&senddevice_addr, sizeof(senddevice_addr))!=(uint16_t)buffer.at(i).item.size())
			{
				logger->log_warn(__FILE__,__LINE__,"Mismatch in number of bytes sent");
			}
			else
			{
				process->new_message_sent(buffer.at(i).id);
			}
		}
	}
	{
		std::vector<NetworkTransceiverNodeProcess::QueueElement> buffer = process->get_sendqueue(NetworkTransceiverNodeProcess::PriorityLevel::LOW);
		for(std::size_t i = 0; i < buffer.size(); i++)
		{
			if(sendto(senddevice_sock, buffer.at(i).item.c_str(), buffer.at(i).item.size(), 0, 
				(struct sockaddr *)&senddevice_addr, sizeof(senddevice_addr))!=(uint16_t)buffer.at(i).item.size())
			{
				logger->log_warn(__FILE__,__LINE__,"Mismatch in number of bytes sent");
			}
			else
			{
				process->new_message_sent(buffer.at(i).id);
			}
		}
	}
	return true;
}
bool NetworkTransceiverNode::run_loop2()
{
	return true;
}
bool NetworkTransceiverNode::run_loop3()
{
	return true;
}
void NetworkTransceiverNode::systemstate_Callback(const eros::system_state::ConstPtr& msg)
{
	std::string send_string = udpmessagehandler->encode_SystemStateUDP(
		msg->State,
		msg->Option1,
		msg->Option2,
		msg->Option3,
		msg->StateText,
		msg->Description);
	process->push_sendqueue(SYSTEMSTATE_ID,send_string);
}
void NetworkTransceiverNode::PPS1_Callback(const std_msgs::Bool::ConstPtr& msg)
{
	new_ppsmsg(msg);
}

void NetworkTransceiverNode::Command_Callback(const eros::command::ConstPtr& t_msg)
{
	std::vector<eros::diagnostic> diaglist = process->new_commandmsg(t_msg);
	new_commandmsg_result(t_msg,diaglist);
}
bool NetworkTransceiverNode::new_devicemsg(std::string query,eros::device t_device)
{
	if(query == "SELF")
	{
		if(t_device.DeviceName == std::string(host_name))
		{
			set_mydevice(t_device);
			process->set_mydevice(t_device);
		}
	}

	if ((process->get_taskstate() == TASKSTATE_INITIALIZED))
	{
		eros::device::ConstPtr device_ptr(new eros::device(t_device));
		eros::diagnostic diag = process->new_devicemsg(device_ptr);
	}
	return true;
}
eros::diagnostic NetworkTransceiverNode::rescan_topics()
{
	eros::diagnostic diag = diagnostic;
	int found_new_topics = 0;

	ros::master::V_TopicInfo master_topics;
	ros::master::getTopics(master_topics);
	for (ros::master::V_TopicInfo::iterator it = master_topics.begin() ; it != master_topics.end(); it++)
	{
		const ros::master::TopicInfo& info = *it;
		if(info.datatype == "eros/resource")
		{
			int v = process->push_topiclist(info.datatype,info.name);
			if(v == 1)
			{
				found_new_topics++;
				char tempstr[255];
				sprintf(tempstr,"Subscribing to resource topic: %s",info.name.c_str());
				logger->log_info(__FILE__,__LINE__,tempstr);
				ros::Subscriber sub = n->subscribe<eros::resource>(info.name,20,&NetworkTransceiverNode::resource_Callback,this);
				resource_subs.push_back(sub);
			}
		}
		else if(info.datatype == "eros/diagnostic")
		{
			int v = process->push_topiclist(info.datatype,info.name);
			if(v == 1)
			{
				found_new_topics++;
				char tempstr[255];
				sprintf(tempstr,"Subscribing to diagnostic topic: %s",info.name.c_str());
				logger->log_info(__FILE__,__LINE__,tempstr);
				ros::Subscriber sub = n->subscribe<eros::diagnostic>(info.name,20,&NetworkTransceiverNode::diagnostic_Callback,this);
				diagnostic_subs.push_back(sub);
			}
		}
		else if(info.datatype == "eros/firmware")
		{
			int v = process->push_topiclist(info.datatype,info.name);
			if(v == 1)
			{
				found_new_topics++;
				char tempstr[255];
				sprintf(tempstr,"Subscribing to firmware topic: %s",info.name.c_str());
				logger->log_info(__FILE__,__LINE__,tempstr);
				ros::Subscriber sub = n->subscribe<eros::firmware>(info.name,1,&NetworkTransceiverNode::firmware_Callback,this);
				firmware_subs.push_back(sub);
			}
		}

	}

	char tempstr[255];
	if(found_new_topics > 0)
	{
		sprintf(tempstr,"Rescanned and found %d new topics.",found_new_topics);
	}
	else
	{
		sprintf(tempstr,"Rescanned and found no new topics.");
	}
	diag = process->update_diagnostic(SOFTWARE,INFO,NOERROR,std::string(tempstr));
	logger->log_info(__FILE__,__LINE__,tempstr);
	return diag;
}
void NetworkTransceiverNode::viewControlGroup_Callback(const eros::view_controlgroup::ConstPtr& msg)
{
	std::string send_string = udpmessagehandler->encode_ControlGroupValueUDP(
		msg->tov,
		msg->Name,
		msg->command_value,
		msg->sensor_value,
		msg->error_value,
		msg->errorperc_value,
		msg->output_value,
		msg->integral_error,
		msg->derivative_error,
		msg->P_output,
		msg->I_output,
		msg->D_output);
	process->push_sendqueue(CONTROLGROUPVALUE_ID,send_string);
}
void NetworkTransceiverNode::ArmedState_Callback(const std_msgs::UInt8::ConstPtr& msg)
{
	std::string send_string = udpmessagehandler->encode_Arm_StatusUDP(msg->data);
	process->push_sendqueue(ARM_STATUS_ID,send_string);
}
void NetworkTransceiverNode::diagnostic_Callback(const eros::diagnostic::ConstPtr& msg)
{
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
	process->push_sendqueue(DIAGNOSTIC_ID,send_string);
}
void NetworkTransceiverNode::firmware_Callback(const eros::firmware::ConstPtr& msg)
{
	std::string send_string = udpmessagehandler->encode_FirmwareUDP(
			msg->Node_Name,
			msg->Description,
			(uint8_t)msg->Major_Release,
			(uint8_t)msg->Minor_Release,
			(uint8_t)msg->Build_Number);

	process->push_sendqueue(FIRMWARE_ID,send_string);
}
void NetworkTransceiverNode::subsystem_diagnostic_Callback(const eros::subsystem_diagnostic::ConstPtr& msg)
{
	std::string send_string = udpmessagehandler->encode_SubsystemDiagnosticUDP(
		msg->Level[0],
		msg->Level[1],
		msg->Level[2],
		msg->Level[3],
		msg->Level[4],
		msg->Level[5],
		msg->Level[6],
		msg->Level[7],
		msg->Level[8],
		msg->Level[9],
		msg->Level[10]);
		process->push_sendqueue(SUBSYSTEMDIAGNOSTIC_ID,send_string);
}
void NetworkTransceiverNode::resource_Callback(const eros::resource::ConstPtr& msg)
{
	std::string send_string = udpmessagehandler->encode_ResourceUDP(msg->Node_Name,
			msg->RAM_MB,
			msg->CPU_Perc);
	process->push_sendqueue(RESOURCE_ID,send_string);
}
void NetworkTransceiverNode::systemSnapshotState_Callback(const eros::systemsnapshot_state::ConstPtr& msg)
{
	std::string send_string = udpmessagehandler->encode_SystemSnapshotStateUDP(msg->state,
		msg->percent_complete,
		msg->systemsnapshot_count,
		msg->source_device,
		msg->snapshot_path);
	process->push_sendqueue(SYSTEMSNAPSHOTSTATE_ID,send_string);
}
bool NetworkTransceiverNode::initialize_recvsocket()
{
	if((recvdevice_sock = socket(AF_INET,SOCK_DGRAM,0)) < 0)
	{
		logger->log_error(__FILE__,__LINE__,"Failed to create recv socket. Exiting.");
		return false;
	}
	struct timeval timeout;
	timeout.tv_sec = 1;
	timeout.tv_usec = 0;
	setsockopt(recvdevice_sock,SOL_SOCKET,SO_RCVTIMEO,(char *)&timeout,sizeof(struct timeval));
	memset((char*)&my_addr,0,sizeof(my_addr));
	my_addr.sin_family = AF_INET;
	my_addr.sin_addr.s_addr = htonl(INADDR_ANY);
	my_addr.sin_port = htons(process->get_recv_unicast_port());
	if(bind(recvdevice_sock,(struct sockaddr *)&my_addr,sizeof(my_addr)) < 0)
	{
		logger->log_error(__FILE__,__LINE__,"Failed to bind recv socket. Exiting.");
		return false;
	}
	else
	{
		logger->log_info(__FILE__,__LINE__,"Bound to recv socket.");
	}
	return true;
}
bool NetworkTransceiverNode::initialize_sendsocket()
{
	memset(&senddevice_addr, 0, sizeof(senddevice_addr));
	senddevice_addr.sin_family=AF_INET;
	//Create the socket
	if((senddevice_sock=socket(AF_INET, SOCK_DGRAM, 0))<0)
	{
		logger->log_error(__FILE__,__LINE__,"Failed to create send socket. Exiting.");
		return false;
	}

	if(bind(senddevice_sock,( struct sockaddr *) &senddevice_addr, sizeof(senddevice_addr))<0)
	{
		logger->log_error(__FILE__,__LINE__,"Failed to bind send socket. Exiting.");
		return false;
	}
	inet_pton(AF_INET,process->get_multicast_group().c_str(),&senddevice_addr.sin_addr.s_addr);
	senddevice_addr.sin_port=htons(process->get_send_multicast_port());
	return true;
}
void NetworkTransceiverNode::thread_loop()
{
	while(kill_node == false)
	{
		if(process->get_taskstate() == TASKSTATE_RUNNING)
		{
			char buf[MAX_UDP_RX_BUFFER_SIZE] = {0};
			socklen_t addrlen = sizeof(remote_addr);
			int recvlen = recvfrom(recvdevice_sock,buf,MAX_UDP_RX_BUFFER_SIZE,0,(struct sockaddr *)&remote_addr,&addrlen);
			if(recvlen < 0) { continue; }
			buf[recvlen] = '\0';
			std::string buffer(buf);
			std::vector<std::string> items;
			boost::split(items,buffer,boost::is_any_of(","));
			int success;
			char tempstr[8];
			sprintf(tempstr,"0x%s",items.at(0).c_str());
			int id = (int)strtol(tempstr,NULL,0);
			uint8_t device;
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
					eros::command newcommand;
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
					logger->log_warn(__FILE__,__LINE__,"Couldn't decode Message.");
				}
				break;
			case UDPMessageHandler::UDP_Heartbeat_ID:
				success = udpmessagehandler->decode_HeartbeatUDP(items,&tempstr1,&t,&t2);
				if(success == 1)
				{
					eros::diagnostic diag = process->new_remoteheartbeatmsg(ros::Time::now().toSec(),tempstr1,(double)t/1000.0,(double)t2/1000.0);
					if(diag.Level > NOTICE)
					{
						logger->log_diagnostic(diag);
					}
				}
				else
				{
					logger->log_warn(__FILE__,__LINE__,"Couldn't decode Message.");
				}
				break;
			case UDPMessageHandler::UDP_RemoteControl_ID:
				success = udpmessagehandler->decode_RemoteControlUDP(items,&t,&axis1,&axis2,&axis3,&axis4,&axis5,&axis6,&axis7,&axis8,
						&button1,&button2,&button3,&button4,&button5,&button6,&button7,&button8);
				if(success == 1)
				{
					process->update_diagnostic(REMOTE_CONTROL,INFO,NOERROR,"Remote Control OK.");
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
					logger->log_warn(__FILE__,__LINE__,"Couldn't decode Message.");
				}
				break;
			case UDPMessageHandler::UDP_TuneControlGroup_ID:
					success = udpmessagehandler->decode_TuneControlGroupUDP(items,&tempstr1,&v1,&v2,&v3,&int_1,&int_2,&int_3);
				if(success == 1)
				{
					eros::tune_controlgroup msg;
					msg.Name = tempstr1;
					msg.param1 = v1;
					msg.param2 = v2;
					msg.param3 = v3;
					msg.max_value = int_1;
					msg.min_value = int_2;
					msg.default_value = int_3;
					tune_controlgroup_pub.publish(msg);
				}
				else
				{
					logger->log_warn(__FILE__,__LINE__,"Couldn't decode Message.");
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
					logger->log_warn(__FILE__,__LINE__,"Couldn't decode Message.");
				}
				break;
			default:
				char tempstr[512];

				sprintf(tempstr,"Message: %d Not Supported.",id);
				printf("%s\n",tempstr);
				logger->log_warn(__FILE__,__LINE__,std::string(tempstr));
				break;
			}
		}
		ros::Duration(0.01).sleep();

	}
}
void NetworkTransceiverNode::cleanup()
{
	close(recvdevice_sock);
	close(senddevice_sock);
	base_cleanup();
	get_logger()->log_info(__FILE__,__LINE__,"[NetworkTransceiverNode] Finished Safely.");
}
/*! \brief Attempts to kill a node when an interrupt is received.
 *
 */
void signalinterrupt_handler(int sig)
{
	printf("Killing NetworkTransceiverNode with Signal: %d", sig);
	kill_node = true;
	exit(0);
}
int main(int argc, char **argv) {
	signal(SIGINT, signalinterrupt_handler);
	signal(SIGTERM, signalinterrupt_handler);
	NetworkTransceiverNode *node = new NetworkTransceiverNode();
	bool status = node->start(argc,argv);
	std::thread thread(&NetworkTransceiverNode::thread_loop, node);
	while((status == true) and (kill_node == false))
	{
		status = node->update(node->get_process()->get_taskstate());
	}
	node->cleanup();
	thread.detach();
	return 0;
}

