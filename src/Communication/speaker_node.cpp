#include "speaker_node.h"
bool kill_node = false;
bool SpeakerNode::start(int argc, char **argv)
{
	bool status = false;
	process = new SpeakerNodeProcess();
	set_basenodename(BASE_NODE_NAME);
	initialize_firmware(MAJOR_RELEASE_VERSION, MINOR_RELEASE_VERSION, BUILD_NUMBER, FIRMWARE_DESCRIPTION);
	diagnostic = preinitialize_basenode(argc, argv);
	if (diagnostic.Level > WARN)
	{
		return false;
	}
	diagnostic = read_launchparameters();
	if (diagnostic.Level > WARN)
	{
		return false;
	}

	process->initialize(get_basenodename(), get_nodename(), get_hostname(), DIAGNOSTIC_SYSTEM, DIAGNOSTIC_SUBSYSTEM, DIAGNOSTIC_COMPONENT);
	std::vector<uint8_t> diagnostic_types;
	diagnostic_types.push_back(SOFTWARE);
	diagnostic_types.push_back(DATA_STORAGE);
	diagnostic_types.push_back(SYSTEM_RESOURCE);
	diagnostic_types.push_back(COMMUNICATIONS);
	process->enable_diagnostics(diagnostic_types);
	//sc.reset(new sound_play::SoundClient());
	process->finish_initialization();
	diagnostic = finish_initialization();
	if (diagnostic.Level > WARN)
	{
		return false;
	}
	if (diagnostic.Level < WARN)
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

eros::diagnostic SpeakerNode::read_launchparameters()
{
	eros::diagnostic diag = diagnostic;
	logger->log_notice(__FILE__,__LINE__,"Configuration Files Loaded.");
	return diagnostic;
}
eros::diagnostic SpeakerNode::finish_initialization()
{
	eros::diagnostic diag = diagnostic;
	pps1_sub = n->subscribe<std_msgs::Bool>("/1PPS", 1, &SpeakerNode::PPS1_Callback, this);
	command_sub = n->subscribe<eros::command>("/command", 1, &SpeakerNode::Command_Callback, this);
	std::string device_topic = "/" + std::string(host_name) + "_master_node/srv_device";
	srv_device = n->serviceClient<eros::srv_device>(device_topic);
	UserMessage_sub = n->subscribe<eros::usermessage>("/usermessage",10,&SpeakerNode::UserMessage_Callback,this);
	soundplaystatus_sub = n->subscribe<actionlib_msgs::GoalStatusArray>("/sound_play/status",1,&SpeakerNode::SoundPlayStatus_Callback,this);
	robot_sound_pub = n->advertise<sound_play::SoundRequest>("/robotsound",1);
	return diagnostic;
}
bool SpeakerNode::run_001hz()
{
	
	return true;
}
bool SpeakerNode::run_01hz()
{
	return true;
}
bool SpeakerNode::run_01hz_noisy()
{
	std::vector<eros::diagnostic> diaglist = process->get_diagnostics();
	for (std::size_t i = 0; i < diaglist.size(); ++i)
	{
		get_logger()->log_diagnostic(diaglist.at(i));
		diagnostic_pub.publish(diaglist.at(i));
	}
	eros::diagnostic diag = rescan_topics();
	get_logger()->log_diagnostic(diag);
	diagnostic_pub.publish(diag);
	return true;
}
bool SpeakerNode::run_1hz()
{
	process->update_diagnostic(get_resource_diagnostic());
	if(process->get_taskstate() == TASKSTATE_RUNNING)
	{
	}
	else if(process->get_taskstate() == TASKSTATE_INITIALIZED)
	{
	}
	else if (process->get_taskstate() == TASKSTATE_INITIALIZING)
	{
		{
			eros::srv_device srv;
			srv.request.query = "SELF";
			if (srv_device.call(srv) == true)
			{
				if (srv.response.data.size() != 1)
				{

					get_logger()->log_error(__FILE__,__LINE__,"Got unexpected device message.");
				}
				else
				{
					new_devicemsg(srv.request.query, srv.response.data.at(0));
				}
			}
			else
			{
			}
		}
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
bool SpeakerNode::run_10hz()
{
	ready_to_arm = process->get_ready_to_arm();
	eros::diagnostic diag = process->update(0.1, ros::Time::now().toSec());
	if (diag.Level > WARN)
	{
		get_logger()->log_diagnostic(diag);
		diagnostic_pub.publish(diag);
	}
	if(process->readytospeak() == true)
	{
		//std::string speechout = process->get_speechoutput();
		//sc->say(speechout);
		sound_play::SoundRequest msg = process->get_speechoutput_client();
		robot_sound_pub.publish(process->get_speechoutput_client());
	}
	std::vector<eros::diagnostic> diaglist = process->get_diagnostics();
	for (std::size_t i = 0; i < diaglist.size(); ++i)
	{
		if (diaglist.at(i).Level > WARN)
		{
			get_logger()->log_diagnostic(diaglist.at(i));
			diagnostic_pub.publish(diaglist.at(i));
		}
	}
	return true;
}
bool SpeakerNode::run_loop1()
{
	return true;
}
bool SpeakerNode::run_loop2()
{
	return true;
}
bool SpeakerNode::run_loop3()
{
	return true;
}

void SpeakerNode::PPS1_Callback(const std_msgs::Bool::ConstPtr &msg)
{
	new_ppsmsg(msg);
}

void SpeakerNode::Command_Callback(const eros::command::ConstPtr &t_msg)
{
	std::vector<eros::diagnostic> diaglist = process->new_commandmsg(t_msg);
	new_commandmsg_result(t_msg, diaglist);
}
bool SpeakerNode::new_devicemsg(std::string query, eros::device t_device)
{
	if (query == "SELF")
	{
		if (t_device.DeviceName == std::string(host_name))
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
void SpeakerNode::UserMessage_Callback(const eros::usermessage::ConstPtr& msg)
{
	eros::diagnostic diag = process->new_usermessage(msg);
	if(diag.Level > NOTICE)
	{
		diagnostic_pub.publish(diag);
		logger->log_diagnostic(diag);
	}
}
void SpeakerNode::SoundPlayStatus_Callback(__attribute__((unused))const actionlib_msgs::GoalStatusArray::ConstPtr& msg)
{
	process->new_soundplaystatus();
}
void SpeakerNode::diagnostic_Callback(const eros::diagnostic::ConstPtr& msg)
{
	if(msg->Level > NOTICE)
	{
		std::string tempstr;
		switch(msg->Level)
		{
		case NOTICE:
			tempstr += "NOTICE ";
			break;
		case WARN:
			tempstr += "WARN ";
			break;
		case ERROR:
			tempstr += "ERROR ";
			break;
		case FATAL:
			tempstr += "FATAL ";
			break;
		default:
			break;
		}
		std::string tempstr2;
		tempstr2 = msg->Node_Name;
		boost::replace_all(tempstr2,"/"," ");
		boost::replace_all(tempstr2,"_"," ");
		tempstr += tempstr2;
		tempstr += " is ";
		tempstr += msg->Description;
		//speak(tempstr,true);
        eros::usermessage usermsg;
        usermsg.Level = msg->Level;
        usermsg.message = tempstr;
		eros::usermessage::ConstPtr user_msg(new eros::usermessage(usermsg));
        eros::diagnostic diag = process->new_usermessage(user_msg);
		if(diag.Level > NOTICE)
		{
			diagnostic_pub.publish(diag);
			logger->log_diagnostic(diag);
		}
	}
}
eros::diagnostic SpeakerNode::rescan_topics()
{
	eros::diagnostic diag = diagnostic;
	int found_new_topics = 0;

	ros::master::V_TopicInfo master_topics;
	ros::master::getTopics(master_topics);
	for (ros::master::V_TopicInfo::iterator it = master_topics.begin() ; it != master_topics.end(); it++)
	{
		const ros::master::TopicInfo& info = *it;
		if(info.datatype == "eros/diagnostic")
		{
			int v = process->push_topiclist(info.datatype,info.name);
			if(v == 1)
			{
				found_new_topics++;
				char tempstr[255];
				sprintf(tempstr,"Subscribing to diagnostic topic: %s",info.name.c_str());
				logger->log_info(__FILE__,__LINE__,tempstr);
				ros::Subscriber sub = n->subscribe<eros::diagnostic>(info.name,20,&SpeakerNode::diagnostic_Callback,this);
				diagnostic_subs.push_back(sub);
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
void SpeakerNode::thread_loop()
{
	while (kill_node == false)
	{
		ros::Duration(1.0).sleep();
	}
}
void SpeakerNode::cleanup()
{
	base_cleanup();
	get_logger()->log_info(__FILE__,__LINE__,"[SpeakerNode] Finished Safely.");
}
/*! \brief Attempts to kill a node when an interrupt is received.
 *
 */
void signalinterrupt_handler(int sig)
{
	printf("Killing SpeakerNode with Signal: %d", sig);
	kill_node = true;
	exit(0);
}
int main(int argc, char **argv)
{
	signal(SIGINT, signalinterrupt_handler);
	signal(SIGTERM, signalinterrupt_handler);
	SpeakerNode *node = new SpeakerNode();
	bool status = node->start(argc, argv);
	std::thread thread(&SpeakerNode::thread_loop, node);
	while ((status == true) and (kill_node == false))
	{
		status = node->update(node->get_process()->get_taskstate());
	}
	node->cleanup();
	thread.detach();
	return 0;
}

