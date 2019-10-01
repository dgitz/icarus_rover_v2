#include "topicremapper_node.h"
bool kill_node = false;
bool TopicRemapperNode::start(int argc,char **argv)
{
	bool status = false;
	process = new TopicRemapperNodeProcess();
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
		diagnostic = process->update_diagnostic(SOFTWARE,INFO,INITIALIZING,"Node Configured.  Initializing.");
		get_logger()->log_diagnostic(diagnostic);
	}
	status = true;
	return status;
}

eros::diagnostic TopicRemapperNode::read_launchparameters()
{
	eros::diagnostic diag = diagnostic;
	get_logger()->log_notice("Configuration Files Loaded.");
	return diagnostic;
}
eros::diagnostic TopicRemapperNode::finish_initialization()
{
	eros::diagnostic diag = diagnostic;
	diagnostic = process->load("/home/robot/config/TopicMap.xml");
	if(diagnostic.Level > NOTICE)
	{
		logger->log_diagnostic(diagnostic);
		return diagnostic;
	}
	logger->log_debug(process->print_topicmaps().c_str());
	pps1_sub = n->subscribe<std_msgs::Bool>("/1PPS",1,&TopicRemapperNode::PPS1_Callback,this);
	command_sub = n->subscribe<eros::command>("/command",1,&TopicRemapperNode::Command_Callback,this);
	std::string device_topic = "/" + std::string(host_name) + "_master_node/srv_device";
	srv_device = n->serviceClient<eros::srv_device>(device_topic);
	std::vector<TopicRemapperNodeProcess::TopicMap> TopicMaps = process->get_topicmaps();
	for(std::size_t i = 0; i < TopicMaps.size();i++)
	{
		if(TopicMaps.at(i).in.type == "sensor_msgs/Joy")
		{
			ros::Subscriber sub = n->subscribe<sensor_msgs::Joy>(TopicMaps.at(i).in.topic,10,boost::bind(&TopicRemapperNode::Joystick_Callback,this,_1,TopicMaps.at(i).in.topic));
			char tempstr[255];
			sprintf(tempstr,"Subscribing to: %s",TopicMaps.at(i).in.topic.c_str());
			logger->log_info(tempstr);
			subs.push_back(sub);
		}
		for(std::size_t j = 0; j < TopicMaps.at(i).outs.size();j++)
		{
			if(TopicMaps.at(i).outs.at(j).type == "eros/pin")
			{
				ros::Publisher pub = n->advertise<eros::pin>(TopicMaps.at(i).outs.at(j).topic,10);
				pin_pubs.push_back(pub);
			}
			else if(TopicMaps.at(i).outs.at(j).type == "std_msgs/Float32")
			{
				ros::Publisher pub = n->advertise<std_msgs::Float32>(TopicMaps.at(i).outs.at(j).topic,10);
				float32_pubs.push_back(pub);
			}
			else if(TopicMaps.at(i).outs.at(j).type == "sensor_msgs/JointState")
			{
				std::size_t found = TopicMaps.at(i).outs.at(j).topic.substr(1).find("/");
				std::string topic = TopicMaps.at(i).outs.at(j).topic.substr(0,found+1);
				ros::Publisher pub = n->advertise<sensor_msgs::JointState>(topic,10);
			}
		}
	}
	return diagnostic;
}
bool TopicRemapperNode::run_001hz()
{
	return true;
}
bool TopicRemapperNode::run_01hz()
{
	return true;
}
bool TopicRemapperNode::run_01hz_noisy()
{
	std::vector<eros::diagnostic> diaglist = process->get_diagnostics();
	for (std::size_t i = 0; i < diaglist.size(); ++i)
	{
		get_logger()->log_diagnostic(diaglist.at(i));
		diagnostic_pub.publish(diaglist.at(i));
	}
	return true;
}
bool TopicRemapperNode::run_1hz()
{
	process->update_diagnostic(get_resource_diagnostic());
	if ((process->is_initialized() == true) and (process->is_ready() == true))
	{
	}
	else if((process->is_ready() == false) and (process->is_initialized() == true))
	{
	}
	else if(process->is_initialized() == false)
	{
		{
			eros::srv_device srv;
			srv.request.query = "SELF";
			if(srv_device.call(srv) == true)
			{
				if(srv.response.data.size() != 1)
				{

					get_logger()->log_error("Got unexpected device message.");
				}
				else
				{
					new_devicemsg(srv.request.query,srv.response.data.at(0));
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
bool TopicRemapperNode::run_10hz()
{
	ready_to_arm = process->get_ready_to_arm();
	eros::diagnostic diag = process->update(0.1,ros::Time::now().toSec());
	if(diag.Level > WARN)
	{
		get_logger()->log_diagnostic(diag);
		diagnostic_pub.publish(diag);
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
bool TopicRemapperNode::run_loop1()
{
	{
		std::vector<eros::pin> outs = process->get_outputs_pins();
		for(std::size_t i = 0; i < pin_pubs.size(); i++)
		{
			outs.at(i).stamp = ros::Time::now();
			pin_pubs.at(i).publish(outs.at(i));
		}
	}
	{
		std::vector<std_msgs::Float32> outs = process->get_outputs_float32();
		for(std::size_t i = 0; i < float32_pubs.size(); i++)
		{
			float32_pubs.at(i).publish(outs.at(i));
		}
	}
	return true;
}
bool TopicRemapperNode::run_loop2()
{
	return true;
}
bool TopicRemapperNode::run_loop3()
{
	return true;
}

void TopicRemapperNode::PPS1_Callback(const std_msgs::Bool::ConstPtr& msg)
{
	new_ppsmsg(msg);
}

void TopicRemapperNode::Command_Callback(const eros::command::ConstPtr& t_msg)
{
	std::vector<eros::diagnostic> diaglist = process->new_commandmsg(t_msg);
	new_commandmsg_result(t_msg,diaglist);
}
void TopicRemapperNode::Joystick_Callback(const sensor_msgs::Joy::ConstPtr& msg,const std::string &topic)
{
	//eros::iopins p_pwmoutputs;
	//eros::iopins p_digitaloutputs;
	sensor_msgs::Joy joy;
	joy.header = msg->header;
	joy.buttons = msg->buttons;
	joy.axes = msg->axes;
	eros::diagnostic diag = process->new_joymsg(joy,topic);
	if(diag.Level > NOTICE)
	{
		logger->log_diagnostic(diag);
		diagnostic_pub.publish(diag);
	}
	/*

    //pwmoutput_pub.publish(p_pwmoutputs);
    //digitaloutput_pub.publish(p_digitaloutputs);
	 */
}

bool TopicRemapperNode::new_devicemsg(std::string query,eros::device t_device)
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
		eros::device::ConstPtr device_ptr(new eros::device(t_device));
		eros::diagnostic diag = process->new_devicemsg(device_ptr);
	}
	return true;
}

void TopicRemapperNode::thread_loop()
{
	while(kill_node == false)
	{
		ros::Duration(1.0).sleep();
	}
}
void TopicRemapperNode::cleanup()
{
}
/*! \brief Attempts to kill a node when an interrupt is received.
 *
 */
void signalinterrupt_handler(__attribute__((unused)) int sig)
{
	kill_node = true;
	exit(0);
}
int main(int argc, char **argv) {
	signal(SIGINT, signalinterrupt_handler);
	signal(SIGTERM, signalinterrupt_handler);
	TopicRemapperNode *node = new TopicRemapperNode();
	bool status = node->start(argc,argv);
	std::thread thread(&TopicRemapperNode::thread_loop, node);
	while((status == true) and (kill_node == false))
	{
		status = node->update();
	}
	node->get_logger()->log_info("Node Finished Safely.");
	return 0;
}

