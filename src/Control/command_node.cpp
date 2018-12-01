#include "command_node.h"
bool kill_node = false;
bool CommandNode::start(int argc,char **argv)
{
	bool status = false;
	process = new CommandNodeProcess();
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

icarus_rover_v2::diagnostic CommandNode::read_launchparameters()
{
	icarus_rover_v2::diagnostic diag = diagnostic;
	get_logger()->log_notice("Configuration Files Loaded.");
	return diagnostic;
}
icarus_rover_v2::diagnostic CommandNode::finish_initialization()
{
	icarus_rover_v2::diagnostic diag = diagnostic;
	pps1_sub = n->subscribe<std_msgs::Bool>("/1PPS",1,&CommandNode::PPS1_Callback,this);
	command_sub = n->subscribe<icarus_rover_v2::command>("/command",1,&CommandNode::Command_Callback,this);
	std::string device_topic = "/" + std::string(host_name) + "_master_node/srv_device";
	srv_device = n->serviceClient<icarus_rover_v2::srv_device>(device_topic);
	std::string command_topic = "/command";
	command_pub =  n->advertise<icarus_rover_v2::command>(command_topic,1000);

	std::vector<std::string> ready_to_arm_topics;

	bool search_for_topics = true;
	int topicindex = 1;
	while(search_for_topics == true)
	{
		std::string topic;
		bool add_new_topic = false;
		std::string param_topic = node_name +"/ready_to_arm_topic" + boost::lexical_cast<std::string>(topicindex);
		if(n->getParam(param_topic,topic) == false)
		{
			char tempstr[255];
			sprintf(tempstr,"Didn't find %s Not adding anymore.",param_topic.c_str());
			logger->log_info(tempstr);
			add_new_topic = false;
			search_for_topics = false;
		}
		else
		{
			char tempstr[255];
			sprintf(tempstr,"Adding Ready to Arm Topic: %s",topic.c_str());
			logger->log_info(tempstr);
			ready_to_arm_topics.push_back(topic);
			search_for_topics = true;
			topicindex++;
		}
	}
	for(int i = 0; i < ready_to_arm_topics.size();i++)
	{
		ros::Subscriber sub = n->subscribe<std_msgs::Bool>(ready_to_arm_topics.at(i),10,boost::bind(&CommandNode::ReadyToArm_Callback,this,_1,ready_to_arm_topics.at(i)));
		ready_to_arm_subs.push_back(sub);
	}
	diag = process->init_readytoarm_list(ready_to_arm_topics);
	if(diag.Level >= WARN)
	{
		diag.Diagnostic_Type = DATA_STORAGE;
		diag.Level = ERROR;
		diag.Diagnostic_Message = INITIALIZING_ERROR;
		diag.Description = "Unable to initialize Ready To Arm List.  Exiting.";
		logger->log_diagnostic(diagnostic);
		return diag;
	}

	std::string param_user_command_topic = node_name +"/user_command_topic";
	std::string user_command_topic;
	if(n->getParam(param_user_command_topic,user_command_topic) == false)
	{
		diag.Diagnostic_Type = DATA_STORAGE;
		diag.Level = ERROR;
		diag.Diagnostic_Message = INITIALIZING_ERROR;
		diag.Description = "Missing parameter: user_command_topic. Exiting.";
		logger->log_diagnostic(diagnostic);
		return diag;
	}
	user_command_sub = n->subscribe<icarus_rover_v2::command>(user_command_topic,10,&CommandNode::User_Command_Callback,this);

	std::string armeddisarmed_state_topic = "/armed_state";
	armeddisarmed_state_pub = n->advertise<std_msgs::UInt8>(armeddisarmed_state_topic,10);
	return diagnostic;
}
bool CommandNode::run_001hz()
{
	return true;
}
bool CommandNode::run_01hz()
{
	icarus_rover_v2::diagnostic diag = process->get_diagnostic();
	{
		get_logger()->log_diagnostic(diag);
		diagnostic_pub.publish(diag);
	}
	return true;
}
bool CommandNode::run_1hz()
{
	if((process->is_initialized() == true) and (process->is_ready() == true))
	{
	}
	else if((process->is_ready() == false) and (process->is_initialized() == true))
	{
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
bool CommandNode::run_10hz()
{
	ready_to_arm = process->get_ready_to_arm();
	std_msgs::UInt8 state;
	state.data = process->get_armeddisarmed_state();
	armeddisarmed_state_pub.publish(state);

	icarus_rover_v2::diagnostic diag = process->update(0.1,ros::Time::now().toSec());
	if(diag.Level > WARN)
	{
		get_logger()->log_diagnostic(diag);
		diagnostic_pub.publish(diag);
	}
	std::vector<icarus_rover_v2::command> periodiccommands = process->get_PeriodicCommands();
	for(std::size_t i = 0; i < periodiccommands.size(); i++)
	{
		command_pub.publish(periodiccommands.at(i));
	}
	return true;
}
bool CommandNode::run_loop1()
{
	return true;
}
bool CommandNode::run_loop2()
{
	return true;
}
bool CommandNode::run_loop3()
{
	return true;
}

void CommandNode::PPS1_Callback(const std_msgs::Bool::ConstPtr& msg)
{
	new_ppsmsg(msg);
}

void CommandNode::Command_Callback(const icarus_rover_v2::command::ConstPtr& t_msg)
{
	std::vector<icarus_rover_v2::diagnostic> diaglist = process->new_commandmsg(t_msg);
	new_commandmsg_result(t_msg,diaglist);
}
bool CommandNode::new_devicemsg(std::string query,icarus_rover_v2::device t_device)
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
	return true;
}
void CommandNode::ReadyToArm_Callback(const std_msgs::Bool::ConstPtr& msg,const std::string &topic)
{
	process->new_readytoarmmsg(topic,msg->data);
}
void CommandNode::User_Command_Callback(const icarus_rover_v2::command::ConstPtr& msg)
{
	icarus_rover_v2::diagnostic diagnostic = process->new_user_commandmsg(msg);
	logger->log_diagnostic(diagnostic);
	diagnostic_pub.publish(diagnostic);
}

void CommandNode::thread_loop()
{
	while(kill_node == false)
	{
		ros::Duration(1.0).sleep();
	}
}
void CommandNode::cleanup()
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
	CommandNode *node = new CommandNode();
	bool status = node->start(argc,argv);
	std::thread thread(&CommandNode::thread_loop, node);
	while((status == true) and (kill_node == false))
	{
		status = node->update();
	}
	node->get_logger()->log_info("Node Finished Safely.");
	return 0;
}

