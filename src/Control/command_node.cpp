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

	process->initialize(get_basenodename(),get_nodename(),get_hostname(),DIAGNOSTIC_SYSTEM,DIAGNOSTIC_SUBSYSTEM,DIAGNOSTIC_COMPONENT);
	std::vector<uint8_t> diagnostic_types;
	diagnostic_types.push_back(SOFTWARE);
	diagnostic_types.push_back(DATA_STORAGE);
	diagnostic_types.push_back(SYSTEM_RESOURCE);
	diagnostic_types.push_back(REMOTE_CONTROL);
	diagnostic_types.push_back(TARGET_ACQUISITION);
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

eros::diagnostic CommandNode::read_launchparameters()
{
	eros::diagnostic diag = diagnostic;
	get_logger()->log_notice("Configuration Files Loaded.");
	return diagnostic;
}
eros::diagnostic CommandNode::finish_initialization()
{
	eros::diagnostic diag = diagnostic;
	pps1_sub = n->subscribe<std_msgs::Bool>("/1PPS",1,&CommandNode::PPS1_Callback,this);
	command_sub = n->subscribe<eros::command>("/command",1,&CommandNode::Command_Callback,this);
	std::string device_topic = "/" + std::string(host_name) + "_master_node/srv_device";
	srv_device = n->serviceClient<eros::srv_device>(device_topic);
	std::string command_topic = "/command";
	command_pub =  n->advertise<eros::command>(command_topic,20);
	systemstate_sub = n->advertise<eros::system_state>("/System/State",20);
	clock_sub = n->subscribe<rosgraph_msgs::Clock>("/clock",1,&CommandNode::gazeboclock_Callback,this);
	gazeboupdaterate_sub = n->subscribe<std_msgs::Float64>("/gazebo/update_rate",1,&CommandNode::gazeboupdaterate_Callback,this);
	std::vector<std::string> ready_to_arm_topics;

	bool search_for_topics = true;
	int topicindex = 1;
	while(search_for_topics == true)
	{
		std::string topic;
		std::string param_topic = node_name +"/ready_to_arm_topic" + boost::lexical_cast<std::string>(topicindex);
		if(n->getParam(param_topic,topic) == false)
		{
			char tempstr[255];
			sprintf(tempstr,"Didn't find %s Not adding anymore.",param_topic.c_str());
			logger->log_info(tempstr);
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
	for(std::size_t i = 0; i < ready_to_arm_topics.size();i++)
	{
		ros::Subscriber sub = n->subscribe<std_msgs::Bool>(ready_to_arm_topics.at(i),10,boost::bind(&CommandNode::ReadyToArm_Callback,this,_1,ready_to_arm_topics.at(i)));
		ready_to_arm_subs.push_back(sub);
	}
	diag = process->load_loadscriptingfiles("/home/robot/config/scriptfiles/");
	diag = process->update_diagnostic(diag);
	if(diag.Level >= WARN)
	{
		logger->log_diagnostic(diagnostic);
		return diag;
	}
	process->print_scriptcommand_list();
	diag = process->init_readytoarm_list(ready_to_arm_topics);
	if(diag.Level >= WARN)
	{
		logger->log_diagnostic(diag);
		return diag;
	}

	std::string param_user_command_topic = node_name +"/user_command_topic";
	std::string user_command_topic;
	if(n->getParam(param_user_command_topic,user_command_topic) == false)
	{

		diag = process->update_diagnostic(DATA_STORAGE,ERROR,INITIALIZING_ERROR,"Missing parameter: user_command_topic. Exiting.");
		logger->log_diagnostic(diag);
		return diag;
	}
	user_command_sub = n->subscribe<eros::command>(user_command_topic,10,&CommandNode::User_Command_Callback,this);

	std::string armeddisarmed_state_topic = "/armed_state";
	armeddisarmed_state_pub = n->advertise<std_msgs::UInt8>(armeddisarmed_state_topic,10);
	return diag;
}
bool CommandNode::run_001hz()
{
	return true;
}
bool CommandNode::run_01hz()
{
	return true;
}
bool CommandNode::run_01hz_noisy()
{
	std::vector<eros::diagnostic> diaglist = process->get_diagnostics();
	for (std::size_t i = 0; i < diaglist.size(); ++i)
	{
		get_logger()->log_diagnostic(diaglist.at(i));
		diagnostic_pub.publish(diaglist.at(i));
	}
	return true;
}
bool CommandNode::run_1hz()
{
	process->update_diagnostic(get_resource_diagnostic());
	if((process->is_initialized() == true) and (process->is_ready() == true))
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
	std::vector<eros::system_state> states = process->get_statelist();
	for(std::size_t i = 0; i < states.size(); ++i)
	{
		systemstate_sub.publish(states.at(i));
	}
	return true;
}
bool CommandNode::run_10hz()
{
	ready_to_arm = process->get_ready_to_arm();
	std_msgs::UInt8 state;
	state.data = process->get_armeddisarmed_state();
	armeddisarmed_state_pub.publish(state);

	eros::diagnostic diag = process->update(0.1,ros::Time::now().toSec());
	if(diag.Level > WARN)
	{
		get_logger()->log_diagnostic(diag);
		diagnostic_pub.publish(diag);
	}
	std::vector<eros::command> periodiccommands = process->get_PeriodicCommands();
	for(std::size_t i = 0; i < periodiccommands.size(); ++i)
	{
		command_pub.publish(periodiccommands.at(i));
	}
	std::vector<eros::command> command_buffer = process->get_command_buffer();
	for(std::size_t i = 0; i < command_buffer.size(); ++i)
	{
		command_pub.publish(command_buffer.at(i));
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
bool CommandNode::run_loop1()
{
	return true;
}
bool CommandNode::run_loop2()
{
	uint8_t gazebo_message = process->get_gazebomessagetopublish();
	if(gazebo_message != 0)
	{
		ros::ServiceClient client;
		std_srvs::Empty srv;
		switch(gazebo_message)
		{
			case ROVERCOMMAND_SIMULATIONCONTROL_RESETWORLD:
				client = n->serviceClient<std_srvs::Empty>("/gazebo/reset_simulation");
				client.call(srv);
				break;
			case ROVERCOMMAND_SIMULATIONCONTROL_STARTSIM:
				client = n->serviceClient<std_srvs::Empty>("/gazebo/unpause_physics");
				client.call(srv);
				break;
			case ROVERCOMMAND_SIMULATIONCONTROL_PAUSESIM:
				client = n->serviceClient<std_srvs::Empty>("/gazebo/pause_physics");
				client.call(srv);
				break;
			default:
				break;
		}
	}
	return true;
}
bool CommandNode::run_loop3()
{
	return true;
}
void CommandNode::gazeboupdaterate_Callback(const std_msgs::Float64::ConstPtr& msg)
{
	process->new_gazebo_updaterate(msg->data);
}
void CommandNode::gazeboclock_Callback(const rosgraph_msgs::Clock::ConstPtr& msg)
{
	process->new_gazeboclockmsg();
}
void CommandNode::PPS1_Callback(const std_msgs::Bool::ConstPtr& msg)
{
	new_ppsmsg(msg);
}

void CommandNode::Command_Callback(const eros::command::ConstPtr& t_msg)
{
	std::vector<eros::diagnostic> diaglist = process->new_commandmsg(t_msg);
	new_commandmsg_result(t_msg,diaglist);
}
bool CommandNode::new_devicemsg(std::string query,eros::device t_device)
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
void CommandNode::ReadyToArm_Callback(const std_msgs::Bool::ConstPtr& msg,const std::string &topic)
{
	process->new_readytoarmmsg(topic,msg->data);
}
void CommandNode::User_Command_Callback(const eros::command::ConstPtr& msg)
{
	eros::diagnostic diagnostic = process->new_user_commandmsg(msg);
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

