#include "commandlauncher_node.h"
bool kill_node = false;
bool CommandLauncherNode::start(int argc,char **argv)
{
	bool status = false;
	process = new CommandLauncherNodeProcess();
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
	process->enable_diagnostics(diagnostic_types);
	
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

eros::diagnostic CommandLauncherNode::read_launchparameters()
{
	eros::diagnostic diag = diagnostic;
	get_logger()->log_notice("Configuration Files Loaded.");
	return diagnostic;
}
eros::diagnostic CommandLauncherNode::finish_initialization()
{
	eros::diagnostic diag = diagnostic;
	pps1_sub = n->subscribe<std_msgs::Bool>("/1PPS",1,&CommandLauncherNode::PPS1_Callback,this);
	command_sub = n->subscribe<eros::command>("/command",1,&CommandLauncherNode::Command_Callback,this);
	std::string device_topic = "/" + std::string(host_name) + "_master_node/srv_device";
	srv_device = n->serviceClient<eros::srv_device>(device_topic);
	std::vector<CommandLauncherNodeProcess::ProcessCommand> process_list = process->get_processlist();
	for(std::size_t i = 0; i < process_list.size(); i++)
	{
		char tempstr[1024];
		sprintf(tempstr,"pkill %s",process_list.at(i).kill_name.c_str());
		system(tempstr);
	}
	return diagnostic;
}
bool CommandLauncherNode::run_001hz()
{
	return true;
}
bool CommandLauncherNode::run_01hz()
{
	
	return true;
}
bool CommandLauncherNode::run_01hz_noisy()
{
	std::vector<eros::diagnostic> diaglist = process->get_diagnostics();
	for (std::size_t i = 0; i < diaglist.size(); ++i)
	{
		get_logger()->log_diagnostic(diaglist.at(i));
		diagnostic_pub.publish(diaglist.at(i));
	}
	return true;
}
bool CommandLauncherNode::run_1hz()
{
	process->update_diagnostic(get_resource_diagnostic());
	if((process->is_initialized() == true) and (process->is_ready() == true))
	{
		std::vector<CommandLauncherNodeProcess::ProcessCommand> processlist = process->get_processlist();
		for(std::size_t i = 0; i < processlist.size(); i++)
		{
			bool check_pid = false;
			if((processlist.at(i).restart_counter >= processlist.at(i).max_restarts) and (processlist.at(i).max_restarts > 0))
			{
				char tempstr[512];
				sprintf(tempstr,"Process: %s restarted %d times.  Not restarting anymore.",processlist.at(i).name.c_str(),processlist.at(i).restart_counter);
				logger->log_warn(std::string(tempstr));
			}
			else if(processlist.at(i).running == false)
			{
				char tempstr[1024];
				sprintf(tempstr,"Trying to restart process with command: %s",processlist.at(i).command_text.c_str());

				logger->log_info(std::string(tempstr));
				system (processlist.at(i).command_text.c_str());
				process->set_process_restarted(processlist.at(i).name);
				check_pid = true;
			}
			if((processlist.at(i).running == true) || (check_pid == true))
			{
				uint32_t pid = get_pid_byname(processlist.at(i).process_name);
				if(pid == 0)
				{
					process->set_processrunning(processlist.at(i).name,false);
					char tempstr[512];
					sprintf(tempstr,"Process: %s is not running.  Restarting (%d) times so far.",processlist.at(i).name.c_str(),processlist.at(i).restart_counter);
					logger->log_warn(std::string(tempstr));
				}
				else
				{
					process->set_processpid(processlist.at(i).name,pid);
					process->set_processrunning(processlist.at(i).name,true);
				}
			}
		}
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
bool CommandLauncherNode::run_10hz()
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
bool CommandLauncherNode::run_loop1()
{
	return true;
}
bool CommandLauncherNode::run_loop2()
{
	return true;
}
bool CommandLauncherNode::run_loop3()
{
	return true;
}

void CommandLauncherNode::PPS1_Callback(const std_msgs::Bool::ConstPtr& msg)
{
	new_ppsmsg(msg);
}

void CommandLauncherNode::Command_Callback(const eros::command::ConstPtr& t_msg)
{
	std::vector<eros::diagnostic> diaglist = process->new_commandmsg(t_msg);
	new_commandmsg_result(t_msg,diaglist);
}
bool CommandLauncherNode::new_devicemsg(std::string query,eros::device t_device)
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
uint32_t CommandLauncherNode::get_pid_byname(std::string name)
{
	char buf[512];
	char tempstr[256];
	sprintf(tempstr,"pidof -s %s",name.c_str());
	FILE *cmd_pipe = popen(tempstr, "r");

	fgets(buf, 512, cmd_pipe);
	pid_t pid = strtoul(buf, NULL, 10);
	pclose( cmd_pipe );
	return pid;
}
void CommandLauncherNode::thread_loop()
{
	while(kill_node == false)
	{
		ros::Duration(1.0).sleep();
	}
}
void CommandLauncherNode::cleanup()
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
	CommandLauncherNode *node = new CommandLauncherNode();
	bool status = node->start(argc,argv);
	std::thread thread(&CommandLauncherNode::thread_loop, node);
	while((status == true) and (kill_node == false))
	{
		status = node->update();
	}
	node->get_logger()->log_info("Node Finished Safely.");
	return 0;
}

