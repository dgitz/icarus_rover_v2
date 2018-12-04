#include "commandlauncher_node.h"
bool kill_node = false;
bool CommandLauncherNode::start(int argc,char **argv)
{
	bool status = false;
	process = new CommandLauncherNodeProcess();
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

icarus_rover_v2::diagnostic CommandLauncherNode::read_launchparameters()
{
	icarus_rover_v2::diagnostic diag = diagnostic;
	get_logger()->log_notice("Configuration Files Loaded.");
	return diagnostic;
}
icarus_rover_v2::diagnostic CommandLauncherNode::finish_initialization()
{
	icarus_rover_v2::diagnostic diag = diagnostic;
	pps1_sub = n->subscribe<std_msgs::Bool>("/1PPS",1,&CommandLauncherNode::PPS1_Callback,this);
	command_sub = n->subscribe<icarus_rover_v2::command>("/command",1,&CommandLauncherNode::Command_Callback,this);
	std::string device_topic = "/" + std::string(host_name) + "_master_node/srv_device";
	srv_device = n->serviceClient<icarus_rover_v2::srv_device>(device_topic);
	std::string param_cameraport = node_name + "/CameraStreamPort";
	std::string camerastream_port;
	if(n->getParam(param_cameraport,camerastream_port) == false)
	{
		diag.Diagnostic_Type = DATA_STORAGE;
		diag.Level = ERROR;
		diag.Diagnostic_Message = INITIALIZING_ERROR;
		diag.Description = "Missing parameter: CameraStreamPort.  Exiting.";
		logger->log_diagnostic(diag);
		return diag;
	}
	process->set_camerastream_port(camerastream_port);
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
	icarus_rover_v2::diagnostic diag = process->get_diagnostic();
	{
		get_logger()->log_diagnostic(diag);
		diagnostic_pub.publish(diag);
	}
	return true;
}
bool CommandLauncherNode::run_1hz()
{

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
				int ret = system (processlist.at(i).command_text.c_str());
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
bool CommandLauncherNode::run_10hz()
{
	ready_to_arm = process->get_ready_to_arm();
	icarus_rover_v2::diagnostic diag = process->update(0.1,ros::Time::now().toSec());
	if(diag.Level > WARN)
	{
		get_logger()->log_diagnostic(diag);
		diagnostic_pub.publish(diag);
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

void CommandLauncherNode::Command_Callback(const icarus_rover_v2::command::ConstPtr& t_msg)
{
	std::vector<icarus_rover_v2::diagnostic> diaglist = process->new_commandmsg(t_msg);
	new_commandmsg_result(t_msg,diaglist);
}
bool CommandLauncherNode::new_devicemsg(std::string query,icarus_rover_v2::device t_device)
{
	if(query == "SELF")
	{
		if(t_device.DeviceName == std::string(host_name))
		{
			set_mydevice(t_device);
			process->set_mydevice(t_device);
			if(process->set_camerastream(process->get_camerastream_port()) == false)
			{
				char tempstr[512];
				sprintf(tempstr,"Couldn't set Camera Stream Port. Exiting.");
				logger->log_error(tempstr);
				return false;
			}
		}
	}

	if((process->is_initialized() == true))
	{
		icarus_rover_v2::device::ConstPtr device_ptr(new icarus_rover_v2::device(t_device));
		icarus_rover_v2::diagnostic diag = process->new_devicemsg(device_ptr);
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
void signalinterrupt_handler(int sig)
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

