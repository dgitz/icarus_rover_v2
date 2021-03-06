#include "safety_node.h"
bool kill_node = false;
bool SafetyNode::start(int argc,char **argv)
{
	bool status = false;
	process = new SafetyNodeProcess();
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
		diagnostic = process->update_diagnostic(DATA_STORAGE,INFO,INITIALIZING,"Node Configured.  Initializing.");
		get_logger()->log_diagnostic(diagnostic);
	}
	status = true;
	return status;
}

eros::diagnostic SafetyNode::read_launchparameters()
{
	eros::diagnostic diag = diagnostic;
	get_logger()->log_notice(__FILE__,__LINE__,"Configuration Files Loaded.");
	return diagnostic;
}
eros::diagnostic SafetyNode::finish_initialization()
{
	eros::diagnostic diag = diagnostic;
	pps1_sub = n->subscribe<std_msgs::Bool>("/1PPS",1,&SafetyNode::PPS1_Callback,this);
	command_sub = n->subscribe<eros::command>("/command",1,&SafetyNode::Command_Callback,this);
	std::string device_topic = "/" + std::string(host_name) + "_master_node/srv_device";
	srv_device = n->serviceClient<eros::srv_device>(device_topic);
	return diagnostic;
}
bool SafetyNode::run_001hz()
{
	return true;
}
bool SafetyNode::run_01hz()
{
	return true;
}
bool SafetyNode::run_01hz_noisy()
{
	std::vector<eros::diagnostic> diaglist = process->get_diagnostics();
	for (std::size_t i = 0; i < diaglist.size(); ++i)
	{
		get_logger()->log_diagnostic(diaglist.at(i));
		diagnostic_pub.publish(diaglist.at(i));
	}
	return true;
}
bool SafetyNode::run_1hz()
{
	process->update_diagnostic(get_resource_diagnostic());
	if(process->get_taskstate() == TASKSTATE_RUNNING)
	{
	}
	else if(process->get_taskstate() == TASKSTATE_INITIALIZED)
	{
		eros::diagnostic diag = diagnostic;
		eros::srv_device srv;
		srv.request.query = (std::string("DeviceType=") + std::string(DEVICETYPE_TERMINALHAT)).c_str();
		if(srv_device.call(srv) == true)
		{
			for(std::size_t i = 0; i < srv.response.data.size(); i++)
			{
				bool status = new_devicemsg(srv.request.query,srv.response.data.at(i));
			}
		}
		else
		{
		}
		if(process->is_hat_running(DEVICETYPE_TERMINALHAT,0) == false)
		{
			bool init = TerminalHat.init(process->get_mydevice().PartNumber);
			if(init == false)
			{
				diag = process->update_diagnostic(DATA_STORAGE,ERROR,INITIALIZING_ERROR,"TerminalHatDriver Could not initialize.");
				get_logger()->log_diagnostic(diag);
				diagnostic_pub.publish(diag);

			}
			{
				bool any_error = false;
				std::vector<eros::pin> pins = process->get_terminalhatpins("");
				for(std::size_t i = 0; i < pins.size(); i++)
				{
					if(TerminalHat.configure_pin(pins.at(i).Name,pins.at(i).Function) == false)
					{
						any_error = true;
						char tempstr[512];
						sprintf(tempstr,"[%s] Could not configure Pin: %s with Function: %s",DEVICETYPE_TERMINALHAT,pins.at(i).Name.c_str(),pins.at(i).Function.c_str());
						diag = process->update_diagnostic(DATA_STORAGE,ERROR,INITIALIZING_ERROR,std::string(tempstr));
						logger->log_error(__FILE__,__LINE__,std::string(tempstr));
						kill_node = 1;
					}
				}
				if(any_error == false)
				{
					diag = process->set_terminalhat_initialized();
					logger->log_diagnostic(diag);
					if(diag.Level > NOTICE) { diagnostic_pub.publish(diag); }
				}
			}
		}
	}
	else if(process->get_taskstate() == TASKSTATE_INITIALIZING)
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
bool SafetyNode::run_10hz()
{
	ready_to_arm = process->get_ready_to_arm();
	eros::diagnostic diag = process->update(0.1,ros::Time::now().toSec());
	if(diag.Level > WARN)
	{
		get_logger()->log_diagnostic(diag);
		diagnostic_pub.publish(diag);
	}
	if(process->is_hat_running(DEVICETYPE_TERMINALHAT,0) == true)
	{
		int pin_value = TerminalHat.read_pin("ArmSwitch");
		bool v = process->set_pinvalue("ArmSwitch",pin_value);
		if(v == false)
		{
			char tempstr[512];
			sprintf(tempstr,"[%s] Could not read Arm Switch",DEVICETYPE_TERMINALHAT);
			diag = process->update_diagnostic(DATA_STORAGE,ERROR,DEVICE_NOT_AVAILABLE,std::string(tempstr));
			logger->log_error(__FILE__,__LINE__,std::string(tempstr));
			kill_node = 1;
		}
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
bool SafetyNode::run_loop1()
{
	return true;
}
bool SafetyNode::run_loop2()
{
	return true;
}
bool SafetyNode::run_loop3()
{
	return true;
}

void SafetyNode::PPS1_Callback(const std_msgs::Bool::ConstPtr& msg)
{
	new_ppsmsg(msg);
}

void SafetyNode::Command_Callback(const eros::command::ConstPtr& t_msg)
{
	std::vector<eros::diagnostic> diaglist = process->new_commandmsg(t_msg);
	new_commandmsg_result(t_msg,diaglist);
}
bool SafetyNode::new_devicemsg(std::string query,eros::device t_device)
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

void SafetyNode::thread_loop()
{
	while(kill_node == false)
	{
		ros::Duration(1.0).sleep();
	}
}
void SafetyNode::cleanup()
{
	base_cleanup();
	get_logger()->log_info(__FILE__,__LINE__,"[SafetyNode] Finished Safely.");
}
/*! \brief Attempts to kill a node when an interrupt is received.
 *
 */
void signalinterrupt_handler(int sig)
{
	printf("Killing SafetyNode with Signal: %d", sig);
	kill_node = true;
	exit(0);
}
int main(int argc, char **argv) {
	signal(SIGINT, signalinterrupt_handler);
	signal(SIGTERM, signalinterrupt_handler);
	SafetyNode *node = new SafetyNode();
	bool status = node->start(argc,argv);
	std::thread thread(&SafetyNode::thread_loop, node);
	while((status == true) and (kill_node == false))
	{
		status = node->update(node->get_process()->get_taskstate());
	}
	node->cleanup();
	thread.detach();
	return 0;
}

