#include "navigation_node.h"
bool kill_node = false;
bool NavigationNode::start(int argc,char **argv)
{
	bool status = false;
	process = new NavigationNodeProcess();
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
		diagnostic.Diagnostic_Type = NOERROR;
		diagnostic.Level = INFO;
		diagnostic.Diagnostic_Message = NOERROR;
		diagnostic.Description = "Node Configured.  Initializing.";
		get_logger()->log_diagnostic(diagnostic);
	}
	status = true;
	return status;
}

eros::diagnostic NavigationNode::read_launchparameters()
{
	eros::diagnostic diag = diagnostic;
	get_logger()->log_notice("Configuration Files Loaded.");
	return diagnostic;
}
eros::diagnostic NavigationNode::finish_initialization()
{
	eros::diagnostic diag = diagnostic;
	pps1_sub = n->subscribe<std_msgs::Bool>("/1PPS",1,&NavigationNode::PPS1_Callback,this);
	command_sub = n->subscribe<eros::command>("/command",1,&NavigationNode::Command_Callback,this);
	std::string device_topic = "/" + std::string(host_name) + "_master_node/srv_device";
	srv_device = n->serviceClient<eros::srv_device>(device_topic);
	process->set_filepaths("/home/robot/config/ControlGroup.xml");
	return diagnostic;
}
bool NavigationNode::run_001hz()
{
	return true;
}
bool NavigationNode::run_01hz()
{
	return true;
}
bool NavigationNode::run_01hz_noisy()
{
	std::vector<eros::diagnostic> diaglist = process->get_diagnostics();
	for (std::size_t i = 0; i < diaglist.size(); ++i)
	{
		get_logger()->log_diagnostic(diaglist.at(i));
		diagnostic_pub.publish(diaglist.at(i));
	}
	print_controlgroups();
	return true;
}
bool NavigationNode::run_1hz()
{
process->update_diagnostic(get_resource_diagnostic());
	if((process->is_initialized() == true) and (process->is_ready() == true))
	{
	}
	else if((process->is_ready() == false) and (process->is_initialized() == true))
	{
		{
			eros::srv_device srv;
			srv.request.query = "ALL";
			if(srv_device.call(srv) == true)
			{
				for(std::size_t i = 0; i < srv.response.data.size(); i++)
				{
					bool status = new_devicemsg(srv.request.query,srv.response.data.at(i));

				}
			}
			eros::diagnostic diag = process->load_controlgroupfile();
			process->fetch_complete();
			if(diag.Level > NOTICE)
			{
				get_logger()->log_diagnostic(diag);
				diagnostic_pub.publish(diag);
			}
			std::vector<eros::pin> pins = process->get_pins();
			for(std::size_t i = 0; i < pins.size(); ++i)
			{
				ros::Publisher pub =  n->advertise<eros::pin>(pins.at(i).ConnectedDevice,10);
				pin_pubs.push_back(pub);
			}

		}
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
bool NavigationNode::run_10hz()
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
bool NavigationNode::run_loop1()
{
	std::vector<eros::pin> pins = process->get_pins();
	for(std::size_t i = 0; i < pins.size(); ++i)
	{
		pin_pubs.at(i).publish(pins.at(i));
	}
	return true;
}
bool NavigationNode::run_loop2()
{
	return true;
}
bool NavigationNode::run_loop3()
{
	return true;
}

void NavigationNode::PPS1_Callback(const std_msgs::Bool::ConstPtr& msg)
{
	new_ppsmsg(msg);
}

void NavigationNode::Command_Callback(const eros::command::ConstPtr& t_msg)
{
	std::vector<eros::diagnostic> diaglist = process->new_commandmsg(t_msg);
	for(std::size_t i = 0; i < diaglist.size(); ++i)
	{
		if(diaglist.at(i).Level >= NOTICE)
		{
			get_logger()->log_diagnostic(diaglist.at(i));
		}
	}
	new_commandmsg_result(t_msg,diaglist);
}
bool NavigationNode::new_devicemsg(std::string query,eros::device t_device)
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
void NavigationNode::print_controlgroups()
{
	std::vector<NavigationNodeProcess::ControlGroup> control_groups = process->get_controlgroups();
	std::string tempstr = "Control Group Map\n";
	for(std::size_t i = 0; i < control_groups.size(); ++i)
	{
		tempstr += "--- Control Group: "  + control_groups.at(i).name + " --- \n";
		tempstr += "\t Last Update: " + std::to_string(control_groups.at(i).time_since_lastupdate) + "\n";
	}
	//printf("%s",tempstr.c_str());
	get_logger()->log_info(tempstr);
}
void NavigationNode::thread_loop()
{
	while(kill_node == false)
	{
		ros::Duration(1.0).sleep();
	}
}
void NavigationNode::cleanup()
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
	NavigationNode *node = new NavigationNode();
	bool status = node->start(argc,argv);
	std::thread thread(&NavigationNode::thread_loop, node);
	while((status == true) and (kill_node == false))
	{
		status = node->update();
	}
	node->cleanup();
	node->get_logger()->log_info("Node Finished Safely.");
	return 0;
}

