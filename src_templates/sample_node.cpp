#include "sample_node.h"
bool kill_node = false;
bool SampleNode::start(int argc,char **argv)
{
	bool status = false;
	process = new SampleNodeProcess();
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

eros::diagnostic SampleNode::read_launchparameters()
{
	eros::diagnostic diag = diagnostic;
	get_logger()->log_notice("Configuration Files Loaded.");
	return diagnostic;
}
eros::diagnostic SampleNode::finish_initialization()
{
	eros::diagnostic diag = diagnostic;
	pps1_sub = n->subscribe<std_msgs::Bool>("/1PPS",1,&SampleNode::PPS1_Callback,this);
	command_sub = n->subscribe<eros::command>("/command",1,&SampleNode::Command_Callback,this);
	std::string device_topic = "/" + std::string(host_name) + "_master_node/srv_device";
	srv_device = n->serviceClient<eros::srv_device>(device_topic);
	return diagnostic;
}
bool SampleNode::run_001hz()
{
	return true;
}
bool SampleNode::run_01hz()
{
	eros::diagnostic diag = process->get_diagnostic();
	{
		get_logger()->log_diagnostic(diag);
		diagnostic_pub.publish(diag);
	}
	return true;
}
bool SampleNode::run_1hz()
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
	eros::diagnostic diag = process->get_diagnostic();
	if(diag.Level >= NOTICE)
	{
		get_logger()->log_diagnostic(diag);
		diagnostic_pub.publish(diag);
	}

	return true;
}
bool SampleNode::run_10hz()
{
	ready_to_arm = process->get_ready_to_arm();
	eros::diagnostic diag = process->update(0.1,ros::Time::now().toSec());
	if(diag.Level > WARN)
	{
		get_logger()->log_diagnostic(diag);
		diagnostic_pub.publish(diag);
	}
	return true;
}
bool SampleNode::run_loop1()
{
	return true;
}
bool SampleNode::run_loop2()
{
	return true;
}
bool SampleNode::run_loop3()
{
	return true;
}

void SampleNode::PPS1_Callback(const std_msgs::Bool::ConstPtr& msg)
{
	new_ppsmsg(msg);
}

void SampleNode::Command_Callback(const eros::command::ConstPtr& t_msg)
{
	std::vector<eros::diagnostic> diaglist = process->new_commandmsg(t_msg);
	new_commandmsg_result(t_msg,diaglist);
}
bool SampleNode::new_devicemsg(std::string query,eros::device t_device)
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
void SampleNode::thread_loop()
{
	while(kill_node == false)
	{
		ros::Duration(1.0).sleep();
	}
}
void SampleNode::cleanup()
{
}
/*! \brief Attempts to kill a node when an interrupt is received.
 *
 */
void signalinterrupt_handler(int sig)
{
	printf("Killing Node with Signal: %d",sig);
	kill_node = true;
	exit(0);
}
int main(int argc, char **argv) {
	signal(SIGINT, signalinterrupt_handler);
	signal(SIGTERM, signalinterrupt_handler);
	SampleNode *node = new SampleNode();
	bool status = node->start(argc,argv);
	std::thread thread(&SampleNode::thread_loop, node);
	while((status == true) and (kill_node == false))
	{
		status = node->update();
	}
	node->cleanup();
	node->get_logger()->log_info("Node Finished Safely.");
	return 0;
}

