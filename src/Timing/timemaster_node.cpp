#include "timemaster_node.h"
bool kill_node = false;
bool TimeMasterNode::start(int argc,char **argv)
{
	bool status = false;
	process = new TimeMasterNodeProcess();
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
	diagnostic_types.push_back(TIMING);
	diagnostic_types.push_back(SENSORS);
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

eros::diagnostic TimeMasterNode::read_launchparameters()
{
	eros::diagnostic diag = diagnostic;
	std::string pps_source;
	std::string param_pps_source = node_name +"/pps_source";
	if(n->getParam(param_pps_source,pps_source) == false)
	{
		logger->log_warn("Missing Parameter: pps_source.  Using default: self");
		pps_source = "self";
	}
	if(process->set_ppssource(pps_source) == false)
	{
		char tempstr[512];
		sprintf(tempstr,"PPS Source: %s Not Supported",pps_source.c_str());
		logger->log_error(std::string(tempstr));
	}
	if(pps_source == "self")
	{
		publish_1pps = true;
		std::string pps1_topic = "/1PPS";
		pps1_pub =  n->advertise<std_msgs::Bool>(pps1_topic,10);
	}

	get_logger()->log_notice("Configuration Files Loaded.");
	return diagnostic;
}
eros::diagnostic TimeMasterNode::finish_initialization()
{
	eros::diagnostic diag = diagnostic;
	pps1_sub = n->subscribe<std_msgs::Bool>("/1PPS",1,&TimeMasterNode::PPS1_Callback,this);
	command_sub = n->subscribe<eros::command>("/command",1,&TimeMasterNode::Command_Callback,this);
	std::string device_topic = "/" + std::string(host_name) + "_master_node/srv_device";
	srv_device = n->serviceClient<eros::srv_device>(device_topic);
	last_pps1_timer = ros::Time::now();
	return diagnostic;
}
bool TimeMasterNode::run_001hz()
{
	return true;
}
bool TimeMasterNode::run_01hz()
{
	return true;
}
bool TimeMasterNode::run_01hz_noisy()
{
	std::vector<eros::diagnostic> diaglist = process->get_diagnostics();
	for (std::size_t i = 0; i < diaglist.size(); ++i)
	{
		get_logger()->log_diagnostic(diaglist.at(i));
		diagnostic_pub.publish(diaglist.at(i));
	}
	return true;
}
bool TimeMasterNode::run_1hz()
{
	process->update_diagnostic(get_resource_diagnostic());
	if(process->get_taskstate() == TASKSTATE_INITIALIZING)
	{
		logger->log_warn("Node Not Initialized Yet.  Waiting on comms with Master Node.");
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
bool TimeMasterNode::run_10hz()
{
	ready_to_arm = process->get_ready_to_arm();
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
bool TimeMasterNode::run_loop1()
{
	if(process->publish_1pps())
	{
		std_msgs::Bool msg;
		msg.data = true;
		pps1_pub.publish(msg);
	}
	return true;
}
bool TimeMasterNode::run_loop2()
{
	eros::diagnostic diag = process->update(measure_time_diff(ros::Time::now(),last_loop2_timer),ros::Time::now().toSec());
	if(diag.Level > WARN)
	{
		get_logger()->log_diagnostic(diag);
		diagnostic_pub.publish(diag);
	}
	return true;
}
bool TimeMasterNode::run_loop3()
{
	return true;
}

void TimeMasterNode::PPS1_Callback(const std_msgs::Bool::ConstPtr& msg)
{
	new_ppsmsg(msg);
}

void TimeMasterNode::Command_Callback(const eros::command::ConstPtr& t_msg)
{
	std::vector<eros::diagnostic> diaglist = process->new_commandmsg(t_msg);
	new_commandmsg_result(t_msg,diaglist);
}
bool TimeMasterNode::new_devicemsg(std::string query,eros::device t_device)
{
	if(query == "SELF")
	{
		if(t_device.DeviceName == std::string(host_name))
		{
			set_mydevice(t_device);
			process->set_mydevice(t_device);
		}
	}

	if((process->get_taskstate() == TASKSTATE_INITIALIZED))
	{
		eros::device::ConstPtr device_ptr(new eros::device(t_device));
		eros::diagnostic diag = process->new_devicemsg(device_ptr);
	}
	return true;
}

void TimeMasterNode::thread_loop()
{
	while(kill_node == false)
	{
		ros::Duration(1.0).sleep();
	}
}
void TimeMasterNode::cleanup()
{
}
/*! \brief Attempts to kill a node when an interrupt is received.
 *
 */
void signalinterrupt_handler(int sig)
{
	printf("Killing Node with Signal: %d", sig);
	kill_node = true;
	exit(0);
}
int main(int argc, char **argv) {
	signal(SIGINT, signalinterrupt_handler);
	signal(SIGTERM, signalinterrupt_handler);
	TimeMasterNode *node = new TimeMasterNode();
	bool status = node->start(argc,argv);
	std::thread thread(&TimeMasterNode::thread_loop, node);
	while((status == true) and (kill_node == false))
	{
		status = node->update(node->get_process()->get_taskstate());
	}
	node->cleanup();
	node->get_logger()->log_info("Node Finished Safely.");
	return 0;
}

