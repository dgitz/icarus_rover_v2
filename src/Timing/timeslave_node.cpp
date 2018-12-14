#include "timeslave_node.h"
bool kill_node = false;
bool TimeSlaveNode::start(int argc,char **argv)
{
	bool status = false;
	process = new TimeSlaveNodeProcess();
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

icarus_rover_v2::diagnostic TimeSlaveNode::read_launchparameters()
{
	icarus_rover_v2::diagnostic diag = diagnostic;
	get_logger()->log_notice("Configuration Files Loaded.");
	return diagnostic;
}
icarus_rover_v2::diagnostic TimeSlaveNode::finish_initialization()
{
	icarus_rover_v2::diagnostic diag = diagnostic;
	pps1_sub = n->subscribe<std_msgs::Bool>("/1PPS",1,&TimeSlaveNode::PPS1_Callback,this);
	command_sub = n->subscribe<icarus_rover_v2::command>("/command",1,&TimeSlaveNode::Command_Callback,this);
	std::string device_topic = "/" + std::string(host_name) + "_master_node/srv_device";
	srv_device = n->serviceClient<icarus_rover_v2::srv_device>(device_topic);
	std::string timesyncinfo_topic = "/" + process->get_mydevice().DeviceName + "/timesyncinfo";
	timesyncinfo_pub = n->advertise<icarus_rover_v2::timesyncinfo>(timesyncinfo_topic,1);
	return diagnostic;
}
bool TimeSlaveNode::run_001hz()
{
	return true;
}
bool TimeSlaveNode::run_01hz()
{
	icarus_rover_v2::diagnostic diag = process->update_timeservers();
	{
		get_logger()->log_diagnostic(diag);
		diagnostic_pub.publish(diag);
	}
	timesyncinfo_pub.publish(process->get_timesyncinfo());
	print_timeserverinfo();

	return true;
}
bool TimeSlaveNode::run_1hz()
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
	{
		get_logger()->log_diagnostic(diag);
		diagnostic_pub.publish(diag);
	}

	return true;
}
bool TimeSlaveNode::run_10hz()
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
bool TimeSlaveNode::run_loop1()
{
	return true;
}
bool TimeSlaveNode::run_loop2()
{
	return true;
}
bool TimeSlaveNode::run_loop3()
{
	return true;
}

void TimeSlaveNode::PPS1_Callback(const std_msgs::Bool::ConstPtr& msg)
{
	new_ppsmsg(msg);
}

void TimeSlaveNode::Command_Callback(const icarus_rover_v2::command::ConstPtr& t_msg)
{
	std::vector<icarus_rover_v2::diagnostic> diaglist = process->new_commandmsg(t_msg);
	new_commandmsg_result(t_msg,diaglist);
}
bool TimeSlaveNode::new_devicemsg(std::string query,icarus_rover_v2::device t_device)
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
void TimeSlaveNode::print_timeserverinfo()
{
	std::vector<TimeSlaveNodeProcess::TimeServer> time_servers = process->get_timeservers();
	for(std::size_t i = 0; i < time_servers.size(); i++)
	{
		char tempstr[512];
		sprintf(tempstr,"[%d] %s Cnt: %ld T:%4.2f Delay: %4.2f Offset: %4.2f Jitter: %4.2f",
				(int)i,time_servers.at(i).name.c_str(),time_servers.at(i).update_count,
				time_servers.at(i).updated_time,time_servers.at(i).delay,time_servers.at(i).offset,time_servers.at(i).jitter);
		switch(time_servers.at(i).level)
		{
		case INFO:
			get_logger()->log_info(tempstr);
			break;
		case NOTICE:
			get_logger()->log_notice(tempstr);
			break;
		case WARN:
			get_logger()->log_info(tempstr);
			break;
		case ERROR:
			get_logger()->log_error(tempstr);
			break;
		case FATAL:
			get_logger()->log_fatal(tempstr);
			break;

		}

	}

}
void TimeSlaveNode::thread_loop()
{
	while(kill_node == false)
	{
		ros::Duration(1.0).sleep();
	}
}
void TimeSlaveNode::cleanup()
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
	TimeSlaveNode *node = new TimeSlaveNode();
	bool status = node->start(argc,argv);
	std::thread thread(&TimeSlaveNode::thread_loop, node);
	while((status == true) and (kill_node == false))
	{
		status = node->update();
	}
	node->cleanup();
	node->get_logger()->log_info("Node Finished Safely.");
	return 0;
}

