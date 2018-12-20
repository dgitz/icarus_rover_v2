#include "imu_node.h"
bool kill_node = false;
bool IMUNode::start(int argc,char **argv)
{
	bool status = false;
	process = new IMUNodeProcess();
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

icarus_rover_v2::diagnostic IMUNode::read_launchparameters()
{
	icarus_rover_v2::diagnostic diag = diagnostic;
	get_logger()->log_notice("Configuration Files Loaded.");
	return diagnostic;
}
icarus_rover_v2::diagnostic IMUNode::finish_initialization()
{
	icarus_rover_v2::diagnostic diag = diagnostic;
	pps1_sub = n->subscribe<std_msgs::Bool>("/1PPS",1,&IMUNode::PPS1_Callback,this);
	command_sub = n->subscribe<icarus_rover_v2::command>("/command",1,&IMUNode::Command_Callback,this);
	std::string device_topic = "/" + std::string(host_name) + "_master_node/srv_device";
	srv_device = n->serviceClient<icarus_rover_v2::srv_device>(device_topic);

	return diagnostic;
}
bool IMUNode::run_001hz()
{
	return true;
}
bool IMUNode::run_01hz()
{

	icarus_rover_v2::diagnostic diag = process->get_diagnostic();
	{
		get_logger()->log_diagnostic(diag);
		diagnostic_pub.publish(diag);
	}
	return true;
}
bool IMUNode::run_1hz()
{
	if((process->is_initialized() == true) and (process->is_ready() == true))
	{
		if((process->get_imus_initialized() == true) and (process->get_imus_running() == false))
		{
			std::vector<IMUNodeProcess::IMU> imus = process->get_imus();
			for(std::size_t i = 0; i < imus.size(); ++i)
			{
				IMUDriver imu_driver;
				int status = imu_driver.init(imus.at(i).connection_method,imus.at(i).device_path,imus.at(i).comm_rate);
				if(status <= 0)
				{
					diagnostic.Diagnostic_Type = SENSORS;
					diagnostic.Level = ERROR;
					diagnostic.Diagnostic_Message = INITIALIZING_ERROR;
					char tempstr[512];
					sprintf(tempstr,"Failed to Initialize IMU at: %s.  Exiting.",imus.at(i).device_path.c_str());
					diagnostic.Description = std::string(tempstr);
					logger->log_diagnostic(diagnostic);
					kill_node = 1;
				}
				else
				{
					std::string imu_topic = "/" + imus.at(i).devicename;
					ros::Publisher pub = n->advertise<icarus_rover_v2::imu>(imu_topic,10);
					imu_pubs.push_back(pub);
					if(process->set_imu_running(imus.at(i).devicename) == true)
					{
						imu_drivers.push_back(imu_driver);
						char tempstr[512];
						sprintf(tempstr,"IMU Driver Started: %s",imus.at(i).devicename.c_str());
						logger->log_notice(tempstr);
					}
					else
					{
						char tempstr[512];
						sprintf(tempstr,"IMU Driver Failed to Start: %s. Exiting.",imus.at(i).devicename.c_str());
						logger->log_error(tempstr);
						kill_node = 1;
					}
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
		{
			icarus_rover_v2::srv_device srv;
			srv.request.query = "DeviceType=IMU";
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
		}
	}
	icarus_rover_v2::diagnostic diag = process->get_diagnostic();
	//if(diag.Level >= NOTICE)
	{
		get_logger()->log_diagnostic(diag);
		diagnostic_pub.publish(diag);
	}

	return true;
}
bool IMUNode::run_10hz()
{
	icarus_rover_v2::diagnostic diag = process->get_diagnostic();
	ready_to_arm = process->get_ready_to_arm();
	diag = process->update(0.1,ros::Time::now().toSec());
	if(diag.Level > WARN)
	{
		get_logger()->log_diagnostic(diag);
		diagnostic_pub.publish(diag);
	}
	return true;
}
bool IMUNode::run_loop1()
{
	icarus_rover_v2::diagnostic diag;
	bool imus_ok = false;
	if((process->is_initialized() == true) and (process->is_ready() == true))
	{
		if((process->get_imus_initialized() == true) and (process->get_imus_running() == true))
		{
			std::vector<IMUNodeProcess::IMU> imus = process->get_imus();
			for(std::size_t i = 0; i < imus.size(); ++i)
			{
				icarus_rover_v2::imu proc_imu;
				diag = process->new_imumsg(imus.at(i).devicename,imu_drivers.at(i).update(),proc_imu);
				if(diag.Level <= NOTICE)
				{
					imu_pubs.at(i).publish(proc_imu);
				}
				else
				{
					imus_ok = true;
					diagnostic_pub.publish(diag);
				}
			}
		}
	}
	return true;
}
bool IMUNode::run_loop2()
{
	return true;
}
bool IMUNode::run_loop3()
{
	return true;
}

void IMUNode::PPS1_Callback(const std_msgs::Bool::ConstPtr& msg)
{
	new_ppsmsg(msg);
}

void IMUNode::Command_Callback(const icarus_rover_v2::command::ConstPtr& t_msg)
{
	std::vector<icarus_rover_v2::diagnostic> diaglist = process->new_commandmsg(t_msg);
	new_commandmsg_result(t_msg,diaglist);
}
bool IMUNode::new_devicemsg(std::string query,icarus_rover_v2::device t_device)
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

void IMUNode::thread_loop()
{
	while(kill_node == false)
	{
		ros::Duration(1.0).sleep();
	}
}
void IMUNode::cleanup()
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
	IMUNode *node = new IMUNode();
	bool status = node->start(argc,argv);
	std::thread thread(&IMUNode::thread_loop, node);
	while((status == true) and (kill_node == false))
	{
		status = node->update();
	}
	node->cleanup();
	node->get_logger()->log_info("Node Finished Safely.");
	return 0;
}

