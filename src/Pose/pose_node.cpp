#include "pose_node.h"
bool kill_node = false;
bool PoseNode::start(int argc,char **argv)
{
	bool status = false;
	process = new PoseNodeProcess();
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
	diagnostic_types.push_back(SENSORS);
	diagnostic_types.push_back(POSE);
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

eros::diagnostic PoseNode::read_launchparameters()
{
	eros::diagnostic diag = diagnostic;
	get_logger()->log_notice("Configuration Files Loaded.");
	return diagnostic;
}
eros::diagnostic PoseNode::finish_initialization()
{
	eros::diagnostic diag = diagnostic;
	pps1_sub = n->subscribe<std_msgs::Bool>("/1PPS",1,&PoseNode::PPS1_Callback,this);
	command_sub = n->subscribe<eros::command>("/command",1,&PoseNode::Command_Callback,this);
	std::string device_topic = "/" + std::string(host_name) + "_master_node/srv_device";
	srv_device = n->serviceClient<eros::srv_device>(device_topic);
	return diagnostic;
}
bool PoseNode::run_001hz()
{
	return true;
}
bool PoseNode::run_01hz()
{
	return true;
}
bool PoseNode::run_01hz_noisy()
{
	std::vector<eros::diagnostic> diaglist = process->get_diagnostics();
	for (std::size_t i = 0; i < diaglist.size(); ++i)
	{
		get_logger()->log_diagnostic(diaglist.at(i));
		diagnostic_pub.publish(diaglist.at(i));
	}
	return true;
}
bool PoseNode::run_1hz()
{
	eros::diagnostic diag = diagnostic;
	diag = process->update_diagnostic(get_resource_diagnostic());
	if ((process->is_initialized() == true) and (process->is_ready() == true))
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
		{
			eros::srv_device srv;
			srv.request.query = (std::string("DeviceType=") + std::string(DEVICETYPE_IMU)).c_str();
			srv.request.filter = "*";
			if(srv_device.call(srv) == true)
			{
				if(process->set_imucount((int)srv.response.data.size()) == false)
				{
					diag = process->update_diagnostic(DATA_STORAGE,ERROR,INITIALIZING_ERROR,
					"IMU Count: " + std::to_string((int)srv.response.data.size()) + " is Invalid.");
					logger->log_diagnostic(diag);
					diagnostic_pub.publish(diag);
				}
				for(std::size_t i = 0; i < srv.response.data.size(); ++i)
				{
					new_devicemsg(srv.request.query,srv.response.data.at(i));
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
bool PoseNode::run_10hz()
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
bool PoseNode::run_loop1()
{
	std::vector<PoseNodeProcess::IMUSensor> imus = process->get_imus();
	for(std::size_t i = 0; i < imus.size(); ++i)
	{
		static tf::TransformBroadcaster br;
		br.sendTransform(tf::StampedTransform(imus.at(i).transform, ros::Time::now(), "world", imus.at(i).device.DeviceName));
		printf("%s X: %4.2f Y: %4.2f Z: %4.2f Pitch: %4.2f Roll: %4.2f\n",imus.at(i).device.DeviceName.c_str(),
				imus.at(i).imu_data.xacc.value,
				imus.at(i).imu_data.yacc.value,
				imus.at(i).imu_data.zacc.value,
				imus.at(i).orientation_pitch.value*180.0/M_PI,imus.at(i).orientation_roll.value*180.0/M_PI);
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
bool PoseNode::run_loop2()
{
	return true;
}
bool PoseNode::run_loop3()
{
	return true;
}
void PoseNode::imumsg_Callback(const eros::imu::ConstPtr& msg,const std::string &topic)
{
	process->new_imumsg(topic,msg);
}
void PoseNode::PPS1_Callback(const std_msgs::Bool::ConstPtr& msg)
{
	new_ppsmsg(msg);
}

void PoseNode::Command_Callback(const eros::command::ConstPtr& t_msg)
{
	std::vector<eros::diagnostic> diaglist = process->new_commandmsg(t_msg);
	new_commandmsg_result(t_msg,diaglist);
}
bool PoseNode::new_devicemsg(std::string query,eros::device t_device)
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
		if(diag.Level <= NOTICE)
		{
			PoseNodeProcess::IMUSensor imu = process->get_imudata(device_ptr->DeviceName);
			ros::Subscriber sub = n->subscribe<eros::imu>(imu.topicname,1,boost::bind(&PoseNode::imumsg_Callback,this,_1,imu.topicname));
			imu_subs.push_back(sub);
		}
	}
	return true;
}

void PoseNode::thread_loop()
{
	while(kill_node == false)
	{
		ros::Duration(1.0).sleep();
	}
}
void PoseNode::cleanup()
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
	PoseNode *node = new PoseNode();
	bool status = node->start(argc,argv);
	std::thread thread(&PoseNode::thread_loop, node);
	while((status == true) and (kill_node == false))
	{
		status = node->update();
	}
	node->cleanup();
	node->get_logger()->log_info("Node Finished Safely.");
	return 0;
}

