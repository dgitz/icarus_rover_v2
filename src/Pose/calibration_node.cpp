#include "calibration_node.h"
bool kill_node = false;
bool CalibrationNode::start(int argc,char **argv)
{
	bool status = false;
	process = new CalibrationNodeProcess();
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

eros::diagnostic CalibrationNode::read_launchparameters()
{
	eros::diagnostic diag = diagnostic;
	get_logger()->log_notice(__FILE__,__LINE__,"Configuration Files Loaded.");
	return diagnostic;
}
eros::diagnostic CalibrationNode::finish_initialization()
{
	eros::diagnostic diag = diagnostic;
	pps1_sub = n->subscribe<std_msgs::Bool>("/1PPS",1,&CalibrationNode::PPS1_Callback,this);
	command_sub = n->subscribe<eros::command>("/command",1,&CalibrationNode::Command_Callback,this);
	std::string device_topic = "/" + std::string(host_name) + "_master_node/srv_device";
	srv_device = n->serviceClient<eros::srv_device>(device_topic);
	imu_sub = n->subscribe<eros::imu>("/RightIMU",1,&CalibrationNode::IMU_Callback,this);
	std::string marker_topic = "/RightIMU_mag_marker";
	imu_mag_marker_pub = n->advertise<visualization_msgs::Marker>(marker_topic, 1);

	return diagnostic;
}
void CalibrationNode::IMU_Callback(const eros::imu::ConstPtr& t_msg)
{
	if(point_list.size() > 1000000)
	{
		point_list.erase (point_list.begin());
	}
	geometry_msgs::Point p;
	p.x = t_msg->xmag.value;
	p.y = t_msg->ymag.value;
	p.z = t_msg->zmag.value;
	point_list.push_back(p);

}
bool CalibrationNode::run_001hz()
{
	return true;
}
bool CalibrationNode::run_01hz()
{
	return true;
}
bool CalibrationNode::run_01hz_noisy()
{
	std::vector<eros::diagnostic> diaglist = process->get_diagnostics();
	for (std::size_t i = 0; i < diaglist.size(); ++i)
	{
		get_logger()->log_diagnostic(diaglist.at(i));
		diagnostic_pub.publish(diaglist.at(i));
	}
	return true;
}
bool CalibrationNode::run_1hz()
{
	visualization_msgs::Marker points;
	points.header.frame_id = "world";
	points.header.stamp = ros::Time();
	points.ns = "IMU";
	points.id = 0;
	points.type = visualization_msgs::Marker::POINTS;
	points.action = visualization_msgs::Marker::ADD;
	points.points = point_list;
	points.pose.orientation.x = 0.0;
	points.pose.orientation.y = 0.0;
	points.pose.orientation.z = 0.0;
	points.pose.orientation.w = 1.0;
	points.scale.x = .1;
	points.scale.y = .1;
	points.scale.z = 0.1;
	points.color.a = 1.0; // Don't forget to set the alpha!
	points.color.r = 0.0;
	points.color.g = 1.0;
	points.color.b = 0.0;
	imu_mag_marker_pub.publish( points );
	process->update_diagnostic(get_resource_diagnostic());
	if(process->get_taskstate() == TASKSTATE_RUNNING)
	{
	}
	else if(process->get_taskstate() == TASKSTATE_INITIALIZED)
	{
	}
	else if (process->get_taskstate() == TASKSTATE_INITIALIZING)
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
bool CalibrationNode::run_10hz()
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
bool CalibrationNode::run_loop1()
{
	return true;
}
bool CalibrationNode::run_loop2()
{
	return true;
}
bool CalibrationNode::run_loop3()
{
	return true;
}

void CalibrationNode::PPS1_Callback(const std_msgs::Bool::ConstPtr& msg)
{
	new_ppsmsg(msg);
}

void CalibrationNode::Command_Callback(const eros::command::ConstPtr& t_msg)
{
	std::vector<eros::diagnostic> diaglist = process->new_commandmsg(t_msg);
	new_commandmsg_result(t_msg,diaglist);
}
bool CalibrationNode::new_devicemsg(std::string query,eros::device t_device)
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
void CalibrationNode::thread_loop()
{
	while(kill_node == false)
	{
		ros::Duration(1.0).sleep();
	}
}
void CalibrationNode::cleanup()
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
	CalibrationNode *node = new CalibrationNode();
	bool status = node->start(argc,argv);
	std::thread thread(&CalibrationNode::thread_loop, node);
	while((status == true) and (kill_node == false))
	{
		status = node->update(node->get_process()->get_taskstate());
	}
	node->get_logger()->log_info(__FILE__,__LINE__,"Node Finished Safely.");
	node->cleanup();
	thread.detach();
	return 0;
}

