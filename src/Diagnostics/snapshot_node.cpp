#include "snapshot_node.h"
bool kill_node = false;
bool SnapshotNode::start(int argc, char **argv)
{
	bool status = false;
	process = new SnapshotNodeProcess();
	set_basenodename(BASE_NODE_NAME);
	initialize_firmware(MAJOR_RELEASE_VERSION, MINOR_RELEASE_VERSION, BUILD_NUMBER, FIRMWARE_DESCRIPTION);
	diagnostic = preinitialize_basenode(argc, argv);
	if (diagnostic.Level > WARN)
	{
		return false;
	}
	diagnostic = read_launchparameters();
	if (diagnostic.Level > WARN)
	{
		return false;
	}

	process->initialize(get_basenodename(), get_nodename(), get_hostname(), DIAGNOSTIC_SYSTEM, DIAGNOSTIC_SUBSYSTEM, DIAGNOSTIC_COMPONENT);
	process->set_config_filepaths("/home/robot/config/SnapshotConfig.xml");
	std::vector<uint8_t> diagnostic_types;
	diagnostic_types.push_back(SOFTWARE);
	diagnostic_types.push_back(DATA_STORAGE);
	diagnostic_types.push_back(SYSTEM_RESOURCE);
	process->enable_diagnostics(diagnostic_types);
	process->finish_initialization();
	diagnostic = finish_initialization();
	if (diagnostic.Level > WARN)
	{
		return false;
	}
	if (diagnostic.Level < WARN)
	{
		diagnostic.Diagnostic_Type = NOERROR;
		diagnostic.Level = INFO;
		diagnostic.Diagnostic_Message = NOERROR;
		diagnostic.Description = "Node Configured.  Initializing.";
		get_logger()->log_diagnostic(diagnostic);
	}
	snapshot_state = process->getSnapshotState();
	prev_snapshot_state = process->getSnapshotState();
	status = true;
	return status;
}

eros::diagnostic SnapshotNode::read_launchparameters()
{
	eros::diagnostic diag = diagnostic;
	get_logger()->log_notice("Configuration Files Loaded.");
	return diagnostic;
}
eros::diagnostic SnapshotNode::finish_initialization()
{
	eros::diagnostic diag = diagnostic;
	pps1_sub = n->subscribe<std_msgs::Bool>("/1PPS", 1, &SnapshotNode::PPS1_Callback, this);
	command_sub = n->subscribe<eros::command>("/command", 1, &SnapshotNode::Command_Callback, this);
	std::string device_topic = "/" + std::string(host_name) + "_master_node/srv_device";
	srv_device = n->serviceClient<eros::srv_device>(device_topic);

	std::string param_instance_mode = node_name +"/Mode";
    std::string mode;
	if(n->getParam(param_instance_mode,mode) == false)
	{
		diag = process->update_diagnostic(DATA_STORAGE,ERROR,INITIALIZING,"Missing Parameter: Mode.  Exiting.");
		logger->log_diagnostic(diag);
		return diag;
	}
	diag = process->setInstanceMode(mode);
	printf("Diag: %s\n",diag.Description.c_str());
	if(diag.Level > NOTICE)
	{
		logger->log_diagnostic(diag);
	}
	return diag;
}
bool SnapshotNode::run_001hz()
{
	
	return true;
}
bool SnapshotNode::run_01hz()
{
	return true;
}
bool SnapshotNode::run_01hz_noisy()
{
	std::vector<eros::diagnostic> diaglist = process->get_diagnostics();
	for (std::size_t i = 0; i < diaglist.size(); ++i)
	{
		get_logger()->log_diagnostic(diaglist.at(i));
		diagnostic_pub.publish(diaglist.at(i));
	}
	return true;
}
bool SnapshotNode::run_1hz()
{
	process->update_diagnostic(get_resource_diagnostic());
	if ((process->is_initialized() == true) and (process->is_ready() == true))
	{
	}
	else if ((process->is_ready() == false) and (process->is_initialized() == true))
	{
	}
	else if (process->is_initialized() == false)
	{
		{
			eros::srv_device srv;
			srv.request.query = "SELF";
			if (srv_device.call(srv) == true)
			{
				if (srv.response.data.size() != 1)
				{

					get_logger()->log_error("Got unexpected device message.");
				}
				else
				{
					new_devicemsg(srv.request.query, srv.response.data.at(0));
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
bool SnapshotNode::run_10hz()
{
	ready_to_arm = process->get_ready_to_arm();
	eros::diagnostic diag = process->update(0.1, ros::Time::now().toSec());
	if (diag.Level > WARN)
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
	prev_snapshot_state = snapshot_state;
	snapshot_state = process->getDeviceSnapshotState();
	if(prev_snapshot_state != snapshot_state)
	{
		char tempstr[512];
		sprintf(tempstr,"Snapshot State Changed: %s\n",
			process->map_state_tostring(snapshot_state).c_str());
		logger->log_notice(std::string(tempstr));
	}
	std::string snapshot_path,snapshot_name;
	if(process->isSnapshotComplete(snapshot_path,snapshot_name))
	{
		char tempstr[512];
		sprintf(tempstr,"Snapshot: %s Complete.",snapshot_name.c_str());
		logger->log_notice(std::string(tempstr));
	}
	return true;
}
bool SnapshotNode::run_loop1()
{
	return true;
}
bool SnapshotNode::run_loop2()
{
	return true;
}
bool SnapshotNode::run_loop3()
{
	return true;
}

void SnapshotNode::PPS1_Callback(const std_msgs::Bool::ConstPtr &msg)
{
	new_ppsmsg(msg);
}

void SnapshotNode::Command_Callback(const eros::command::ConstPtr &t_msg)
{
	std::vector<eros::diagnostic> diaglist = process->new_commandmsg(t_msg);
	new_commandmsg_result(t_msg, diaglist);
}
bool SnapshotNode::new_devicemsg(std::string query, eros::device t_device)
{
	if (query == "SELF")
	{
		if (t_device.DeviceName == std::string(host_name))
		{
			set_mydevice(t_device);
			process->set_mydevice(t_device);
		}
	}

	if ((process->is_initialized() == true))
	{
		eros::device::ConstPtr device_ptr(new eros::device(t_device));
		eros::diagnostic diag = process->new_devicemsg(device_ptr);
	}
	return true;
}
void SnapshotNode::thread_loop()
{
	while (kill_node == false)
	{
		ros::Duration(1.0).sleep();
	}
}
void SnapshotNode::cleanup()
{
	base_cleanup();
	get_logger()->log_info("Node Finished Safely.");
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
int main(int argc, char **argv)
{
	signal(SIGINT, signalinterrupt_handler);
	signal(SIGTERM, signalinterrupt_handler);
	SnapshotNode *node = new SnapshotNode();
	bool status = node->start(argc, argv);
	std::thread thread(&SnapshotNode::thread_loop, node);
	while ((status == true) and (kill_node == false))
	{
		status = node->update();
	}
	node->cleanup();
	thread.detach();
	return 0;
}
