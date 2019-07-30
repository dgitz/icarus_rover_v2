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
	snapshot_state = process->getDeviceSnapshotState();
	prev_snapshot_state = process->getDeviceSnapshotState();
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
	std::string srv_snapshot_topic = "/" + node_name + "/srv_snapshotstate";
	snapshot_srv = n->advertiseService(srv_snapshot_topic,&SnapshotNode::snapshot_service,this);
	std::string datalogger_snapshot_topic = "/snapshot_trigger";
	datalogger_snapshot_pub = n->advertise<std_msgs::Empty>(datalogger_snapshot_topic,1);
	std::string param_instance_mode = node_name +"/Mode";
    std::string mode;
	if(n->getParam(param_instance_mode,mode) == false)
	{
		diag = process->update_diagnostic(DATA_STORAGE,ERROR,INITIALIZING,"Missing Parameter: Mode.  Exiting.");
		logger->log_diagnostic(diag);
		return diag;
	}
	diag = process->setInstanceMode(mode);
	if(process->getInstanceMode() == SnapshotNodeProcess::InstanceMode::MASTER)
	{
		std::string topic = "/System/Snapshot/State";
		snapshotstate_pub =  n->advertise<eros::systemsnapshot_state>(topic,1);
	}
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
		if(process->getInstanceMode() == SnapshotNodeProcess::InstanceMode::MASTER)
		{	//If this is a Master,get all Devices that support the SnapshotTask
			
			eros::srv_device srv;
			srv.request.query = "ALL";
			if (srv_device.call(srv) == true)
			{
				for(std::size_t i = 0; i < srv.response.data.size(); ++i)
				{
					new_devicemsg(srv.request.query, srv.response.data.at(i));
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
	process->update_slow();
	if(process->getInstanceMode() == SnapshotNodeProcess::InstanceMode::MASTER)
	{
		snapshotstate_pub.publish(process->getROSSnapshotState());
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
	if(process->isDeviceSnapshotComplete(snapshot_path,snapshot_name))
	{
		char tempstr[512];
		sprintf(tempstr,"Device Snapshot: %s Complete.",snapshot_name.c_str());
		logger->log_notice(std::string(tempstr));
		
	}
	if((process->getInstanceMode() == SnapshotNodeProcess::InstanceMode::MASTER) and 
	   (process->getSystemSnapshotState() == SnapshotNodeProcess::SnapshotState::RUNNING))//Generate new System Snapshot
	{
		std::vector<std::string> snapshot_devices = process->get_snapshot_devices_toquery();
		for(std::size_t i = 0; i < snapshot_devices.size(); ++i)
		{
			std::string snapshot_topic = "/" + std::string(snapshot_devices.at(i)) + "_snapshot_node/srv_snapshotstate";
			srv_snapshotstate = n->serviceClient<eros::srv_snapshotstate>(snapshot_topic);
			eros::srv_snapshotstate srv;
			srv.request.query = "SNAPSHOTSTATE";
			if (srv_snapshotstate.call(srv) == true)
			{
				get_logger()->log_debug("Got snapshot state.");
				if(srv.response.state == (uint8_t)SnapshotNodeProcess::SnapshotState::READY)
				{
					//transfer via scp
					char tempstr[1024];
					std::string snap_path = srv.response.active_snapshot_path;
					std::string device = srv.response.device;
					std::string dest_directory = process->getSystemActiveSnapshotDirectory(); 
					std::string snap_name = srv.response.active_snapshot_name;
					sprintf(tempstr,"scp robot@%s:%s %s",device.c_str(),snap_path.c_str(),dest_directory.c_str());
					process->exec(tempstr,true);
					int file_match = process->count_files_indirectory(dest_directory+"/",snap_name+".zip");
					if(file_match == 1)
					{
						if(process->received_snapshot_fromdevice(device))
						{
						}
						else
						{
							char tempstr[512];
							sprintf(tempstr,"Received Snapshot from: %s But I shouldn't have.",device.c_str());
							diag = process->update_diagnostic(DATA_STORAGE,WARN,DROPPING_PACKETS,std::string(tempstr));
							logger->log_diagnostic(diag);
						}
						
					}
					else
					{
					}
					
				}
			}
			else
			{
			}
		}

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
	if(process->getInstanceMode() == SnapshotNodeProcess::InstanceMode::MASTER)
	{
		if(t_msg->Command == ROVERCOMMAND_GENERATESNAPSHOT)
		{
			if(t_msg->Option1 == ROVERCOMMAND_SNAPSHOT_NEWSNAPSHOT)
			{
				if((process->getDeviceSnapshotState() == SnapshotNodeProcess::SnapshotState::RUNNING) or 
					(process->getDeviceSnapshotState() == SnapshotNodeProcess::SnapshotState::READY) or 
					(process->getSystemSnapshotState() == SnapshotNodeProcess::SnapshotState::RUNNING))
					{

					}
				else
				{
					std_msgs::Empty empty_msg;
					datalogger_snapshot_pub.publish(empty_msg);
				}
			}
		}
	}
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
bool SnapshotNode::snapshot_service(eros::srv_snapshotstate::Request &req,
		eros::srv_snapshotstate::Response &res)
{
	if(req.query == "SNAPSHOTSTATE")
	{
		SnapshotNodeProcess::SnapshotState state = process->getDeviceSnapshotState();
		res.state = (uint8_t)state;
		res.device = process->get_mydevice().DeviceName;
		if(state == SnapshotNodeProcess::SnapshotState::READY)
		{	
			res.active_snapshot_path = process->getDeviceActiveSnapshotCompletePath();
			res.active_snapshot_name = process->getDeviceActiveSnapshotName();
			process->resetDeviceSnapshotState();
		}
		else
		{
			res.active_snapshot_path = "";
		}
		
		return true;
	}
	return false;
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
	kill_node = true;
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
