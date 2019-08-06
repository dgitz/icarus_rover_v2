#include "datalogger_node.h"
bool kill_node = false;
bool DataLoggerNode::start(int argc,char **argv)
{
	bool status = false;
	process = new DataLoggerNodeProcess();
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

eros::diagnostic DataLoggerNode::read_launchparameters()
{
	eros::diagnostic diag = diagnostic;
	return diag;
}
eros::diagnostic DataLoggerNode::finish_initialization()
{
	eros::diagnostic diag = diagnostic;
	pps1_sub = n->subscribe<std_msgs::Bool>("/1PPS",1,&DataLoggerNode::PPS1_Callback,this);
	command_sub = n->subscribe<eros::command>("/command",1,&DataLoggerNode::Command_Callback,this);
	std::string device_topic = "/" + std::string(host_name) + "_master_node/srv_device";
	srv_device = n->serviceClient<eros::srv_device>(device_topic);

	std::string param_logfile_duration = node_name +"/LogFile_Duration";
    double logfile_duration;
	if(n->getParam(param_logfile_duration,logfile_duration) == false)
	{
		diag = process->update_diagnostic(DATA_STORAGE,ERROR,INITIALIZING,"Missing Parameter: LogFile_Duration.  Exiting.");
		logger->log_diagnostic(diag);
		return diag;
	}
    std::string param_logfile_directory = node_name +"/LogFile_Directory";
    std::string logfile_directory;
	if(n->getParam(param_logfile_directory,logfile_directory) == false)
	{
		diag = process->update_diagnostic(DATA_STORAGE,ERROR,INITIALIZING_ERROR,"Missing Parameter: LogFile_Directory.  Exiting.");
		logger->log_diagnostic(diag);
		return diag;
	}
    process->set_logfileduration(logfile_duration);
    bool available = process->set_logdirectory(logfile_directory);
    if(available == false)
    {
        char tempstr[512];  
        sprintf(tempstr,"LogFile_Directory: %s Does Not Exist. Not logging anything.",logfile_directory.c_str());
		diag = process->update_diagnostic(DATA_STORAGE,WARN,DEVICE_NOT_AVAILABLE,std::string(tempstr));
		logger->log_diagnostic(diag);
    }
	std::string param_snapshot_mode = node_name +"/SnapshotMode";
    bool snapshot_mode = false;
	if(n->getParam(param_snapshot_mode,snapshot_mode) == false)
	{
		diag = process->update_diagnostic(DATA_STORAGE,WARN,NOERROR,"Missing Parameter: SnapshotMode.");
		logger->log_diagnostic(diag);
	}
    process->setSnapshotMode(snapshot_mode);
	if(snapshot_mode == false)
	{
		diag = process->update_diagnostic(DATA_STORAGE,WARN,NOERROR,"SnapshotMode Disabled.  Logging Everything.");
		logger->log_diagnostic(diag);
	}
	else
	{
		diag = process->update_diagnostic(DATA_STORAGE,NOTICE,NOERROR,"SnapshotMode Enabled.  All logs stored in RAM until Snapshot is triggered.");
		logger->log_diagnostic(diag);
	}
	get_logger()->log_notice("Configuration Files Loaded.");
	return diagnostic;
}
bool DataLoggerNode::run_001hz()
{
	return true;
}
bool DataLoggerNode::run_01hz()
{
	return true;
}
bool DataLoggerNode::run_01hz_noisy()
{
	std::vector<eros::diagnostic> diaglist = process->get_diagnostics();
	for (std::size_t i = 0; i < diaglist.size(); ++i)
	{
		get_logger()->log_diagnostic(diaglist.at(i));
		diagnostic_pub.publish(diaglist.at(i));
	}
	return true;
}
bool DataLoggerNode::run_1hz()
{
	process->update_diagnostic(get_resource_diagnostic());
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
bool DataLoggerNode::run_10hz()
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
bool DataLoggerNode::run_loop1()
{
	return true;
}
bool DataLoggerNode::run_loop2()
{
	return true;
}
bool DataLoggerNode::run_loop3()
{
	return true;
}

void DataLoggerNode::PPS1_Callback(const std_msgs::Bool::ConstPtr& msg)
{
	new_ppsmsg(msg);
}

void DataLoggerNode::Command_Callback(const eros::command::ConstPtr& t_msg)
{
	std::vector<eros::diagnostic> diaglist = process->new_commandmsg(t_msg);
	new_commandmsg_result(t_msg,diaglist);
}
bool DataLoggerNode::new_devicemsg(std::string query,eros::device t_device)
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
void DataLoggerNode::thread_loop()
{
	while(kill_node == false)
	{
		ros::Duration(1.0).sleep();
	}
}
void DataLoggerNode::cleanup()
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
void DataLoggerNode::run_logger(DataLoggerNode *node)
{
	if(node->get_process()->is_logging_enabled() == true)
	{
		rosbag::RecorderOptions opts;
		opts.record_all = true;
		opts.quiet = true;
		opts.verbose=false;
		opts.prefix = node->get_process()->get_logdirectory() + "BAG";
		opts.append_date = true;
		opts.max_duration = ros::Duration(node->get_process()->get_logfile_duration()); //30 minutes
		opts.split = true;
		opts.snapshot = node->get_process()->getSnapshotMode();
		rosbag::Recorder recorder(opts);
		recorder.run();
		node->get_logger()->log_info("Logger Finished.");
	}

	return;

}
int main(int argc, char **argv) {
	signal(SIGINT, signalinterrupt_handler);
	signal(SIGTERM, signalinterrupt_handler);
	DataLoggerNode *node = new DataLoggerNode();
	bool status = node->start(argc,argv);
    if(status == true)
    {
        std::thread thread(&DataLoggerNode::thread_loop, node);
        std::thread thread2(&DataLoggerNode::run_logger,node,node);
        while((status == true) and (kill_node == false))
        {
            status = node->update();
        }
        node->cleanup();
        thread2.join();
    }
	node->get_logger()->log_info("Node Finished Safely.");
	return 0;
}

