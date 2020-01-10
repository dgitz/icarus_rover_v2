#include "diagnostic_node.h"
bool kill_node = false;
bool DiagnosticNode::start(int argc,char **argv)
{
	bool status = false;
	process = new DiagnosticNodeProcess();
	set_basenodename(BASE_NODE_NAME);
	initialize_firmware(MAJOR_RELEASE_VERSION,MINOR_RELEASE_VERSION,BUILD_NUMBER,FIRMWARE_DESCRIPTION);
	eros::diagnostic diag = preinitialize_basenode(argc,argv);
	if(diag.Level > WARN)
	{
		return false;
	}
	diag = read_launchparameters();
	if(diagnostic.Level > WARN)
	{
		return false;
	}

	process->initialize(get_basenodename(),get_nodename(),get_hostname(),DIAGNOSTIC_SYSTEM,DIAGNOSTIC_SUBSYSTEM,DIAGNOSTIC_COMPONENT);
	std::vector<uint8_t> diagnostic_types;
	diagnostic_types.push_back(SOFTWARE);
	diagnostic_types.push_back(DATA_STORAGE);
	diagnostic_types.push_back(SYSTEM_RESOURCE);
	diagnostic_types.push_back(COMMUNICATIONS);
	diagnostic_types.push_back(REMOTE_CONTROL);
	resourcemonitor->disable_memoryleakdetection(); 
	process->enable_diagnostics(diagnostic_types);
	std::string subsystem_diagnostic_topic = "/System/Diagnostic/State";
	subsystem_diagnostic_pub =  n->advertise<eros::subsystem_diagnostic>(subsystem_diagnostic_topic,1);

	process->finish_initialization();
	diag = finish_initialization();
	if(diag.Level > WARN)
	{
		return false;
	}
	if(diagnostic.Level < WARN)
	{

		diag = process->update_diagnostic(SOFTWARE,INFO,NOERROR,"Node Configured. Initializing.");
		get_logger()->log_diagnostic(diag);
	}
	status = true;
	return status;
}

eros::diagnostic DiagnosticNode::read_launchparameters()
{
	eros::diagnostic diag = diagnostic;
	get_logger()->log_notice(__FILE__,__LINE__,"Configuration Files Loaded.");
	return diag;
}
eros::diagnostic DiagnosticNode::finish_initialization()
{
	eros::diagnostic diag = diagnostic;
	last_armedstate = ARMEDSTATUS_UNDEFINED;
	logging_initialized = false;
	pps1_sub = n->subscribe<std_msgs::Bool>("/1PPS",1,&DiagnosticNode::PPS1_Callback,this);
	command_sub = n->subscribe<eros::command>("/command",20,&DiagnosticNode::Command_Callback,this);
	std::string device_topic = "/" + std::string(host_name) + "_master_node/srv_device";
	srv_device = n->serviceClient<eros::srv_device>(device_topic);
	std::string param_ram_usage_threshold = node_name +"/RAM_usage_threshold_MB";
	int RAM_usage_threshold_MB;
	if(n->getParam(param_ram_usage_threshold,RAM_usage_threshold_MB) == false)
	{
		diag = process->update_diagnostic(DATA_STORAGE,ERROR,INITIALIZING_ERROR,"Missing Parameter: RAM_usage_threshold_MB.  Exiting.");
		logger->log_diagnostic(diag);
		return diag;
	}
	std::string param_CPU_usage_threshold = node_name + "/CPU_usage_threshold_percent";
	int CPU_usage_threshold_percent;
	if(n->getParam(param_CPU_usage_threshold,CPU_usage_threshold_percent) == false)
	{
		diag = process->update_diagnostic(DATA_STORAGE,ERROR,INITIALIZING_ERROR,"Missing Parameter: CPU_usage_threshold_percent.  Exiting.");
		logger->log_diagnostic(diag);
		return diag;
	}
	process->set_resourcethresholds(RAM_usage_threshold_MB,CPU_usage_threshold_percent);
	std::string param_log_resources_used = node_name +"/Log_Resources_Used";
	bool logging_resources_used;
	if(n->getParam(param_log_resources_used,logging_resources_used) == false)
	{
		logger->log_warn(__FILE__,__LINE__,"Missing Parameter: Log_Resources Used.  Using Default: false");
		logging_resources_used = false;
	}
	process->set_log_resources_used(logging_resources_used);

	std::string armed_state_topic = "/armed_state";
	armed_state_sub = n->subscribe<std_msgs::UInt8>(armed_state_topic,1,&DiagnosticNode::ArmedState_Callback,this);
	diag = rescan_topics();
	diag = process->update_diagnostic(diag);
	if(diagnostic.Level > NOTICE)
	{
		
		diagnostic_pub.publish(diagnostic);
		logger->log_diagnostic(diagnostic);
	}
	return diagnostic;
}
bool DiagnosticNode::run_001hz()
{

	return true;
}
bool DiagnosticNode::run_01hz()
{
	eros::diagnostic diag = rescan_topics();
	diag = process->update_diagnostic(diag);
	if(diag.Level > NOTICE)
	{
		diagnostic_pub.publish(diag);
		logger->log_diagnostic(diag);
	}
	logger->log_info(__FILE__,__LINE__,process->print_subsystem_diagnostics());
	if(measure_time_diff(ros::Time::now(),boot_time) > process->get_waitnode_bringup_time()) //Wait 20 seconds for all Nodes to start.
	{
		if(process->get_log_resources_used())
		{

			log_resources();
		}
		std::vector<eros::diagnostic> diaglist = process->check_tasks();
		for(std::size_t i = 0; i < diaglist.size(); ++i)
		{
			get_logger()->log_diagnostic(diaglist.at(i));
			diagnostic_pub.publish(diaglist.at(i));
		}
	}

	return true;
}
bool DiagnosticNode::run_01hz_noisy()
{
	std::vector<eros::diagnostic> diaglist = process->get_diagnostics();
	for (std::size_t i = 0; i < diaglist.size(); ++i)
	{
		get_logger()->log_diagnostic(diaglist.at(i));
		diagnostic_pub.publish(diaglist.at(i));
	}
	return true;
}
bool DiagnosticNode::run_1hz()
{
	if(process->get_taskstate() == TASKSTATE_RUNNING)
	{
		if(process->get_lcdavailable() == true)
		{
			if(lcd.get_initialized() == true)
			{
				std::string msg = process->build_lcdmessage();
				lcd.send(msg);
			}
		}
	}
	else if(process->get_taskstate() == TASKSTATE_INITIALIZED)
	{
		{
			eros::srv_device srv;
			srv.request.query = (std::string("DeviceType=") + std::string(DEVICETYPE_LCD)).c_str();
			if(srv_device.call(srv) == true)
			{
				if(srv.response.data.size() == 0)
				{
					process->no_connectedlcd();
				}
				else
				{
					for(std::size_t i = 0; i < srv.response.data.size(); i++)
					{
						new_devicemsg(srv.request.query,srv.response.data.at(i));
					}
				}
			}
			else
			{
			}
		}
		{
			eros::srv_device srv;
			srv.request.query = (std::string("DeviceType=") + std::string(DEVICETYPE_BATTERY)).c_str();
			if(srv_device.call(srv) == true)
			{
				if(srv.response.data.size() == 0)
				{
				}
				else
				{
					for(std::size_t i = 0; i < srv.response.data.size(); i++)
					{
						new_devicemsg(srv.request.query,srv.response.data.at(i));
					}
				}
			}
			else
			{
			}
		}
	}
	else if(process->get_taskstate() == TASKSTATE_INITIALIZING)
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
	process->update_diagnostic(get_resource_diagnostic());
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
bool DiagnosticNode::run_10hz()
{
	bool last_ready_to_arm = ready_to_arm;
	ready_to_arm = process->get_ready_to_arm();
	if(last_ready_to_arm != ready_to_arm)
	{
		logger->log_notice(__FILE__,__LINE__,"Ready To Arm Changed From: " + std::to_string(last_ready_to_arm) + " To: " + std::to_string(ready_to_arm));
	}
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
	if(process->get_armedstate() == ARMEDSTATUS_DISARMED_CANNOTARM)
	{
		logger->log_warn(__FILE__,__LINE__,"Checking Tasks More Frequently for now.");
		std::vector<eros::diagnostic> diaglist = process->check_tasks();
		for(std::size_t i = 0; i < diaglist.size(); ++i)
		{
			get_logger()->log_diagnostic(diaglist.at(i));
			diagnostic_pub.publish(diaglist.at(i));
		}
	}
	return true;
}
bool DiagnosticNode::run_loop1()
{
	subsystem_diagnostic_pub.publish(process->get_eros_subsystem_diagnostic());
	return true;
}
bool DiagnosticNode::run_loop2()
{
	return true;
}
bool DiagnosticNode::run_loop3()
{
	return true;
}

void DiagnosticNode::PPS1_Callback(const std_msgs::Bool::ConstPtr& msg)
{
	new_ppsmsg(msg);
}
void DiagnosticNode::Battery_Callback(const eros::battery::ConstPtr& msg)
{
	printf("got bat\n");
	process->set_batterylevel(msg->level_perc);
	process->set_batteryvoltage(msg->voltage);
}
void DiagnosticNode::Command_Callback(const eros::command::ConstPtr& t_msg)
{
	std::vector<eros::diagnostic> diaglist = process->new_commandmsg(t_msg);
	new_commandmsg_result(t_msg,diaglist);
}
bool DiagnosticNode::new_devicemsg(std::string query,eros::device t_device)
{
	if(query == "SELF")
	{
		if(t_device.DeviceName == std::string(host_name))
		{
			set_mydevice(t_device);
			process->set_mydevice(t_device);
		}
	}
	else if(t_device.DeviceType == DEVICETYPE_BATTERY)
	{
		std::string topic = "/" + t_device.DeviceName;
		battery_sub = n->subscribe<eros::battery>(topic,1,&DiagnosticNode::Battery_Callback,this);
	}
	else
	{
		if(process->get_taskstate() == TASKSTATE_INITIALIZED)
		{
			eros::device::ConstPtr dev_ptr(new eros::device(t_device));
			eros::diagnostic diag = process->new_devicemsg(dev_ptr);
			logger->log_diagnostic(diag);
			if(process->get_taskstate() == TASKSTATE_RUNNING)
			{
				logger->log_notice(__FILE__,__LINE__,"Device is now ready.");
				if(process->get_lcdavailable() == true)
				{
					uint8_t lcd_width,lcd_height;
					lcd_width = process->get_lcdwidth();
					lcd_height = process->get_lcdheight();
					int v = lcd.init(lcd_width,lcd_height);
					if(v <= 0)
					{
						logger->log_error(__FILE__,__LINE__,"Couldn't Initialize LCD. Exiting.");
						kill_node = 1;
					}
					else
					{
						lcd.set_color(LCDDriver::YELLOW);
						logger->log_notice(__FILE__,__LINE__,"Initialized LCD.");
					}
				}
			}
		}
	}
	return true;
}
eros::diagnostic DiagnosticNode::rescan_topics()
{
	eros::diagnostic diag = diagnostic;
	ros::master::V_TopicInfo master_topics;
	ros::master::getTopics(master_topics);
	std::vector<std::string> topics_to_add;
	std::vector<DiagnosticNodeProcess::Task> TaskList = process->get_TaskList();
	for (ros::master::V_TopicInfo::iterator it = master_topics.begin() ; it != master_topics.end(); it++)
	{
		const ros::master::TopicInfo& info = *it;
		if(info.datatype == "eros/diagnostic")
		{
			bool add_me = true;
			for(std::size_t i = 0; i < TaskList.size();i++)
			{
				if(TaskList.at(i).diagnostic_topic == info.name)
				{
					add_me = false;
					break;
				}
			}
			if(add_me == true)
			{
				topics_to_add.push_back(info.name);
			}
		}
	}
	for(std::size_t i = 0; i < topics_to_add.size(); i++)
	{
		std::string taskname = topics_to_add.at(i).substr(1,topics_to_add.at(i).find("/diagnostic")-1);;
		DiagnosticNodeProcess::Task newTask;
		newTask.Task_Name = taskname;
		newTask.diagnostic_topic = topics_to_add.at(i);
		newTask.heartbeat_topic = "/" + taskname + "/heartbeat";
		newTask.resource_topic = "/" + taskname + "/resource";
		ros::Subscriber resource_sub =
				n->subscribe<eros::resource>(newTask.resource_topic,10,boost::bind(&DiagnosticNode::resource_Callback,this,_1,newTask.resource_topic));
		resource_subs.push_back(resource_sub);
		ros::Subscriber diagnostic_sub =
				n->subscribe<eros::diagnostic>(newTask.diagnostic_topic,10,boost::bind(&DiagnosticNode::diagnostic_Callback,this,_1,newTask.diagnostic_topic));
		diagnostic_subs.push_back(diagnostic_sub);
		ros::Subscriber heartbeat_sub =
				n->subscribe<eros::heartbeat>(newTask.heartbeat_topic,10,boost::bind(&DiagnosticNode::heartbeat_Callback,this,_1,newTask.heartbeat_topic));
		heartbeat_subs.push_back(heartbeat_sub);
		process->add_Task(newTask);
		//TaskList.push_back(newTask);

	}
	char tempstr[512];
	if(topics_to_add.size() > 0)
	{
		sprintf(tempstr,"Rescanned and found %d new topics.",(int)topics_to_add.size());
	}
	else
	{
		sprintf(tempstr,"Rescanned and found no new topics.");
	}
	diag = process->update_diagnostic(SOFTWARE,INFO,NOERROR,std::string(tempstr));
	logger->log_diagnostic(diag);
	return diag;
}
void DiagnosticNode::heartbeat_Callback(__attribute__((unused)) const eros::heartbeat::ConstPtr& msg,const std::string &topicname)
{
	process->new_heartbeatmsg(topicname);
}
void DiagnosticNode::ArmedState_Callback(const std_msgs::UInt8::ConstPtr& msg)
{
	process->new_armedstatemsg(msg->data);
	if(msg->data != last_armedstate)
	{
		if(process->get_lcdavailable() == true)
		{
			switch(msg->data)
			{
			case ARMEDSTATUS_UNDEFINED:
				lcd.set_color(LCDDriver::RED);
				break;
			case ARMEDSTATUS_DISARMING:
				lcd.set_color(LCDDriver::GREEN);
				break;
			case ARMEDSTATUS_DISARMED_CANNOTARM:
				lcd.set_color(LCDDriver::YELLOW);
				break;
			case ARMEDSTATUS_DISARMED:
				lcd.set_color(LCDDriver::GREEN);
				break;
			case ARMEDSTATUS_ARMING:
				if(process->get_RobotUnderRemoteControl() == true)
				{
					lcd.set_color(LCDDriver::PURPLE);
				}
				else
				{
					lcd.set_color(LCDDriver::BLUE);
				}
				break;
			case ARMEDSTATUS_ARMED:
				if(process->get_RobotUnderRemoteControl() == true)
				{
					lcd.set_color(LCDDriver::PURPLE);
				}
				else
				{
					lcd.set_color(LCDDriver::BLUE);
				}
				break;
			}
		}
	}
	last_armedstate = msg->data;
}
bool DiagnosticNode::log_resources()
{
	std::vector<DiagnosticNodeProcess::Task> TaskList = process->get_TaskList();
	std::vector<DiagnosticNodeProcess::DeviceResourceAvailable> DeviceResourceAvailableList = process->get_DeviceResourceAvailableList();
	std::string ram_used_file_path = "/home/robot/var/log/ram_used.csv";
	std::string cpu_used_file_path = "/home/robot/var/log/cpu_used.csv";
	std::string ram_free_file_path = "/home/robot/var/log/ram_free.csv";
	std::string cpu_free_file_path = "/home/robot/var/log/cpu_free.csv";
	if(logging_initialized == false)
	{
		ram_used_file.open(ram_used_file_path.c_str(),ios::out);
		if(ram_used_file.is_open() == false)
		{
			logger->log_error(__FILE__,__LINE__,"Couldn't open ram_used.csv file.  Exiting.");
			kill_node = 1;
		}
		else
		{
			ram_used_file << "Time (s),";
			for(std::size_t i = 0; i < TaskList.size();i++)
			{
				ram_used_file << TaskList.at(i).Task_Name << ",";
			}
			ram_used_file << endl;
		}
		ram_used_file.close();
		cpu_used_file.open(cpu_used_file_path.c_str(),ios::out);
		if(cpu_used_file.is_open() == false)
		{
			logger->log_error(__FILE__,__LINE__,"Couldn't open cpuused.csv file.  Exiting.");
			kill_node = 1;
		}
		else
		{
			cpu_used_file << "Time (s),";
			for(std::size_t i = 0; i < TaskList.size();i++)
			{
				cpu_used_file << TaskList.at(i).Task_Name << ",";
			}
			cpu_used_file << endl;
		}
		cpu_used_file.close();
		ram_free_file.open(ram_free_file_path.c_str(),ios::out);
		if(ram_free_file.is_open() == false)
		{
			logger->log_error(__FILE__,__LINE__,"Couldn't open ram_free.csv file.  Exiting.");
		}
		else
		{
			ram_free_file << "Time (s),";

			for(std::size_t i = 0; i < DeviceResourceAvailableList.size();i++)
			{
				ram_free_file << DeviceResourceAvailableList.at(i).Device_Name << ",";
			}
			ram_free_file << endl;
		}
		ram_free_file.close();

		cpu_free_file.open(cpu_free_file_path.c_str(),ios::out);
		if(cpu_free_file.is_open() == false)
		{
			logger->log_error(__FILE__,__LINE__,"Couldn't open cpu_free.csv file.  Exiting.");
			kill_node = 1;
		}
		else
		{
			cpu_free_file << "Time (s),";
			for(std::size_t i = 0; i < DeviceResourceAvailableList.size();i++)
			{
				cpu_free_file << DeviceResourceAvailableList.at(i).Device_Name << ",";
			}
			cpu_free_file << endl;
		}
		cpu_free_file.close();

		logging_initialized = true;
	}
	else
	{
		double mtime = measure_time_diff(ros::Time::now(),boot_time);
		ram_used_file.open(ram_used_file_path.c_str(),ios::app);
		if(ram_used_file.is_open() == false)
		{
			logger->log_error(__FILE__,__LINE__,"Couldn't open ram_used.csv file.  Exiting.");
			kill_node = 1;
		}
		else
		{
			ram_used_file << mtime << ",";
			for(std::size_t i = 0; i < TaskList.size();i++)
			{
				ram_used_file << TaskList.at(i).RAM_MB << ",";
			}
			ram_used_file << endl;
		}
		ram_used_file.close();
		cpu_used_file.open(cpu_used_file_path.c_str(),ios::app);
		if(cpu_used_file.is_open() == false)
		{
			logger->log_error(__FILE__,__LINE__,"Couldn't open cpu_used.csv file.  Exiting.");
			kill_node = 1;
		}
		else
		{
			cpu_used_file << mtime << ",";
			for(std::size_t i = 0; i < TaskList.size();i++)
			{
				cpu_used_file << TaskList.at(i).CPU_Perc << ",";
			}
			cpu_used_file << endl;
		}
		cpu_used_file.close();

		ram_free_file.open(ram_free_file_path.c_str(),ios::app);
		if(ram_free_file.is_open() == false)
		{
			logger->log_error(__FILE__,__LINE__,"Couldn't open ram_free.csv file.  Exiting.");
			kill_node = 1;
		}
		else
		{
			ram_free_file << mtime << ",";
			for(std::size_t i = 0; i < DeviceResourceAvailableList.size();i++)
			{
				ram_free_file << DeviceResourceAvailableList.at(i).RAM_Mb_Available << ",";
			}
			ram_free_file << endl;
		}
		ram_free_file.close();

		cpu_free_file.open(cpu_free_file_path.c_str(),ios::app);
		if(cpu_free_file.is_open() == false)
		{
			logger->log_error(__FILE__,__LINE__,"Couldn't open cpu_free.csv file.  Exiting.");
			kill_node = 1;
		}
		else
		{
			cpu_free_file << mtime << ",";
			for(std::size_t i = 0; i < DeviceResourceAvailableList.size();i++)
			{
				cpu_free_file << DeviceResourceAvailableList.at(i).CPU_Perc_Available << ",";
			}
			cpu_free_file << endl;
		}
		cpu_free_file.close();

	}
	return true;
}
void DiagnosticNode::resource_Callback(const eros::resource::ConstPtr& msg,const std::string &topicname)
{
	process->new_resourcemsg(topicname,msg);
}
void DiagnosticNode::diagnostic_Callback(const eros::diagnostic::ConstPtr& msg,const std::string &topicname)
{
	if(msg->Diagnostic_Type == 0)
	{
		char tempstr[512];
		sprintf(tempstr,"Improper Diagnostic Formatting: Diag: topic: %s type: %d dev: %s level: %d",
		topicname.c_str(),
		msg->Diagnostic_Type,
		msg->DeviceName.c_str(),
		msg->Level);
		logger->log_error(__FILE__,__LINE__,tempstr);
	}

	process->new_diagnosticmsg(topicname,msg);

}
void DiagnosticNode::thread_loop()
{
	while(kill_node == false)
	{
		ros::Duration(1.0).sleep();
	}
}
void DiagnosticNode::cleanup()
{
	if(process->get_lcdavailable() == true)
	{
		lcd.set_color(LCDDriver::RED);
		lcd.send("SHUTTING DOWN");
	}
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
	DiagnosticNode *node = new DiagnosticNode();
	bool status = node->start(argc,argv);
	std::thread thread(&DiagnosticNode::thread_loop, node);
	while((status == true) and (kill_node == false))
	{
		status = node->update(node->get_process()->get_taskstate());
	}
	node->cleanup();
	node->get_logger()->log_info(__FILE__,__LINE__,"Node Finished Safely.");
	return 0;
}

