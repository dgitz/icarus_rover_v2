#include "diagnostic_node.h"
//Start User Code: Firmware Definition
#define DIAGNOSTICNODE_MAJOR_RELEASE 2
#define DIAGNOSTICNODE_MINOR_RELEASE 2
#define DIAGNOSTICNODE_BUILD_NUMBER 2
//End User Code: Firmware Definition
//Start User Code: Functions
icarus_rover_v2::diagnostic rescan_topics(icarus_rover_v2::diagnostic diag)
{
	int found_new_topics = 0;
	ros::master::V_TopicInfo master_topics;
	ros::master::getTopics(master_topics);
	std::vector<std::string> topics_to_add;
	for (ros::master::V_TopicInfo::iterator it = master_topics.begin() ; it != master_topics.end(); it++)
	{
		const ros::master::TopicInfo& info = *it;
		if(info.datatype == "icarus_rover_v2/diagnostic")
		{
			bool add_me = true;
			for(int i = 0; i < TaskList.size();i++)
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
	for(int i = 0; i < topics_to_add.size(); i++)
	{
		std::string taskname = topics_to_add.at(i).substr(1,topics_to_add.at(i).find("/diagnostic")-1);;
		Task newTask;
		newTask.Task_Name = taskname;
		newTask.diagnostic_topic = topics_to_add.at(i);
		newTask.heartbeat_topic = "/" + taskname + "/heartbeat";
		newTask.resource_topic = "/" + taskname + "/resource";
		newTask.CPU_Perc = 0;
		newTask.PID = -1;
		newTask.RAM_MB = 0;
		newTask.last_diagnostic_level = WARN;
		ros::Subscriber resource_sub = n->subscribe<icarus_rover_v2::resource>(newTask.resource_topic,1000,boost::bind(resource_Callback,_1,newTask.resource_topic));
		newTask.resource_sub = resource_sub;
		ros::Subscriber diagnostic_sub = n->subscribe<icarus_rover_v2::diagnostic>(newTask.diagnostic_topic,1000,boost::bind(diagnostic_Callback,_1,newTask.diagnostic_topic));
		newTask.diagnostic_sub = diagnostic_sub;
		ros::Subscriber heartbeat_sub = n->subscribe<icarus_rover_v2::heartbeat>(newTask.heartbeat_topic,1000,boost::bind(heartbeat_Callback,_1,newTask.heartbeat_topic));
		newTask.heartbeat_sub = heartbeat_sub;
		newTask.last_diagnostic_received = ros::Time::now();
		newTask.last_heartbeat_received = ros::Time::now();
		newTask.last_resource_received = ros::Time::now();
		TaskList.push_back(newTask);

	}

	diag.Diagnostic_Type = SOFTWARE;
	diag.Level = INFO;
	diag.Diagnostic_Message = NOERROR;

	char tempstr[512];
	if(topics_to_add.size() > 0)
	{
		sprintf(tempstr,"Rescanned and found %d new topics.",topics_to_add.size());
	}
	else
	{
		sprintf(tempstr,"Rescanned and found no new topics.");
	}
	logger->log_info(tempstr);
	diag.Description = tempstr;
	return diag;
}
void heartbeat_Callback(const icarus_rover_v2::heartbeat::ConstPtr& msg,const std::string &topicname)
{
	for(int i = 0; i < TaskList.size(); i++)
	{
		if(	TaskList.at(i).heartbeat_topic == topicname)
		{
			TaskList.at(i).last_heartbeat_received = msg->stamp;
		}
	}

}
bool log_resources()
{
	std::string ram_used_file_path = "/home/robot/logs/ram_used.csv";
	std::string cpu_used_file_path = "/home/robot/logs/cpu_used.csv";
	std::string ram_free_file_path = "/home/robot/logs/ram_free.csv";
	std::string cpu_free_file_path = "/home/robot/logs/cpu_free.csv";
	if(logging_initialized == false)
	{
		ram_used_file.open(ram_used_file_path.c_str(),ios::out);
		if(ram_used_file.is_open() == false)
		{
			logger->log_error("Couldn't open ram_used.csv file.  Exiting.");
		}
		else
		{
			ram_used_file << "Time (s),";
			for(int i = 0; i < TaskList.size();i++)
			{
				ram_used_file << TaskList.at(i).Task_Name << ",";
			}
			ram_used_file << endl;
		}
		ram_used_file.close();
		cpu_used_file.open(cpu_used_file_path.c_str(),ios::out);
		if(cpu_used_file.is_open() == false)
		{
			logger->log_error("Couldn't open cpuused.csv file.  Exiting.");
		}
		else
		{
			cpu_used_file << "Time (s),";
			for(int i = 0; i < TaskList.size();i++)
			{
				cpu_used_file << TaskList.at(i).Task_Name << ",";
			}
			cpu_used_file << endl;
		}
		cpu_used_file.close();
		ram_free_file.open(ram_free_file_path.c_str(),ios::out);
		if(ram_free_file.is_open() == false)
		{
			logger->log_error("Couldn't open ram_free.csv file.  Exiting.");
		}
		else
		{
			ram_free_file << "Time (s),";
			for(int i = 0; i < DeviceResourceAvailableList.size();i++)
			{
				ram_free_file << DeviceResourceAvailableList.at(i).Device_Name << ",";
			}
			ram_free_file << endl;
		}
		ram_free_file.close();

		cpu_free_file.open(cpu_free_file_path.c_str(),ios::out);
		if(cpu_free_file.is_open() == false)
		{
			logger->log_error("Couldn't open cpu_free.csv file.  Exiting.");
		}
		else
		{
			cpu_free_file << "Time (s),";
			for(int i = 0; i < DeviceResourceAvailableList.size();i++)
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
			logger->log_error("Couldn't open ram_used.csv file.  Exiting.");
		}
		else
		{
			ram_used_file << mtime << ",";
			for(int i = 0; i < TaskList.size();i++)
			{
				ram_used_file << TaskList.at(i).RAM_MB << ",";
			}
			ram_used_file << endl;
		}
		ram_used_file.close();
		cpu_used_file.open(cpu_used_file_path.c_str(),ios::app);
		if(cpu_used_file.is_open() == false)
		{
			logger->log_error("Couldn't open cpu_used.csv file.  Exiting.");
		}
		else
		{
			cpu_used_file << mtime << ",";
			for(int i = 0; i < TaskList.size();i++)
			{
				cpu_used_file << TaskList.at(i).CPU_Perc << ",";
			}
			cpu_used_file << endl;
		}
		cpu_used_file.close();

		ram_free_file.open(ram_free_file_path.c_str(),ios::app);
		if(ram_free_file.is_open() == false)
		{
			logger->log_error("Couldn't open ram_free.csv file.  Exiting.");
		}
		else
		{
			ram_free_file << mtime << ",";
			for(int i = 0; i < DeviceResourceAvailableList.size();i++)
			{
				ram_free_file << DeviceResourceAvailableList.at(i).RAM_Mb_Available << ",";
			}
			ram_free_file << endl;
		}
		ram_free_file.close();

		cpu_free_file.open(cpu_free_file_path.c_str(),ios::app);
		if(cpu_free_file.is_open() == false)
		{
			logger->log_error("Couldn't open cpu_free.csv file.  Exiting.");
		}
		else
		{
			cpu_free_file << mtime << ",";
			for(int i = 0; i < DeviceResourceAvailableList.size();i++)
			{
				cpu_free_file << DeviceResourceAvailableList.at(i).CPU_Perc_Available << ",";
			}
			cpu_free_file << endl;
		}
		cpu_free_file.close();

	}
	return true;
}
bool check_tasks()
{
	std::vector<Task> TasksToCheck = TaskList;
	int task_ok_counter = 0;
	for(int i = 0; i < TasksToCheck.size(); i++)
	{
		bool task_ok = true;
		Task newTask = TasksToCheck.at(i);
		if(newTask.CPU_Perc > CPU_usage_threshold_percent)
		{
			task_ok = false;
			char tempstr[512];
			sprintf(tempstr,"Task: %s is using high CPU resource: %d/%d %",
					newTask.Task_Name.c_str(),newTask.CPU_Perc,CPU_usage_threshold_percent);
			logger->log_warn(tempstr);
			diagnostic_status.Diagnostic_Message = HIGH_RESOURCE_USAGE;
			diagnostic_status.Level = WARN;
			diagnostic_status.Description = tempstr;
			diagnostic_pub.publish(diagnostic_status);
		}
		if(newTask.RAM_MB > RAM_usage_threshold_MB)
		{
			task_ok = false;
			char tempstr[512];
			sprintf(tempstr,"Task: %s is using high RAM resource: %ld/%ld (MB)",
					newTask.Task_Name.c_str(),newTask.RAM_MB,RAM_usage_threshold_MB);
			logger->log_warn(tempstr);
			diagnostic_status.Diagnostic_Message = HIGH_RESOURCE_USAGE;
			diagnostic_status.Level = WARN;
			diagnostic_status.Description = tempstr;
			diagnostic_pub.publish(diagnostic_status);
		}
		/*
		if(newTask.PID <= 0)
		{
			task_ok = false;
			char tempstr[512];
			sprintf(tempstr,"Task: %s does not have a valid PID.",newTask.Task_Name.c_str());
			logger->log_warn(tempstr);
			diagnostic_status.Diagnostic_Message = HIGH_RESOURCE_USAGE;
			diagnostic_status.Level = WARN;
			diagnostic_status.Description = tempstr;
			diagnostic_pub.publish(diagnostic_status);
		}
		double resource_time_duration = measure_time_diff(ros::Time::now(),newTask.last_resource_received);
		//printf("Task: %s Resource Time: %f\r\n",newTask.Task_Name.c_str(),resource_time_duration);
		if( resource_time_duration > 5.0)
		{
			task_ok = false;
			char tempstr[512];
			sprintf(tempstr,"Task: %s has not reported resources used in %.1f seconds",newTask.Task_Name.c_str(),resource_time_duration);
			logger->log_warn(tempstr);
			diagnostic_status.Diagnostic_Message = DEVICE_NOT_AVAILABLE;
			diagnostic_status.Level = WARN;
			diagnostic_status.Description = tempstr;
			diagnostic_pub.publish(diagnostic_status);
		}

		double diagnostic_time_duration = measure_time_diff(ros::Time::now(),newTask.last_diagnostic_received);
		if( resource_time_duration > 5.0)
		{
			task_ok = false;
			char tempstr[512];
			sprintf(tempstr,"Task: %s has not reported diagnostics in %.1f seconds",newTask.Task_Name.c_str(),diagnostic_time_duration);
			logger->log_warn(tempstr);
			diagnostic_status.Diagnostic_Message = DEVICE_NOT_AVAILABLE;
			diagnostic_status.Level = WARN;
			diagnostic_status.Description = tempstr;
			diagnostic_pub.publish(diagnostic_status);
		}
		*/
		double heartbeat_time_duration = measure_time_diff(ros::Time::now(),newTask.last_heartbeat_received);
		if(heartbeat_time_duration > 2.0)
		{
			task_ok = false;
			char tempstr[512];
			sprintf(tempstr,"Task: %s has not reported heartbeats in %.1f seconds",newTask.Task_Name.c_str(),heartbeat_time_duration);
			logger->log_fatal(tempstr);
			diagnostic_status.Diagnostic_Message = MISSING_HEARTBEATS;
			diagnostic_status.Level = FATAL;
			diagnostic_status.Description = tempstr;
			diagnostic_pub.publish(diagnostic_status);

		}

		if(task_ok == true){task_ok_counter++;}
	}
	if(task_ok_counter == TasksToCheck.size())
	{
		if(TasksToCheck.size() > 0)
		{
			ready_to_arm = true;
		}
		else
		{
			ready_to_arm = false;
		}
		char tempstr[255];
		sprintf(tempstr,"%d/%d (All) Tasks Operational.",task_ok_counter,TasksToCheck.size());
        
		logger->log_info(tempstr);
	}
	else
	{
		ready_to_arm = false;
		char tempstr[255];
		sprintf(tempstr,"%d/%d Tasks are in WARN state or Higher!",TasksToCheck.size()-task_ok_counter,TasksToCheck.size());
		logger->log_warn(tempstr);
		diagnostic_status.Diagnostic_Message = DEVICE_NOT_AVAILABLE;
		diagnostic_status.Level = WARN;
		diagnostic_status.Description = tempstr;
		diagnostic_pub.publish(diagnostic_status);
	}
	return true;
}
bool run_fastrate_code()
{
	std_msgs::Bool bool_ready_to_arm;
	bool_ready_to_arm.data = ready_to_arm;
    ready_to_arm_pub.publish(bool_ready_to_arm);
	//logger->log_debug("Running fast rate code.");
	return true;
}
bool run_mediumrate_code()
{
	beat.stamp = ros::Time::now();
	heartbeat_pub.publish(beat);
	//logger->log_debug("Running medium rate code.");
	if(measure_time_diff(ros::Time::now(),boot_time) > 20.0) //Wait 5 seconds for all Nodes to start.
	{
		if(Log_Resources_Used == 1)
		{
			log_resources();
		}
	}
	diagnostic_status.Diagnostic_Type = SOFTWARE;
	diagnostic_status.Level = INFO;
	diagnostic_status.Diagnostic_Message = NOERROR;
	diagnostic_status.Description = "Node Executing.";
	diagnostic_pub.publish(diagnostic_status);
	return true;
}
bool run_slowrate_code()
{
	if(device_initialized == true)
	{
		icarus_rover_v2::diagnostic resource_diagnostic;
		resource_diagnostic = resourcemonitor->update();
		if(resource_diagnostic.Diagnostic_Message == DEVICE_NOT_AVAILABLE)
		{
			diagnostic_pub.publish(resource_diagnostic);
			logger->log_warn("Couldn't read resources used.");
		}
		else if(resource_diagnostic.Level >= WARN)
		{
			resources_used = resourcemonitor->get_resourceused();
			resource_pub.publish(resources_used);
			diagnostic_pub.publish(resource_diagnostic);
		}
		else if(resource_diagnostic.Level <= NOTICE)
		{
			resources_used = resourcemonitor->get_resourceused();
			resource_pub.publish(resources_used);
		}
	}
	if(check_tasks() == false)
	{
		logger->log_warn("Not able to check Tasks.");
	}
	//logger->log_debug("Running slow rate code.");

	return true;
}
bool run_veryslowrate_code()
{
	//logger->log_debug("Running very slow rate code.");
	diagnostic_status = rescan_topics(diagnostic_status);
	diagnostic_pub.publish(diagnostic_status);
	logger->log_info("Node Running.");
	icarus_rover_v2::firmware fw;
	fw.Generic_Node_Name = "diagnostic_node";
	fw.Node_Name = node_name;
	fw.Description = "Latest Rev: 11-Jan-2017";
	fw.Major_Release = DIAGNOSTICNODE_MAJOR_RELEASE;
	fw.Minor_Release = DIAGNOSTICNODE_MINOR_RELEASE;
	fw.Build_Number = DIAGNOSTICNODE_BUILD_NUMBER;
	firmware_pub.publish(fw);
	return true;
}
void resource_Callback(const icarus_rover_v2::resource::ConstPtr& msg,const std::string &topicname)
{
	for(int i = 0; i < TaskList.size();i++)
	{
		//printf("------------------\r\n%s/%s\r\n",TaskList.at(i).Task_Name.c_str(),msg->Node_Name.c_str());
		if(	TaskList.at(i).resource_topic == topicname)
		{
			TaskList.at(i).last_resource_received = ros::Time::now();//measure_time_diff(ros::Time::now(),boot_time);
			TaskList.at(i).CPU_Perc = msg->CPU_Perc;
			TaskList.at(i).RAM_MB = msg->RAM_MB;
			TaskList.at(i).PID = msg->PID;
		}
	}
	std::size_t resource_available_topic = topicname.find("resource_available");
	if(resource_available_topic != std::string::npos)
	{
		bool found = true;
		for(int i = 0; i < DeviceResourceAvailableList.size();i++)
		{
			if(DeviceResourceAvailableList.at(i).Device_Name == msg->Node_Name)
			{
				found = false;
				DeviceResourceAvailableList.at(i).CPU_Perc_Available = msg->CPU_Perc;
				DeviceResourceAvailableList.at(i).RAM_Mb_Available = msg->RAM_MB;
				break;
			}
		}
		if(found == true)
		{
			DeviceResourceAvailable newdevice;
			newdevice.Device_Name = msg->Node_Name;
			newdevice.CPU_Perc_Available = msg->CPU_Perc;
			newdevice.RAM_Mb_Available = msg->RAM_MB;
			DeviceResourceAvailableList.push_back(newdevice);
		}
	}
}
void diagnostic_Callback(const icarus_rover_v2::diagnostic::ConstPtr& msg,const std::string &topicname)
{
	bool add_me = true;
	for(int i = 0; i < TaskList.size();i++)
	{
		if(	TaskList.at(i).diagnostic_topic == topicname)
		{
			TaskList.at(i).last_diagnostic_received = ros::Time::now();//measure_time_diff(ros::Time::now(),boot_time);
			TaskList.at(i).last_diagnostic_level = msg->Level;
		}
	}
	//sprintf(tempstr,"TaskList size: %d",TaskList.size());
	//logger->log_debug(tempstr);
	char tempstr[512];
	sprintf(tempstr,"Node: %s: %s",msg->Node_Name.c_str(),msg->Description.c_str());

	switch(msg->Level)
	{
		case DEBUG:
			//logger->log_debug(tempstr);
			break;
		case INFO:
			//logger->log_info(tempstr);
			break;
		case NOTICE:
			logger->log_notice(tempstr);
			break;
		case WARN:
			logger->log_warn(tempstr);
			break;
		case ERROR:
			logger->log_error(tempstr);
			break;
		case FATAL:
			logger->log_fatal(tempstr);
			break;
		default:
			break;
	}
}
std::vector<icarus_rover_v2::diagnostic> check_program_variables()
{
	std::vector<icarus_rover_v2::diagnostic> diaglist;
	bool status = true;
	logger->log_notice("checking program variables.");

	if(status == true)
	{
		icarus_rover_v2::diagnostic diag=diagnostic_status;
		diag.Diagnostic_Type = SOFTWARE;
		diag.Level = NOTICE;
		diag.Diagnostic_Message = DIAGNOSTIC_PASSED;
		diag.Description = "Checked Program Variables -> PASSED";
		diaglist.push_back(diag);
	}
	else
	{
		icarus_rover_v2::diagnostic diag=diagnostic_status;
		diag.Diagnostic_Type = SOFTWARE;
		diag.Level = WARN;
		diag.Diagnostic_Message = DIAGNOSTIC_FAILED;
		diag.Description = "Checked Program Variables -> FAILED";
		diaglist.push_back(diag);
	}
	return diaglist;
}

void Command_Callback(const icarus_rover_v2::command::ConstPtr& msg)
{
	//logger->log_info("Got command");
	if (msg->Command ==  DIAGNOSTIC_ID)
	{
		if(msg->Option1 == LEVEL1)
		{
			diagnostic_pub.publish(diagnostic_status);
		}
		else if(msg->Option1 == LEVEL2)
		{
			std::vector<icarus_rover_v2::diagnostic> diaglist = check_program_variables();
			for(int i = 0; i < diaglist.size();i++) { diagnostic_pub.publish(diaglist.at(i)); }
		}
		else if(msg->Option1 == LEVEL3)
		{
		}
		else if(msg->Option1 == LEVEL4)
		{
		}
		else
		{
			logger->log_error("Shouldn't get here!!!");
		}
	}
}
//End User Code: Functions

int main(int argc, char **argv)
{
	node_name = "diagnostic_node";
    ros::init(argc, argv, node_name);
    n.reset(new ros::NodeHandle);
    node_name = ros::this_node::getName();

    if(initializenode() == false)
    {
        logger->log_fatal("Unable to Initialize.  Exiting.");
    	diagnostic_status.Diagnostic_Type = SOFTWARE;
		diagnostic_status.Level = FATAL;
		diagnostic_status.Diagnostic_Message = INITIALIZING_ERROR;
		diagnostic_status.Description = "Node Initializing Error.";
		diagnostic_pub.publish(diagnostic_status);
		kill_node = 1;
    }
    ros::Rate loop_rate(rate);
	boot_time = ros::Time::now();
    now = ros::Time::now();
    fast_timer = now;
    medium_timer = now;
    slow_timer = now;
    veryslow_timer = now;
    while (ros::ok() && (kill_node == 0))
    {
    	bool ok_to_start = false;
		if(require_pps_to_start == false) { ok_to_start = true;}
		else if(require_pps_to_start == true && received_pps == true) { ok_to_start = true; }
    	if(ok_to_start == true)
    	{
    		now = ros::Time::now();
    		mtime = measure_time_diff(now,fast_timer);
			if(mtime > .02)
			{
				run_fastrate_code();
				fast_timer = ros::Time::now();
			}
			mtime = measure_time_diff(now,medium_timer);
			if(mtime > 0.1)
			{
				run_mediumrate_code();
				medium_timer = ros::Time::now();
			}
			mtime = measure_time_diff(now,slow_timer);
			if(mtime > 1.0)
			{
				run_slowrate_code();
				slow_timer = ros::Time::now();
			}
			mtime = measure_time_diff(now,veryslow_timer);
			if(mtime > 10.0)
			{
				run_veryslowrate_code();
				veryslow_timer = ros::Time::now();
			}
		}
		else
		{
			logger->log_warn("Waiting on PPS to Start.");
		}
		ros::spinOnce();
		loop_rate.sleep();
    }
    logger->log_notice("Node Finished Safely.");
    return 0;
}
bool initializenode()
{
    //Start Template Code: Initialization, Parameters and Topics
    myDevice.DeviceName = "";
    myDevice.Architecture = "";
    device_initialized = false;
    std::string diagnostic_topic = "/" + node_name + "/diagnostic";
	diagnostic_pub =  n->advertise<icarus_rover_v2::diagnostic>(diagnostic_topic,1000);
	std::string resource_topic = "/" + node_name + "/resource";
	resource_pub = n->advertise<icarus_rover_v2::resource>(resource_topic,1000);

	std::string param_verbosity_level = node_name +"/verbosity_level";
	if(n->getParam(param_verbosity_level,verbosity_level) == false)
	{
		logger = new Logger("FATAL",ros::this_node::getName());
		logger->log_fatal("Missing Parameter: verbosity_level. Exiting");
		return false;
	}
	else
	{
		logger = new Logger(verbosity_level,ros::this_node::getName());
	}
	std::string param_loop_rate = node_name +"/loop_rate";
	if(n->getParam(param_loop_rate,rate) == false)
	{
		logger->log_fatal("Missing Parameter: loop_rate.  Exiting.");
		return false;
	}
	hostname[1023] = '\0';
	gethostname(hostname,1023);
	std::string heartbeat_topic = "/" + node_name + "/heartbeat";
	heartbeat_pub = n->advertise<icarus_rover_v2::heartbeat>(heartbeat_topic,1000);
	beat.Node_Name = node_name;
	std::string device_topic = "/" + std::string(hostname) +"_master_node/device";
	device_sub = n->subscribe<icarus_rover_v2::device>(device_topic,1000,Device_Callback);
	pps_sub = n->subscribe<std_msgs::Bool>("/pps",1000,PPS_Callback);  //This is a pps consumer.
 	std::string param_require_pps_to_start = node_name +"/require_pps_to_start";
    if(n->getParam(param_require_pps_to_start,require_pps_to_start) == false)
	{
		logger->log_fatal("Missing Parameter: require_pps_to_start.  Exiting.");
		return false;
	}
	std::string firmware_topic = "/" + node_name + "/firmware";
    firmware_pub =  n->advertise<icarus_rover_v2::firmware>(firmware_topic,1000);
	//End Template Code: Initialization, Parameters and Topics

	//Start User Code: Initialization, Parameters and Topics
    ready_to_arm = false;
    diagnostic_status.DeviceName = hostname;
	diagnostic_status.Node_Name = node_name;
	diagnostic_status.System = ROVER;
	diagnostic_status.SubSystem = ROBOT_CONTROLLER;
	diagnostic_status.Component = TIMING_NODE;

	diagnostic_status.Diagnostic_Type = NOERROR;
	diagnostic_status.Level = INFO;
	diagnostic_status.Diagnostic_Message = INITIALIZING;
	diagnostic_status.Description = "Node Initializing";
	diagnostic_pub.publish(diagnostic_status);
	std::string param_ram_usage_threshold = node_name +"/RAM_usage_threshold_MB";
	if(n->getParam(param_ram_usage_threshold,RAM_usage_threshold_MB) == false)
	{
		logger->log_fatal("Missing Parameter: RAM_usage_threshold_MB.  Exiting.");
		return false;
	}
	std::string param_CPU_usage_threshold = node_name + "/CPU_usage_threshold_percent";
	if(n->getParam(param_CPU_usage_threshold,CPU_usage_threshold_percent) == false)
	{
		logger->log_fatal("Missing Parameter: CPU_usage_threshold_percent.  Exiting.");
		return false;
	}

	std::string param_log_resources_used = node_name +"/Log_Resources_Used";
	if(n->getParam(param_log_resources_used,Log_Resources_Used) == false)
	{
		logger->log_warn("Missing Parameter: Log_Resources Used.  Using Default: 0");
	}
	logging_initialized = false;
	std::string ready_to_arm_topic = "/" + node_name + "/ready_to_arm";
	ready_to_arm_pub =  n->advertise<std_msgs::Bool>(ready_to_arm_topic,1000);
	//End User Code: Initialization, Parameters and Topics
    logger->log_info("Initialized!");
    return true;
}
//End Initialize Function

//Start Template Code: Functions
double measure_time_diff(ros::Time timer_a, ros::Time timer_b)
{
	ros::Duration etime = timer_a - timer_b;
	return etime.toSec();
}
void PPS_Callback(const std_msgs::Bool::ConstPtr& msg)
{
	//logger->log_info("Got pps");
	received_pps = true;
}
void Device_Callback(const icarus_rover_v2::device::ConstPtr& msg)
{
	icarus_rover_v2::device newdevice;
	newdevice.DeviceName = msg->DeviceName;
	newdevice.Architecture = msg->Architecture;

	if((newdevice.DeviceName == hostname) && (device_initialized == false))
	{
		myDevice = newdevice;
		resourcemonitor = new ResourceMonitor(diagnostic_status,myDevice.Architecture,myDevice.DeviceName,node_name);
		device_initialized = true;
	}
}
void signalinterrupt_handler(int sig)
{
	kill_node = 1;
}
//End Template Code: Functions
