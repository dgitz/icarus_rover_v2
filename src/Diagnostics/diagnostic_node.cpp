#include "diagnostic_node.h"

//Start User Code: Functions
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
			char tempstr[200];
			sprintf(tempstr,"Task: %s is using high CPU resource: %d/%d %",
					newTask.Task_Name.c_str(),newTask.CPU_Perc,CPU_usage_threshold_percent);
			logger->log_warn(tempstr);
		}
		if(newTask.RAM_MB > RAM_usage_threshold_MB)
		{
			task_ok = false;
			char tempstr[200];
			sprintf(tempstr,"Task: %s is using high RAM resource: %ld/%ld (MB)",
					newTask.Task_Name.c_str(),newTask.RAM_MB,RAM_usage_threshold_MB);
			logger->log_warn(tempstr);
		}

		if(task_ok == true){task_ok_counter++;}
	}
	if(task_ok_counter == TasksToCheck.size())
	{
		char tempstr[100];
		sprintf(tempstr,"%d/%d (All) Tasks Operational.",task_ok_counter,TasksToCheck.size());
		logger->log_info(tempstr);
	}
	else
	{
		char tempstr[100];
		sprintf(tempstr,"%d/%d Tasks are in WARN state or Higher!",TasksToCheck.size()-task_ok_counter,TasksToCheck.size());
		logger->log_warn(tempstr);
	}
	return true;
}
bool run_fastrate_code()
{
	//logger->log_debug("Running fast rate code.");
	return true;
}
bool run_mediumrate_code()
{
	//logger->log_debug("Running medium rate code.");
	return true;
}
bool run_slowrate_code()
{
	if(device_initialized == true)
	{
		pid = get_pid();
		if(pid < 0)
		{
			logger->log_warn("Couldn't retrieve PID.");
		}
		else
		{
			if(check_resources(pid))
			{
				resource_pub.publish(resources_used);
			}
			else
			{
				logger->log_warn("Couldn't read resources used.");
			}
		}
	}
	if(check_tasks() == false)
	{
		logger->log_warn("Not able to check Tasks.");
	}
	//logger->log_debug("Running slow rate code.");
	diagnostic_status.Diagnostic_Type = SOFTWARE;
	diagnostic_status.Level = DEBUG;
	diagnostic_status.Diagnostic_Message = NOERROR;
	diagnostic_status.Description = "Node Executing.";
	diagnostic_pub.publish(diagnostic_status);
	return true;
}
bool run_veryslowrate_code()
{
	//logger->log_debug("Running very slow rate code.");

	return true;
}
void resource_Callback(const icarus_rover_v2::resource::ConstPtr& msg)
{
	bool add_me = true;
	for(int i = 0; i < TaskList.size();i++)
	{
		if(	TaskList.at(i).Task_Name == msg->Node_Name)
		{
			add_me = false;
			TaskList.at(i).last_message_received = ros::Time::now();//measure_time_diff(ros::Time::now(),boot_time);
			TaskList.at(i).CPU_Perc = msg->CPU_Perc;
			TaskList.at(i).RAM_MB = msg->RAM_MB;
		}
	}
	if(add_me == true)
	{
		Task newTask;
		newTask.Task_Name = msg->Node_Name;
		newTask.last_message_received = ros::Time::now();//measure_time_diff(ros::Time::now(),boot_time);
		newTask.CPU_Perc = msg->CPU_Perc;
		newTask.RAM_MB = msg->RAM_MB;
		TaskList.push_back(newTask);
	}
}
void diagnostic_Callback(const icarus_rover_v2::diagnostic::ConstPtr& msg)
{
	bool add_me = true;
	for(int i = 0; i < TaskList.size();i++)
	{
		if(	TaskList.at(i).Task_Name == msg->Node_Name)
		{
			add_me = false;

			TaskList.at(i).last_message_received = ros::Time::now();//measure_time_diff(ros::Time::now(),boot_time);
			TaskList.at(i).last_diagnostic_level = msg->Level;
		}
	}
	if(add_me == true)
	{
		Task newTask;
		newTask.Task_Name = msg->Node_Name;
		newTask.last_message_received = ros::Time::now();//measure_time_diff(ros::Time::now(),boot_time);
		newTask.last_diagnostic_level = msg->Level;
		TaskList.push_back(newTask);
	}
	//sprintf(tempstr,"TaskList size: %d",TaskList.size());
	//logger->log_debug(tempstr);
	char tempstr[50];
	sprintf(tempstr,"Node: %s: %s",msg->Node_Name.c_str(),msg->Description.c_str());

	switch(msg->Level)
	{
		case DEBUG:
			logger->log_debug(tempstr);
			break;
		case INFO:
			logger->log_info(tempstr);
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
//End User Code: Functions

//Start Initialize Function
bool initialize(ros::NodeHandle nh)
{
    //Start Template Code: Initialization, Parameters and Topics
    printf("Node name: %s\r\n",node_name.c_str());
    myDevice.DeviceName = "";
    myDevice.Architecture = "";
    device_initialized = false;
    pid = -1;
    std::string diagnostic_topic = "/" + node_name + "/diagnostic";
	diagnostic_pub =  nh.advertise<icarus_rover_v2::diagnostic>(diagnostic_topic,1000);
	std::string resource_topic = "/" + node_name + "/resource";
	resource_pub = nh.advertise<icarus_rover_v2::resource>(resource_topic,1000);

	std::string param_verbosity_level = node_name +"/verbosity_level";
	if(nh.getParam(param_verbosity_level,verbosity_level) == false)
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
	if(nh.getParam(param_loop_rate,rate) == false)
	{
		logger->log_fatal("Missing Parameter: loop_rate.  Exiting.");
		return false;
	}
	char hostname[1024];
	hostname[1023] = '\0';
	gethostname(hostname,1023);
	std::string device_topic = "/" + std::string(hostname) +"_master_node/device";
	device_sub = nh.subscribe<icarus_rover_v2::device>(device_topic,1000,Device_Callback);
	pps_sub = nh.subscribe<std_msgs::Bool>("/pps",1000,PPS_Callback);  //This is a pps consumer.
	 std::string param_require_pps_to_start = node_name +"/require_pps_to_start";
	    if(nh.getParam(param_require_pps_to_start,require_pps_to_start) == false)
		{
			logger->log_fatal("Missing Parameter: require_pps_to_start.  Exiting.");
			return false;
		}
	//End Template Code: Initialization, Parameters and Topics

	//Start User Code: Initialization, Parameters and Topics
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
	if(nh.getParam(param_ram_usage_threshold,RAM_usage_threshold_MB) == false)
	{
		logger->log_fatal("Missing Parameter: RAM_usage_threshold_MB.  Exiting.");
		return false;
	}
	std::string param_CPU_usage_threshold = node_name + "/CPU_usage_threshold_percent";
	if(nh.getParam(param_CPU_usage_threshold,CPU_usage_threshold_percent) == false)
	{
		logger->log_fatal("Missing Parameter: CPU_usage_threshold_percent.  Exiting.");
		return false;
	}

	ros::master::V_TopicInfo master_topics;
	ros::master::getTopics(master_topics);
	std::vector<std::string> resource_topics;
	std::vector<std::string> diagnostic_topics;
	for (ros::master::V_TopicInfo::iterator it = master_topics.begin() ; it != master_topics.end(); it++)
	{
		const ros::master::TopicInfo& info = *it;
		if(info.datatype == "icarus_rover_v2/resource")
		{
			resource_topics.push_back(info.name);
		}
		else if(info.datatype == "icarus_rover_v2/diagnostic")
		{
			diagnostic_topics.push_back(info.name);
		}

	}
	ros::Subscriber * resource_subs;
	resource_subs = new ros::Subscriber[resource_topics.size()];
	for(int i = 0; i < resource_topics.size();i++)
	{
		char tempstr[50];
		sprintf(tempstr,"Subscribing to resource topic: %s",resource_topics.at(i).c_str());
		logger->log_info(tempstr);
		resource_subs[i] = nh.subscribe<icarus_rover_v2::resource>(resource_topics.at(i),1000,resource_Callback);
	}

	ros::Subscriber * diagnostic_subs;
	diagnostic_subs = new ros::Subscriber[diagnostic_topics.size()];
	for(int i = 0; i < diagnostic_topics.size();i++)
	{
		char tempstr[50];
		sprintf(tempstr,"Subscribing to diagnostic topic: %s",diagnostic_topics.at(i).c_str());
		logger->log_info(tempstr);
		diagnostic_subs[i] = nh.subscribe<icarus_rover_v2::diagnostic>(diagnostic_topics.at(i),1000,diagnostic_Callback);
	}
	//End User Code: Initialization, Parameters and Topics
    logger->log_info("Initialized!");
    return true;
}
//End Initialize Function

//Start Main Loop
int main(int argc, char **argv)
{
	usleep(2000000); //Wait 2 seconds for other nodes to start.
	node_name = "diagnostic_node";
    ros::init(argc, argv, node_name);
    node_name = ros::this_node::getName();
    ros::NodeHandle n;
    if(initialize(n) == false)
    {
        logger->log_fatal("Unable to Initialize.  Exiting.");
    	diagnostic_status.Diagnostic_Type = SOFTWARE;
		diagnostic_status.Level = FATAL;
		diagnostic_status.Diagnostic_Message = INITIALIZING_ERROR;
		diagnostic_status.Description = "Node Initializing Error.";
		diagnostic_pub.publish(diagnostic_status);
        return 0; 
    }
    ros::Rate loop_rate(rate);
    now = ros::Time::now();
    fast_timer = now;
    medium_timer = now;
    slow_timer = now;
    veryslow_timer = now;
    boot_time = now;
    while (ros::ok())
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
    return 0;
}
//End Main Loop

//Start Template Code: Functions
void PPS_Callback(const std_msgs::Bool::ConstPtr& msg)
{
	//logger->log_info("Got pps");
	received_pps = true;
}
int get_pid()
{
	int id = -1;
	std::string local_node_name;
	local_node_name = node_name.substr(1,node_name.size());
	std::string pid_filename;
	pid_filename = "/home/robot/logs/output/PID" + node_name;
	char tempstr[130];
	sprintf(tempstr,"ps aux | grep __name:=%s > %s",local_node_name.c_str(),pid_filename.c_str());
	system(tempstr);
	ifstream myfile;
	myfile.open(pid_filename.c_str());
	if(myfile.is_open())
	{
		std::string line;
		getline(myfile,line);
		//printf("Line:%s\r\n",line.c_str());
		std::size_t found = line.find("icarus_rover_v2/diagnostic_node");
		if(found != std::string::npos)
		{
			std::vector <string> fields;
			boost::split(fields,line,boost::is_any_of("\t "),boost::token_compress_on);
			id =  atoi(fields.at(1).c_str());
		}
	}
	else
	{
		id = -1;
	}
	myfile.close();
	//printf("ID: %d\r\n",id);
	//id = -1;
	return id;

}
void Device_Callback(const icarus_rover_v2::device::ConstPtr& msg)
{
	icarus_rover_v2::device newdevice;
	newdevice.DeviceName = msg->DeviceName;
	newdevice.Architecture = msg->Architecture;
	myDevice = newdevice;
	if(myDevice.DeviceName != "")
	{
		device_initialized = true;
	}
}
double measure_time_diff(ros::Time timer_a, ros::Time timer_b)
{
	ros::Duration etime = timer_a - timer_b;
	return etime.toSec();
}
bool check_resources(int procid)
{
	std::string resource_filename;
	resource_filename = "/home/robot/logs/output/RESOURCE/" + node_name;
	char tempstr[130];
	sprintf(tempstr,"top -bn1 | grep %d > %s",procid,resource_filename.c_str());
	//printf("Command: %s\r\n",tempstr);
	system(tempstr); //RAM used is column 6, in KB.  CPU used is column 8, in percentage.
	ifstream myfile;
	myfile.open(resource_filename.c_str());
	if(myfile.is_open())
	{
		std::string line;
		getline(myfile,line);
		std::vector<std::string> strs;
		boost::split(strs,line,boost::is_any_of(" "),boost::token_compress_on);
		resources_used.Node_Name = node_name;
		resources_used.PID = pid;
		resources_used.CPU_Perc = atoi(strs.at(8).c_str());
		resources_used.RAM_MB = atoi(strs.at(6).c_str())/1000.0;
		return true;
	}
	else
	{
		return false;
	}
	myfile.close();
	return false;
}
//End Template Code: Functions