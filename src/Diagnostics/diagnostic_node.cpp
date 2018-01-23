#include "diagnostic_node.h"
//Start User Code: Firmware Definition
#define DIAGNOSTICNODE_MAJOR_RELEASE 2
#define DIAGNOSTICNODE_MINOR_RELEASE 3
#define DIAGNOSTICNODE_BUILD_NUMBER 1
//End User Code: Firmware Definition
//Start User Code: Functions
/*! \brief User Loop1 Code
 */
bool run_loop1_code()
{
	return true;
}
/*! \brief User Loop2 Code
 */
bool run_loop2_code()
{
	if(Log_Resources_Used == 1)
	{
		if(measure_time_diff(ros::Time::now(),boot_time) > 20.0) //Wait 20 seconds for all Nodes to start.
		{
			log_resources();
		}
	}
	std_msgs::Bool bool_ready_to_arm;
	bool_ready_to_arm.data = process->get_readytoarm();
    ready_to_arm_pub.publish(bool_ready_to_arm);
 	return true;
}
/*! \brief User Loop3 Code
 */
bool run_loop3_code()
{
	icarus_rover_v2::diagnostic diag = process->update(1.0/(double)loop3_rate);
	if(diag.Level > NOTICE)
	{
		diagnostic_pub.publish(diag);
		printf("[%s]: %s\n",node_name.c_str(),diag.Description.c_str());
	}
 	return true;
}
icarus_rover_v2::diagnostic rescan_topics(icarus_rover_v2::diagnostic diag)
{
	int found_new_topics = 0;
	ros::master::V_TopicInfo master_topics;
	ros::master::getTopics(master_topics);
	std::vector<std::string> topics_to_add;
	std::vector<DiagnosticNodeProcess::Task> TaskList = process->get_TaskList();
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
		DiagnosticNodeProcess::Task newTask;
		newTask.Task_Name = taskname;
		newTask.diagnostic_topic = topics_to_add.at(i);
		newTask.heartbeat_topic = "/" + taskname + "/heartbeat";
		newTask.resource_topic = "/" + taskname + "/resource";
		ros::Subscriber resource_sub = n->subscribe<icarus_rover_v2::resource>(newTask.resource_topic,1000,boost::bind(resource_Callback,_1,newTask.resource_topic));
		resource_subs.push_back(resource_sub);
		ros::Subscriber diagnostic_sub = n->subscribe<icarus_rover_v2::diagnostic>(newTask.diagnostic_topic,1000,boost::bind(diagnostic_Callback,_1,newTask.diagnostic_topic));
		diagnostic_subs.push_back(diagnostic_sub);
		ros::Subscriber heartbeat_sub = n->subscribe<icarus_rover_v2::heartbeat>(newTask.heartbeat_topic,1000,boost::bind(heartbeat_Callback,_1,newTask.heartbeat_topic));
		heartbeat_subs.push_back(heartbeat_sub);
		process->add_Task(newTask);
		//TaskList.push_back(newTask);

	}

	diag.Diagnostic_Type = SOFTWARE;
	diag.Level = INFO;
	diag.Diagnostic_Message = NOERROR;

	char tempstr[512];
	if(topics_to_add.size() > 0)
	{
		sprintf(tempstr,"Rescanned and found %d new topics.",(int)topics_to_add.size());
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
	process->new_heartbeatmsg(topicname);
}
bool log_resources()
{
	std::vector<DiagnosticNodeProcess::Task> TaskList = process->get_TaskList();
	std::vector<DiagnosticNodeProcess::DeviceResourceAvailable> DeviceResourceAvailableList = process->get_DeviceResourceAvailableList();
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

/*! \brief 0.1 PULSE PER SECOND User Code
 */
void PPS01_Callback(const std_msgs::Bool::ConstPtr& msg)
{
	icarus_rover_v2::diagnostic diagnostic = rescan_topics(process->get_diagnostic());
	if(diagnostic.Level > NOTICE)
	{
		diagnostic_pub.publish(diagnostic);
		logger->log_diagnostic(diagnostic);
	}
	diagnostic = process->new_01ppsmsg();
	if(diagnostic.Level > NOTICE)
	{
		diagnostic_pub.publish(diagnostic);
		logger->log_diagnostic(diagnostic);
	}
	icarus_rover_v2::firmware fw;
	fw.Generic_Node_Name = "diagnostic_node";
	fw.Node_Name = node_name;
	fw.Description = "Latest Rev: 1-January-2018";
	fw.Major_Release = DIAGNOSTICNODE_MAJOR_RELEASE;
	fw.Minor_Release = DIAGNOSTICNODE_MINOR_RELEASE;
	fw.Build_Number = DIAGNOSTICNODE_BUILD_NUMBER;
	firmware_pub.publish(fw);

	printf("t=%4.2f (sec) [%s]: %s\n",ros::Time::now().toSec(),node_name.c_str(),process->get_diagnostic().Description.c_str());
}
/*! \brief 1.0 PULSE PER SECOND User Code
 */
void PPS1_Callback(const std_msgs::Bool::ConstPtr& msg)
{
	received_pps = true;
	icarus_rover_v2::diagnostic diagnostic = process->new_1ppsmsg();
	if(diagnostic.Level > NOTICE)
	{
		diagnostic_pub.publish(diagnostic);
		logger->log_diagnostic(diagnostic);
	}
	if((process->get_initialized() == true) and (process->get_ready() == true))
	{
		icarus_rover_v2::diagnostic resource_diagnostic = resourcemonitor->update();
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
			logger->log_diagnostic(resource_diagnostic);
		}
		else if(resource_diagnostic.Level <= NOTICE)
		{
			resources_used = resourcemonitor->get_resourceused();
			resource_pub.publish(resources_used);
		}
	}
    else if((process->get_ready() == false) and (process->get_initialized() == true))
    {
        
    }
	else if(process->get_initialized() == false)
    {
    	{
			icarus_rover_v2::srv_device srv;
			srv.request.query = "SELF";
			if(srv_device.call(srv) == true)
			{
				if(srv.response.data.size() != 1)
				{
					logger->log_error("Got unexpected device message");
				}
				else
				{
					bool status = new_devicemsg(srv.request.query,srv.response.data.at(0));
				}
			}
    	}
    }
    diagnostic_pub.publish(process->get_diagnostic());
}
void Command_Callback(const icarus_rover_v2::command::ConstPtr& msg)
{
	icarus_rover_v2::command command;
	command.Command = msg->Command;
	command.Option1 = msg->Option1;
	command.Option2 = msg->Option2;
	command.Option3 = msg->Option3;
	command.CommandText = msg->CommandText;
	command.Description = msg->Description;
	std::vector<icarus_rover_v2::diagnostic> diaglist = process->new_commandmsg(command);
	for(std::size_t i = 0; i < diaglist.size(); i++)
	{
		if(diaglist.at(i).Level >= NOTICE)
        {
            logger->log_diagnostic(diaglist.at(i));
            diagnostic_pub.publish(diaglist.at(i));
        }
        if((diaglist.at(i).Level > NOTICE) and (process->get_runtime() > 10.0))
        {
            printf("[%s]: %s\n",node_name.c_str(),diaglist.at(i).Description.c_str());
        }
	}
}

void resource_Callback(const icarus_rover_v2::resource::ConstPtr& msg,const std::string &topicname)
{
	icarus_rover_v2::resource resource;
	resource.Node_Name = msg->Node_Name;
	resource.CPU_Perc = msg->CPU_Perc;
	resource.PID = msg->PID;
	resource.RAM_MB = msg->RAM_MB;
	resource.stamp = msg->stamp;
	process->new_resourcemsg(topicname,resource);
}
void diagnostic_Callback(const icarus_rover_v2::diagnostic::ConstPtr& msg,const std::string &topicname)
{
	icarus_rover_v2::diagnostic diag;
	diag.DeviceName = msg->DeviceName;
	diag.Component = msg->Component;
	diag.Description = msg->Description;
	diag.Diagnostic_Message = msg->Diagnostic_Message;
	diag.Diagnostic_Type = msg->Diagnostic_Type;
	diag.Level = msg->Level;
	diag.Node_Name = msg->Node_Name;
	diag.SubSystem = msg->SubSystem;
	diag.System = msg->System;
	process->new_diagnosticmsg(topicname,diag);
}

//End User Code: Functions
bool run_10Hz_code()
{
    beat.stamp = ros::Time::now();
	heartbeat_pub.publish(beat);
    if(process->get_diagnostic().Level > NOTICE)
    {
        diagnostic_pub.publish(process->get_diagnostic());
        logger->log_diagnostic(process->get_diagnostic());
    }
    return true;
}
int main(int argc, char **argv)
{
	node_name = "diagnostic_node";
    ros::init(argc, argv, node_name);
    n.reset(new ros::NodeHandle);
    node_name = ros::this_node::getName();
    ros::NodeHandle n;
    
    if(initializenode() == false)
    {
        char tempstr[256];
        sprintf(tempstr,"Unable to Initialize. Exiting.");
        printf("[%s]: %s\n",node_name.c_str(),tempstr);
        logger->log_fatal(tempstr);
		kill_node = 1;
    }
    ros::Rate loop_rate(ros_rate);
	boot_time = ros::Time::now();
    last_10Hz_timer = ros::Time::now();
    double mtime;
    while (ros::ok() && (kill_node == 0))
    {
    	bool ok_to_start = false;
		if(require_pps_to_start == false) { ok_to_start = true;}
		else if(require_pps_to_start == true && received_pps == true) { ok_to_start = true; }
    	if(ok_to_start == true)
    	{
            if(run_loop1 == true)
            {
                mtime = measure_time_diff(ros::Time::now(),last_loop1_timer);
                if(mtime >= (1.0/loop1_rate))
                {
                    run_loop1_code();
                    last_loop1_timer = ros::Time::now();
                }
            }
            if(run_loop2 == true)
            {
                mtime = measure_time_diff(ros::Time::now(),last_loop2_timer);
                if(mtime >= (1.0/loop2_rate))
                {
                    run_loop2_code();
                    last_loop2_timer = ros::Time::now();
                }
            }
            if(run_loop3 == true)
            {
                mtime = measure_time_diff(ros::Time::now(),last_loop3_timer);
                if(mtime >= (1.0/loop3_rate))
                {
                    run_loop3_code();
                    last_loop3_timer = ros::Time::now();
                }
            }
            
            mtime = measure_time_diff(ros::Time::now(),last_10Hz_timer);
            if(mtime >= 0.1)
            {
                run_10Hz_code();
                last_10Hz_timer = ros::Time::now();
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
	kill_node = 0;
	signal(SIGINT,signalinterrupt_handler);
    hostname[1023] = '\0';
    gethostname(hostname,1023);
    std::string diagnostic_topic = "/" + node_name + "/diagnostic";
	diagnostic_pub =  n->advertise<icarus_rover_v2::diagnostic>(diagnostic_topic,5);
    icarus_rover_v2::diagnostic diagnostic;
    diagnostic.DeviceName = hostname;
	diagnostic.Node_Name = node_name;
	diagnostic.System = ROVER;
	diagnostic.SubSystem = ROBOT_CONTROLLER;
	diagnostic.Component = DIAGNOSTIC_NODE;

	diagnostic.Diagnostic_Type = NOERROR;
	diagnostic.Level = INFO;
	diagnostic.Diagnostic_Message = INITIALIZING;
	diagnostic.Description = "Node Initializing";
	diagnostic_pub.publish(diagnostic);

	std::string resource_topic = "/" + node_name + "/resource";
	resource_pub = n->advertise<icarus_rover_v2::resource>(resource_topic,5);

    std::string param_verbosity_level = node_name +"/verbosity_level";
    if(n->getParam(param_verbosity_level,verbosity_level) == false)
    {
        logger = new Logger("WARN",ros::this_node::getName());
        logger->log_warn("Missing Parameter: verbosity_level");
        return false;
    }
    else
    {
        logger = new Logger(verbosity_level,ros::this_node::getName());      
    }
	std::string param_disabled = node_name +"/disable";
    bool disable_node;
    if(n->getParam(param_disabled,disable_node) == true)
    {
    	if(disable_node == true)
    	{
    		logger->log_notice("Node Disabled in Launch File.  Exiting.");
    		printf("[%s]: Node Disabled in Launch File. Exiting.\n",node_name.c_str());
    		return false;
    	}
    }
    
    std::string heartbeat_topic = "/" + node_name + "/heartbeat";
    heartbeat_pub = n->advertise<icarus_rover_v2::heartbeat>(heartbeat_topic,5);
    beat.Node_Name = node_name;
    std::string device_topic = "/" + std::string(hostname) + "_master_node/srv_device";
    srv_device = n->serviceClient<icarus_rover_v2::srv_device>(device_topic);

    pps01_sub = n->subscribe<std_msgs::Bool>("/01PPS",5,PPS01_Callback); 
    pps1_sub = n->subscribe<std_msgs::Bool>("/1PPS",5,PPS1_Callback); 
    command_sub = n->subscribe<icarus_rover_v2::command>("/command",5,Command_Callback);
    std::string param_require_pps_to_start = node_name +"/require_pps_to_start";
    if(n->getParam(param_require_pps_to_start,require_pps_to_start) == false)
	{
		logger->log_warn("Missing Parameter: require_pps_to_start.");
		return false;
	}
    std::string firmware_topic = "/" + node_name + "/firmware";
    firmware_pub =  n->advertise<icarus_rover_v2::firmware>(firmware_topic,1);
    
    double max_rate = 0.0;
    std::string param_loop1_rate = node_name + "/loop1_rate";
    if(n->getParam(param_loop1_rate,loop1_rate) == false)
    {
        logger->log_warn("Missing parameter: loop1_rate.  Not running loop1 code.");
        run_loop1 = false;
    }
    else 
    { 
        last_loop1_timer = ros::Time::now();
        run_loop1 = true; 
        if(loop1_rate > max_rate) { max_rate = loop1_rate; }
    }
    
    std::string param_loop2_rate = node_name + "/loop2_rate";
    if(n->getParam(param_loop2_rate,loop2_rate) == false)
    {
        logger->log_warn("Missing parameter: loop2_rate.  Not running loop2 code.");
        run_loop2 = false;
    }
    else 
    { 
        last_loop2_timer = ros::Time::now();
        run_loop2 = true; 
        if(loop2_rate > max_rate) { max_rate = loop2_rate; }
    }
    
    std::string param_loop3_rate = node_name + "/loop3_rate";
    if(n->getParam(param_loop3_rate,loop3_rate) == false)
    {
        logger->log_warn("Missing parameter: loop3_rate.  Not running loop3 code.");
        run_loop3 = false;
    }
    else 
    { 
        last_loop3_timer = ros::Time::now();
        run_loop3 = true; 
        if(loop3_rate > max_rate) { max_rate = loop3_rate; }
    }
    ros_rate = max_rate * 50.0;
    if(ros_rate < 100.0) { ros_rate = 100.0; }
    char tempstr[512];
    sprintf(tempstr,"Running Node at Rate: %f",ros_rate);
    logger->log_notice(std::string(tempstr));
    //End Template Code: Initialization and Parameters

	//Start User Code: Initialization, Parameters and Topics
	process = new DiagnosticNodeProcess;
	diagnostic = process->init(diagnostic,std::string(hostname));
	if(diagnostic.Level > NOTICE)
	{
		logger->log_fatal(diagnostic.Description);
		printf("[%s]: %s\n",node_name.c_str(),diagnostic.Description.c_str());
		return false;
	}
	diagnostic_pub.publish(diagnostic);

	std::string param_ram_usage_threshold = node_name +"/RAM_usage_threshold_MB";
	int RAM_usage_threshold_MB;
	if(n->getParam(param_ram_usage_threshold,RAM_usage_threshold_MB) == false)
	{
		logger->log_fatal("Missing Parameter: RAM_usage_threshold_MB.  Exiting.");
		return false;
	}
	std::string param_CPU_usage_threshold = node_name + "/CPU_usage_threshold_percent";
	int CPU_usage_threshold_percent;
	if(n->getParam(param_CPU_usage_threshold,CPU_usage_threshold_percent) == false)
	{
		logger->log_fatal("Missing Parameter: CPU_usage_threshold_percent.  Exiting.");
		return false;
	}
	process->set_resourcethresholds(RAM_usage_threshold_MB,CPU_usage_threshold_percent);
	process->set_nodename(node_name);
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
    //Finish User Code: Initialization and Parameters

    //Start Template Code: Final Initialization.
	diagnostic.Diagnostic_Type = NOERROR;
	diagnostic.Level = INFO;
	diagnostic.Diagnostic_Message = NOERROR;
	diagnostic.Description = "Node Initialized";
	process->set_diagnostic(diagnostic);
	diagnostic_pub.publish(diagnostic);
    logger->log_info("Initialized!");
    return true;
    //End Template Code: Finish Initialization.
}
//Start Template Code: Functions
double measure_time_diff(ros::Time timer_a, ros::Time timer_b)
{
	ros::Duration etime = timer_a - timer_b;
	return etime.toSec();
}
bool new_devicemsg(std::string query,icarus_rover_v2::device device)
{

	if(query == "SELF")
	{
		if((device.DeviceName == hostname))
		{
			resourcemonitor = new ResourceMonitor(process->get_diagnostic(),device.Architecture,device.DeviceName,node_name);
			process->set_mydevice(device);
		}
	}

	if((process->get_initialized() == true))
	{
		icarus_rover_v2::diagnostic diag = process->new_devicemsg(device);
	}
	return true;
}
void signalinterrupt_handler(int sig)
{
	kill_node = 1;
}
//End Template Code: Functions
