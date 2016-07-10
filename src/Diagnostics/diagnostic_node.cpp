#include "ros/ros.h"
#include "std_msgs/String.h"
#include "logger.h"
#include <boost/algorithm/string.hpp>
#include <std_msgs/Bool.h>
#include <sstream>
#include <stdlib.h>
#include <unistd.h>
#include "message_filters/subscriber.h"
#include <icarus_rover_v2/Definitions.h>
#include <icarus_rover_v2/resource.h>
#include <icarus_rover_v2/diagnostic.h>


//Template Code.  This should not be removed.
//Function Prototypes
bool initialize(ros::NodeHandle nh);
int get_pid();
bool check_resources();
bool run_fastrate_code();
bool run_mediumrate_code();
bool run_slowrate_code();
bool run_veryslowrate_code();
double measure_time_diff(ros::Time timer_a, ros::Time tiber_b);
bool check_tasks();

//Define general variables, these should be defined for every node.
std::string node_name;
int rate = 1;
std::string verbosity_level = "";
ros::Publisher pps_pub;  //Not used as this is a pps consumer only.
ros::Subscriber pps_sub;  
ros::Publisher resource_pub;
Logger *logger;
bool require_pps_to_start = false;
bool received_pps = false;
int pid = -1;
icarus_rover_v2::resource resources_used;
ros::Time fast_timer; //50 Hz
ros::Time medium_timer; //10 Hz
ros::Time slow_timer; //1 Hz
ros::Time veryslow_timer; //1 Hz
ros::Time now;
ros::Time boot_time;
double mtime;


//Define program variables.  These will vary based on the application.
struct Task
{
	std::string Task_Name;
	ros::Time last_message_received;
	int RAM_MB;
	int CPU_Perc;
	int last_diagnostic_level;
};
std::vector<Task> TaskList;
int RAM_usage_threshold_MB;
int CPU_usage_threshold_percent;
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
	//logger->log_debug("Running slow rate code.");
	if(check_resources() == false)
	{
		logger->log_fatal("Not able to check Node Resources.  Exiting.");
		return false;
	}
	resource_pub.publish(resources_used);
	if(check_tasks() == false)
	{
		logger->log_fatal("Not able to check Tasks. Exiting");
	}
	return true;
}
bool run_veryslowrate_code()
{
	//logger->log_debug("Running very slow rate code.");

	return true;
}
void PPS_Callback(const std_msgs::Bool::ConstPtr& msg)
{
	//logger->log_info("Got pps");
	received_pps = true;
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
int main(int argc, char **argv)
{
	usleep(2000000); //Wait 2 seconds for other nodes to start.
	node_name = "diagnostic_node";
    ros::init(argc, argv, node_name);
    ros::NodeHandle n;
    if(initialize(n) == false)
    {
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

bool initialize(ros::NodeHandle nh)
{
    //Template code.  This should not be changed.
    if(nh.getParam("diagnostic_node/verbosity_level",verbosity_level) == false)
    {
        logger = new Logger("FATAL",ros::this_node::getName());    
        logger->log_fatal("Missing Parameter: verbosity_level. Exiting");
        return false;
    }
    else
    {
        logger = new Logger(verbosity_level,ros::this_node::getName());      
    }
    if(nh.getParam("diagnostic_node/loop_rate",rate) == false)
    {
        logger->log_fatal("Missing Parameter: loop_rate.  Exiting.");
        return false;
    }
    pps_sub = nh.subscribe<std_msgs::Bool>("/pps",1000,PPS_Callback);  //This is a pps consumer.

    if(nh.getParam("diagnostic_node/require_pps_to_start",require_pps_to_start) == false)
	{
		logger->log_fatal("Missing Parameter: require_pps_to_start.  Exiting.");
		return false;
	}
    //Free to edit code from here.
    if(nh.getParam("diagnostic_node/RAM_usage_threshold_MB",RAM_usage_threshold_MB) == false)
	{
		logger->log_fatal("Missing Parameter: RAM_usage_threshold_MB.  Exiting.");
		return false;
	}
    if(nh.getParam("diagnostic_node/CPU_usage_threshold_percent",CPU_usage_threshold_percent) == false)
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
    //More Template code here.  Do not edit.
    pid = get_pid();
    if(pid < 0)
    {
    	logger->log_fatal("Couldn't retrieve PID. Exiting");
    	return false;
    }
    resource_pub =  nh.advertise<icarus_rover_v2::resource>("/diagnostic_node/resource",1000); //This is a pps source.
    logger->log_info("Initialized!");
    return true;
}

int get_pid()
{
	int id = -1;
	std::string pid_filename;
	pid_filename = "/home/robot/logs/output/PID/" + node_name;
	char tempstr[130];

	sprintf(tempstr,"top -bn1 | grep %s | awk ' { print $1 }' > %s",node_name.c_str(),pid_filename.c_str());
	system(tempstr);  //First entry should be PID
	ifstream myfile;
	myfile.open(pid_filename.c_str());
	if(myfile.is_open())
	{
		std::string line;
		getline(myfile,line);
		id =  atoi(line.c_str());
	}
	else
	{
		id = -1;
	}
	myfile.close();
	return id;
}
bool check_tasks()
{
	int task_ok_counter = 0;
	for(int i = 0; i < TaskList.size();i++)
	{
		bool task_ok = false;
		//char tempstr[100];
		//sprintf(tempstr,"Task: %s last msg: %f",TaskList.at(i).Task_Name.c_str(),TaskList.at(i).last_message_received);
		//logger->log_debug(std::string(tempstr));
		if(TaskList.at(i).RAM_MB > RAM_usage_threshold_MB)
		{
			task_ok = false;
			char tempstr[100];
			sprintf(tempstr,"Task: %s exceeds RAM:  %d/%d",
					TaskList.at(i).Task_Name.c_str(),TaskList.at(i).RAM_MB,RAM_usage_threshold_MB);
			double ram_exceeded_factor = (double)(TaskList.at(i).RAM_MB/RAM_usage_threshold_MB);
			if(ram_exceeded_factor < 2.0)
			{
				logger->log_warn(std::string(tempstr));
			}
			else if(ram_exceeded_factor < 4.0)
			{
				logger->log_error(std::string(tempstr));
			}
			else
			{
				logger->log_fatal(std::string(tempstr));
			}
		}
		if(TaskList.at(i).CPU_Perc > CPU_usage_threshold_percent)
		{
			task_ok = false;
			char tempstr[100];
			sprintf(tempstr,"Task: %s exceeds CPU Usage:  %d/%d",
					TaskList.at(i).Task_Name.c_str(),TaskList.at(i).CPU_Perc,CPU_usage_threshold_percent);
			double cpu_exceeded_factor = (double)(TaskList.at(i).CPU_Perc/CPU_usage_threshold_percent);
			if(cpu_exceeded_factor < 2.0)
			{
				logger->log_warn(std::string(tempstr));
			}
			else if(cpu_exceeded_factor < 4.0)
			{
				logger->log_error(std::string(tempstr));
			}
			else
			{
				logger->log_fatal(std::string(tempstr));
			}
		}
		double etime;
		etime = measure_time_diff(ros::Time::now(),TaskList.at(i).last_message_received);
		char tempstr[100];
		sprintf(tempstr,"Task: %s not heard from for: %f seconds",TaskList.at(i).Task_Name.c_str(),etime);
		if(etime < 1.0)
		{
			task_ok = true;
		}
		else if(etime < 2.0)
		{
			task_ok = false;
			logger->log_warn(std::string(tempstr));
		}
		else
		{
			task_ok = false;
			logger->log_fatal(std::string(tempstr));
		}
		if(TaskList.at(i).last_diagnostic_level >= WARN)
		{
			task_ok = false;
		}
		if(task_ok == true){ task_ok_counter++;}

	}
	if(task_ok_counter == TaskList.size())
	{
		logger->log_info("All Tasks Operational!");
	}
	else
	{
		char tempstr[100];
		sprintf(tempstr,"%d/%d Tasks are WARN or Higher!",TaskList.size()-task_ok_counter,TaskList.size());
		logger->log_error(std::string(tempstr));
	}
	return true;
}
bool check_resources()
{
	std::string resource_filename;
	resource_filename = "/home/robot/logs/output/RESOURCE/" + node_name;
	char tempstr[130];
	sprintf(tempstr,"top -bn1 | grep %d > %s",pid,resource_filename.c_str());
	system(tempstr); //RAM used is column 6, in KB.  CPU used is column 9, in percentage.
	ifstream myfile;
	myfile.open(resource_filename.c_str());
	if(myfile.is_open())
	{
		std::string line;
		getline(myfile,line);
		std::vector<std::string> strs;
		boost::split(strs,line,boost::is_any_of(" "),boost::token_compress_on);
		resources_used.PID = pid;
		resources_used.CPU_Perc = atoi(strs.at(9).c_str());
		resources_used.RAM_MB = atoi(strs.at(6).c_str())/1000.0;
		return true;
	}
	else
	{
		logger->log_fatal("Unable to check Node Resources. Exiting");
		return false;
	}
	myfile.close();

}
double measure_time_diff(ros::Time timer_a, ros::Time timer_b)
{
	ros::Duration etime = timer_a - timer_b;
	return etime.toSec();
}
