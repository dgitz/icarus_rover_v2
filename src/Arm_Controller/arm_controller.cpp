#include "ros/ros.h"
#include "std_msgs/String.h"
#include "logger.h"
#include <boost/algorithm/string.hpp>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <sstream>
#include <stdlib.h>
#include <icarus_rover_v2/resource.h>
#include <urdf/model.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <math.h>
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

//Define general variables, these should be defined for every node.
std::string node_name;
int rate = 1;
std::string verbosity_level = "";
ros::Publisher pps_pub;  //Not used as this is a pps consumer only.
ros::Subscriber pps_sub;  
ros::Publisher resource_pub;
ros::Publisher diagnostic_pub;
icarus_rover_v2::diagnostic diagnostic_status;
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
double mtime;


//Define program variables.  These will vary based on the application.
urdf::Model arm_model;
ros::Publisher joint_pub;
ros::Subscriber joint_base_rotate_sub;
ros::Subscriber joint_shoulder_sub;
ros::Subscriber joint_elbow_sub;
ros::Subscriber joint_wrist_sub;
std::string path_to_model;
geometry_msgs::TransformStamped odom_trans;
sensor_msgs::JointState joint_state;
float joint_base_rotate_angle_command_rad = 0;
float joint_shoulder_angle_command_rad = 0;
float joint_elbow_angle_command_rad = 0; //Stowed
float joint_wrist_angle_command_rad = 0; //Stowed
bool run_fastrate_code()
{
	//logger->log_debug("Running fast rate code.");
	return true;
}
bool run_mediumrate_code()
{
	//logger->log_debug("Running medium rate code.");
	joint_state.header.stamp = ros::Time::now();
	joint_state.name.resize(4);
	joint_state.position.resize(4);
	joint_state.name[0] = "joint_base_rotate";
	joint_state.position[0] = joint_base_rotate_angle_command_rad;
	joint_state.name[1] = "joint_shoulder";
	joint_state.position[1] = joint_shoulder_angle_command_rad;
	joint_state.name[2] = "joint_elbow";
	joint_state.position[2] = joint_elbow_angle_command_rad;
	joint_state.name[3] = "joint_wrist";
	joint_state.position[3] = joint_wrist_angle_command_rad;
	joint_state.effort.resize(4);
	joint_state.effort[0] = 0;
	joint_state.effort[1] = 0;
	joint_state.effort[2] = 0;
	joint_state.effort[3] = 0;
	joint_state.velocity.resize(4);
	joint_state.velocity[0] = 0;
	joint_state.velocity[1] = 0;
	joint_state.velocity[2] = 0;
	joint_state.velocity[3] = 0;

	odom_trans.header.frame_id = "odom";
	odom_trans.child_frame_id = "base_link";
	odom_trans.header.stamp = ros::Time::now();
	odom_trans.transform.translation.x = 0;
	odom_trans.transform.translation.y = 0;
	odom_trans.transform.translation.z = 0.0;
	odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(0+M_PI/2);
	joint_pub.publish(joint_state);


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
void PPS_Callback(const std_msgs::Bool::ConstPtr& msg)
{
	//logger->log_info("Got pps");
	received_pps = true;
}
void joint_base_rotate_Callback(const std_msgs::Float64::ConstPtr& msg)
{
	joint_base_rotate_angle_command_rad = msg->data;
}
void joint_shoulder_Callback(const std_msgs::Float64::ConstPtr& msg)
{
	logger->log_debug("Got here.");
	joint_shoulder_angle_command_rad = msg->data;
}
void joint_elbow_Callback(const std_msgs::Float64::ConstPtr& msg)
{
	joint_elbow_angle_command_rad = msg->data;
}
void joint_wrist_Callback(const std_msgs::Float64::ConstPtr& msg)
{
	joint_wrist_angle_command_rad = msg->data;
}
int main(int argc, char **argv)
{
	node_name = "arm_controller";
    ros::init(argc, argv, node_name);
    ros::NodeHandle n;
    tf::TransformBroadcaster broadcaster;
    if(initialize(n) == false)
    {
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
				broadcaster.sendTransform(odom_trans); //Have to put broadcaster here, otherwise program will crash.
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
    //Start Template Code: Initialization and Parameters
	diagnostic_pub =  nh.advertise<icarus_rover_v2::diagnostic>("/arm_controller/diagnostic",1000);
	diagnostic_status.Node_Name = node_name;
	diagnostic_status.System = ROVER;
	diagnostic_status.SubSystem = ROBOT_CONTROLLER;
	diagnostic_status.Component = TIMING_NODE;

	diagnostic_status.Diagnostic_Type = NOERROR;
	diagnostic_status.Level = INFO;
	diagnostic_status.Diagnostic_Message = INITIALIZING;
	diagnostic_status.Description = "Node Initializing";
	diagnostic_pub.publish(diagnostic_status);
    if(nh.getParam("arm_controller/verbosity_level",verbosity_level) == false)
    {
        logger = new Logger("FATAL",ros::this_node::getName());    
        logger->log_fatal("Missing Parameter: verbosity_level. Exiting");
        return false;
    }
    else
    {
        logger = new Logger(verbosity_level,ros::this_node::getName());      
    }
    if(nh.getParam("arm_controller/loop_rate",rate) == false)
    {
        logger->log_fatal("Missing Parameter: loop_rate.  Exiting.");
        return false;
    }
    pps_sub = nh.subscribe<std_msgs::Bool>("/pps",1000,PPS_Callback);  //This is a pps consumer.
    if(nh.getParam("arm_controller/require_pps_to_start",require_pps_to_start) == false)
	{
		logger->log_fatal("Missing Parameter: require_pps_to_start.  Exiting.");
		return false;
	}
    //Free to edit code from here.

    if(nh.getParam("arm_controller/path_to_urdf",path_to_model) == false)
    {
    	logger->log_fatal("Missing Parameter: path_urdf");
    	return false;
    }
    else
    {
		if(!arm_model.initFile(path_to_model))
		{
			logger->log_fatal("Couldn't parse URDF file.");
			return false;
		}
    }
    joint_pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 1);
    joint_base_rotate_sub = nh.subscribe<std_msgs::Float64>("/joint_base_rotate_command",1000,joint_base_rotate_Callback);
    joint_shoulder_sub = nh.subscribe<std_msgs::Float64>("/joint_shoulder_command",1000,joint_shoulder_Callback);
    joint_elbow_sub = nh.subscribe<std_msgs::Float64>("/joint_elbow_command",1000,joint_elbow_Callback);
    joint_wrist_sub = nh.subscribe<std_msgs::Float64>("/joint_wrist_command",1000,joint_wrist_Callback);

    //More Template code here.  Do not edit.
    resource_pub =  nh.advertise<icarus_rover_v2::resource>("/arm_controller/resource",1000);
    pid = get_pid();
    if(pid < 0)
    {
    	logger->log_fatal("Couldn't retrieve PID. Exiting");
    	return false;
    }
	resource_pub =  nh.advertise<icarus_rover_v2::resource>("/arm_controller/resource",1000); //This is a pps source.
	diagnostic_status.Diagnostic_Type = NOERROR;
	diagnostic_status.Level = INFO;
	diagnostic_status.Diagnostic_Message = NOERROR;
	diagnostic_status.Description = "Node Initialized";
	diagnostic_pub.publish(diagnostic_status);
    logger->log_info("Initialized!");
    return true;
}

int get_pid()
{
	int id = -1;
	std::string pid_filename;
	pid_filename = "/tmp/output/PID/" + node_name;
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
bool check_resources()
{
	std::string resource_filename;
	resource_filename = "/tmp/output/RESOURCE/" + node_name;
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
		resources_used.Node_Name = node_name;
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
