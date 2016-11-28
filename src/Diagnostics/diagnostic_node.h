#ifndef DIAGNOSTIC_H
#define DIAGNOSTIC_H
//Start Template Code: Includes
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "logger.h"
#include "resourcemonitor.h"
#include <boost/algorithm/string.hpp>
#include <std_msgs/Bool.h>
#include <sstream>
#include <stdlib.h>
#include "Definitions.h"
#include <icarus_rover_v2/diagnostic.h>
#include <icarus_rover_v2/device.h>
#include <icarus_rover_v2/resource.h>
#include <icarus_rover_v2/firmware.h>
#include <icarus_rover_v2/heartbeat.h>
//End Template Code: Includes

//Start User Code: Includes
//End User Code: Includes

//Start User Code: Data Structures
struct Task
{
	std::string Task_Name;
	ros::Time last_diagnostic_received;
	ros::Time last_resource_received;
	ros::Time last_heartbeat_received;
	int16_t PID;
	int16_t CPU_Perc;
	int64_t RAM_MB;
	uint8_t last_diagnostic_level;
	std::string resource_topic;
	std::string diagnostic_topic;
	std::string heartbeat_topic;
	ros::Subscriber resource_sub;
	ros::Subscriber diagnostic_sub;
	ros::Subscriber heartbeat_sub;
};
struct DeviceResourceAvailable
{
	std::string Device_Name;
	int16_t CPU_Perc_Available;
	int64_t RAM_Mb_Available;
};
//End User Code: Data Structures

//Start Template Code: Function Prototypes
bool initializenode();//ros::NodeHandle nh);
bool run_fastrate_code();
bool run_mediumrate_code();
bool run_slowrate_code();
bool run_veryslowrate_code();
double measure_time_diff(ros::Time timer_a, ros::Time tiber_b);
void PPS_Callback(const std_msgs::Bool::ConstPtr& msg);
void Device_Callback(const icarus_rover_v2::device::ConstPtr& msg);
//Stop Template Code: Function Prototypes

//Start User Code: Function Prototypes
icarus_rover_v2::diagnostic rescan_topics(icarus_rover_v2::diagnostic diag);
void heartbeat_Callback(const icarus_rover_v2::heartbeat::ConstPtr& msg,const std::string &topicname);
void resource_Callback(const icarus_rover_v2::resource::ConstPtr& msg,const std::string &topicname);
void diagnostic_Callback(const icarus_rover_v2::diagnostic::ConstPtr& msg,const std::string &topicname);
bool check_tasks();
//End User Code: Function Prototypes

//Start Template Code: Define Global variables
std::string node_name;
int rate = 1;
std::string verbosity_level = "";
ros::Publisher pps_pub;  //Not used as this is a pps consumer only.
ros::Subscriber pps_sub;  
ros::Subscriber device_sub;
ros::Publisher diagnostic_pub;
ros::Publisher resource_pub;
ros::Publisher firmware_pub;
icarus_rover_v2::diagnostic diagnostic_status;
icarus_rover_v2::device myDevice;
icarus_rover_v2::resource resources_used;
Logger *logger;
ResourceMonitor *resourcemonitor;
bool require_pps_to_start = false;
bool received_pps = false;
ros::Time fast_timer; //50 Hz
ros::Time medium_timer; //10 Hz
ros::Time slow_timer; //1 Hz
ros::Time veryslow_timer; //1 Hz
ros::Time now;
double mtime;
bool device_initialized;
char hostname[1024];
ros::Publisher heartbeat_pub;
icarus_rover_v2::heartbeat beat;
//End Template Code: Define Global Variables

//Start User Code: Define Global Variables
ros::Publisher ready_to_arm_pub;
bool ready_to_arm;
boost::shared_ptr<ros::NodeHandle> n;
std::vector<Task> TaskList;
std::vector<DeviceResourceAvailable> DeviceResourceAvailableList;
int RAM_usage_threshold_MB;
int CPU_usage_threshold_percent;
ros::Time boot_time;
int Log_Resources_Used;
bool logging_initialized;
ofstream ram_used_file;
ofstream cpu_used_file;
ofstream ram_free_file;
ofstream cpu_free_file;
/*
std::vector<std::string> resource_topics;
std::vector<std::string> diagnostic_topics;
std::vector<std::string> heartbeat_topics;
std::vector<ros::Subscriber> heartbeat_subs;
std::vector<ros::Subscriber> resource_subs;
std::vector<ros::Subscriber> diagnostic_subs;
*/
//End User Code: Define Global Variables

#endif
