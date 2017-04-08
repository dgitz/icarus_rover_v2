#ifndef DIAGNOSTIC_H
#define DIAGNOSTIC_H
//Start Template Code: Includes
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "logger.h"
#include "resourcemonitor.h"
#include <boost/algorithm/string.hpp>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8.h>
#include <sstream>
#include <stdlib.h>
#include <icarus_rover_v2/Definitions.h>
#include <icarus_rover_v2/diagnostic.h>
#include <icarus_rover_v2/device.h>
#include <icarus_rover_v2/resource.h>
#include <icarus_rover_v2/pin.h>
#include <icarus_rover_v2/command.h>
#include <icarus_rover_v2/firmware.h>
#include <icarus_rover_v2/heartbeat.h>
#include <signal.h>
//End Template Code: Includes

//Start User Code: Defines
//End User Code: Defines

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
void PPS01_Callback(const std_msgs::Bool::ConstPtr& msg);
void PPS1_Callback(const std_msgs::Bool::ConstPtr& msg);
void PPS10_Callback(const std_msgs::Bool::ConstPtr& msg);
void PPS100_Callback(const std_msgs::Bool::ConstPtr& msg);
void PPS1000_Callback(const std_msgs::Bool::ConstPtr& msg);
double measure_time_diff(ros::Time timer_a, ros::Time tiber_b);
void Device_Callback(const icarus_rover_v2::device::ConstPtr& msg);
void Command_Callback(const icarus_rover_v2::command& msg);
std::vector<icarus_rover_v2::diagnostic> check_program_variables();
void signalinterrupt_handler(int sig);
//End Template Code: Function Prototypes

//Start User Code: Function Prototypes
icarus_rover_v2::diagnostic rescan_topics(icarus_rover_v2::diagnostic diag);
void heartbeat_Callback(const icarus_rover_v2::heartbeat::ConstPtr& msg,const std::string &topicname);
void resource_Callback(const icarus_rover_v2::resource::ConstPtr& msg,const std::string &topicname);
void diagnostic_Callback(const icarus_rover_v2::diagnostic::ConstPtr& msg,const std::string &topicname);
bool check_tasks();
//End User Code: Function Prototypes

//Start Template Code: Define Global variables
std::string node_name;
std::string verbosity_level;
ros::Subscriber pps01_sub;
ros::Subscriber pps1_sub;
ros::Subscriber pps10_sub;
ros::Subscriber pps100_sub;
ros::Subscriber pps1000_sub;
ros::Subscriber device_sub;
ros::Publisher diagnostic_pub;
ros::Publisher resource_pub;
ros::Subscriber command_sub;
ros::Publisher firmware_pub;
icarus_rover_v2::diagnostic diagnostic_status;
icarus_rover_v2::device myDevice;
icarus_rover_v2::resource resources_used;
Logger *logger;
ResourceMonitor *resourcemonitor;
bool require_pps_to_start;
bool received_pps;
ros::Time now;
ros::Time boot_time;
double mtime;
bool device_initialized;
char hostname[1024];
ros::Publisher heartbeat_pub;
icarus_rover_v2::heartbeat beat;
volatile sig_atomic_t kill_node;
//End Template Code: Define Global Variables

//Start User Code: Define Global Variables
ros::Publisher ready_to_arm_pub;
bool ready_to_arm;
boost::shared_ptr<ros::NodeHandle> n;
std::vector<Task> TaskList;
std::vector<DeviceResourceAvailable> DeviceResourceAvailableList;
int RAM_usage_threshold_MB;
int CPU_usage_threshold_percent;
int Log_Resources_Used;
bool logging_initialized;
ofstream ram_used_file;
ofstream cpu_used_file;
ofstream ram_free_file;
ofstream cpu_free_file;
//End User Code: Define Global Variables
#endif
