#ifndef AUTODRIVENODE_H
#define AUTODRIVENODE_H
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
#include <icarus_rover_v2/srv_device.h>
#include <icarus_rover_v2/srv_connection.h>
#include <icarus_rover_v2/srv_leverarm.h>
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
#include <icarus_rover_v2/controlgroup.h>
#include <icarus_rover_v2/pose.h>
#include "sensor_msgs/Joy.h"
#include "autodrive_node_process.h"
//End User Code: Includes

//Start User Code: Data Structures
struct CGSensorSub
{
	ros::Subscriber sub;
	std::string topic;
	std::string type;
};
struct CGCommandSub
{
	ros::Subscriber sub;
	std::string topic;
	std::string type;
};
//End User Code: Data Structures

//Start Template Code: Function Prototypes
bool initializenode();
void PPS01_Callback(const std_msgs::Bool::ConstPtr& msg);
void PPS1_Callback(const std_msgs::Bool::ConstPtr& msg);
double measure_time_diff(ros::Time timer_a, ros::Time tiber_b);
bool new_devicemsg(std::string query,icarus_rover_v2::device device);
void Command_Callback(const icarus_rover_v2::command& msg);
std::vector<icarus_rover_v2::diagnostic> check_program_variables();
bool run_loop3_code();
bool run_loop2_code();
bool run_loop1_code();
bool run_10Hz_code();
void signalinterrupt_handler(int sig);
//End Template Code: Function Prototypes

//Start User Code: Function Prototypes
void ArmedState_Callback(const std_msgs::UInt8::ConstPtr& msg);
void ControlGroup_Callback(const icarus_rover_v2::controlgroup::ConstPtr& msg);
void Pose_Callback(const icarus_rover_v2::pose::ConstPtr& msg);
void Joy_Callback(const sensor_msgs::Joy::ConstPtr& msg);
//End User Code: Function Prototypes

//Start Template Code: Define Global variables
ros::ServiceClient srv_device;
boost::shared_ptr<ros::NodeHandle> n;
std::string node_name;
std::string verbosity_level;
ros::Subscriber pps01_sub;
ros::Subscriber pps1_sub;
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
ros::Time boot_time;
bool device_initialized;
char hostname[1024];
ros::Publisher heartbeat_pub;
icarus_rover_v2::heartbeat beat;
volatile sig_atomic_t kill_node;
ros::Time last_10Hz_timer;
double loop1_rate;
double loop2_rate;
double loop3_rate;
bool run_loop1;
bool run_loop2;
bool run_loop3;
ros::Time last_loop1_timer;
ros::Time last_loop2_timer;
ros::Time last_loop3_timer;
double ros_rate;
//End Template Code: Define Global Variables

//Start User Code: Define Global Variables
ros::Subscriber armed_state_sub;
std::vector<std::string> device_topics;
std::vector<ros::Subscriber> device_subs;
AutoDriveNodeProcess *process;
bool node_initialized;
std::vector<ros::Publisher> pin_pubs;
std::vector<std::string> pin_topics;
ros::Subscriber controlgroup_sub;
std::vector<CGSensorSub> CGSensorSubs;
std::vector<CGCommandSub> CGCommandSubs;
//End User Code: Define Global Variables
#endif
