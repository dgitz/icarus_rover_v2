#ifndef TIMEMASTERNODE_H
#define TIMEMASTERNODE_H
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
//End User Code: Data Structures

//Start Template Code: Function Prototypes
bool initialize(ros::NodeHandle nh);
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
bool run_1000Hz_code();
bool run_100Hz_code();
bool run_10Hz_code();
bool run_1Hz_code();
bool run_01Hz_code();
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
ros::Publisher pps01_pub;
ros::Publisher pps1_pub;
ros::Publisher pps10_pub;
ros::Publisher pps100_pub;
ros::Publisher pps1000_pub;
ros::Time pps01_timer;
ros::Time pps1_timer;
ros::Time pps10_timer;
ros::Time pps100_timer;
ros::Time pps1000_timer;
ros::Time last_pps01_timer;
ros::Time last_pps1_timer;
ros::Time last_pps10_timer;
ros::Time last_pps100_timer;
ros::Time last_pps1000_timer;
double pps01_delay;
double pps1_delay;
double pps10_delay;
double pps100_delay;
double pps1000_delay;
bool publish_1pps;
//End User Code: Define Global Variables
#endif