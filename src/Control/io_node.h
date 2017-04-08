#ifndef IONODE_H
#define IONODE_H
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

//Start User Code: Includes
#include "io_node_process.h"
//End User Code: Includes

//Start User Code: Defines
#define WARN_ON_SOFTWARE_NOT_IMPLEMENTED 1
//End User Code: Defines

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
void DigitalOutput_Callback(const icarus_rover_v2::pin::ConstPtr& msg);
void ArmedState_Callback(const std_msgs::UInt8::ConstPtr& msg);
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
IONodeProcess *process;
ros::Publisher digitalinput_pub;
ros::Subscriber digitaloutput_sub;
ros::Subscriber armed_state_sub;
uint8_t remote_armed_state;
uint8_t local_armed_state;
uint8_t arm_command;
bool enable_actuators;
bool last_enable_actuators;
//End User Code: Define Global Variables
#endif
