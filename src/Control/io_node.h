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
#include "Definitions.h"
#include <icarus_rover_v2/diagnostic.h>
#include <icarus_rover_v2/device.h>
#include <icarus_rover_v2/resource.h>
#include <icarus_rover_v2/command.h>
#include <icarus_rover_v2/firmware.h>
#include <icarus_rover_v2/heartbeat.h>
//End Template Code: Includes

//Start User Code: Includes
#include "io_node_process.h"
//End User Code: Includes
//Start User Code: Defines
#define WARN_ON_SOFTWARE_NOT_IMPLEMENTED 1
//End User Code: Defines


//Start Template Code: Function Prototypes
bool initialize(ros::NodeHandle nh);
bool run_fastrate_code();
bool run_mediumrate_code();
bool run_slowrate_code();
bool run_veryslowrate_code();
double measure_time_diff(ros::Time timer_a, ros::Time tiber_b);
void PPS_Callback(const std_msgs::Bool::ConstPtr& msg);
void Device_Callback(const icarus_rover_v2::device::ConstPtr& msg);
std::vector<icarus_rover_v2::diagnostic> check_program_variables();
//Stop Template Code: Function Prototypes

//Start User Code: Function Prototypes
void DigitalOutput_Callback(const icarus_rover_v2::pin::ConstPtr& msg);
void ArmedState_Callback(const std_msgs::UInt8::ConstPtr& msg);
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
ros::Subscriber command_sub;
ros::Publisher firmware_pub;
ros::Publisher heartbeat_pub;
icarus_rover_v2::heartbeat beat;
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
ros::Time boot_time;
double mtime;
bool device_initialized;
char hostname[1024];
bool kill_node;
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
