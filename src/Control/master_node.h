#ifndef MASTER_H
#define MASTER_H
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
#include <icarus_rover_v2/pin.h>
#include <icarus_rover_v2/command.h>
#include <icarus_rover_v2/firmware.h>
#include <icarus_rover_v2/heartbeat.h>
//End Template Code: Includes
//Start User Code: Includes
#include <boost/algorithm/string.hpp>
#include <tinyxml.h>
#include <iostream>
#include <string>
#include <ros/package.h>
#include <stdlib.h>
//End User Code: Includes


//Start Template Code: Function Prototypes
bool initialize(ros::NodeHandle nh);
bool run_fastrate_code();
bool run_mediumrate_code();
bool run_slowrate_code();
bool run_veryslowrate_code();
double measure_time_diff(ros::Time timer_a, ros::Time tiber_b);
void PPS_Callback(const std_msgs::Bool::ConstPtr& msg);
void Command_Callback(const icarus_rover_v2::command& msg);
void print_myDevice();
void print_otherDevices();
void publish_deviceinfo();
std::vector<icarus_rover_v2::diagnostic> check_program_variables();
//End Template Code: Function Prototypes

//Start User Code: Function Prototypes
double read_device_temperature();
bool parse_devicefile(TiXmlDocument doc);
//End User Code: Function Prototypes

//Start Template Code: Global Variables
std::string node_name;
int rate;
std::string verbosity_level;
ros::Subscriber pps_sub;
ros::Subscriber command_sub;
ros::Publisher diagnostic_pub;
ros::Publisher resource_pub;
ros::Publisher firmware_pub;
icarus_rover_v2::diagnostic diagnostic_status;
ros::Publisher device_pub;
icarus_rover_v2::resource resources_used;
Logger *logger;
ResourceMonitor *resourcemonitor;
bool require_pps_to_start;
bool received_pps;
ros::Time fast_timer;
ros::Time medium_timer;
ros::Time slow_timer;
ros::Time veryslow_timer;
ros::Time now;
double mtime;
ros::Publisher heartbeat_pub;
icarus_rover_v2::heartbeat beat;
//End Template Code: Global Variables

//Start User Code: Global Variables
icarus_rover_v2::device myDevice;
std::vector<icarus_rover_v2::device> otherDevices;
std::vector<std::string> NodeList;
double device_temperature;
ros::Publisher device_resourceavail_pub;
ofstream process_file;
//End User Code: Global Variables

#endif
