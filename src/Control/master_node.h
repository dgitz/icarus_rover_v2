#ifndef MASTER_H
#define MASTER_H
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "logger.h"
#include <boost/algorithm/string.hpp>
#include <std_msgs/Bool.h>
#include <sstream>
#include <stdlib.h>
#include <icarus_rover_v2/Definitions.h>
#include <icarus_rover_v2/diagnostic.h>
#include <icarus_rover_v2/device.h>
#include <icarus_rover_v2/resource.h>
#include <tinyxml.h>


//Start Template Code: Function Prototypes
bool initialize(ros::NodeHandle nh);
bool run_fastrate_code();
bool run_mediumrate_code();
bool run_slowrate_code();
bool run_veryslowrate_code();
double measure_time_diff(ros::Time timer_a, ros::Time tiber_b);
void parse_devicefile(TiXmlDocument doc);
void PPS_Callback(const std_msgs::Bool::ConstPtr& msg);
void print_myDevice();
void print_otherDevices();
void publish_deviceinfo();
int get_pid();
bool check_resources(int procid);

std::string node_name;
int rate;
std::string verbosity_level;
ros::Subscriber pps_sub;
ros::Publisher diagnostic_pub;
ros::Publisher resource_pub;
icarus_rover_v2::diagnostic diagnostic_status;
ros::Publisher device_pub;
icarus_rover_v2::resource resources_used;
Logger *logger;
bool require_pps_to_start;
bool received_pps;
ros::Time fast_timer;
ros::Time medium_timer;
ros::Time slow_timer;
ros::Time veryslow_timer;
ros::Time now;
double mtime;
int pid;
icarus_rover_v2::device myDevice;
std::vector<icarus_rover_v2::device> otherDevices;

std::vector<std::string> NodeList;
//End Template Code: Function Definitions
#endif