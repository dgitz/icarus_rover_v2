#ifndef MASTER_H
#define MASTER_H
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
#include "master_node_process.h"
#include <boost/algorithm/string.hpp>
#include "serialmessage.h"
#include <string>
#include <ros/package.h>
#include <stdlib.h>
#include <dirent.h>
#include <errno.h>

#include <fcntl.h>      // File control definitions
#include <errno.h>      // Error number definitions
#include <termios.h>    // POSIX terminal control definitions
//End User Code: Includes

//Start User Code: Data Structures
//End User Code: Data Structures

//Start Template Code: Function Prototypes
bool initializenode();
void PPS01_Callback(const std_msgs::Bool::ConstPtr& msg);
void PPS1_Callback(const std_msgs::Bool::ConstPtr& msg);
double measure_time_diff(ros::Time timer_a, ros::Time tiber_b);
void Command_Callback(const icarus_rover_v2::command& msg);
bool run_loop3_code();
bool run_loop2_code();
bool run_loop1_code();
bool run_10Hz_code();
void signalinterrupt_handler(int sig);
//End Template Code: Function Prototypes

//Start User Code: Function Prototypes
bool device_service(icarus_rover_v2::srv_device::Request &req,
				icarus_rover_v2::srv_device::Response &res);
bool connection_service(icarus_rover_v2::srv_connection::Request &req,
				icarus_rover_v2::srv_connection::Response &res);
bool leverarm_service(icarus_rover_v2::srv_leverarm::Request &req,
				icarus_rover_v2::srv_leverarm::Response &res);
double read_device_temperature();
std::vector<std::string> find_serialports();
bool check_serialports();
//End User Code: Function Prototypes


//Start Template Code: Define Global variables
boost::shared_ptr<ros::NodeHandle> n;
std::string node_name;
std::string verbosity_level;
ros::Subscriber pps01_sub;
ros::Subscriber pps1_sub;
ros::Publisher diagnostic_pub;
ros::Publisher resource_pub;
ros::Subscriber command_sub;
ros::Publisher firmware_pub;
icarus_rover_v2::resource resources_used;
Logger *logger;
ResourceMonitor *resourcemonitor;
bool require_pps_to_start;
bool received_pps;
ros::Time boot_time;
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

//Start User Code: Global Variables
MasterNodeProcess *process;
SerialMessageHandler *serialmessagehandler;
std::vector<icarus_rover_v2::device> devices_to_publish;
std::vector<std::string> NodeList;
double device_temperature;
ros::Publisher device_resourceavail_pub;
ofstream process_file;
ros::ServiceServer device_srv;
ros::ServiceServer connection_srv;
ros::ServiceServer leverarm_srv;
//End User Code: Global Variables
#endif
