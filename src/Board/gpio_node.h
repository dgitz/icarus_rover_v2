#ifndef GPIONODE_H
#define GPIONODE_H
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
#include <icarus_rover_v2/command.h>
#include <icarus_rover_v2/firmware.h>
#include <icarus_rover_v2/heartbeat.h>
//End Template Code: Includes

//Start User Code: Includes
#include "gpio_node_process.h"
#include <stdio.h>
#include <string.h>
#include <serialmessage.h>
#include <unistd.h>			//Used for UART
#include <fcntl.h>			//Used for UART
#include <termios.h>		//Used for UART
#include <boost/thread.hpp>

//End User Code: Includes
//Start User Code: Defines
#define USE_UART 1
#define WARN_ON_SOFTWARE_NOT_IMPLEMENTED 0
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
ros::Time last_message_received_time;
GPIONodeProcess *process;
ros::Publisher digitalinput_pub;
ros::Subscriber pwmoutput_sub;
ros::Subscriber digitaloutput_sub;
ros::Publisher analoginput_pub;
ros::Publisher forcesensorinput_pub;
ros::Time gpio_comm_test_start;
bool checking_gpio_comm;
int message_receive_counter;
int device_fid;

int current_num;
int last_num;
int missed_counter;
int bad_checksum_counter;
int good_checksum_counter;
bool new_message;
int packet_type;
unsigned char packet[8];
bool message_started;
bool message_completed;
int message_buffer_index;
unsigned char message_buffer[64];
int packet_length;


//End User Code: Define Global Variables
#endif
