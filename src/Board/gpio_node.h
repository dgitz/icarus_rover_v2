#ifndef GPIONODE_H
#define GPIONODE_H
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
#define USE_UART 1
#define WARN_ON_SOFTWARE_NOT_IMPLEMENTED 0
//End User Code: Defines

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

//Start User Code: Data Structures
//End User Code: Data Structures

//Start Template Code: Function Prototypes
bool initialize(ros::NodeHandle nh);
bool run_fastrate_code();
bool run_mediumrate_code();
bool run_slowrate_code();
bool run_veryslowrate_code();
double measure_time_diff(ros::Time timer_a, ros::Time tiber_b);
void PPS_Callback(const std_msgs::Bool::ConstPtr& msg);
void Device_Callback(const icarus_rover_v2::device::ConstPtr& msg);
void Command_Callback(const icarus_rover_v2::command& msg);
std::vector<icarus_rover_v2::diagnostic> check_program_variables();
void signalinterrupt_handler(int sig);
//End Template Code: Function Prototypes

//Start User Code: Function Prototypes
void ArmedState_Callback(const std_msgs::UInt8::ConstPtr& msg);
void signalinterrupt_handler(int sig);
//End User Code: Function Prototypes


//Start Template Code: Define Global variables
std::string node_name;
int rate;
std::string verbosity_level;
ros::Subscriber pps_sub;  
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
ros::Time fast_timer;
ros::Time medium_timer;
ros::Time slow_timer;
ros::Time veryslow_timer;
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
ros::Subscriber armed_state_sub;
bool ready_to_arm;
ros::Publisher ready_to_arm_pub;
//End User Code: Define Global Variables
#endif
