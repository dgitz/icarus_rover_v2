OBSOLETE!!!
#ifndef BOARDCONTROLLERNODE_H
#define BOARDCONTROLLERNODE_H
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
#define USE_UART 0
#define WARN_ON_SOFTWARE_NOT_IMPLEMENTED 0
//End User Code: Defines

//Start User Code: Includes
#include "boardcontroller_node_process.h"
#include <stdio.h>
#include <string.h>
#include <serialmessage.h>
#include <unistd.h>			//Used for UART
#include <fcntl.h>			//Used for UART
#include <termios.h>		//Used for UART
#include <sys/ioctl.h>
#include <boost/thread.hpp>
#include <dirent.h>
#include <sys/types.h>
//End User Code: Includes

//Start User Code: Data Structures
struct Task
{
	std::string Task_Name;
	ros::Time last_diagnostic_received;
	uint8_t last_diagnostic_level;
	std::string diagnostic_topic;
	ros::Subscriber diagnostic_sub;
};
struct UsbDevice
{
	int device_fid;
	std::string location;
	int valid; //0 is no, 1 is yes, 2 is unknown
	int boardcontrollernode_id;
	long long bytesreceived;
	long long bytestransmitted;
	int index;
	long long good_checksum_counter;
	long long bad_checksum_counter;
};
//End User Code: Data Structures

//Start Template Code: Function Prototypes
bool initializenode();
void PPS01_Callback(const std_msgs::Bool::ConstPtr& msg);
void PPS1_Callback(const std_msgs::Bool::ConstPtr& msg);
void PPS10_Callback(const std_msgs::Bool::ConstPtr& msg);
void PPS100_Callback(const std_msgs::Bool::ConstPtr& msg);
void PPS1000_Callback(const std_msgs::Bool::ConstPtr& msg);
double measure_time_diff(ros::Time timer_a, ros::Time tiber_b);
void Device_Callback(const icarus_rover_v2::device::ConstPtr& msg);
void Command_Callback(const icarus_rover_v2::command& msg);
void diagnostic_Callback(const icarus_rover_v2::diagnostic::ConstPtr& msg,const std::string &topicname);
std::vector<icarus_rover_v2::diagnostic> check_program_variables();
void signalinterrupt_handler(int sig);
int checkmessage();
icarus_rover_v2::diagnostic rescan_topics(icarus_rover_v2::diagnostic diag);

//End Template Code: Function Prototypes

//Start User Code: Function Prototypes
void ArmedState_Callback(const std_msgs::UInt8::ConstPtr& msg);
void signalinterrupt_handler(int sig);
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
boost::shared_ptr<ros::NodeHandle> n;
ros::Time last_message_received_time;
std::vector<BoardControllerNodeProcess> boardprocesses;
ros::Publisher digitalinput_pub;
ros::Subscriber pwmoutput_sub;
ros::Time last_pwmoutput_sub_time;
std::vector<ros::Subscriber> diagnostic_subs;
ros::Time last_diagnostic_sub_time;
ros::Subscriber digitaloutput_sub;
ros::Time last_digitaloutput_time;
ros::Publisher analoginput_pub;
ros::Publisher forcesensorinput_pub;
ros::Time gpio_comm_test_start;
bool checking_gpio_comm;
int message_receive_counter;
std::vector<UsbDevice> UsbDevices;
std::vector<boost::thread> threads;

int current_num;
int last_num;
int missed_counter;
bool new_message;
//int packet_type;
//unsigned char packet[8];
bool message_started;
bool message_completed;
int message_buffer_index;
unsigned char message_buffer[64];
int packet_length;
ros::Subscriber armed_state_sub;
bool ready_to_arm;
ros::Publisher ready_to_arm_pub;
std::vector<Task> TaskList;
//End User Code: Define Global Variables
#endif
