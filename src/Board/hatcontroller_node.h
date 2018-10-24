#ifndef HATCONTROLLERNODE_H
#define HATCONTROLLERNODE_H
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
#include <wiringPiI2C.h>
#include "hatcontroller_node_process.h"
#include "Driver/ServoHatDriver.h"
#include "Driver/TerminalHatDriver.h"
#include "Driver/GPIOHatDriver.h"
#include <stdio.h>
#include <string.h>
#include <serialmessage.h>
#include <i2cmessage.h>
#include <unistd.h>			//Used for UART
#include <fcntl.h>			//Used for UART
#include <termios.h>		//Used for UART
#include <sys/ioctl.h>
#include <boost/thread.hpp>
#include <dirent.h>
#include <sys/types.h>
//End User Code: Includes

//Start User Code: Data Structures
//End User Code: Data Structures

//Start Template Code: Function Prototypes
bool initializenode();
void PPS01_Callback(const std_msgs::Bool::ConstPtr& msg);
void PPS1_Callback(const std_msgs::Bool::ConstPtr& msg);
double measure_time_diff(ros::Time timer_a, ros::Time tiber_b);
bool new_devicemsg(std::string query,icarus_rover_v2::device device);
void Command_Callback(const icarus_rover_v2::command& msg);
bool run_loop3_code();
bool run_loop2_code();
bool run_loop1_code();
bool run_10Hz_code();
void signalinterrupt_handler(int sig);
//End Template Code: Function Prototypes

//Start User Code: Function Prototypes
void ArmedState_Callback(const std_msgs::UInt8::ConstPtr& msg);
void signalinterrupt_handler(int sig);
//End User Code: Function Prototypes

//Start Template Code: Define Global variables
boost::shared_ptr<ros::NodeHandle> n;
ros::ServiceClient srv_device;
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

//Start User Code: Define Global Variables
ros::Time last_message_received_time;
HatControllerNodeProcess* process;
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
std::vector<boost::thread> threads;

int current_num;
int last_num;
int missed_counter;
bool new_message;
bool message_started;
bool message_completed;
int message_buffer_index;
unsigned char message_buffer[64];
int packet_length;
ros::Subscriber armed_state_sub;
bool ready_to_arm;
ros::Publisher ready_to_arm_pub;

std::vector<ServoHatDriver> ServoHats;
std::vector<GPIOHatDriver> GPIOHats;
TerminalHatDriver TerminalHat;
I2CMessageHandler *i2cmessagehandler;
std::vector<ros::Publisher> signal_sensor_pubs;
std::vector<std::string> signal_sensor_names;
icarus_rover_v2::device myDevice;
std::vector<ros::Publisher> digitalinput_pubs;
std::vector<std::string> digitalinput_names;
//TerminalHatDriver TerminalHat;
//End User Code: Define Global Variables
#endif
