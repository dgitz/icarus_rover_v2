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
//End Template Code: Includes

//Start User Code: Includes
#include <stdio.h>
#include <string.h>
#include <serialmessage.h>
#include <unistd.h>			//Used for UART
#include <fcntl.h>			//Used for UART
#include <termios.h>		//Used for UART
#include <boost/thread.hpp>
struct Port_Info{
	std::string PortName;
	bool Available[8];
	int Value[8];
	int Mode[8];
};
//End User Code: Includes


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
bool initialize_gpioboard_ports();
bool configure_pin(std::string BoardName,std::string Port, uint8_t Number, std::string Function);
void DigitalOutput_Callback(const icarus_rover_v2::pin::ConstPtr& msg);
void PwmOutput_Callback(const icarus_rover_v2::pin::ConstPtr& msg);
bool publish_port_info(Port_Info pi);
std::string map_PinFunction_ToString(int function);
int map_PinFunction_ToInt(std::string Function);
void process_message_thread();
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
//End Template Code: Define Global Variables

//Start User Code: Define Global Variables
ros::Publisher digitalinput_pub;
ros::Subscriber pwmoutput_sub;
ros::Subscriber digitaloutput_sub;
ros::Publisher analoginput_pub;
ros::Publisher forcesensorinput_pub;
int gpio_board_mode;
ros::Time gpio_comm_test_start;
bool checking_gpio_comm;
int message_receive_counter;
SerialMessageHandler *serialmessagehandler;
int device_fid;
std::vector<icarus_rover_v2::device> boards;
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

Port_Info DIO_PortA;
Port_Info DIO_PortB;
Port_Info ANA_PortA;
Port_Info ANA_PortB;
//End User Code: Define Global Variables
#endif
