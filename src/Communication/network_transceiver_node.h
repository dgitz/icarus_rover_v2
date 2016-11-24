#ifndef NETWORK_TRANSCEIVER_NODE_H
#define NETWORK_TRANSCEIVER_NODE_H
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
#include <icarus_rover_v2/firmware.h>
#include <icarus_rover_v2/heartbeat.h>
//End Template Code: Includes

//Start User Code: Includes
#include "sensor_msgs/Joy.h"
#include "std_msgs/UInt8.h"
#include "udpmessage.h"
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <iomanip>
#include <iostream>
#include <string>
#include <boost/thread.hpp>
#define RECV_BUFFERSIZE 2048
//End User Code: Includes


//Start Template Code: Function Prototypes
bool initializenode();//ros::NodeHandle nh);
bool run_fastrate_code();
bool run_mediumrate_code();
bool run_slowrate_code();
bool run_veryslowrate_code();
double measure_time_diff(ros::Time timer_a, ros::Time tiber_b);
void PPS_Callback(const std_msgs::Bool::ConstPtr& msg);
void Device_Callback(const icarus_rover_v2::device::ConstPtr& msg);
//Stop Template Code: Function Prototypes

//Start User Code: Function Prototypes
icarus_rover_v2::diagnostic rescan_topics(icarus_rover_v2::diagnostic diag);
bool initialize_sendsocket();
bool initialize_recvsocket();
void process_udp_receive();
void diagnostic_Callback(const icarus_rover_v2::diagnostic::ConstPtr& msg);
void device_Callback(const icarus_rover_v2::device::ConstPtr& msg);
void resource_Callback(const icarus_rover_v2::resource::ConstPtr& msg);
void ArmedState_Callback(const std_msgs::UInt8::ConstPtr& msg);
bool check_remoteHeartbeats();
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
double mtime;
bool device_initialized;
char hostname[1024];
ros::Publisher heartbeat_pub;
icarus_rover_v2::heartbeat beat;
//End Template Code: Define Global Variables

//Start User Code: Define Global Variables
struct RemoteDevice
{
	std::string Name;
	double current_beatepoch_sec;
	double expected_beatepoch_sec;
	double offset_sec;

};
bool remote_heartbeat_pass;
bool ready_to_arm;
boost::shared_ptr<ros::NodeHandle> n;
UDPMessageHandler *udpmessagehandler;
struct sockaddr_in senddevice_addr;
struct sockaddr_in my_addr;
struct sockaddr_in remote_addr;
int senddevice_sock;
int recvdevice_sock;
std::string send_multicast_group;
int send_multicast_port;
int recv_unicast_port;
std::string Mode;
ros::Publisher joy_pub;
ros::Publisher arm1_joy_pub;
ros::Publisher arm2_joy_pub;
std::vector<std::string> resource_topics;
std::vector<std::string> diagnostic_topics;
std::vector<std::string> device_topics;
std::vector<ros::Subscriber> device_subs;
std::vector<ros::Subscriber> resource_subs;
std::vector<ros::Subscriber> diagnostic_subs;
ros::Subscriber armed_disarmed_state_sub;
ros::Publisher arm_command_pub;
ros::Publisher ready_to_arm_pub;
std::vector<RemoteDevice> remote_devices;
//End User Code: Define Global Variables
#endif
