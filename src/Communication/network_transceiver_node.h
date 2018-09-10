#ifndef NETWORK_TRANSCEIVER_NODE_H
#define NETWORK_TRANSCEIVER_NODE_H
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
#include "network_transceiver_node_process.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/UInt8.h"
#include <icarus_rover_v2/estop.h>
#include <icarus_rover_v2/controlgroup.h>
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
#include <sys/time.h>
#define RECV_BUFFERSIZE 2048
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
icarus_rover_v2::diagnostic rescan_topics(icarus_rover_v2::diagnostic diag);
bool initialize_sendsocket();
bool initialize_recvsocket();
void process_udp_receive();
void diagnostic_Callback(const icarus_rover_v2::diagnostic::ConstPtr& msg);
void resource_Callback(const icarus_rover_v2::resource::ConstPtr& msg);
void firmware_Callback(const icarus_rover_v2::firmware::ConstPtr& msg);
void ArmedState_Callback(const std_msgs::UInt8::ConstPtr& msg);
void estop_Callback(const icarus_rover_v2::estop::ConstPtr& msg);
void device_callback(std::vector<icarus_rover_v2::device> devicelist);
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


bool ready_to_arm;
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
ros::Publisher controlgroup_pub;
std::vector<std::string> resource_topics;
std::vector<std::string> diagnostic_topics;
std::vector<std::string> device_topics;
std::vector<std::string> firmware_topics;
std::vector<ros::Subscriber> device_subs;
std::vector<ros::Subscriber> resource_subs;
std::vector<ros::Subscriber> diagnostic_subs;
std::vector<ros::Subscriber> firmware_subs;
ros::Subscriber armed_disarmed_state_sub;
ros::Subscriber estop_sub;
ros::Publisher estop_pub;
ros::Publisher user_command_pub;
ros::Publisher ready_to_arm_pub;

struct timeval now2;
NetworkTransceiverNodeProcess* process;
//End User Code: Define Global Variables
#endif
