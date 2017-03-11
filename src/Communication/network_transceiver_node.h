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
#include "sensor_msgs/Joy.h"
#include "std_msgs/UInt8.h"
#include "ipmessage.h"
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <iomanip>
#include <iostream>
#include <string>
#include <boost/thread.hpp>
#include <cv_bridge/cv_bridge.h>
#include "sensor_msgs/Image.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#define RECV_BUFFERSIZE 2048
//End User Code: Includes

//Start User Code: Data Structures
//End User Code: Data Structures

//Start Template Code: Function Prototypes
bool initializenode();//ros::NodeHandle nh);
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
icarus_rover_v2::diagnostic rescan_topics(icarus_rover_v2::diagnostic diag);
bool initialize_udp_sendsocket();
bool initialize_udp_recvsocket();
bool initialize_tcp_socket();
void process_udp_receive();
void diagnostic_Callback(const icarus_rover_v2::diagnostic::ConstPtr& msg);
void device_Callback(const icarus_rover_v2::device::ConstPtr& msg);
void resource_Callback(const icarus_rover_v2::resource::ConstPtr& msg);
void ArmedState_Callback(const std_msgs::UInt8::ConstPtr& msg);
bool check_remoteHeartbeats();
void image_Callback(const sensor_msgs::Image::ConstPtr& msg);
void tcpprocess(int);
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
struct RemoteDevice
{
	std::string Name;
	double current_beatepoch_sec;
	double expected_beatepoch_sec;
	double offset_sec;

};
struct ImageInfo
{
    bool image_ready;
    cv::Mat image;
    uint32_t width;
    uint32_t height;
    std::string name;
    uint8_t encoding;
};
bool remote_heartbeat_pass;
bool ready_to_arm;
boost::shared_ptr<ros::NodeHandle> n;
IPMessageHandler *ipmessagehandler;
struct sockaddr_in udp_senddevice_addr;
struct sockaddr_in udp_my_addr;
struct sockaddr_in udp_remote_addr;
int udp_senddevice_sock;
int udp_recvdevice_sock;
int tcp_device_sock;
socklen_t clilen;
struct sockaddr_in cli_addr;
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
std::vector<std::string> image_topics;
std::vector<ros::Subscriber> image_subs;
std::vector<int> tcpprocesses_to_close;
cv::Mat image_to_send;
bool image_ready;
std::vector<ImageInfo> image_streams;
//End User Code: Define Global Variables
#endif
