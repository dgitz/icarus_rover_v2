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
#include <icarus_rover_v2/firmware.h>
#include <icarus_rover_v2/heartbeat.h>
//Start User Code: Includes
#include "visionhelper.h"
#include "opencv2/opencv.hpp"
#include "sensor_msgs/Image.h"
#include <cv_bridge/cv_bridge.h>
#include "opencv2/imgproc/imgproc.hpp"
//End User Code: Includes

//Start User Code: Data Structures
struct imageresample_map
{
	std::string input_topic;
	ros::Subscriber image_sub;
	std::string encoding;
	std::string output_topic;
	ros::Publisher image_pub;
};
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
//Stop Template Code: Function Prototypes

//Start User Code: Function Prototypes
bool check_tasks();
bool capture_image(cv::VideoCapture cap);
bool Edge_Detection(cv::Mat gray_image,int, void*);
void Edge_Detect_Threshold_Callback(const std_msgs::UInt8::ConstPtr& msg);
void ResizeImage_Callback(const sensor_msgs::Image::ConstPtr& msg,const std::string &topicname);
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
std::string operation_mode;
VisionHelper *visionhelper;
cv::VideoCapture capture;
vector<int> compression_params;
ros::Publisher raw_image_pub;
ros::Publisher proc_image_pub;
ros::Subscriber edge_threshold_sub;
int16_t counter;
int edge_detect_threshold;

int image_width;
int image_height;
std::vector<imageresample_map> resample_maps;
//CvCapture *capture;
//End User Code: Define Global Variables



