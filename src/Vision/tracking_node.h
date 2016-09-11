#ifndef TRACKING_H
#define TRACKING_H
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
#include <icarus_rover_v2/pin.h>
#include <icarus_rover_v2/command.h>
#include <icarus_rover_v2/firmware.h>
//End Template Code: Includes

//Start User Code: Includes
#include "visionhelper.h"
#include "opencv2/opencv.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "sensor_msgs/Image.h"
#include <cv_bridge/cv_bridge.h>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/nonfree/features2d.hpp"
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
void Command_Callback(const icarus_rover_v2::command& msg);
std::vector<icarus_rover_v2::diagnostic> check_program_variables();
//End Template Code: Function Prototypes

//Start User Code: Function Prototypes
void Image_Callback(const sensor_msgs::Image::ConstPtr& msg);
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
double mtime;
bool device_initialized;
//End Template Code: Define Global Variables

//Start User Code: Define Global Variables
VisionHelper *visionhelper;
ros::Subscriber image_sub;
std::string image_to_track_topic;
std::vector<cv::Mat> target_images;
ros::Publisher tracked_image_pub;
std::vector<std::vector<cv::KeyPoint> > keypoints_target_images;
std::vector<cv::Mat> descriptors_target_images;
int minHessian;
std::vector<cv::SurfFeatureDetector> target_feature_detectors;
std::vector<cv::SurfDescriptorExtractor> target_descriptor_extractors;
//End User Code: Define Global Variables
#endif
