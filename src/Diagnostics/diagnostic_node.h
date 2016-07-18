#include "ros/ros.h"
#include "std_msgs/String.h"
#include "logger.h"
#include <boost/algorithm/string.hpp>
#include <std_msgs/Bool.h>
#include <sstream>
#include <stdlib.h>
#include <icarus_rover_v2/Definitions.h>
#include <icarus_rover_v2/diagnostic.h>
#include <icarus_rover_v2/device.h>
#include <icarus_rover_v2/resource.h>

//Start User Code: Data Structures
struct Task
{
	std::string Task_Name;
	ros::Time last_diagnostic_received;
	ros::Time last_resource_received;
	int16_t PID;
	int16_t CPU_Perc;
	int64_t RAM_MB;
	uint8_t last_diagnostic_level;
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
int get_pid();
bool check_resources(int procid);
//Stop Template Code: Function Prototypes

//Start User Code: Function Prototypes
bool check_tasks();
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
icarus_rover_v2::diagnostic diagnostic_status;
icarus_rover_v2::device myDevice;
icarus_rover_v2::resource resources_used;
Logger *logger;
bool require_pps_to_start = false;
bool received_pps = false;
ros::Time fast_timer; //50 Hz
ros::Time medium_timer; //10 Hz
ros::Time slow_timer; //1 Hz
ros::Time veryslow_timer; //1 Hz
ros::Time now;
double mtime;
bool device_initialized;
int pid;
//End Template Code: Define Global Variables

//Start User Code: Define Global Variables
std::vector<Task> TaskList;
int RAM_usage_threshold_MB;
int CPU_usage_threshold_percent;
ros::Time boot_time;
//End User Code: Define Global Variables



