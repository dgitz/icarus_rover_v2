#ifndef WEBSERVER_H
#define WEBSERVER_H
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
#include <boost/signals2/mutex.hpp>

#include "MongooseHelper.h"
#include "webserver_node_process.h"
//End User Code: Includes

//Start User Code: Data Structures
class WebController: public Mongoose::IMongooseCallback
{
public:
	WebController();
	bool initialize(std::string doc_path,int port);
	void HttpServicePostCommandCallback(Mongoose::CHttpConnWrapper& httpConn, const std::string& URI, const std::string& URIParameters,
			const char *postBody, size_t postBodyLength, bool& keepOpen );
	bool update(double dt);
	bool refreshConn(Mongoose::CHttpConnWrapper& httpConn);

private:
	bool queryResponse(uint8_t msg,Mongoose::CHttpConnWrapper& httpConn,const char *postBody, size_t postBodyLength,bool& keepOpen);
	boost::signals2::mutex             m_currentConn_Mutex;    ///< avoid race condition for mconnection resource.
	Mongoose::CHttpConnWrapper         *m_currentConn;         ///< current connection being serviced
	Mongoose::MongooseHelper m_mongoose;
};


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

bool refreshConn(Mongoose::CHttpConnWrapper& httpConn);


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
WebServerNodeProcess *process;
WebController wc;
bool processing_command;
//End User Code: Define Global Variables
#endif
