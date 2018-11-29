// Derived class
#include "MasterNodeProcess.cpp"
#include "../include/Base/BaseNode.cpp"
//C System Files
#include <stdlib.h>
#include <dirent.h>
#include <fcntl.h>      // File control definitions
#include <errno.h>      // Error number definitions
#include <termios.h>    // POSIX terminal control definitions
//C++ System Files
//ROS Base Functionality
//ROS Messages
//Project
/*! \class MasterNode MasterNode.h "MasterNode.h"
 *  \brief This is a MasterNode class.  Used for the master_node node. */
class MasterNode: public BaseNode {
public:

	const string BASE_NODE_NAME = "master_node";

	const uint8_t MAJOR_RELEASE_VERSION = 3;
	const uint8_t MINOR_RELEASE_VERSION = 0;
	const uint8_t BUILD_NUMBER = 0;
	const string FIRMWARE_DESCRIPTION = "Latest Rev: 26-November-2018";

	const uint8_t DIAGNOSTIC_SYSTEM = ROVER;
	const uint8_t DIAGNOSTIC_SUBSYSTEM = ROBOT_CONTROLLER;
	const uint8_t DIAGNOSTIC_COMPONENT = CONTROLLER_NODE;
	~MasterNode()
	{
	}
	/*! \brief Initialize */
	bool start(int argc,char **argv);
	MasterNodeProcess* get_process() { return process; }
	void thread_loop();
	//Cleanup
	void cleanup();


private:
	//Initialization Functions
	/*! \brief Read Node Specific Launch Parameters */
	icarus_rover_v2::diagnostic read_launchparameters();
	/*! \brief Setup other pubs/subs, other node specific init stuff */
	icarus_rover_v2::diagnostic finish_initialization();
	//Update Functions
	bool run_001hz();
	bool run_01hz();
	bool run_1hz();
	bool run_10hz();
	bool run_loop1();
	bool run_loop2();
	bool run_loop3();
	//Attribute Functions
	//Message Functions
	void PPS1_Callback(const std_msgs::Bool::ConstPtr& t_msg);
	void Command_Callback(const icarus_rover_v2::command::ConstPtr& t_msg);
	bool new_devicemsg(std::string query,icarus_rover_v2::device t_device);
	//Support Functions
	/*! \brief Request Information in the DeviceFile */
	bool device_service(icarus_rover_v2::srv_device::Request &req,
					icarus_rover_v2::srv_device::Response &res);
	/*! \brief Request a connection over a limited bus, such as a Serial Port. */
	bool connection_service(icarus_rover_v2::srv_connection::Request &req,
					icarus_rover_v2::srv_connection::Response &res);
	/*! \brief Request a Lever Arm */
	bool leverarm_service(icarus_rover_v2::srv_leverarm::Request &req,
					icarus_rover_v2::srv_leverarm::Response &res);
	double read_device_temperature();
	std::vector<std::string> find_serialports();
	bool check_serialports();
	void print_deviceinfo();


	std::string base_node_name;
	ros::Subscriber pps1_sub;
	ros::Publisher device_resourceavail_pub;\
	std::vector<icarus_rover_v2::device> devices_to_publish;
	ros::Subscriber command_sub;
	MasterNodeProcess *process;
	ros::ServiceServer device_srv;
	ros::ServiceServer connection_srv;
	ros::ServiceServer leverarm_srv;
	SerialMessageHandler *serialmessagehandler;

};
