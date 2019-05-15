// Derived class
#include "NavigationNodeProcess.cpp"
#include "../include/Base/BaseNode.cpp"
//C System Files
//C++ System Files
//ROS Base Functionality
//ROS Messages
//Project
/*! \class NavigationNode NavigationNode.h "NavigationNode.h"
 *  \brief This is a NavigationNode class.  Used for the navigation_node node.
 *
 */
class NavigationNode: public BaseNode {
public:

	const string BASE_NODE_NAME = "navigation_node";

	const uint8_t MAJOR_RELEASE_VERSION = 0;
	const uint8_t MINOR_RELEASE_VERSION = 0;
	const uint8_t BUILD_NUMBER = 1;
	const string FIRMWARE_DESCRIPTION = "Latest Rev: 21-March-2019";

	const uint8_t DIAGNOSTIC_SYSTEM = ROVER;
	const uint8_t DIAGNOSTIC_SUBSYSTEM = ROBOT_CONTROLLER;
	const uint8_t DIAGNOSTIC_COMPONENT = NAVIGATION_NODE;
	~NavigationNode()
	{
	}
	/*! \brief Initialize
	 *
	 */
	bool start(int argc,char **argv);
	NavigationNodeProcess* get_process() { return process; }
	void thread_loop();

	//Cleanup
	void cleanup();
private:
	//Initialization Functions
	/*! \brief Read Node Specific Launch Parameters
	 *
	 */
	eros::diagnostic read_launchparameters();
	/*! \brief Setup other pubs/subs, other node specific init stuff
	 *
	 */
	eros::diagnostic finish_initialization();
	//Update Functions
	bool run_001hz();
	bool run_01hz();
	bool run_01hz_noisy();
	bool run_1hz();
	bool run_10hz();
	bool run_loop1();
	bool run_loop2();
	bool run_loop3();
	//Attribute Functions
	//Message Functions
	void PPS1_Callback(const std_msgs::Bool::ConstPtr& t_msg);
	void Command_Callback(const eros::command::ConstPtr& t_msg);
	bool new_devicemsg(std::string query,eros::device t_device);
	//Support Functions
	void print_controlgroups();


	std::string base_node_name;
	ros::Subscriber pps1_sub;
	ros::Subscriber command_sub;
	ros::ServiceClient srv_device;
	NavigationNodeProcess *process;

	std::vector<ros::Publisher> pin_pubs;

};
