// Derived class
#include "SafetyNodeProcess.cpp"
#include "../../include/Base/BaseNode.cpp"
//C System Files
//C++ System Files
//ROS Base Functionality
//ROS Messages
//Project
#include "../Board/Driver/TerminalHatDriver.h"
/*! \class SafetyNode SafetyNode.h "SafetyNode.h"
 *  \brief This is a SafetyNode class.  Used for the safety_node node.
 *
 */
class SafetyNode: public BaseNode {
public:

	const string BASE_NODE_NAME = "safety_node";

	const uint8_t MAJOR_RELEASE_VERSION = 2;
	const uint8_t MINOR_RELEASE_VERSION = 0;
	const uint8_t BUILD_NUMBER = 1;
	const string FIRMWARE_DESCRIPTION = "Latest Rev: 2-December-2018";

	const uint8_t DIAGNOSTIC_SYSTEM = ROVER;
	const uint8_t DIAGNOSTIC_SUBSYSTEM = ROBOT_CONTROLLER;
	const uint8_t DIAGNOSTIC_COMPONENT = DIAGNOSTIC_NODE;
	~SafetyNode()
	{
	}
	/*! \brief Initialize
	 *
	 */
	bool start(int argc,char **argv);
	SafetyNodeProcess* get_process() { return process; }
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



	std::string base_node_name;
	ros::Subscriber pps1_sub;
	ros::Subscriber command_sub;
	ros::ServiceClient srv_device;
	SafetyNodeProcess *process;
	TerminalHatDriver TerminalHat;
	std::vector<ros::Subscriber> diagnostic_subs;

};
