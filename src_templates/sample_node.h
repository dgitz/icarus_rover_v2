// Derived class
#include "SampleNodeProcess.cpp"
#include "../include/Base/BaseNode.cpp"
//C System Files
//C++ System Files
//ROS Base Functionality
//ROS Messages
//Project
/*! \class SampleNode SampleNode.h "SampleNode.h"
 *  \brief This is a SampleNode class.  Used for the sample_node node.
 *
 */
class SampleNode: public BaseNode {
public:

	const string BASE_NODE_NAME = "sample_node";

	const uint8_t MAJOR_RELEASE_VERSION = 4;
	const uint8_t MINOR_RELEASE_VERSION = 1;
	const uint8_t BUILD_NUMBER = 0;
	const string FIRMWARE_DESCRIPTION = "Latest Rev: 3-Aug-2019";

	const uint8_t DIAGNOSTIC_SYSTEM = ROVER;
	const uint8_t DIAGNOSTIC_SUBSYSTEM = ROBOT_CONTROLLER;
	const uint8_t DIAGNOSTIC_COMPONENT = CONTROLLER_NODE;
	~SampleNode()
	{
	}
	/*! \brief Initialize
	 *
	 */
	bool start(int argc,char **argv);
	SampleNodeProcess* get_process() { return process; }
	void thread_loop();

	//Cleanup
	void cleanup();
private:
	//Initialization Functions
	/*! \brief Read Node Specific Launch Parameters
	 *
	 */
	eros::diagnostic read_launchparameters();
	eros::diagnostic rescan_topics();
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
	SampleNodeProcess *process;
	std::vector<ros::Subscriber> multiple_subs;

};
