// Derived class
#include "ControlGroupNodeProcess.cpp"
#include "../include/Base/BaseNode.cpp"
//C System Files
//C++ System Files
//ROS Base Functionality
//ROS Messages
//Project
/*! \class ControlGroupNode controlgroup_node.h "controlgroup_node.h"
 *  \brief This is a ControlGroupNode class.  Used for the controlgroup_node.
 *
 */
class ControlGroupNode: public BaseNode {
public:

	const string BASE_NODE_NAME = "controlgroup_node";

	const uint8_t MAJOR_RELEASE_VERSION = 0;
	const uint8_t MINOR_RELEASE_VERSION = 0;
	const uint8_t BUILD_NUMBER = 1;
	const string FIRMWARE_DESCRIPTION = "Latest Rev: 29-Sep-2019";

	const uint8_t DIAGNOSTIC_SYSTEM = ROVER;
	const uint8_t DIAGNOSTIC_SUBSYSTEM = ROBOT_CONTROLLER;
	const uint8_t DIAGNOSTIC_COMPONENT = CONTROLLER_NODE;
	~ControlGroupNode()
	{
	}
	/*! \brief Initialize
	 *
	 */
	bool start(int argc,char **argv);
	ControlGroupNodeProcess* get_process() { return process; }
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
	void Signal_Callback(const eros::signal::ConstPtr& t_msg);
	void TuneControlGroup_Callback(const eros::tune_controlgroup::ConstPtr& t_msg);
	bool new_devicemsg(std::string query,eros::device t_device);
	//Support Functions



	std::string base_node_name;
	ros::Subscriber pps1_sub;
	ros::Subscriber command_sub;
	ros::Subscriber tune_controlgroup_sub;
	ros::ServiceClient srv_device;
	ros::ServiceClient srv_pin;
	ControlGroupNodeProcess *process;

	std::vector<ros::Publisher> outputs;
	std::vector<ros::Subscriber> inputs;
	ros::Publisher controlgroup_view_pub;

};
