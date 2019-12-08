// Derived class
#include "CommandNodeProcess.cpp"
#include "../../include/Base/BaseNode.cpp"
//C System Files
//C++ System Files
//ROS Base Functionality
//ROS Messages

//Project
/*! \class CommandNode CommandNode.h "CommandNode.h"
 *  \brief This is a CommandNode class.  Used for the sample_node node.
 *
 */
class CommandNode: public BaseNode {
public:

	const string BASE_NODE_NAME = "command_node";

	const uint8_t MAJOR_RELEASE_VERSION = 4;
	const uint8_t MINOR_RELEASE_VERSION = 0;
	const uint8_t BUILD_NUMBER = 1;
	const string FIRMWARE_DESCRIPTION = "Latest Rev: 29-November-2018";

	const uint8_t DIAGNOSTIC_SYSTEM = ROVER;
	const uint8_t DIAGNOSTIC_SUBSYSTEM = ROBOT_CONTROLLER;
	const uint8_t DIAGNOSTIC_COMPONENT = CONTROLLER_NODE;
	~CommandNode()
	{
	}
	/*! \brief Initialize
	 *
	 */
	bool start(int argc,char **argv);
	CommandNodeProcess* get_process() { return process; }
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
	void ReadyToArm_Callback(const std_msgs::Bool::ConstPtr& msg,const std::string &topic);
	void User_Command_Callback(const eros::command::ConstPtr& msg);
	void gazeboclock_Callback(const rosgraph_msgs::Clock::ConstPtr& msg);
	void gazeboupdaterate_Callback(const std_msgs::Float64::ConstPtr& msg);
	//Support Functions



	std::string base_node_name;
	ros::Subscriber pps1_sub;
	ros::Subscriber command_sub;
	ros::ServiceClient srv_device;
	CommandNodeProcess *process;

	ros::Publisher armeddisarmed_state_pub;
	ros::Publisher command_pub;
	std::vector<ros::Subscriber> ready_to_arm_subs;
	ros::Subscriber user_command_sub;
	ros::Subscriber clock_sub;
	ros::Publisher systemstate_sub;
	ros::Subscriber gazeboupdaterate_sub;

};
