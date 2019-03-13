// Derived class
#include "TopicRemapperNodeProcess.cpp"
#include "../include/Base/BaseNode.cpp"
//C System Files
//C++ System Files
//ROS Base Functionality
//ROS Messages
//Project
/*! \class TopicRemapperNode TopicRemapperNode.h "TopicRemapperNode.h"
 *  \brief This is a TopicRemapperNode class.  Used for the topicremapper_node node.
 *
 */
class TopicRemapperNode: public BaseNode {
public:

	const string BASE_NODE_NAME = "topicremapper_node";

	const uint8_t MAJOR_RELEASE_VERSION = 3;
	const uint8_t MINOR_RELEASE_VERSION = 0;
	const uint8_t BUILD_NUMBER = 1;
	const string FIRMWARE_DESCRIPTION = "Latest Rev: 3-December-2018";

	const uint8_t DIAGNOSTIC_SYSTEM = ROVER;
	const uint8_t DIAGNOSTIC_SUBSYSTEM = ROBOT_CONTROLLER;
	const uint8_t DIAGNOSTIC_COMPONENT = COMMUNICATION_NODE;
	~TopicRemapperNode()
	{
	}
	/*! \brief Initialize
	 *
	 */
	bool start(int argc,char **argv);
	TopicRemapperNodeProcess* get_process() { return process; }
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
	void Joystick_Callback(const sensor_msgs::Joy::ConstPtr& msg,const std::string &topic);
	//Support Functions



	std::string base_node_name;
	ros::Subscriber pps1_sub;
	ros::Subscriber command_sub;
	ros::ServiceClient srv_device;
	TopicRemapperNodeProcess *process;
	std::vector<ros::Subscriber> subs;
	ros::Subscriber joy_sub;
	std::vector<ros::Publisher> pin_pubs;
	std::vector<ros::Publisher> float32_pubs;

};
