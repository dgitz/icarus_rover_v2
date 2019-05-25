// Derived class
#include "NetworkTransceiverNodeProcess.cpp"
#include "../include/Base/BaseNode.cpp"
//C System Files
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
//C++ System Files
#include <iomanip>
#include <iostream>
#include <string>
//ROS Base Functionality
//ROS Messages
//Project
#include "udpmessage.h"
/*! \class NetworkTransceiverNode NetworkTransceiverNode.h "NetworkTransceiverNode.h"
 *  \brief This is a NetworkTransceiverNode class.  Used for the networktransceiver_node node.
 *
 */
class NetworkTransceiverNode: public BaseNode {
public:

	const string BASE_NODE_NAME = "networktransceiver_node";

	const uint8_t MAJOR_RELEASE_VERSION = 4;
	const uint8_t MINOR_RELEASE_VERSION = 0;
	const uint8_t BUILD_NUMBER = 0;
	const string FIRMWARE_DESCRIPTION = "Latest Rev: 27-November-2018";

	const uint8_t DIAGNOSTIC_SYSTEM = ROVER;
	const uint8_t DIAGNOSTIC_SUBSYSTEM = ROBOT_CONTROLLER;
	const uint8_t DIAGNOSTIC_COMPONENT = COMMUNICATION_NODE;
	~NetworkTransceiverNode()
	{
	}
	/*! \brief Initialize
	 *
	 */
	bool start(int argc,char **argv);
	NetworkTransceiverNodeProcess* get_process() { return process; }
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
	bool initialize_sendsocket();
	bool initialize_recvsocket();
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
	void diagnostic_Callback(const eros::diagnostic::ConstPtr& msg);
	void resource_Callback(const eros::resource::ConstPtr& msg);
	void firmware_Callback(const eros::firmware::ConstPtr& msg);
	void ArmedState_Callback(const std_msgs::UInt8::ConstPtr& msg);
	void device_callback(std::vector<eros::device> devicelist);
	void subsystem_diagnostic_Callback(const eros::subsystem_diagnostic::ConstPtr& msg);
	//Support Functions
	eros::diagnostic rescan_topics();

	void process_udp_receive();



	std::string base_node_name;
	ros::Subscriber pps1_sub;
	ros::Subscriber command_sub;
	ros::ServiceClient srv_device;
	NetworkTransceiverNodeProcess *process;

	UDPMessageHandler *udpmessagehandler;
	struct sockaddr_in senddevice_addr;
	struct sockaddr_in my_addr;
	struct sockaddr_in remote_addr;
	int senddevice_sock;
	int recvdevice_sock;
	ros::Publisher joy_pub;
	ros::Publisher arm1_joy_pub;
	ros::Publisher arm2_joy_pub;
	ros::Publisher controlgroup_pub;
	std::vector<ros::Subscriber> device_subs;
	std::vector<ros::Subscriber> resource_subs;
	std::vector<ros::Subscriber> diagnostic_subs;
	std::vector<ros::Subscriber> firmware_subs;
	ros::Subscriber armed_disarmed_state_sub;
	ros::Subscriber estop_sub;
	ros::Subscriber subsystem_diagnostic_sub;
	ros::Publisher estop_pub;
	ros::Publisher user_command_pub;
	ros::Publisher ready_to_arm_pub;

};
